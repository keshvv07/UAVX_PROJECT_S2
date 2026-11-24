"""Ground Control Station: tasking, area coverage, relay chains, the ferry decision, watchdog.

Talks to the swarm only through the emulated mesh radio, so everything it knows can be
stale or missing - which is the point of the resilience levels.

What it decides

  Coverage     with a mission area on the map, the area is split into compact sectors
               (coverage.py) and handed out as `sweep` tasks: a surveyor covers ground with
               its camera rather than flying to coordinates somebody already knew. This is
               what the organisers' Q19/Q20 force - the PoIs appear during the mission, at
               times nobody tells the swarm.

  PoIs         an announced or newly found PoI becomes a `survey` task. Each pending PoI,
               highest priority first, goes to the UAV with the earliest finish time
               (queue + travel + survey). Priority >= emergency_priority goes to the head of
               the nearest UAV's queue and pre-empts its transit.

  Relays and   the organisers' 100 m range (Q13) against a search area 365 m away (Q6)
  the ferry    means a live link to the far corner needs seven relays and a 4-5 UAV fleet
               cannot buy that. So the GCS plans the chain it CAN afford (relay_plan.py):

                 LIVE   the chain to the focus target fits: relays take their anchors
                        before the surveyor leaves coverage and the report arrives inside
                        the organisers' 10 s (Q23).
                 FERRY  it does not fit: relays go as far out as the fleet allows, the
                        nearest UAV surveys alone, and the chain tip and handoff point ride
                        along in its task message so it can ferry the reports back to the
                        chain on its own - the GCS cannot talk to it out there.
                 BLIND  nothing can be spared: plain store-and-forward.

               In quiet sweeping, `base_relays` UAVs hold the chain that reaches into the
               near edge of the forest. When a PoI of priority >= escort_priority appears,
               the fleet switches to escort: the nearest UAV surveys it and the others
               become its chain, for as long as that takes.

  Watchdog     a UAV is heard through its 1 Hz STATUS broadcast, directly or forwarded.
               Silence for fail_timeout_s is judged against where the UAV was and what it
               was doing: if the swarm's own topology says it should have been audible
               (predicted route PDR >= alive_pdr) it is LOST and its work is re-allocated
               now; otherwise it is OUT_OF_RANGE, presumed alive and buffering, and
               declared LOST only after lost_timeout_s.

  Sightings    a UAV that finds something while sweeping has no idea which PoI it is, so it
               reports a sighting under its own id. The GCS matches it to the nearest
               announced PoI within match_radius_m, and keeps it as a new find otherwise.

Policy. Everything above is the `legacy` policy every level was scored with. A scenario
that sets `policy: connected` (Level 5) hands tasking to mission_plan.ConnectedPlanner
instead: one surveyor at a time, a relay chain that is on station before the surveyor
leaves coverage, a ferry sortie only for a PoI the chain cannot reach, one UAV per far PoI,
and a watchdog that expects a ferrying UAV to be silent until its sortie is overdue.
"""
from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uavx_interfaces.msg import RadioPacket

from ..common import EVENT_QOS, LATCHED_QOS, EventBus, dist2, load_scenario, now_s, spin
from ..coverage import lane_spacing_m, partition, sweep_time_s, lanes
from ..mission_plan import ConnectedParams, ConnectedPlanner, PoiView, UavView
from ..radio_endpoint import RadioEndpoint, packet_age_s
from ..radio_model import load_radio_params, predict_pdr
from ..relay_plan import RelayParams, RelayPlanner
from ..routing import RouteParams, RouteTable
from ..world_map import WorldMap

DEFAULTS = {"heartbeat_s": 1.0, "fail_timeout_s": 6.0, "lost_timeout_s": 90.0, "alive_pdr": 0.5,
            "min_battery_pct": 30.0, "emergency_priority": 4, "speed_mps": 5.0, "climb_mps": 2.5,
            "poi_window_s": 0.0, "mission_time_s": 1800.0, "out_of_contact_penalty_s": 30.0,
            # coverage and relay policy
            "coverage": True, "sectors": 0, "survey_alt_m": 0.0, "sweep_overlap": 0.3,
            "base_relays": 1, "max_relays": 4, "min_surveyors": 1, "escort_priority": 4,
            "match_radius_m": 25.0, "replan_interval_s": 10.0}
UNAVAILABLE = {"RETURN", "LANDING", "LANDED", "FAILED", "DONE", "RECHARGE"}
BUSY_STATES = {"NAVIGATE", "SURVEY", "SWEEP", "FERRY"}


class Gcs(Node):
    def __init__(self):
        super().__init__("gcs")
        self.declare_parameter("scenario", "level3")
        self.declare_parameter("run_dir", ".")
        self.sc = load_scenario(self.get_parameter("scenario").value)
        self.cfg = dict(DEFAULTS, **(self.sc.get("gcs_logic") or {}))
        self.id = self.sc["gcs"].get("id", "gcs")
        self.pos = tuple(self.sc["gcs"]["position_m"])
        self.radio_p = load_radio_params(self.sc)
        self.map = WorldMap.for_scenario(self.sc)
        self.events = EventBus(self, self.id)
        self.route = RouteTable(self.id, self.id, RouteParams(**(self.sc.get("routing") or {})))
        self.radio = RadioEndpoint(self, self.id, self._on_radio, self._on_acked,
                                   next_hop=self._next_hop, on_any=self._on_any_frame)

        self.uavs = {}
        for u in self.sc["uavs"]:
            self.uavs[u["id"]] = {"id": u["id"], "sysid": u["sysid"], "pad": tuple(u["pad_m"][:2]),
                                  "alt": float(u["cruise_alt_m"]), "link": "UNKNOWN", "last_seen": None,
                                  "status": {}, "pos": (u["pad_m"][0], u["pad_m"][1], 0.0), "queue": [],
                                  "version": 0, "acked_version": 0, "recall_sent": False,
                                  "role": "IDLE", "relay_at": None, "sector": None}
        self.pois: dict[str, dict] = {}
        self.sightings: dict[str, str] = {}      # sighting id -> PoI id it was matched to
        self.t0 = None
        self.recovery_pending: list[dict] = []
        self.recalled = False
        self.dirty = False
        self.last_plan_t = -1e9

        # coverage and relay planning
        self.relay = RelayPlanner(self.radio_p, RelayParams(**(self.sc.get("relay") or {})), self.map)
        self.relay.set_gcs(self.pos)
        self.survey_alt = float(self.cfg["survey_alt_m"]) or min(u["alt"] for u in self.uavs.values())
        self.lane_spacing = lane_spacing_m(self.survey_alt, float(self.cfg["sweep_overlap"]))
        self.sectors = []
        self.covered_sectors: set[str] = set()
        self.chain = None
        self.escort_of = ""
        self.policy = str(self.sc.get("policy") or "legacy")
        self.cplan = None
        if self.policy == "connected":
            battery = ((self.sc.get("companion") or {}).get("battery")) or {}
            self.cplan = ConnectedPlanner(self.relay, self.pos, ConnectedParams.from_cfg(self.cfg, battery))
            for u in self.uavs.values():
                u["tasks"] = None
        if self.cfg["coverage"] and self.map is not None and self.map.mission_area:
            count = int(self.cfg["sectors"]) or max(1, len(self.uavs) - int(self.cfg["base_relays"]))
            self.sectors = partition(self.map.mission_area, count, self.pos)
            self.events.emit("COVERAGE_PLAN", sectors={s.id: s.to_list() for s in self.sectors},
                             survey_alt_m=self.survey_alt, lane_spacing_m=round(self.lane_spacing, 1),
                             sweep_s={s.id: round(sweep_time_s(lanes(s, self.lane_spacing),
                                                               self.cfg["speed_mps"], self.pos))
                                      for s in self.sectors})

        self.create_subscription(String, "/scenario/poi", self._on_poi, EVENT_QOS)
        self.create_subscription(String, "/scenario/clock", self._on_clock, LATCHED_QOS)
        self.create_timer(float(self.cfg["heartbeat_s"]), self._heartbeat)
        self.create_timer(0.2, self.radio.tick)
        self.create_timer(1.0, self._supervise)
        self.get_logger().info(
            f"GCS at {self.pos}: watchdog {self.cfg['fail_timeout_s']} s, emergency priority >= "
            f"{self.cfg['emergency_priority']}, {len(self.sectors)} sectors, "
            f"relay hop {self.relay.hop_m:.0f} m (first {self.relay.gcs_hop_m:.0f} m)")

    # ---------------------------------------------------------------- scenario feed
    def _on_clock(self, msg: String) -> None:
        self.t0 = json.loads(msg.data).get("t0")

    def _mission_t(self) -> float:
        return now_s(self) - self.t0 if self.t0 is not None else 0.0

    def _on_poi(self, msg: String) -> None:
        p = json.loads(msg.data)
        if p["id"] in self.pois:
            return
        self.pois[p["id"]] = {"id": p["id"], "pos": p["reported_m"][:2], "priority": int(p.get("priority", 1)),
                              "survey_s": float(p.get("survey_s", 20.0)), "status": "PENDING", "owner": None,
                              "known_t": now_s(self), "found_by": None}
        self.events.emit("POI_KNOWN", poi_id=p["id"], pos=p["reported_m"][:2], priority=p.get("priority", 1))
        self.dirty = True

    # ---------------------------------------------------------------- radio
    def _next_hop(self, final_dst: str) -> str:
        """The GCS reaches a UAV directly when it can, otherwise through the relay chain."""
        if final_dst == "*":
            return "*"
        t = now_s(self)
        peer = self.route.neighbours.get(final_dst)
        if peer is not None and peer.fresh(t, self.route.p.stale_s) and peer.pdr >= self.route.p.min_usable_pdr:
            return final_dst
        # The way that UAV's own frames came in is the way back to it.
        back = self.radio.reverse_hop(final_dst, t)
        if back:
            return back
        # Nothing learned yet: the neighbour that claims the shortest route of its own.
        best, best_cost = "", math.inf
        for n in self.route.neighbours.values():
            if n.id == final_dst or not n.fresh(t, self.route.p.stale_s):
                continue
            if n.pdr < self.route.p.min_usable_pdr or n.cost is math.inf:
                continue
            if n.cost < best_cost:
                best, best_cost = n.id, n.cost
        return best or final_dst

    def _on_any_frame(self, pkt: RadioPacket) -> None:
        if int(pkt.hops) == 0 and pkt.src != self.id:
            self.route.on_frame(pkt.src, int(pkt.seq), now_s(self))

    def _heartbeat(self) -> None:
        self.route.update(now_s(self))
        self.radio.send("*", "HEARTBEAT", {"t": round(now_s(self), 2), "pois": len(self.pois),
                                           "cost": 0.0, "gcs": self.id})

    def _on_radio(self, pkt: RadioPacket, payload: dict) -> None:
        u = self.uavs.get(pkt.src)
        if u is None:
            return
        t = now_s(self)
        u["last_seen"] = t
        u["hops"] = int(pkt.hops)
        if u["link"] in ("UNKNOWN", "OUT_OF_RANGE", "LOST"):
            if u["link"] == "LOST":
                # its work went to others while it was silent: tell it what it owns now
                self.events.emit("UAV_REJOINED", uav=u["id"], queue=list(u["queue"]), hops=int(pkt.hops))
                if self.cplan is not None:
                    u["tasks"] = None            # the next planning pass sends it a fresh list
                else:
                    self._send_tasks(u)
            elif u["link"] == "OUT_OF_RANGE":
                self.events.emit("UAV_BACK_IN_RANGE", uav=u["id"], hops=int(pkt.hops),
                                 silent_s=round(t - u.get("silent_since", t), 1))
            else:
                self.events.emit("UAV_JOINED", uav=u["id"])
            u["link"] = "OK"
            self.dirty = True
        if pkt.kind == "STATUS":
            if self.cplan is not None:
                # the chain gate opens on a relay's own "on station": re-plan as soon as it does
                prev = u["status"]
                if any(prev.get(k) != payload.get(k) for k in ("state", "on_station", "task")):
                    self.dirty = True
            u["status"] = payload
            u["pos"] = tuple(payload.get("pos", u["pos"]))
            u["role"] = payload.get("role", u["role"])
            route = payload.get("route") or {}
            if route.get("cost") is not None:
                self.route.on_advert(pkt.src, float(route["cost"]), route.get("next_hop", ""),
                                     int(route.get("hops", 0)), t)
            for pid in payload.get("done", []):
                self._mark_done(self.sightings.get(pid, pid), u["id"])
        elif pkt.kind == "POI_REPORT":
            self._on_report(u, pkt, payload, t)
        elif pkt.kind == "TASK_DONE":
            pid = payload.get("poi_id")
            if payload.get("kind") == "sweep":
                self.covered_sectors.add(pid)
                self.events.emit("SECTOR_COVERED", uav=u["id"], sector=pid,
                                 remaining=[s.id for s in self.sectors if s.id not in self.covered_sectors])
                u["sector"] = None
                u["queue"] = [q for q in u["queue"] if q != pid]
                self.dirty = True
            else:
                self._mark_done(self.sightings.get(pid, pid), u["id"], payload.get("outcome"))
        elif pkt.kind == "TASK_RELEASE":
            released = [self.sightings.get(p, p) for p in payload.get("poi_ids", [])]
            for pid in released:
                poi = self.pois.get(pid)
                if poi and poi["status"] != "DONE":
                    poi["status"], poi["owner"] = "PENDING", None
            u["queue"] = [q for q in u["queue"] if q not in released]
            u["sector"] = None
            u["relay_at"] = None
            # it is heading home: its last 1 Hz STATUS may still say otherwise, so never hand
            # the released work straight back to it
            u["status"] = dict(u["status"], state="RETURN")
            self.events.emit("TASKS_RELEASED", uav=u["id"], pois=released, reason=payload.get("reason"),
                             was_relay=bool(payload.get("was_relay")), sweep=payload.get("sweep"))
            self._reallocate(u["id"], released, payload.get("reason", "released"))
        elif pkt.kind == "READY_AGAIN":
            self.events.emit("UAV_READY_AGAIN", uav=u["id"], cycle=payload.get("cycle"),
                             battery=payload.get("battery"))
            u["link"] = "OK"
            self.dirty = True

    def _on_report(self, u: dict, pkt: RadioPacket, payload: dict, t: float) -> None:
        """A find. It may be an announced PoI, or a sighting the swarm made on its own."""
        raw = payload.get("poi_id")
        pid = self._match_sighting(raw, payload.get("estimate"))
        latency = t - float(payload.get("detect_t", t))
        poi = self.pois.get(pid)
        if poi is None:
            poi = self.pois[pid] = {"id": pid, "pos": (payload.get("estimate") or [0, 0])[:2],
                                    "priority": 2, "survey_s": 0.0, "status": "DETECTED",
                                    "owner": u["id"], "known_t": t, "found_by": u["id"]}
            self.events.emit("POI_FOUND_BY_SWARM", poi_id=pid, uav=u["id"], pos=poi["pos"],
                             sighting=raw)
        if poi["status"] != "DONE":
            poi["status"] = "DETECTED"
            poi["found_by"] = poi["found_by"] or u["id"]
        self.events.emit("POI_RECEIVED", poi_id=pid, sighting=raw, uav=u["id"],
                         latency_s=round(latency, 2), frame_age_s=round(packet_age_s(self, pkt), 2),
                         attempts=int(pkt.attempt), hops=int(pkt.hops), route=list(pkt.route),
                         estimate=payload.get("estimate"), image=payload.get("image"),
                         found_while=payload.get("found_while"))

    def _match_sighting(self, raw: str, estimate) -> str:
        """Map a UAV's own sighting id onto the PoI the organisers announced, if it is one."""
        if raw in self.pois or raw is None:
            return raw
        if raw in self.sightings:
            return self.sightings[raw]
        if estimate:
            best, best_d = None, float(self.cfg["match_radius_m"])
            for poi in self.pois.values():
                d = dist2(estimate[:2], poi["pos"])
                if d <= best_d:
                    best, best_d = poi["id"], d
            if best:
                self.sightings[raw] = best
                self.events.emit("SIGHTING_MATCHED", sighting=raw, poi_id=best, distance_m=round(best_d, 1))
                return best
        self.sightings[raw] = raw
        return raw

    def _on_acked(self, entry: dict, latency_s: float) -> None:
        if entry["kind"] != "TASKS":
            return
        uid = entry["dst"]
        version = entry["payload"]["version"]
        self.uavs[uid]["acked_version"] = max(self.uavs[uid]["acked_version"], version)
        for rec in list(self.recovery_pending):
            if rec["to"] == uid and version >= rec["version"]:
                self.events.emit("RECOVERED", lost=rec["lost"], to=uid, pois=rec["pois"],
                                 recovery_s=round(now_s(self) - rec["declared_t"], 2))
                self.recovery_pending.remove(rec)

    def _mark_done(self, pid, uid, outcome=None) -> None:
        poi = self.pois.get(pid)
        if poi is None or poi["status"] == "DONE":
            return
        poi["status"] = "DONE"
        poi["owner"] = uid
        for u in self.uavs.values():
            u["queue"] = [q for q in u["queue"] if q != pid]
        self.events.emit("POI_COMPLETED", poi_id=pid, uav=uid, outcome=outcome,
                         since_known_s=round(now_s(self) - poi["known_t"], 1))
        self.dirty = True

    # ---------------------------------------------------------------- supervision
    def _supervise(self) -> None:
        t = now_s(self)
        if t <= 0.0:
            return
        for u in self.uavs.values():
            if u["last_seen"] is None or u["link"] == "LOST":
                continue
            silent = t - u["last_seen"]
            state = u["status"].get("state", "")
            if state in ("LANDED", "DONE", "RECHARGE"):
                continue
            if self.cplan is not None and self.cplan.ferry_uav() == u["id"]:
                # Rule 4: a ferrying UAV is out of contact by design. Its PoI stays its own and
                # nobody else is sent after it until the sortie is overdue.
                if u["link"] == "OK" and silent > self.cfg["fail_timeout_s"]:
                    u["link"] = "OUT_OF_RANGE"
                    u["silent_since"] = u["last_seen"]
                    due = self.cplan.work.due_t
                    self.events.emit("UAV_OUT_OF_RANGE", uav=u["id"], silent_s=round(silent, 1),
                                     last_pos=[round(v, 1) for v in u["pos"]], role="FERRY",
                                     reason="ferry sortie", due_in_s=None if due is None else round(due - t, 1))
                elif u["link"] == "OUT_OF_RANGE" and not self.cplan.ferry_expected(u["id"], t):
                    self._declare_lost(u, silent, "ferry sortie overdue")
                continue
            if u["link"] == "OK" and silent > self.cfg["fail_timeout_s"]:
                # Where should it be now, and could the swarm have carried its voice home?
                expected = self._expected_pdr(u["pos"])
                task = self.pois.get(u["status"].get("task") or "")
                if task:
                    expected = min(expected, self._expected_pdr((task["pos"][0], task["pos"][1], u["alt"])))
                u["silent_since"] = u["last_seen"]
                if expected >= self.cfg["alive_pdr"] or state == "FAILED":
                    self._declare_lost(u, silent, f"silent {silent:.1f} s where the link should be {expected:.2f} PDR")
                else:
                    u["link"] = "OUT_OF_RANGE"
                    self.events.emit("UAV_OUT_OF_RANGE", uav=u["id"], silent_s=round(silent, 1),
                                     last_pos=[round(v, 1) for v in u["pos"]], expected_pdr=round(expected, 2),
                                     role=u["role"])
            elif u["link"] == "OUT_OF_RANGE" and silent > self.cfg["lost_timeout_s"]:
                self._declare_lost(u, silent, "out of range too long")
        # safety net: unfinished work held by a UAV that can no longer fly it goes back
        for u in self.uavs.values():
            stuck = [pid for pid in u["queue"] if pid in self.pois
                     and self.pois[pid]["status"] != "DONE"]
            if stuck and u["link"] != "LOST" and u["status"].get("state") in UNAVAILABLE:
                u["queue"] = [q for q in u["queue"] if q not in stuck]
                for pid in stuck:
                    self.pois[pid]["status"], self.pois[pid]["owner"] = "PENDING", None
                self.events.emit("TASKS_RECLAIMED", uav=u["id"], pois=stuck, state=u["status"].get("state"))
                self._reallocate(u["id"], stuck, "owner unavailable")
        if self.dirty or t - self.last_plan_t >= float(self.cfg["replan_interval_s"]):
            self.dirty = False
            self.last_plan_t = t
            self._plan()
        self._maybe_recall()

    def _expected_pdr(self, pos) -> float:
        """Best predicted delivery to the GCS from `pos`: direct, or through a relay on station.

        With relays holding anchors, a UAV going quiet where the chain reaches is a real
        failure, and one going quiet beyond the chain is just out of contact. Judging the
        silence against the topology the GCS itself planned is what stops a ferrying UAV
        from being written off.
        """
        best = predict_pdr(self.radio_p, pos, self.pos, gcs_link=True, world_map=self.map)
        t = now_s(self)
        for u in self.uavs.values():
            anchor = u.get("relay_at")
            if anchor is None or u["link"] == "LOST":
                continue
            if self.cplan is not None and (u["last_seen"] is None
                                           or t - u["last_seen"] > self.cfg["fail_timeout_s"]):
                continue        # a silent relay carries nobody: silence behind it is not a failure
            hop = predict_pdr(self.radio_p, pos, anchor, world_map=self.map)
            best = max(best, hop * self._anchor_to_gcs_pdr(anchor))
        return best

    def _anchor_to_gcs_pdr(self, anchor) -> float:
        """Predicted delivery from a relay anchor back to the GCS along the planned chain."""
        if self.chain is None or not self.chain.points:
            return predict_pdr(self.radio_p, anchor, self.pos, gcs_link=True, world_map=self.map)
        nodes = [self.pos] + list(self.chain.points)
        try:
            idx = next(i for i, p in enumerate(self.chain.points) if dist2(p, anchor) < 1.0) + 1
        except StopIteration:
            return predict_pdr(self.radio_p, anchor, self.pos, gcs_link=True, world_map=self.map)
        out = 1.0
        for i in range(idx):
            out *= predict_pdr(self.radio_p, nodes[i], nodes[i + 1], gcs_link=(i == 0), world_map=self.map)
        return out

    def _declare_lost(self, u: dict, silent: float, why: str) -> None:
        u["link"] = "LOST"
        orphans = [pid for pid in u["queue"] if pid in self.pois and self.pois[pid]["status"] != "DONE"]
        cur = u["status"].get("task")
        if cur and cur in self.pois and cur not in orphans and self.pois[cur]["status"] != "DONE":
            orphans.insert(0, cur)
        for pid in orphans:
            self.pois[pid]["status"], self.pois[pid]["owner"] = "PENDING", None
        was_relay = u.get("relay_at") is not None
        u["queue"] = []
        u["relay_at"] = None
        u["sector"] = None
        self.events.emit("UAV_DECLARED_LOST", uav=u["id"], detection_s=round(silent, 2), reason=why,
                         orphans=orphans, was_relay=was_relay, role=u["role"])
        self._reallocate(u["id"], orphans, "uav lost")

    def _reallocate(self, from_uav: str, pois: list, reason: str) -> None:
        changed = self._plan()
        moved = {}
        for pid in pois:
            owner = self.pois.get(pid, {}).get("owner")
            if owner:
                moved[pid] = owner
        self.events.emit("REALLOCATED", lost=from_uav, reason=reason, moved=moved,
                         unassigned=[p for p in pois if p not in moved])
        t = now_s(self)
        for to in set(moved.values()):
            if to in changed:
                self.recovery_pending.append({"lost": from_uav, "to": to,
                                              "pois": [p for p, o in moved.items() if o == to],
                                              "version": self.uavs[to]["version"], "declared_t": t})

    # ---------------------------------------------------------------- planning
    def _available(self) -> list[dict]:
        out = []
        for u in self.uavs.values():
            st = u["status"]
            if u["link"] not in ("OK", "OUT_OF_RANGE") or st.get("state") in UNAVAILABLE:
                continue
            if float(st.get("batt", 100.0)) < self.cfg["min_battery_pct"]:
                continue
            out.append(u)
        return out

    def _finish_time(self, u: dict, extra=None) -> tuple[float, tuple]:
        """Seconds until u finishes its PoI queue (+ extra PoI) and where it ends up."""
        sp = self.cfg["speed_mps"]
        t = 0.0
        pos = u["pos"][:2]
        if u["status"].get("state") in (None, "READY", "PREFLIGHT", "BOOT", "ARMING", ""):
            t += u["alt"] / self.cfg["climb_mps"] + 8.0
        for pid in [q for q in u["queue"] if q in self.pois] + ([extra] if extra else []):
            poi = self.pois[pid]
            t += dist2(pos, poi["pos"]) / sp + poi["survey_s"]
            pos = poi["pos"]
        return t, pos

    def _plan(self) -> set:
        """One planning pass: PoIs, then the relay chain, then area coverage."""
        if self.cplan is not None:
            changed = self._plan_connected()
            for uid in changed:
                self._send_tasks(self.uavs[uid])
            return changed
        changed = self._allocate_pois()
        changed |= self._plan_relays()
        changed |= self._plan_coverage()
        for uid in changed:
            self._send_tasks(self.uavs[uid])
        return changed

    def _plan_connected(self) -> set:
        """`policy: connected`: one pass of mission_plan.ConnectedPlanner, applied.

        Returns the UAVs whose task list changed (they get a new TASKS message). The PoI
        bookkeeping the rest of this node relies on - owner, status, each UAV's queue and
        relay anchor - is kept in step, so the watchdog, the re-allocation report and the
        referee read the same story as they do for the legacy planner.
        """
        t = now_s(self)
        fleet = []
        for u in self.uavs.values():
            st = u["status"]
            fleet.append(UavView(u["id"], tuple(u["pos"]), u["alt"], u["pad"],
                                 batt=float(st.get("batt", 100.0)), state=st.get("state", ""),
                                 link=u["link"], on_station=bool(st.get("on_station")),
                                 relay_at=tuple(st["relay_at"]) if st.get("relay_at") else None))
        pois = [PoiView(p["id"], tuple(p["pos"][:2]), int(p["priority"]), float(p["survey_s"]),
                        p["status"], float(p["known_t"])) for p in self.pois.values()]
        plan = self.cplan.plan(t, fleet, pois)
        for etype, fields in plan.events:
            self.events.emit(etype, **fields)
        if plan.chain is not None:
            self.chain = plan.chain
        for pid, poi in self.pois.items():
            if poi["status"] == "DONE":
                continue
            owner = plan.owners.get(pid)
            poi["owner"] = owner
            if owner and poi["status"] == "PENDING":
                poi["status"] = "ASSIGNED"
            elif not owner and poi["status"] == "ASSIGNED":
                poi["status"] = "PENDING"
        changed = set()
        for uid, u in self.uavs.items():
            if u["link"] == "LOST":
                continue                     # unreachable: it gets a fresh list when it rejoins
            tasks = plan.tasks.get(uid, [])
            relay = next((b for b in tasks if b["kind"] == "relay"), None)
            u["relay_at"] = None if relay is None else (relay["pos"][0], relay["pos"][1], relay["alt"])
            u["role"] = "RELAY" if relay else ("SURVEY" if tasks and tasks[0]["kind"] in ("survey", "stage")
                                               else u["role"])
            u["queue"] = [b["id"] for b in tasks if b["kind"] in ("survey", "relay")]
            if tasks != u.get("tasks"):
                u["tasks"] = tasks
                changed.add(uid)
        return changed

    def _allocate_pois(self) -> set:
        avail = self._available()
        if not avail:
            return set()
        changed = set()
        pending = sorted((p for p in self.pois.values() if p["status"] == "PENDING"),
                         key=lambda p: (-p["priority"], p["known_t"]))
        for poi in pending:
            emergency = poi["priority"] >= self.cfg["emergency_priority"]
            best, best_t = None, math.inf
            for u in avail:
                if u.get("relay_at") is not None and not emergency:
                    continue                      # a relay on station keeps the chain alive
                if emergency:
                    # the nearest UAV goes, as the user's rule says: travel time only
                    cur = u["status"].get("task")
                    finish = dist2(u["pos"][:2], poi["pos"]) / self.cfg["speed_mps"]
                    if u["status"].get("state") == "SURVEY" and cur in self.pois:
                        finish += self.pois[cur]["survey_s"]
                else:
                    finish, _ = self._finish_time(u, poi["id"])
                    # connectivity-aware: prefer a UAV whose level keeps the GCS in contact
                    if self._expected_pdr((poi["pos"][0], poi["pos"][1], u["alt"])) < self.cfg["alive_pdr"]:
                        finish += self.cfg["out_of_contact_penalty_s"]
                if finish < best_t:
                    best, best_t = u, finish
            if best is None:
                continue
            if emergency:
                cur = best["status"].get("task")
                head = [cur] if best["status"].get("state") == "SURVEY" and cur in best["queue"] else []
                best["queue"] = head + [poi["id"]] + [q for q in best["queue"] if q not in head]
            else:
                best["queue"].append(poi["id"])
            poi["status"], poi["owner"] = "ASSIGNED", best["id"]
            changed.add(best["id"])
            self.events.emit("ASSIGNED", poi_id=poi["id"], uav=best["id"], priority=poi["priority"],
                             eta_s=round(best_t, 1), queue=list(best["queue"]), emergency=emergency)
        return changed

    def _focus(self):
        """What the relay chain is built towards: the most urgent active target."""
        active = [p for p in self.pois.values() if p["status"] in ("PENDING", "ASSIGNED", "DETECTED")]
        if active:
            poi = max(active, key=lambda p: (p["priority"], -dist2(p["pos"], self.pos[:2])))
            return poi["id"], (poi["pos"][0], poi["pos"][1], self.survey_alt), \
                poi["priority"] >= self.cfg["escort_priority"]
        if self.sectors:
            far = max(self.sectors, key=lambda s: dist2(s.centre, self.pos[:2]))
            c = far.centre
            return far.id, (c[0], c[1], self.survey_alt), False
        return "", None, False

    def _plan_relays(self) -> set:
        """Decide the chain and who flies it: LIVE if the fleet can afford it, else FERRY."""
        if int(self.cfg["max_relays"]) <= 0:
            return set()
        target_id, target, escort = self._focus()
        if target is None:
            return set()
        avail = self._available()
        budget = min(int(self.cfg["max_relays"]),
                     max(0, len(avail) - int(self.cfg["min_surveyors"])))
        if not escort:
            budget = min(budget, int(self.cfg["base_relays"]))
        chain = self.relay.decide(target_id, self.pos, target, budget)
        before = None if self.chain is None else self.chain.to_dict()
        self.chain = chain
        self.escort_of = target_id if escort else ""
        if before != chain.to_dict():
            self.events.emit("CHAIN_PLANNED", escort=escort, **chain.to_dict())

        # Who flies the anchors: prefer UAVs that are not holding a PoI of their own.
        candidates = [{"id": u["id"], "pos": u["pos"], "batt": float(u["status"].get("batt", 100.0)),
                       "relaying_at": u.get("relay_at")}
                      for u in avail
                      if not [q for q in u["queue"] if q in self.pois]
                      or u.get("relay_at") is not None]
        picks = self.relay.assign(chain, candidates, self.cfg["speed_mps"]) if chain.points else {}
        changed = set()
        for u in self.uavs.values():
            want = picks.get(u["id"])
            had = u.get("relay_at")
            if want is None and had is not None:
                u["relay_at"] = None
                u["queue"] = [q for q in u["queue"] if not q.startswith("R:")]
                changed.add(u["id"])
                self.events.emit("RELAY_RELEASED", uav=u["id"], reason="chain changed")
            elif want is not None and (had is None or dist2(had, want) > self.relay.p.reposition_threshold_m):
                u["relay_at"] = want
                u["sector"] = None
                u["queue"] = [f"R:{target_id}"] + [q for q in u["queue"] if not q.startswith("R:")]
                changed.add(u["id"])
                self.events.emit("RELAY_ASSIGNED", uav=u["id"], point=[round(v, 1) for v in want],
                                 serves=target_id, mode=chain.mode)
        return changed

    def _plan_coverage(self) -> set:
        """Hand every sector that nobody is covering to a UAV with nothing better to do."""
        if not self.sectors:
            return set()
        taken = {u["sector"] for u in self.uavs.values() if u["sector"]}
        covered = self.covered_sectors
        changed = set()
        free = [u for u in self._available()
                if u.get("relay_at") is None and not [q for q in u["queue"] if q in self.pois]
                and not u["sector"]]
        for sector in self.sectors:
            if sector.id in taken or sector.id in covered or not free:
                continue
            best = min(free, key=lambda u: dist2(u["pos"][:2], sector.centre))
            free.remove(best)
            best["sector"] = sector.id
            best["queue"] = [q for q in best["queue"] if not q.startswith("R:")] + [sector.id]
            taken.add(sector.id)
            changed.add(best["id"])
            self.events.emit("SECTOR_ASSIGNED", uav=best["id"], sector=sector.id, box=sector.to_list(),
                             eta_s=round(sweep_time_s(lanes(sector, self.lane_spacing),
                                                      self.cfg["speed_mps"], best["pos"])))
        return changed

    def _task_body(self, u: dict, pid: str) -> dict | None:
        if pid.startswith("R:") and u.get("relay_at") is not None:
            return {"id": pid, "kind": "relay", "pos": list(u["relay_at"][:2]),
                    "alt": round(float(u["relay_at"][2]), 1), "serves": pid[2:]}
        sector = next((s for s in self.sectors if s.id == pid), None)
        if sector is not None:
            return {"id": sector.id, "kind": "sweep", "sector": sector.to_list(),
                    "alt": self.survey_alt, "spacing_m": round(self.lane_spacing, 1)}
        poi = self.pois.get(pid)
        if poi is not None:
            return {"id": pid, "kind": "survey", "pos": poi["pos"], "priority": poi["priority"],
                    "survey_s": poi["survey_s"]}
        return None

    def _send_tasks(self, u: dict) -> None:
        u["version"] += 1
        self.radio.cancel(lambda e: e["kind"] == "TASKS" and e["dst"] == u["id"])   # newest queue wins
        if self.cplan is not None:
            tasks = list(u.get("tasks") or [])
        else:
            tasks = [b for b in (self._task_body(u, pid) for pid in u["queue"]) if b is not None]
        chain = {}
        if self.chain is not None:
            chain = {"mode": self.chain.mode, "needed": self.chain.needed,
                     "tip": None if self.chain.tip is None else [round(v, 1) for v in self.chain.tip],
                     "handoff": None if self.chain.handoff is None
                     else [round(v, 1) for v in self.chain.handoff]}
        self.radio.send(u["id"], "TASKS", {"version": u["version"], "tasks": tasks, "chain": chain},
                        reliable=True)

    def _maybe_recall(self) -> None:
        if self.recalled or self.t0 is None:
            return
        mt = self._mission_t()
        pois_done = bool(self.pois) and all(p["status"] == "DONE" for p in self.pois.values())
        swept = all(s.id in self.covered_sectors for s in self.sectors)
        all_done = pois_done and swept
        if (all_done and mt >= self.cfg["poi_window_s"]) or mt >= self.cfg["mission_time_s"]:
            self.recalled = True
            reason = "area covered and all PoIs done" if all_done else "mission time"
            self.events.emit("RECALL", reason=reason, mission_t=round(mt, 1),
                             pois_done=sum(p["status"] == "DONE" for p in self.pois.values()),
                             pois=len(self.pois))
            for u in self.uavs.values():
                if u["link"] != "LOST":
                    self.radio.send(u["id"], "RECALL", {"reason": reason}, reliable=True)


def main(args=None):
    rclpy.init(args=args)
    spin(Gcs())


if __name__ == "__main__":
    main()
