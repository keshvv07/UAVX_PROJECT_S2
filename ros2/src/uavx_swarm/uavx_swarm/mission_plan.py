"""The connected-chain mission policy (Level 5): what the GCS decides each planning pass.

Pure Python with no ROS dependency, so the whole decision is unit-tested without a
simulator. `nodes/gcs.py` calls it when the scenario says `policy: connected`; every other
level keeps the planner it was scored with.

The operating rules it enforces
-------------------------------
1. The swarm stays interconnected. Every UAV keeps a multi-hop route to the GCS; the only
   UAV ever allowed out of it is a surveyor on a ferry sortie (rule 3).
2. Reports travel over the multi-hop aerial network, live. Ferrying is not the normal data
   path: a surveyor is only sent to a PoI once the relay chain that keeps it connected
   there is on station, and until then it waits (stages) inside coverage.
3. Ferry only for a PoI the chain cannot reach. The distance is estimated as the number
   of relays a live link needs (relay_plan.relays_needed). When that is more than the fleet
   can spare, the PoI is a FERRY target: the chain is still built as far towards it as the
   fleet allows, the relays hold their anchors and wait, and the surveyor flies out alone,
   surveys, stores the report on board, flies back to the last connected UAV (the handoff
   point inside the chain tip's range) and hands the report over.
4. While a far PoI is being ferried it belongs to that one UAV. Nobody else is given it, no
   second sortie starts, and the GCS does not write the ferrying UAV off for being silent
   until its sortie is overdue.

One surveyor works at a time; the other UAVs are its chain. A UAV that is neither stays on
its pad, or - if it is airborne and the next job will need a relay - holds where it is,
connected.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .radio_model import predict_pdr
from .relay_plan import BLIND, FERRY, LIVE, Chain, RelayPlanner

UNAVAILABLE = {"RETURN", "LANDING", "LANDED", "FAILED", "DONE", "RECHARGE"}
GROUNDED = {"BOOT", "PREFLIGHT", "READY", "ARMING", ""}
ACTIVE_POI = {"PENDING", "ASSIGNED", "DETECTED"}


@dataclass(frozen=True)
class ConnectedParams:
    emergency_priority: int = 4       # a PoI at or above this pre-empts work in progress
    max_relays: int = 4
    speed_mps: float = 5.0
    climb_mps: float = 2.5
    survey_alt_m: float = 0.0         # 0: each surveyor works at its own cruise height
    min_battery_pct: float = 30.0     # below this a UAV is not given NEW work
    on_station_m: float = 12.0        # a relay reporting on station this close to its anchor
    ferry: bool = True                # False: a PoI the chain cannot reach waits instead
    ferry_margin: float = 1.5         # sortie overdue after expected time x this ...
    ferry_grace_s: float = 45.0       # ... plus this
    extra_search_s: float = 15.0      # companion keeps looking this long if nothing is seen
    endurance_min: float = 20.0       # the companions' FakeBattery model, for affordability
    cruise_penalty: float = 0.3
    reserve_pct: float = 8.0
    critical_pct: float = 20.0
    idle_park_s: float = 60.0         # nothing to do for this long: everybody lands

    @classmethod
    def from_cfg(cls, cfg: dict, battery: dict | None = None) -> "ConnectedParams":
        names = set(cls.__dataclass_fields__)
        kw = {k: v for k, v in (cfg or {}).items() if k in names}
        for k in ("endurance_min", "cruise_penalty", "reserve_pct", "critical_pct"):
            if battery and k in battery:
                kw[k] = battery[k]
        return cls(**kw)


@dataclass
class UavView:
    """What the GCS knows about one UAV, from its last status broadcast."""
    id: str
    pos: tuple
    alt: float                        # its own cruise / survey height
    pad: tuple
    batt: float = 100.0
    state: str = ""
    link: str = "UNKNOWN"             # UNKNOWN | OK | OUT_OF_RANGE | LOST
    on_station: bool = False
    relay_at: tuple | None = None

    @property
    def airborne(self) -> bool:
        return self.state not in GROUNDED and self.state not in ("LANDED", "DONE", "RECHARGE")


@dataclass
class PoiView:
    id: str
    pos: tuple
    priority: int = 1
    survey_s: float = 20.0
    status: str = "PENDING"
    known_t: float = 0.0


@dataclass
class Work:
    """The one job the swarm is doing: a PoI, its chain, its surveyor."""
    poi_id: str
    priority: int
    mode: str = LIVE
    surveyor: str | None = None
    chain: Chain | None = None
    relays: dict = field(default_factory=dict)       # uav id -> anchor
    dispatched: bool = False
    dispatched_t: float | None = None
    due_t: float | None = None                       # ferry sortie overdue after this
    expect_s: float = 0.0

    @property
    def ferry(self) -> bool:
        return self.mode in (FERRY, BLIND)


@dataclass
class Plan:
    tasks: dict = field(default_factory=dict)        # uav id -> list of task bodies
    owners: dict = field(default_factory=dict)       # poi id -> uav id (None = unowned)
    events: list = field(default_factory=list)       # (type, fields)
    chain: Chain | None = None


class ConnectedPlanner:
    def __init__(self, relay: RelayPlanner, gcs_pos, params: ConnectedParams | None = None):
        self.relay = relay
        self.gcs = tuple(gcs_pos)
        self.p = params or ConnectedParams()
        self.work: Work | None = None
        self.surveyor: str | None = None             # sticky between jobs
        self.idle_since: float | None = None
        self.deferred: dict[str, float] = {}         # poi id -> battery it was deferred at
        self._needed: dict[tuple, int] = {}
        self._last_chain: dict | None = None
        self._announced: set = set()
        self._holding = False

    # ------------------------------------------------------------------ queries
    def ferry_expected(self, uav_id: str, t: float) -> bool:
        """True while `uav_id` is out on a ferry sortie that is not overdue yet."""
        w = self.work
        return bool(w and w.ferry and w.dispatched and w.surveyor == uav_id
                    and w.due_t is not None and t <= w.due_t)

    def ferry_uav(self) -> str | None:
        w = self.work
        return w.surveyor if w and w.ferry and w.dispatched else None

    def needed(self, poi: PoiView, alt: float) -> int:
        """Relays a live link to this PoI needs: the distance estimate behind rule 3."""
        key = (poi.id, round(alt, 1))
        if key not in self._needed:
            self._needed[key] = self.relay.relays_needed(self.gcs, (poi.pos[0], poi.pos[1], alt))
        return self._needed[key]

    # ------------------------------------------------------------------ helpers
    def _usable(self, u: UavView) -> bool:
        if u.state in UNAVAILABLE:
            return False
        if u.link == "OK" or (u.link == "UNKNOWN" and u.state in GROUNDED):
            return True
        # The surveyor out of contact - on a ferry sortie by design, or cut off and pulling
        # back on its leash - keeps its job until the GCS declares it LOST. Handing its PoI
        # to somebody else now would put two UAVs on one PoI the moment it reconnects.
        w = self.work
        return bool(u.link == "OUT_OF_RANGE" and w and w.surveyor == u.id)

    def _alt(self, u: UavView) -> float:
        return float(self.p.survey_alt_m) or float(u.alt)

    def _travel_s(self, u: UavView, xy) -> float:
        t = math.dist(u.pos[:2], xy[:2]) / max(self.p.speed_mps, 0.1)
        if not u.airborne:
            t += u.alt / max(self.p.climb_mps, 0.1) + 8.0      # arm, take off, climb
        return t

    def _pct(self, seconds: float) -> float:
        hover = 100.0 / max(self.p.endurance_min, 0.1)
        return seconds / 60.0 * hover * (1.0 + self.p.cruise_penalty)

    def _order(self, poi: PoiView, budget: int, alt: float):
        """Highest priority first; within a priority, PoIs the chain reaches live first."""
        far = self.needed(poi, alt) > budget
        return (-poi.priority, far, round(poi.known_t / 10.0), poi.id)

    def _pick_surveyor(self, avail: list[UavView], target) -> UavView | None:
        cands = [u for u in avail if u.batt >= self.p.min_battery_pct]
        if not cands:
            return None
        sticky = next((u for u in cands if u.id == self.surveyor), None)
        if sticky is not None:
            return sticky
        relay_alt = self.relay.p.relay_alt_m

        def key(u: UavView):
            # A surveyor whose own height is a separation layer clear of the relays never
            # has to be deconflicted from the chain it is working under.
            clash = abs(u.alt - relay_alt) < 15.0
            return (clash, self._travel_s(u, target), -u.batt, u.id)
        return min(cands, key=key)

    def _chain_for(self, w: Work, target, cands: list[UavView]) -> tuple[Chain, dict]:
        """The longest chain towards `target` that the candidates can actually fly."""
        budget = min(int(self.p.max_relays), len(cands))
        sticky = [{"id": u.id, "pos": u.pos, "batt": u.batt,
                   "relaying_at": w.relays.get(u.id) or (u.relay_at if u.state == "RELAY" else None)}
                  for u in cands]
        while True:
            chain = self.relay.decide(w.poi_id, self.gcs, target, budget)
            if not chain.points:
                return chain, {}
            picks = self.relay.assign(chain, sticky, self.p.speed_mps, keep_low_battery=True)
            if picks or budget <= 0:
                return chain, picks
            budget -= 1

    def _on_station(self, by_id: dict, uid: str, anchor) -> bool:
        u = by_id.get(uid)
        if u is None or u.state != "RELAY" or not u.on_station or u.relay_at is None:
            return False
        return math.dist(u.relay_at[:2], anchor[:2]) <= self.p.on_station_m

    def _stage_point(self, w: Work, target, by_id: dict, alt: float) -> tuple:
        """Where the surveyor waits: as close to the target as the chain on station allows."""
        node = self.gcs
        if w.chain is not None:
            order = {tuple(p): i for i, p in enumerate(w.chain.points)}
            for anchor in sorted(w.relays.values(), key=lambda p: order.get(tuple(p), 0)):
                uid = next(k for k, v in w.relays.items() if v == anchor)
                if not self._on_station(by_id, uid, anchor):
                    break
                node = anchor
        p = self.relay.safe_handoff_point((target[0], target[1], alt), node, gcs_link=node == self.gcs)
        return (p[0], p[1])

    def _sortie_s(self, start_xy, poi: PoiView, handoff) -> float:
        out = math.dist(start_xy[:2], poi.pos[:2])
        back = math.dist(poi.pos[:2], handoff[:2]) if handoff else out
        return (out + back) / max(self.p.speed_mps, 0.1) + poi.survey_s + self.p.extra_search_s + 10.0

    def _affordable(self, u: UavView, poi: PoiView, chain: Chain, start_xy) -> bool:
        handoff = chain.handoff or chain.tip or self.gcs
        sortie = self._sortie_s(start_xy, poi, handoff)
        home = math.dist(handoff[:2], u.pad[:2]) / max(self.p.speed_mps, 0.1) + 15.0
        need = self._pct(sortie + home)
        return u.batt - need >= self.p.critical_pct + self.p.reserve_pct

    def _connected_spot(self, u: UavView, anchors) -> bool:
        """Would this UAV still be inside the network if it simply held where it is?"""
        floor = self.relay.p.min_planned_pdr
        if predict_pdr(self.relay.radio, u.pos, self.gcs, gcs_link=True, world_map=self.relay.map) >= floor:
            return True
        return any(predict_pdr(self.relay.radio, u.pos, a, world_map=self.relay.map) >= floor for a in anchors)

    # ------------------------------------------------------------------ the decision
    def plan(self, t: float, fleet: list[UavView], pois: list[PoiView]) -> Plan:
        out = Plan()
        by_id = {u.id: u for u in fleet}
        active = {p.id: p for p in pois if p.status in ACTIVE_POI}
        avail = [u for u in fleet if self._usable(u)]
        w = self.work

        # -------------------------------------------- is the current job still a job?
        if w is not None and w.poi_id not in active:
            out.events.append(("WORK_FINISHED", {"poi_id": w.poi_id, "uav": w.surveyor, "mode": w.mode,
                                                 "ferry": w.ferry}))
            w = self.work = None
        if w is not None and w.surveyor is not None and w.surveyor not in {u.id for u in avail}:
            out.events.append(("SURVEYOR_LOST", {"poi_id": w.poi_id, "uav": w.surveyor,
                                                 "state": by_id[w.surveyor].state if w.surveyor in by_id else None}))
            if self.surveyor == w.surveyor:
                self.surveyor = None
            w.surveyor, w.dispatched, w.dispatched_t, w.due_t = None, False, None, None
        frozen = w is not None and w.ferry and w.dispatched

        # -------------------------------------------- pick the job
        if not frozen:
            cands = [p for p in active.values() if not self._is_deferred(p, avail)]
            budget_now = max(0, min(int(self.p.max_relays), len(avail) - 1))
            alt = float(self.p.survey_alt_m) or min((u.alt for u in avail), default=30.0)
            if w is not None and w.poi_id in active:
                urgent = [p for p in cands if p.priority >= self.p.emergency_priority and p.priority > w.priority]
                if urgent:
                    nxt = min(urgent, key=lambda p: self._order(p, budget_now, alt))
                    out.events.append(("PREEMPTED", {"dropped": w.poi_id, "now": nxt.id, "priority": nxt.priority}))
                    w = None
            if w is None and cands:
                nxt = min(cands, key=lambda p: self._order(p, budget_now, alt))
                w = Work(nxt.id, nxt.priority)
                out.events.append(("WORK_SELECTED", {"poi_id": nxt.id, "priority": nxt.priority,
                                                     "relays_needed": self.needed(nxt, alt), "budget": budget_now}))
            self.work = w

        if w is None:
            return self._idle(t, fleet, out)
        self.idle_since = None
        poi = active[w.poi_id]

        # -------------------------------------------- surveyor
        if w.surveyor is None:
            pick = self._pick_surveyor(avail, poi.pos)
            if pick is None:
                out.events.append(("NO_SURVEYOR", {"poi_id": poi.id}))
                return self._idle(t, fleet, out, keep_work=True)
            w.surveyor = self.surveyor = pick.id
        sv = by_id[w.surveyor]
        alt = self._alt(sv)
        target = (poi.pos[0], poi.pos[1], alt)

        # -------------------------------------------- chain (frozen while a sortie is out)
        cands = [u for u in avail if u.id != sv.id]
        lost_relay = [uid for uid in w.relays if uid not in {u.id for u in cands}]
        if not frozen or lost_relay or w.chain is None:
            chain, picks = self._chain_for(w, target, cands)
            if chain.mode in (FERRY, BLIND):
                # The handoff is where the surveyor comes back to: inside the chain tip's range,
                # in free airspace and with an unshadowed hop to it (the last connected UAV).
                tip = chain.tip if chain.points else self.gcs
                chain.handoff = self.relay.safe_handoff_point(target, tip, gcs_link=not chain.points)
            if frozen and lost_relay:
                out.events.append(("CHAIN_REPAIRED", {"poi_id": poi.id, "lost": lost_relay,
                                                      "relays": len(picks)}))
            w.chain, w.mode = chain, chain.mode
            released = [uid for uid in w.relays if uid not in picks]
            for uid in released:
                out.events.append(("RELAY_RELEASED", {"uav": uid, "reason": "chain changed"}))
            for uid, anchor in picks.items():
                if w.relays.get(uid) is None or math.dist(w.relays[uid][:2], anchor[:2]) > 1.0:
                    out.events.append(("RELAY_ASSIGNED", {"uav": uid, "point": [round(v, 1) for v in anchor],
                                                          "serves": poi.id, "mode": chain.mode}))
            w.relays = dict(picks)
        chain = w.chain
        out.chain = chain
        d = dict(chain.to_dict(), escort=False)
        if d != self._last_chain:
            self._last_chain = d
            out.events.append(("CHAIN_PLANNED", d))

        # -------------------------------------------- ferry: one UAV, affordable, or wait
        if w.ferry and not w.dispatched:
            if not self.p.ferry:
                return self._defer(t, fleet, pois, out, w, poi, "ferry disabled")
            if not self._affordable(sv, poi, chain, sv.pos):
                return self._defer(t, fleet, pois, out, w, poi, "battery", sv.batt)

        # -------------------------------------------- gate: is the chain on station?
        ready = all(self._on_station(by_id, uid, a) for uid, a in w.relays.items())
        body = {"id": poi.id, "kind": "survey", "pos": [round(v, 1) for v in poi.pos[:2]],
                "priority": poi.priority, "survey_s": poi.survey_s}
        if w.ferry:
            handoff = chain.handoff
            if not w.dispatched:
                w.expect_s = self._sortie_s(sv.pos, poi, handoff)
            body.update(ferry=True, handoff=[round(v, 1) for v in handoff],
                        tip=None if chain.tip is None else [round(v, 1) for v in chain.tip],
                        expect_s=round(w.expect_s, 1))
        if w.dispatched or ready:
            if not w.dispatched:
                w.dispatched, w.dispatched_t = True, t
                if w.ferry:
                    w.due_t = t + w.expect_s * self.p.ferry_margin + self.p.ferry_grace_s
                    out.events.append(("FERRY_DISPATCHED", {"poi_id": poi.id, "uav": sv.id, "mode": w.mode,
                                                            "needed": chain.needed, "relays": chain.relays,
                                                            "handoff": body["handoff"], "expect_s": body["expect_s"],
                                                            "due_in_s": round(w.due_t - t, 1)}))
                else:
                    out.events.append(("SURVEYOR_DISPATCHED", {"poi_id": poi.id, "uav": sv.id,
                                                               "relays": chain.relays}))
            sv_tasks = [body]
        else:
            sp = self._stage_point(w, target, by_id, alt)
            sv_tasks = [{"id": f"STAGE:{poi.id}", "kind": "stage", "pos": [round(sp[0], 1), round(sp[1], 1)],
                         "alt": round(alt, 1), "serves": poi.id}]
        out.tasks[sv.id] = sv_tasks
        if (poi.id, sv.id) not in self._announced:
            self._announced.add((poi.id, sv.id))
            out.events.append(("ASSIGNED", {"poi_id": poi.id, "uav": sv.id, "priority": poi.priority,
                                            "eta_s": round(self._travel_s(sv, poi.pos) + poi.survey_s, 1),
                                            "queue": [poi.id], "emergency": poi.priority >= self.p.emergency_priority,
                                            "mode": w.mode, "relays_needed": chain.needed}))
        out.owners = {pid: (sv.id if pid == poi.id else None) for pid in active}

        # -------------------------------------------- relays, and everybody else
        # A relay already on station stays there while some other UAV still depends on it
        # for its route home - it moves to its new anchor (or is released) only once that
        # UAV is somewhere it stays connected without it. The surveyor meanwhile waits at its
        # stage point, which is connected without the old station by construction.
        stations = self._stations(fleet)
        keep_old: dict = {}
        if not frozen:
            ready_anchors = [a for uid, a in w.relays.items() if self._on_station(by_id, uid, a)]
            deps = self._dependents(fleet, set(w.relays) | set(stations), ready_anchors)
            if deps:
                for uid, old in stations.items():
                    new = w.relays.get(uid)
                    if new is None or math.dist(new[:2], old[:2]) > self.p.on_station_m:
                        keep_old[uid] = old
                if keep_old and not self._holding:
                    out.events.append(("RELAY_MOVE_HELD", {"relays": sorted(keep_old),
                                                           "dependents": sorted(u.id for u in deps)}))
        self._holding = bool(keep_old)
        for uid, anchor in w.relays.items():
            out.tasks[uid] = [self._relay_body(keep_old.get(uid, anchor), poi.id)]
        for uid, old in keep_old.items():
            out.tasks.setdefault(uid, [self._relay_body(old, poi.id)])
        later_needs_relay = any(self.needed(p, alt) > 0 for p in active.values() if p.id != poi.id)
        for u in fleet:
            if u.id in out.tasks:
                continue
            out.tasks[u.id] = self._spare(u, later_needs_relay, list(w.relays.values()))
        return out

    # ------------------------------------------------------------------ spares, idle, defer
    @staticmethod
    def _relay_body(anchor, serves: str) -> dict:
        return {"id": f"R:{anchor[0]:.0f},{anchor[1]:.0f}", "kind": "relay",
                "pos": [round(anchor[0], 1), round(anchor[1], 1)], "alt": round(float(anchor[2]), 1),
                "serves": serves}

    @staticmethod
    def _stations(fleet) -> dict:
        """uav id -> anchor, for every relay that reports it is on station right now."""
        return {u.id: tuple(u.relay_at) for u in fleet
                if u.state == "RELAY" and u.on_station and u.relay_at is not None and u.link == "OK"}

    def _dependents(self, fleet, relay_ids, anchors) -> list:
        """UAVs in the air - working or flying home - that are only in the network thanks
        to a relay station: without one they would have no route to the GCS."""
        return [u for u in fleet
                if u.airborne and u.link == "OK" and u.id not in relay_ids
                and u.state not in ("FAILED", "LANDED", "DONE", "RECHARGE")
                and not self._connected_spot(u, anchors)]

    def _spare(self, u: UavView, keep: bool, anchors) -> list:
        if not u.airborne or u.state in UNAVAILABLE:
            return []
        if keep and self._connected_spot(u, anchors):
            return []                                   # hold where it is, still in the network
        return [{"id": "PARK", "kind": "park"}]

    def _idle(self, t: float, fleet, out: Plan, keep_work: bool = False) -> Plan:
        """Nothing to do: hold where connected, then land. A relay holds its station as long
        as anybody still airborne needs it to reach the GCS - including a UAV flying home."""
        if not keep_work:
            self.work = None
        self.idle_since = self.idle_since if self.idle_since is not None else t
        park = t - self.idle_since >= self.p.idle_park_s
        stations = self._stations(fleet)
        deps = self._dependents(fleet, set(stations), [])
        for u in fleet:
            if not u.airborne or u.state in UNAVAILABLE:
                out.tasks[u.id] = []
            elif u.id in stations and deps:
                out.tasks[u.id] = [self._relay_body(stations[u.id], "")]
            elif u.id in stations:
                out.tasks[u.id] = [{"id": "PARK", "kind": "park"}] \
                    if park or not self._connected_spot(u, []) else [self._relay_body(stations[u.id], "")]
            elif park or not self._connected_spot(u, list(stations.values())):
                out.tasks[u.id] = [{"id": "PARK", "kind": "park"}]
            else:
                out.tasks[u.id] = []
        out.owners = {}
        return out

    def _is_deferred(self, poi: PoiView, avail) -> bool:
        """A PoI deferred for lack of battery is retried once somebody has more of it."""
        at = self.deferred.get(poi.id)
        if at is None:
            return False
        if max((u.batt for u in avail), default=0.0) > at + 5.0:
            del self.deferred[poi.id]
            return False
        return True

    def _defer(self, t, fleet, pois, out: Plan, w: Work, poi: PoiView, why: str, batt: float = 0.0) -> Plan:
        """Put a far PoI aside (no affordable sortie yet) and plan the next job instead."""
        out.events.append(("FERRY_DEFERRED", {"poi_id": poi.id, "reason": why, "uav": w.surveyor,
                                              "battery_pct": round(batt, 1)}))
        self.deferred[poi.id] = batt if why == "battery" else math.inf
        self.work = None
        nxt = self.plan(t, fleet, pois)            # the deferred PoI is skipped now
        nxt.events[:0] = out.events
        return nxt
