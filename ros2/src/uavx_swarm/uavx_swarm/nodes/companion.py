"""Onboard companion computer of one UAV.

Runs next to the autopilot and drives it only through ArduPilot's ROS 2 interface
(GUIDED mode, /ap/v<SYSID>/...), exactly as a Raspberry Pi / Jetson would on the real
aircraft. Mission sources:

  mission.mode: local   the plan is on board (Levels 1-2): waypoints and/or PoI surveys
  mission.mode: gcs     the GCS assigns work over the mesh radio (Levels 3+)

State machine
  BOOT -> PREFLIGHT (pre-arm OK) -> READY -> ARMING -> TAKEOFF
       -> NAVIGATE <-> SURVEY            fly to a PoI and survey it
       -> SWEEP                          cover an assigned sector, lane by lane
       -> RELAY                          hold a relay anchor and forward other UAVs' traffic
       -> FERRY                          carry buffered reports back into radio contact
       -> HOLD (nothing to do) / TO_COMM (link lost, no plan) -> RETURN
       -> LANDING -> LANDED [-> RECHARGE -> READY]      FAILED if it drops out of the sky

What the companion decides for itself, because the GCS often cannot reach it
  * endurance: a battery sized from `endurance_min` (the organisers' 20 minutes, Q7-Q8 -
    change that one number and every decision below adapts), returning home at the
    critical level or earlier if the trip home needs more; released PoIs go back to the GCS
  * routing: an ETX distance-vector table built from its neighbours' beacons (routing.py),
    so a report can travel through other UAVs without anybody holding a map of the network
  * the partial data ferry: with no route to the GCS and reports waiting, it flies to the
    handoff point - the nearest point to it that is inside the radio range of the outermost
    relay it knows about (relay_plan.py) - delivers over the multi-hop chain, and goes back
    to the lane it broke off from. It gives up a round trip, not its progress.
  * separation: the higher SYSID yields (holds) when a peer is within separation_m

Policy `connected` (Level 5; scenario `policy: connected`). The GCS's ConnectedPlanner
(mission_plan.py) gives one UAV the survey and the others the relay chain, and the
companion adds what only it can do:
  * STAGE: wait inside coverage until the chain the GCS is building is on station
  * a connectivity leash: off a ferry sortie, a UAV that loses its route to the GCS holds
    where it is after leash_hold_s, and after leash_retreat_s flies back towards the
    nearest node it can still hear (or the GCS) until the route is back. It never carries
    on deeper into silence.
  * the ferry sortie, only when the task says `ferry`: fly out of the network to the far
    PoI, survey it, keep the report on board, fly back to the handoff point inside the
    chain tip's range - the last connected UAV - and hand it over before taking new work.
    Reports are never ferried just because they are old: the multi-hop network carries them.
  * PARK: land on its own pad and stay available, instead of hovering with nothing to do
"""
from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from uavx_interfaces.msg import PoiDetection, RadioPacket

from ..ap_client import ArduPilotClient
from ..common import (EVENT_QOS, SENSOR_QOS, EventBus, dist2, dumps, load_scenario,
                      now_s, spin, uav_by_id)
from ..coverage import Sector, SweepPlan, lane_spacing_m, lanes
from ..radio_endpoint import RadioEndpoint, packet_age_s
from ..radio_model import load_radio_params
from ..relay_plan import RelayParams, RelayPlanner
from ..routing import RouteParams, RouteTable
from ..world_map import WorldMap

AIRBORNE = {"TAKEOFF", "NAVIGATE", "SURVEY", "SWEEP", "RELAY", "FERRY", "HOLD", "TO_COMM",
            "STAGE", "RETURN", "LANDING"}
WORKING = {"NAVIGATE", "SURVEY", "SWEEP", "RELAY", "FERRY", "HOLD", "TO_COMM", "STAGE"}
FINAL = {"LANDED", "FAILED", "DONE"}
# states the connectivity leash watches (policy: connected) - never a ferry sortie
LEASHED = {"NAVIGATE", "SURVEY", "HOLD", "STAGE", "RELAY"}
GROUND = {"BOOT", "PREFLIGHT", "READY", "ARMING"}

DEFAULTS = {
    "survey_s": 20.0,
    "arrival_radius_m": 2.0,
    "alt_tolerance_m": 1.5,
    "status_period_s": 1.0,
    "lost_link_s": 4.0,
    "lost_link_rth_s": 150.0,
    "return_to_comm": True,
    "separation_m": 25.0,
    "detection_radius_m": 15.0,
    "extra_search_s": 15.0,
    "start_delay_s": 0.0,
    "cruise_speed_mps": 5.0,
    "obstacle_clearance_m": 10.0,
    "sweep_overlap": 0.3,
    "sweep_arrival_m": 8.0,            # a lane waypoint counts as reached this close
    "ferry_after_s": 45.0,             # buffered report older than this: go and deliver it
    "ferry_min_reports": 1,
    "ferry_deliver_s": 25.0,           # give up at the handoff point after this long
    "relay_hold_radius_m": 6.0,
    "leash_hold_s": 3.0,               # policy connected: route down this long -> hold position
    "leash_retreat_s": 20.0,           # ... and this long -> fly back towards the network
    "recharge": {"enabled": False, "pad_s": 60.0, "to_pct": 100.0, "max_cycles": 3},
    "battery": {"endurance_min": 20.0, "initial_pct": 100.0, "cruise_penalty": 0.3,
                "critical_pct": 20.0, "reserve_pct": 8.0},
}


class FakeBattery:
    """The organisers' endurance limit as a battery (Q7: 20 minutes; Q8: adapt when it changes).

    One number, `endurance_min`, sets everything: hovering burns 100 % over exactly that
    long, and flying burns `cruise_penalty` more at cruise speed. Shrink it to 15 minutes
    and the return-home arithmetic, the relay eligibility and the ferry decision all follow
    without another constant being touched.
    """

    def __init__(self, cfg: dict, cruise_speed: float):
        self.endurance_min = float(cfg.get("endurance_min", 20.0))
        self.pct = float(cfg.get("initial_pct", 100.0))
        self.hover = 100.0 / max(self.endurance_min, 0.1)
        self.penalty = float(cfg.get("cruise_penalty", 0.3))
        self.cruise_speed = max(cruise_speed, 0.1)
        self.used_pct = 0.0

    def update(self, dt: float, airborne: bool, speed: float) -> None:
        if airborne and dt > 0:
            rate = self.hover * (1.0 + self.penalty * min(speed / self.cruise_speed, 1.5))
            spend = rate * dt / 60.0
            self.pct = max(0.0, self.pct - spend)
            self.used_pct += spend

    def cost_pct(self, seconds: float, moving: bool = True) -> float:
        rate = self.hover * (1.0 + (self.penalty if moving else 0.0))
        return seconds / 60.0 * rate

    def needed_to_reach(self, dist_m: float, alt_m: float) -> float:
        return self.cost_pct(dist_m / self.cruise_speed + alt_m / 1.5 + 15.0)

    def reset(self, pct: float) -> None:
        self.pct = float(pct)


class Companion(Node):
    def __init__(self):
        super().__init__("companion")
        self.declare_parameter("scenario", "level1")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("uav_id", "uav1")
        self.sc = load_scenario(self.get_parameter("scenario").value)
        self.uav = uav_by_id(self.sc, self.get_parameter("uav_id").value)
        self.id = self.uav["id"]
        cfg = json.loads(json.dumps(DEFAULTS))
        for k, v in (self.sc.get("companion") or {}).items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
        for k, v in self.uav.get("companion", {}).items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
        self.cfg = cfg
        self.mode = self.sc["mission"].get("mode", "local")
        self.connected_policy = str(self.sc.get("policy") or "legacy") == "connected" and self.mode == "gcs"
        self.alt = float(self.uav["cruise_alt_m"])
        self.pad = tuple(self.uav["pad_m"][:2])
        self.gcs_id = self.sc["gcs"].get("id", "gcs")
        self.gcs_pos = tuple(self.sc["gcs"]["position_m"])

        self.ap = ArduPilotClient(self, self.uav["sysid"])
        self.events = EventBus(self, self.id)
        self.battery = FakeBattery(cfg["battery"], cfg["cruise_speed_mps"])
        self.map = WorldMap.for_scenario(self.sc)
        self.use_radio = "radio_net" in self.sc["components"]
        self.radio_p = load_radio_params(self.sc)
        self.route = RouteTable(self.id, self.gcs_id, RouteParams(**(self.sc.get("routing") or {})))
        self.relay_planner = RelayPlanner(self.radio_p, RelayParams(**(self.sc.get("relay") or {})), self.map)
        self.radio = RadioEndpoint(self, self.id, self._on_radio, self._on_acked,
                                   next_hop=self._next_hop, on_any=self._on_any_frame,
                                   on_forward=self._on_forward) if self.use_radio else None

        self.state = "BOOT"
        self.state_t = 0.0
        self.tasks: list[dict] = self._local_plan() if self.mode == "local" else []
        self.task_version = -1
        self.current: dict | None = None
        self.done_ids: list[str] = []
        self.reported: dict[str, dict] = {}
        self.detections: dict[str, list] = {}
        self.return_reason = ""
        self.future = None
        self.next_cmd_t = 0.0
        self.last_t = None
        self.was_airborne = False
        self.last_gcs_hb = -1e9
        self.link_ok = False
        self.link_lost_t = None
        self.last_good_pos = None
        self.peers: dict[str, dict] = {}
        self.yielding = False
        self.max_speed = 0.0
        self.max_alt = 0.0
        self.takeoff_t = None
        self.wp_arrive_t = None
        self.path: list = []
        self.path_goal = None
        # multi-hop / sweep / ferry state
        self.sweep: SweepPlan | None = None
        self.relay_point: tuple | None = None
        self.relay_serves: str = ""
        self.chain_tip: tuple | None = None
        self.handoff_hint: tuple | None = None
        self.chain_mode = ""
        self.ferry_target: tuple | None = None
        self.ferry_resume: dict | None = None
        self.ferry_t = 0.0
        self.ferries = 0
        self.ferried_reports = 0
        self.forwarded = 0
        self.recharges = 0
        self.recharge_t = 0.0
        self.flight_s = 0.0
        # policy connected: relay on station, connectivity leash
        self.on_station = False
        self.hold_point = None
        self.route_up_t = 0.0
        self.leash: str | None = None
        self.leash_pos = None
        self.leash_target = None

        self.create_subscription(PoiDetection, f"/{self.id}/poi_detections", self._on_detection, SENSOR_QOS)
        self.create_subscription(String, "/scenario/inject", self._on_inject, EVENT_QOS)
        self.status_pub = self.create_publisher(String, f"/{self.id}/companion/status", EVENT_QOS)
        self.batt_pub = self.create_publisher(BatteryState, f"/{self.id}/battery_sim", SENSOR_QOS)
        self.create_timer(0.2, self._tick)
        self.create_timer(float(cfg["status_period_s"]), self._broadcast_status)
        if self.radio:
            self.create_timer(0.5, self._route_tick)
        self.get_logger().info(
            f"companion {self.id}: mission mode '{self.mode}', {len(self.tasks)} on-board tasks, "
            f"cruise {self.alt:.0f} m, {self.battery.endurance_min:.0f} min endurance, "
            f"radio {'on' if self.use_radio else 'off'}")

    # ================================================================ plan
    def _local_plan(self) -> list[dict]:
        m = self.sc["mission"]
        tasks = []
        for i, wp in enumerate(m.get("waypoints_m", [])):
            tasks.append({"id": f"WP{i + 1}", "kind": "waypoint", "pos": [float(wp[0]), float(wp[1])],
                          "alt": float(wp[2]) if len(wp) > 2 else self.alt, "hover_s": float(m.get("hover_s", 0.0)),
                          "priority": 1})
        if m.get("survey_pois"):
            for p in self.sc["pois"]:
                if p.get("assigned_to", self.id) != self.id:
                    continue
                tasks.append(self._poi_task(p))
        return tasks

    def _poi_task(self, p: dict) -> dict:
        reported = p.get("reported_m", p["pos_m"])
        return {"id": p["id"], "kind": "survey", "pos": [float(reported[0]), float(reported[1])],
                "alt": float(p.get("survey_alt_m", self.alt)), "survey_s": float(p.get("survey_s", self.cfg["survey_s"])),
                "priority": int(p.get("priority", 1))}

    # ================================================================ helpers
    def _set(self, state: str, **info) -> None:
        if state != self.state:
            self.events.emit("STATE", state=state, prev=self.state, **info)
            if state == "HOLD":
                # Latch the spot: re-commanding "where I am now" every 2 s lets any small
                # frame offset accumulate into a creep that can carry a UAV out of range.
                self.hold_point = self._pos() if self.ap.has_position else None
        self.state = state
        self.state_t = now_s(self)
        self.future = None
        self.next_cmd_t = 0.0

    def _pos(self):
        return self.ap.position if self.ap.has_position else (self.pad[0], self.pad[1], 0.0)

    def _every(self, period: float) -> bool:
        t = now_s(self)
        if t >= self.next_cmd_t:
            self.next_cmd_t = t + period
            return True
        return False

    def _arrived(self, target, alt) -> bool:
        e, n, z = self._pos()
        return dist2((e, n), target) < self.cfg["arrival_radius_m"] and abs(z - alt) < self.cfg["alt_tolerance_m"]

    # ================================================================ main loop
    def _tick(self) -> None:
        t = now_s(self)
        if t <= 0.0:
            return
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        airborne = self.ap.armed and self.ap.flying
        self.battery.update(dt, airborne, self.ap.ground_speed)
        if airborne:
            self.flight_s += dt
        if self.ap.has_position and airborne:
            self.max_speed = max(self.max_speed, self.ap.ground_speed)
            self.max_alt = max(self.max_alt, self._pos()[2])
        if self.radio:
            self.radio.tick()
            self._update_link(t)

        if self.state == "RECHARGE":
            self._st_recharge()
            return
        if self.state in FINAL:
            return
        # the vehicle stopped flying without being told to land: crash / motor failure
        if self.state in AIRBORNE and self.state != "LANDING" and self.was_airborne and not self.ap.armed:
            self._fail("disarmed in flight")
            return
        if self.state in AIRBORNE:
            # only a vehicle that really got off the ground can "fail in flight"
            # (ArduPilot also auto-disarms a GUIDED vehicle whose take-off was refused)
            if self.ap.has_position and self._pos()[2] > 3.0:
                self.was_airborne = True
            if self.state != "LANDING" and self.ap.mode_name in ("RTL", "LAND", "SMART_RTL") and t - self.state_t > 3.0:
                self.events.emit("AUTOPILOT_TOOK_OVER", mode=self.ap.mode_name)
                self._set("LANDING", reason=f"autopilot {self.ap.mode_name}")
            if self.state not in ("RETURN", "LANDING") and not self.return_reason:
                self._check_battery()
            # A relay never leaves to deliver its own data: the chain it holds is what
            # everybody else's reports are travelling through.
            if self.state in ("SWEEP", "SURVEY", "HOLD") and self._ferry_due(t):
                self._start_ferry("report ageing")
            if self.connected_policy and self._leash(t):
                return
        elif self.connected_policy:
            self.route_up_t = t              # on the ground, beside the GCS
        getattr(self, f"_st_{self.state.lower()}")()

    # ---------------------------------------------------------------- ground states
    def _st_boot(self) -> None:
        if self.ap.connected and self.ap.has_position and self.ap.services_ready():
            self._set("PREFLIGHT")

    def _st_preflight(self) -> None:
        if self.future is not None and self.future.done():
            ok = self.future.result().success
            self.future = None
            if ok:
                self._set("READY")
                return
        if self.future is None and self._every(2.0):
            self.future = self.ap.prearm_check()

    def _st_ready(self) -> None:
        if self.mode == "local":
            if now_s(self) - self.state_t >= float(self.cfg["start_delay_s"]):
                self.events.emit("MISSION_START", tasks=[x["id"] for x in self.tasks])
                self._set("ARMING")
        elif self.tasks:
            self._set("ARMING")
        elif self.return_reason:          # recalled before it ever flew
            if self.connected_policy:
                # a spare or a parked UAV is already on its own pad: that is landed
                self._set("LANDED", reason=self.return_reason)
            else:
                self._set("DONE", reason=self.return_reason)

    def _st_arming(self) -> None:
        if self.ap.mode_name != "GUIDED":
            if self._every(2.0):
                self.ap.set_mode("GUIDED")
        elif not self.ap.armed:
            if self._every(3.0):
                self.ap.arm(True)
        else:
            self.ap.takeoff(self.alt)
            self.takeoff_t = now_s(self)
            self._set("TAKEOFF", alt=self.alt)
        if now_s(self) - self.state_t > 90.0:
            self.events.emit("ARMING_TIMEOUT", mode=self.ap.mode_name)
            self._set("PREFLIGHT")

    def _st_takeoff(self) -> None:
        z = self._pos()[2]
        if not self.ap.armed and not self.was_airborne and now_s(self) - self.state_t > 2.0:
            self.events.emit("TAKEOFF_RETRY", reason="disarmed on the ground")
            self._set("ARMING")
            return
        if z >= self.alt - self.cfg["alt_tolerance_m"]:
            self.events.emit("AIRBORNE", alt=round(z, 1), climb_s=round(now_s(self) - self.takeoff_t, 1))
            self._next_task()
        elif z < 1.0 and now_s(self) - self.state_t > 5.0 and self._every(4.0):
            self.ap.takeoff(self.alt)       # takeoff request lost / refused: ask again

    # ---------------------------------------------------------------- connectivity leash
    def _on_sortie(self) -> bool:
        return self.state == "FERRY" or bool(self.current and self.current.get("ferry"))

    def _leash(self, t: float) -> bool:
        """Policy connected: never fly on into silence. True when it took the controls.

        Off a ferry sortie a UAV belongs inside the multi-hop network. If its route to the
        GCS goes away it first holds where it is (a radio outage or a jammed hop usually
        comes back by itself), and if the silence lasts it flies back towards the nearest
        node it can still hear - or the GCS - until the route returns. A relay on its anchor
        is the network and never moves for this; a surveyor already hovering over its PoI
        keeps surveying while it holds.
        """
        if self.state not in LEASHED or not self.radio or self._on_sortie():
            self.route_up_t = t
            return self._leash_clear(t)
        if self._connected() or self.link_ok:
            self.route_up_t = t
            return self._leash_clear(t)
        if self.state == "RELAY" and self.on_station:
            return False
        down = t - self.route_up_t
        if down < float(self.cfg["leash_hold_s"]):
            return False
        e, n, z = self._pos()
        if down < float(self.cfg["leash_retreat_s"]):
            if self.state == "SURVEY":
                return False                  # hovering over the PoI already: keep looking
            if self.leash != "hold":
                self.leash, self.leash_pos = "hold", (e, n, z)
                self.next_cmd_t = 0.0
                self.events.emit("LEASH_HOLD", down_s=round(down, 1), pos=[round(e, 1), round(n, 1), round(z, 1)],
                                 buffered=self.radio.pending, task=self.current["id"] if self.current else None)
            if self._every(1.0):
                self.ap.goto(self.leash_pos[0], self.leash_pos[1], self.leash_pos[2])
            return True
        if self.leash != "retreat":
            self.leash, self.leash_target = "retreat", self._retreat_point()
            self.path_goal = None
            self.events.emit("LEASH_RETREAT", down_s=round(down, 1), to=[round(v, 1) for v in self.leash_target],
                             buffered=self.radio.pending, task=self.current["id"] if self.current else None)
            if self.state == "SURVEY" and self.current is not None:
                if self.current["id"] in self.reported:
                    self._complete_current("surveyed")    # found already: the report is on board
                else:
                    self._set("NAVIGATE", task=self.current["id"], target=self.current["pos"])
        if self._fly_to(self.leash_target[:2], self.alt, period=1.0, radius=self.cfg["sweep_arrival_m"]):
            # still nothing here: keep closing in on the GCS
            g = self.gcs_pos
            self.leash_target = (0.6 * self.leash_target[0] + 0.4 * g[0], 0.6 * self.leash_target[1] + 0.4 * g[1])
            self.path_goal = None
        return True

    def _leash_clear(self, t: float) -> bool:
        if self.leash:
            self.events.emit("LEASH_CLEAR", was=self.leash, buffered=self.radio.pending if self.radio else 0,
                             hops=self.route.hops, next_hop=self.route.next_hop)
            self.leash = None
            self.path_goal = None
            self.next_cmd_t = 0.0
        return False

    def _retreat_point(self) -> tuple:
        """Back towards the network: a node still heard and connected, or the GCS itself."""
        t = now_s(self)
        me = self._pos()
        best, best_d = None, math.inf
        for peer in self.peers.values():
            if t - peer["t"] > float(self.cfg["leash_hold_s"]) or not peer.get("connected"):
                continue
            d = dist2(me, peer["pos"])
            if d < best_d:
                best, best_d = tuple(peer["pos"]), d
        if best is not None:
            p = self.relay_planner.safe_handoff_point((me[0], me[1], self.alt), best)
        else:
            p = self.relay_planner.safe_handoff_point((me[0], me[1], self.alt), self.gcs_pos, gcs_link=True)
        return (p[0], p[1])

    # ---------------------------------------------------------------- tasking
    def _next_task(self) -> None:
        if self.connected_policy:
            self._next_task_connected()
            return
        self.current = None
        if self.return_reason:
            self._set("RETURN", reason=self.return_reason)
            return
        if self.radio and self.radio.pending and not self._connected() and self._can_ferry():
            self._start_ferry("work finished with reports buffered")
            return
        if (self.radio and not self.link_ok and not self._connected() and self.radio.pending
                and self.cfg["return_to_comm"] and self.last_good_pos is not None and not self._can_ferry()):
            self._set("TO_COMM", buffered=self.radio.pending)
            return
        if self.tasks:
            self.current = self.tasks.pop(0)
            kind = self.current.get("kind", "survey")
            if kind == "relay":
                self._begin_relay(self.current)
            elif kind == "sweep":
                self._begin_sweep(self.current)
            else:
                self._set("NAVIGATE", task=self.current["id"], target=self.current["pos"])
            return
        if self.mode == "local":
            self.return_reason = "mission complete"
            self._set("RETURN", reason=self.return_reason)
        else:
            self._set("HOLD")

    def _next_task_connected(self) -> None:
        """Policy connected: the GCS's list is the whole plan, and nothing else starts a ferry."""
        self.current = None
        if self.return_reason:
            self._set("RETURN", reason=self.return_reason)
            return
        while self.tasks:
            task = self.tasks.pop(0)
            kind = task.get("kind", "survey")
            if kind == "park":
                # nothing for it to do: land on its own pad and stay available
                self.return_reason = "park"
                self._set("RETURN", reason="park")
                return
            self.current = task
            if kind == "relay":
                self._begin_relay(task)
            elif kind == "stage":
                self._set("STAGE", task=task["id"], target=task["pos"], serves=task.get("serves"))
            elif kind == "sweep":
                self._begin_sweep(task)
            else:
                if task.get("ferry"):
                    # Rule 3: out of the network on purpose, to a PoI the chain cannot reach.
                    self.handoff_hint = tuple(task["handoff"]) if task.get("handoff") else None
                    e, n, _ = self._pos()
                    self.events.emit("FERRY_SORTIE", poi=task["id"], to=task["pos"],
                                     leg_m=round(dist2((e, n), task["pos"]), 1), handoff=task.get("handoff"),
                                     tip=task.get("tip"), expect_s=task.get("expect_s"))
                self._set("NAVIGATE", task=task["id"], target=task["pos"], ferry=bool(task.get("ferry")))
            return
        self._set("HOLD")

    def _complete_current(self, outcome: str) -> None:
        task = self.current
        self.done_ids.append(task["id"])
        est = self._estimate(task["id"])
        self.events.emit("TASK_DONE", task=task["id"], kind=task["kind"], outcome=outcome, estimate=est)
        if self.radio and task["kind"] in ("survey", "sweep"):
            self.radio.send(self.gcs_id, "TASK_DONE", {"poi_id": task["id"], "kind": task["kind"],
                                                       "outcome": outcome, "estimate": est,
                                                       "t": now_s(self)}, reliable=True)
        if self.connected_policy and task.get("ferry") and self.radio:
            # Rule 3, second half: the data is on board - take it back to the last
            # connected UAV and hand it over before doing anything else.
            self.current = None
            self._start_ferry("ferry sortie: report on board", force=True)
            if self.state == "FERRY":
                return
        self._next_task()

    def _estimate(self, poi_id: str):
        pts = self.detections.get(poi_id)
        if not pts:
            return None
        return [round(sum(p[0] for p in pts) / len(pts), 2), round(sum(p[1] for p in pts) / len(pts), 2)]

    def _separation_hold(self) -> bool:
        """Yield rule: the higher SYSID holds while a peer is too close."""
        me = self._pos()
        t = now_s(self)
        for pid, peer in self.peers.items():
            if t - peer["t"] > 3.0 or peer["sysid"] > self.uav["sysid"]:
                continue
            p = peer["pos"]
            if dist2(me, p) < self.cfg["separation_m"] and abs(me[2] - p[2]) < 15.0:
                if not self.yielding:
                    self.events.emit("SEPARATION_HOLD", peer=pid, distance_m=round(dist2(me, p), 1))
                self.yielding = True
                return True
        if self.yielding:
            self.events.emit("SEPARATION_CLEAR")
        self.yielding = False
        return False

    # ---------------------------------------------------------------- path following
    def _fly_to(self, target, alt: float, period: float = 1.0, radius: float | None = None) -> bool:
        """Fly an obstacle-free path to target at height alt; True once there.

        With a 3-D map the companion plans around every obstacle taller than alt minus the
        clearance (A*, world_map.py) and feeds ArduPilot one leg at a time; without a map
        the path is the straight line. Re-plans whenever the target changes.
        """
        goal = (round(float(target[0]), 1), round(float(target[1]), 1), round(float(alt), 1))
        if goal != self.path_goal:
            e, n, _ = self._pos()
            if self.map is not None:
                self.path = self.map.plan((e, n), goal[:2], alt, self.cfg["obstacle_clearance_m"])
            else:
                self.path = [goal[:2]]
            self.path_goal = goal
            self.next_cmd_t = 0.0
            if len(self.path) > 1:
                self.events.emit("PATH_PLANNED", goal=list(goal), legs=len(self.path),
                                 path=[[round(p[0], 1), round(p[1], 1)] for p in self.path])
        e, n, _ = self._pos()
        while len(self.path) > 1 and dist2((e, n), self.path[0]) < 4.0:
            self.path.pop(0)             # intermediate waypoint passed: next leg straight away
            self.next_cmd_t = 0.0
        wp = self.path[0]
        if self._every(period):
            self.ap.goto(wp[0], wp[1], alt)
        if len(self.path) > 1:
            return False
        if radius is None:
            return self._arrived(wp, alt)
        return dist2((e, n), wp) < radius and abs(self._pos()[2] - alt) < max(self.cfg["alt_tolerance_m"], 3.0)

    # ---------------------------------------------------------------- airborne states
    def _st_navigate(self) -> None:
        task = self.current
        if self._separation_hold():
            if self._every(1.0):
                e, n, _ = self._pos()
                self.ap.goto(e, n, task["alt"])
            self.path_goal = None        # re-plan from wherever the hold leaves us
            return
        if self._fly_to(task["pos"], task["alt"]):
            if task["kind"] == "waypoint":
                if self.wp_arrive_t is None:
                    self.wp_arrive_t = now_s(self)
                if now_s(self) - self.wp_arrive_t >= task.get("hover_s", 0.0):
                    self.wp_arrive_t = None
                    self._complete_current("reached")
            else:
                self.events.emit("SURVEY_START", task=task["id"])
                self._set("SURVEY", task=task["id"])

    def _st_survey(self) -> None:
        task = self.current
        self._fly_to(task["pos"], task["alt"])      # holds over the end of the planned path
        elapsed = now_s(self) - self.state_t
        found = task["id"] in self.reported
        if elapsed >= task["survey_s"] and (found or elapsed >= task["survey_s"] + self.cfg["extra_search_s"]):
            self._complete_current("surveyed" if found else "not_found")

    def _st_stage(self) -> None:
        """Policy connected: wait inside coverage while the GCS puts the chain on station."""
        task = self.current
        alt = float(task.get("alt", self.alt))
        if self._separation_hold():
            if self._every(1.0):
                e, n, _ = self._pos()
                self.ap.goto(e, n, alt)
            self.path_goal = None
            return
        self._fly_to(task["pos"], alt, period=1.0)

    # ------------------------------------------------- sector sweep (area coverage)
    def _begin_sweep(self, task: dict) -> None:
        """Build the lane path for an assigned sector and start flying it.

        The lanes are computed on board from the sector and the camera swath, so a UAV that
        loses contact keeps covering ground, and a UAV that breaks off to deliver reports
        resumes at the lane it left (coverage.SweepPlan).
        """
        box = task.get("sector") or task.get("area")
        alt = float(task.get("alt", self.alt))
        sector = Sector(task["id"], float(box[0]), float(box[1]), float(box[2]), float(box[3]))
        spacing = float(task.get("spacing_m") or lane_spacing_m(alt, self.cfg["sweep_overlap"]))
        e, n, _ = self._pos()
        pts = lanes(sector, spacing, inset_m=spacing / 2.0, start_near=(e, n))
        self.sweep = SweepPlan(sector.id, pts)
        self.events.emit("SWEEP_START", sector=sector.id, box=sector.to_list(), waypoints=len(pts),
                         lane_spacing_m=round(spacing, 1), alt_m=alt)
        self._set("SWEEP", task=sector.id)

    def _st_sweep(self) -> None:
        task = self.current
        plan = self.sweep
        if plan is None or plan.done:
            self._complete_current("covered")
            return
        if self._separation_hold():
            if self._every(1.0):
                e, n, _ = self._pos()
                self.ap.goto(e, n, float(task.get("alt", self.alt)))
            self.path_goal = None
            return
        # Transit to the first lane at this UAV's own height, so five aircraft can leave one
        # apron without stacking up; drop to the common survey height once spread out.
        alt = self.alt if plan.index == 0 else float(task.get("alt", self.alt))
        if self._fly_to(plan.next_point, alt, radius=self.cfg["sweep_arrival_m"]):
            plan.advance()
            self.path_goal = None
            if plan.done:
                self.events.emit("SWEEP_DONE", sector=plan.sector_id, covered_m=round(plan.covered_m, 1),
                                 found=[p for p in self.reported])
                self._complete_current("covered")

    # ------------------------------------------------- relay
    def _begin_relay(self, task: dict) -> None:
        self.relay_point = (float(task["pos"][0]), float(task["pos"][1]),
                            float(task.get("alt", self.relay_planner.p.relay_alt_m)))
        self.relay_serves = task.get("serves", "")
        self.on_station = False
        self.events.emit("RELAY_ASSIGNED", point=[round(v, 1) for v in self.relay_point],
                         serves=self.relay_serves)
        self._set("RELAY", task=task["id"])

    def _st_relay(self) -> None:
        if self.relay_point is None:
            self._next_task()
            return
        if self._separation_hold():
            if self._every(1.0):
                e, n, _ = self._pos()
                self.ap.goto(e, n, self.relay_point[2])
            self.path_goal = None
            return
        on_station = self._fly_to(self.relay_point[:2], self.relay_point[2], period=2.0,
                                  radius=self.cfg["relay_hold_radius_m"])
        # Once there it stays "on station" while it holds near the anchor, so a metre of
        # drift never closes the GCS's gate on the surveyor again.
        if on_station:
            self.on_station = True
        elif self.on_station and dist2(self._pos(), self.relay_point) > 3.0 * self.cfg["relay_hold_radius_m"]:
            self.on_station = False
        if on_station and self.current is not None and not self.current.get("_announced"):
            self.current["_announced"] = True
            self.events.emit("RELAY_ON_STATION", point=[round(v, 1) for v in self.relay_point],
                             serves=self.relay_serves, cost=self.route.advert()["cost"])
        if self.tasks and any(t.get("kind") != "relay" for t in self.tasks):
            self._release_relay("re-tasked")

    def _release_relay(self, why: str) -> None:
        self.events.emit("RELAY_RELEASED", reason=why, forwarded=self.forwarded,
                         point=None if self.relay_point is None else [round(v, 1) for v in self.relay_point])
        self.on_station = False
        self.relay_point = None
        self.relay_serves = ""
        # Deliberately NOT added to done_ids: a relay task id names the target it serves, and
        # the same target may need a chain again later. Marking it done would make the UAV
        # ignore that second assignment.
        self.current = None
        self._next_task()

    # ------------------------------------------------- partial data ferry
    def _connected(self) -> bool:
        return bool(self.radio) and self.route.connected

    def _can_ferry(self) -> bool:
        return bool(self.radio) and self._ferry_node() is not None

    def _ferry_node(self):
        """Where to take the data: the chain tip the GCS last told us about, else the
        nearest peer that can still reach the GCS, else the GCS itself."""
        if self.handoff_hint is not None:
            return self.handoff_hint
        t = now_s(self)
        best, best_d = None, math.inf
        for pid, peer in self.peers.items():
            if t - peer["t"] > 20.0 or not peer.get("connected"):
                continue
            d = dist2(self._pos(), peer["pos"])
            if d < best_d:
                best, best_d = tuple(peer["pos"]), d
        if best is not None:
            return self.relay_planner.handoff_point(self._pos(), best)
        if self.chain_tip is not None:
            return self.relay_planner.handoff_point(self._pos(), self.chain_tip)
        return self.relay_planner.handoff_point(self._pos(), self.gcs_pos)

    def _ferry_due(self, t: float) -> bool:
        if self.connected_policy:
            return False         # rule 2: reports ride the multi-hop network, not a ferry
        if not self.radio or self.state == "FERRY" or self.return_reason:
            return False
        if self.radio.pending < int(self.cfg["ferry_min_reports"]):
            return False
        if self._connected():
            return False
        if self.radio.oldest_pending_age_s() < float(self.cfg["ferry_after_s"]):
            return False
        return self._can_ferry()

    def _start_ferry(self, why: str, force: bool = False) -> None:
        """Fly buffered reports back into contact. `force`: the return leg of a ferry
        sortie, which is also the way home, so it is never skipped for battery."""
        target = self._ferry_node()
        if target is None:
            return
        e, n, z = self._pos()
        leg = dist2((e, n), target)
        # Only worth it if the round trip fits in what is left after getting home.
        home = self.battery.needed_to_reach(dist2(target[:2], self.pad), z)
        trip = self.battery.cost_pct(2.0 * leg / self.cfg["cruise_speed_mps"])
        if not force and self.battery.pct - trip - home < self.cfg["battery"]["reserve_pct"]:
            self.events.emit("FERRY_SKIPPED", reason="not enough battery", leg_m=round(leg, 1),
                             battery_pct=round(self.battery.pct, 1))
            return
        self.ferry_resume = {"state": self.state, "task": self.current,
                             "pending": self.radio.pending,
                             "sweep_index": None if self.sweep is None else self.sweep.index}
        self.ferry_target = target
        self.ferry_t = now_s(self)
        self.ferries += 1
        self.events.emit("FERRY_START", reason=why, to=[round(v, 1) for v in target], leg_m=round(leg, 1),
                         buffered=self.radio.pending, oldest_s=round(self.radio.oldest_pending_age_s(), 1),
                         resume=None if self.sweep is None else self.sweep.to_dict())
        self._set("FERRY", to=[round(v, 1) for v in target])

    def _st_ferry(self) -> None:
        t = now_s(self)
        pending = self.radio.pending if self.radio else 0
        if pending == 0:
            self._finish_ferry("delivered", t)
            return
        if self._connected():
            self.radio.flush()
            if t - self.ferry_t > float(self.cfg["ferry_deliver_s"]):
                self._finish_ferry("timed out in contact", t)
            elif self._every(2.0):
                e, n, _ = self._pos()
                self.ap.goto(e, n, self.alt)
            return
        if self._separation_hold():
            self.path_goal = None
            return
        if self._fly_to(self.ferry_target[:2], self.alt, radius=self.cfg["sweep_arrival_m"]):
            if t - self.ferry_t > float(self.cfg["ferry_deliver_s"]) + 20.0:
                # nothing at the handoff point: close in on the GCS and try again
                self.ferry_target = self.relay_planner.handoff_point(self.ferry_target, self.gcs_pos)
                self.ferry_t = t
                self.path_goal = None
                self.events.emit("FERRY_CLOSING_IN", to=[round(v, 1) for v in self.ferry_target])

    def _finish_ferry(self, outcome: str, t: float) -> None:
        started_with = (self.ferry_resume or {}).get("pending", 0)
        self.ferried_reports += max(0, started_with - (self.radio.pending if self.radio else 0))
        self.events.emit("FERRY_DONE", outcome=outcome, elapsed_s=round(t - self.ferry_t, 1),
                         still_buffered=self.radio.pending if self.radio else 0,
                         hops=self.route.hops, next_hop=self.route.next_hop)
        resume = self.ferry_resume or {}
        self.ferry_resume = None
        self.ferry_target = None
        self.path_goal = None
        task, state = resume.get("task"), resume.get("state")
        if self.return_reason:
            self._set("RETURN", reason=self.return_reason)
        elif task is not None and state in ("SWEEP", "SURVEY", "NAVIGATE", "RELAY"):
            self.current = task
            if state == "SWEEP" and self.sweep is not None:
                self.events.emit("SWEEP_RESUMED", sector=self.sweep.sector_id, **self.sweep.to_dict())
                self._set("SWEEP", task=self.sweep.sector_id)
            elif state == "RELAY":
                self._set("RELAY", task=task["id"])
            else:
                self._set("NAVIGATE", task=task["id"], target=task["pos"])
        else:
            self._next_task()

    def _st_hold(self) -> None:
        if self.tasks:
            self._next_task()
            return
        hold = self.cfg.get("hold_m")
        if hold:
            self._fly_to(hold, self.alt, period=2.0)
        elif self.connected_policy and getattr(self, "hold_point", None) is not None:
            if self._every(2.0):
                self.ap.goto(self.hold_point[0], self.hold_point[1], self.alt)
        elif self._every(2.0):
            e, n, _ = self._pos()
            self.ap.goto(e, n, self.alt)

    def _st_to_comm(self) -> None:
        if self.link_ok or self._connected():
            self.events.emit("COMM_REGAINED", buffered=self.radio.pending)
            self._next_task()
            return
        target = self.last_good_pos
        if self._fly_to(target, self.alt):
            # still nothing where the link last worked: keep closing in on the GCS
            self.last_good_pos = (0.6 * target[0] + 0.4 * self.pad[0], 0.6 * target[1] + 0.4 * self.pad[1])

    def _st_return(self) -> None:
        self._fly_to(self.pad, self.alt)
        e, n, _ = self._pos()
        if dist2((e, n), self.pad) < 1.5:
            self.ap.set_mode("LAND")
            self._set("LANDING")

    def _st_landing(self) -> None:
        if not self.ap.armed and now_s(self) - self.state_t > 2.0:
            e, n, _ = self._pos()
            self.events.emit("LANDED", pad_error_m=round(dist2((e, n), self.pad), 2),
                             battery_pct=round(self.battery.pct, 1), flight_s=round(self.flight_s, 1),
                             max_speed_mps=round(self.max_speed, 2), max_alt_m=round(self.max_alt, 1),
                             tasks_done=self.done_ids, ferries=self.ferries, forwarded=self.forwarded)
            if self.connected_policy and self.return_reason == "park":
                # parked, not finished: on its pad, beside the GCS, ready to launch again
                self.return_reason = ""
                self.was_airborne = False
                self.takeoff_t = None
                self.path_goal = None
                self.events.emit("PARKED", battery_pct=round(self.battery.pct, 1))
                self._set("READY", parked=True)
                return
            if self._recharge_allowed():
                self.recharge_t = now_s(self)
                self._set("RECHARGE", cycle=self.recharges + 1)
            else:
                self._set("LANDED")
            return
        if self.ap.mode_name not in ("LAND", "RTL", "SMART_RTL") and self._every(3.0):
            self.ap.set_mode("LAND")

    def _st_landed(self) -> None:
        pass

    def _st_done(self) -> None:
        pass

    # ------------------------------------------------- recharge and relaunch
    def _recharge_allowed(self) -> bool:
        """A 20 minute aircraft cannot cover a mission whose PoIs appear over half an hour
        (Q7 against Q19), so a UAV that came home on its battery swaps and goes again."""
        rc = self.cfg.get("recharge") or {}
        if not rc.get("enabled") or self.mode == "local":
            return False
        if self.return_reason not in ("battery", "lost link"):
            return False
        return self.recharges < int(rc.get("max_cycles", 3))

    def _st_recharge(self) -> None:
        rc = self.cfg["recharge"]
        if now_s(self) - self.recharge_t < float(rc.get("pad_s", 60.0)):
            return
        self.recharges += 1
        self.battery.reset(float(rc.get("to_pct", 100.0)))
        self.return_reason = ""
        self.was_airborne = False
        self.takeoff_t = None
        self.path_goal = None
        self.events.emit("RECHARGED", cycle=self.recharges, battery_pct=round(self.battery.pct, 1))
        if self.radio:
            self.radio.send(self.gcs_id, "READY_AGAIN", {"battery": round(self.battery.pct, 1),
                                                         "cycle": self.recharges}, reliable=True)
        self._set("READY")

    def _fail(self, reason: str) -> None:
        self.events.emit("UAV_FAILED", reason=reason, task=self.current["id"] if self.current else None)
        self._set("FAILED", reason=reason)
        if self.radio:
            self.radio.enabled = False

    # ---------------------------------------------------------------- battery
    def _check_battery(self) -> None:
        e, n, z = self._pos()
        need = self.battery.needed_to_reach(dist2((e, n), self.pad), z)
        crit = self.cfg["battery"]["critical_pct"]
        reserve = self.cfg["battery"]["reserve_pct"]
        if self.battery.pct <= crit or self.battery.pct <= need + reserve:
            self.return_reason = "battery"
            released = ([self.current["id"]] if self.current and self.current["kind"] in ("survey", "sweep") else []) + \
                       [x["id"] for x in self.tasks if x["kind"] in ("survey", "sweep")]
            self.events.emit("BATTERY_RTH", battery_pct=round(self.battery.pct, 1), needed_pct=round(need, 1),
                             released=released, endurance_min=self.battery.endurance_min,
                             sweep=None if self.sweep is None else self.sweep.to_dict())
            if self.radio and (released or self.relay_point is not None):
                self.radio.send(self.gcs_id, "TASK_RELEASE",
                                {"poi_ids": released, "reason": "battery",
                                 "was_relay": self.relay_point is not None,
                                 "sweep": None if self.sweep is None else self.sweep.to_dict()}, reliable=True)
            self.tasks = []
            self.current = None
            self.relay_point = None
            self._set("RETURN", reason="battery")

    # ================================================================ radio
    def _next_hop(self, final_dst: str) -> str:
        """Link-level next hop for a frame addressed to `final_dst`.

        Towards the GCS: the ETX table (routing.py). Towards anybody else - an ACK, a reply -
        the neighbour that node's own frames last arrived through, because the routing table
        only ever points one way, at the GCS.
        """
        if final_dst == "*":
            return "*"
        t = now_s(self)
        peer = self.route.neighbours.get(final_dst)
        if peer is not None and peer.fresh(t, self.route.p.stale_s) \
                and peer.pdr >= self.route.p.min_usable_pdr:
            return final_dst
        if final_dst == self.gcs_id:
            return self.route.next_hop
        back = self.radio.reverse_hop(final_dst, t) if self.radio else ""
        return back or self.route.next_hop or ""

    def _on_any_frame(self, pkt: RadioPacket) -> None:
        """Measure the neighbour that put this frame on the air (direct frames only)."""
        if int(pkt.hops) == 0 and pkt.src != self.id:
            self.route.on_frame(pkt.src, int(pkt.seq), now_s(self))

    def _on_forward(self, pkt: RadioPacket, hop: str) -> None:
        self.forwarded += 1
        if self.forwarded in (1, 10, 50) or self.forwarded % 100 == 0:
            self.events.emit("RELAYED", frames=self.forwarded, last_kind=pkt.kind,
                             src=pkt.src, to=hop)

    def _route_tick(self) -> None:
        t = now_s(self)
        was = self.route.connected
        if self.route.update(t) and self.state in WORKING:
            self.events.emit("ROUTE_CHANGED", next_hop=self.route.next_hop, hops=self.route.hops,
                             cost=self.route.advert()["cost"])
        if self.route.connected and not was:
            self.events.emit("ROUTE_UP", next_hop=self.route.next_hop, hops=self.route.hops,
                             buffered=self.radio.pending)
            self.radio.flush()
        elif was and not self.route.connected:
            self.events.emit("ROUTE_DOWN", buffered=self.radio.pending)

    def _update_link(self, t: float) -> None:
        ok = (t - self.last_gcs_hb) < self.cfg["lost_link_s"]
        if ok != self.link_ok:
            if ok:
                down = None if self.link_lost_t is None else round(t - self.link_lost_t, 1)
                self.events.emit("LINK_RESTORED", down_s=down, buffered=self.radio.pending)
                self.radio.flush()       # deliver the store-and-forward backlog now
            elif self.state in AIRBORNE or self.state == "READY":
                self.link_lost_t = t
                self.events.emit("LINK_LOST", pos=[round(v, 1) for v in self._pos()], buffered=self.radio.pending)
            self.link_ok = ok
        if (not ok and not self.route.connected and self.link_lost_t is not None
                and self.state in ("NAVIGATE", "SURVEY", "SWEEP", "HOLD", "TO_COMM", "RELAY", "STAGE")
                and not (self.connected_policy and self._on_sortie())
                and not self.return_reason and t - self.link_lost_t > self.cfg["lost_link_rth_s"]):
            self.return_reason = "lost link"
            self.events.emit("LOST_LINK_RTH", down_s=round(t - self.link_lost_t, 1))
            if self.state in ("HOLD", "TO_COMM", "STAGE"):
                self._set("RETURN", reason=self.return_reason)

    def _on_radio(self, pkt: RadioPacket, payload: dict) -> None:
        if pkt.kind == "HEARTBEAT" and pkt.src == self.gcs_id:
            self.last_gcs_hb = now_s(self)
            self.route.on_advert(self.gcs_id, 0.0, "", 0, now_s(self))
            if self.ap.has_position and self.state in AIRBORNE and int(pkt.hops) == 0:
                e, n, _ = self._pos()
                self.last_good_pos = (e, n)
        elif pkt.kind == "STATUS" and pkt.src != self.gcs_id:
            self.peers[pkt.src] = {"pos": payload.get("pos", [0, 0, 0]), "sysid": payload.get("sysid", 99),
                                   "connected": bool(payload.get("connected")), "t": now_s(self)}
            route = payload.get("route") or {}
            if route.get("cost") is not None:
                self.route.on_advert(pkt.src, float(route["cost"]), route.get("next_hop", ""),
                                     int(route.get("hops", 0)), now_s(self))
        elif pkt.kind == "TASKS":
            self._apply_tasks(payload, packet_age_s(self, pkt))
        elif pkt.kind == "RECALL":
            self.return_reason = payload.get("reason", "recall")
            self.events.emit("RECALLED", reason=self.return_reason)
            if self.state in ("HOLD", "NAVIGATE", "SWEEP", "RELAY", "TO_COMM", "FERRY", "STAGE"):
                self.tasks = []
                self.relay_point = None
                self.on_station = False
                self._set("RETURN", reason=self.return_reason)

    def _apply_tasks(self, payload: dict, age_s: float) -> None:
        version = int(payload.get("version", 0))
        if version <= self.task_version:
            return
        self.task_version = version
        chain = payload.get("chain") or {}
        if chain:
            self.chain_mode = chain.get("mode", "")
            tip = chain.get("tip")
            handoff = chain.get("handoff")
            self.chain_tip = tuple(tip) if tip else None
            self.handoff_hint = tuple(handoff) if handoff else None
        incoming = [t for t in payload.get("tasks", []) if t["id"] not in self.done_ids]
        for t in incoming:
            t.setdefault("kind", "survey")
            t.setdefault("alt", self.alt)
            if t["kind"] == "survey":
                t.setdefault("survey_s", self.cfg["survey_s"])
        ids = [t["id"] for t in incoming]
        cur = self.current["id"] if self.current else None
        self.events.emit("TASKS_RECEIVED", version=version, tasks=ids, age_s=round(age_s, 2),
                         chain=chain or None)
        if self.connected_policy:
            self._apply_tasks_connected(incoming, ids, cur)
            return
        if self.state == "FERRY":
            self.tasks = [t for t in incoming if t["id"] != cur]
            return
        if self.state in ("SURVEY", "SWEEP") and cur:
            # finish what is in progress; the rest of the queue follows the GCS order
            self.tasks = [t for t in incoming if t["id"] != cur]
            if cur not in ids:                               # re-assigned to someone else
                self.events.emit("TASK_WITHDRAWN", task=cur)
                self.current = None
                self.sweep = None
                self._next_task()
            return
        if self.state == "RELAY" and cur:
            if cur not in ids:
                self._release_relay("re-assigned by the GCS")
                return
            self.tasks = [t for t in incoming if t["id"] != cur]
            return
        if self.state == "NAVIGATE" and cur:
            if cur not in ids:
                self.tasks = incoming
                self._next_task()
                return
            if ids and ids[0] != cur:                        # GCS put something more urgent first
                self.events.emit("PREEMPTED", dropped=cur, now=ids[0])
                self.tasks = incoming
                self._next_task()
                return
            self.tasks = [t for t in incoming if t["id"] != cur]
            return
        self.tasks = incoming
        if self.state == "HOLD" and self.tasks:
            self._next_task()

    def _apply_tasks_connected(self, incoming: list, ids: list, cur) -> None:
        """Policy connected: the GCS sends the whole list every time; the first item rules."""
        work = [t for t in incoming if t.get("kind") != "park"]
        if self.state in GROUND:
            self.tasks = work                 # nothing to park from; READY launches on work
            if self.state == "ARMING" and not work:
                self.events.emit("LAUNCH_CANCELLED")
                self._set("READY")            # ArduPilot disarms an idle vehicle by itself
            return
        if self.state == "FERRY":
            self.tasks = [t for t in incoming if t["id"] != cur]    # hand the report over first
            return
        if self.state in ("RETURN", "LANDING"):
            if self.state == "RETURN" and self.return_reason == "park" and work:
                self.events.emit("PARK_CANCELLED", tasks=[t["id"] for t in work])
                self.return_reason = ""
                self.tasks = work
                self._next_task()
            return
        if self.state == "TAKEOFF":
            self.tasks = incoming             # _next_task runs once it is up
            return
        if cur is not None and ids and ids[0] == cur:
            new = incoming[0]
            if self.current.get("kind") == "stage" and new.get("pos") != self.current.get("pos"):
                self.current["pos"] = new["pos"]          # the chain moved on: wait closer in
                self.current["alt"] = new.get("alt", self.current.get("alt"))
            self.tasks = incoming[1:]
            return
        # the job in hand was withdrawn, replaced or pre-empted
        if cur is not None and self.state in ("NAVIGATE", "SURVEY", "STAGE", "SWEEP"):
            self.events.emit("TASK_WITHDRAWN", task=cur, now=ids[0] if ids else None)
            if self.state == "SWEEP":
                self.sweep = None
        self.tasks = incoming
        if self.state == "RELAY":
            self._release_relay("re-assigned by the GCS")     # starts the next task itself
        else:
            self._next_task()

    def _on_acked(self, entry: dict, latency_s: float) -> None:
        if entry["kind"] == "POI_REPORT":
            self.events.emit("REPORT_ACKED", poi=entry["payload"].get("poi_id"), latency_s=round(latency_s, 2),
                             attempts=entry["attempt"], hops=self.route.hops,
                             ferried=self.state == "FERRY")

    def _broadcast_status(self) -> None:
        if not self.ap.has_position:
            return
        e, n, z = self._pos()
        body = {"pos": [round(e, 2), round(n, 2), round(z, 2)], "sysid": self.uav["sysid"],
                "batt": round(self.battery.pct, 2), "state": self.state,
                "task": self.current["id"] if self.current else None,
                "queue": [x["id"] for x in self.tasks], "done": self.done_ids,
                "pending": self.radio.pending if self.radio else 0, "link": self.link_ok,
                "armed": self.ap.armed, "mode": self.ap.mode_name, "speed": round(self.ap.ground_speed, 2),
                "route": self.route.advert() if self.radio else None,
                "connected": self._connected(),
                "role": self._role(), "relay_at": None if self.relay_point is None
                else [round(v, 1) for v in self.relay_point],
                "sweep": None if self.sweep is None else self.sweep.to_dict(),
                "forwarded": self.forwarded, "ferries": self.ferries,
                "endurance_min": self.battery.endurance_min, "recharges": self.recharges,
                "flight_s": round(self.flight_s, 1),
                "on_station": self.on_station, "ferry": self._on_sortie(), "leash": self.leash}
        if self.radio and self.state not in ("FAILED",):
            # The broadcast is how neighbours discover and measure each other: radio_net
            # delivers it over single links only, so a UAV several hops out is invisible to
            # the GCS through it. Those UAVs also send a unicast status along the chain, at
            # half the rate to keep the relay hops from carrying needless traffic - which is
            # what lets the GCS supervise a surveyor it cannot hear directly.
            self.radio.send("*", "STATUS", body)
            self._status_tick = getattr(self, "_status_tick", 0) + 1
            if self.route.connected and not self.route.direct_to_gcs and self._status_tick % 2 == 0:
                self.radio.send(self.gcs_id, "STATUS", body)
        msg = String()
        msg.data = dumps(dict(body, t=round(now_s(self), 2), id=self.id))
        self.status_pub.publish(msg)
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.percentage = float(self.battery.pct / 100.0)
        b.present = True
        self.batt_pub.publish(b)

    def _role(self) -> str:
        if self.state == "RELAY" or self.relay_point is not None:
            return "RELAY"
        if self._on_sortie():
            return "FERRY"
        if self.state == "STAGE":
            return "SURVEY"
        if self.state in ("SWEEP",):
            return "SURVEY"
        if self.state in ("NAVIGATE", "SURVEY") and self.current:
            return "SURVEY"
        return "IDLE"

    # ================================================================ detections / injection
    def _on_detection(self, det: PoiDetection) -> None:
        task = self.current
        if self.state not in ("NAVIGATE", "SURVEY", "SWEEP"):
            return
        if task is not None and task.get("kind") == "survey":
            if dist2((det.east_m, det.north_m), task["pos"]) > self.cfg["detection_radius_m"]:
                return
            poi_id = task["id"]
        else:
            # Found while sweeping: nobody told us this target was here (Q19/Q20 - the PoIs
            # appear as the mission runs). Report the sighting on its own merit, keyed by a
            # local id, and let the GCS match it to the PoI it announced.
            poi_id = self._sighting_id(det)
            if poi_id is None:
                return
        self.detections.setdefault(poi_id, []).append((det.east_m, det.north_m))
        if poi_id in self.reported:
            return
        t_det = det.header.stamp.sec + det.header.stamp.nanosec * 1e-9
        report = {"poi_id": poi_id, "uav": self.id, "estimate": [round(det.east_m, 2), round(det.north_m, 2)],
                  "detect_t": round(t_det, 3), "image": det.image_path, "pixel_area": round(det.pixel_area, 1),
                  "found_while": "sweep" if self.state == "SWEEP" else "survey"}
        self.reported[poi_id] = report
        self.events.emit("POI_DETECTED", **report)
        if self.radio:
            self.radio.send(self.gcs_id, "POI_REPORT", report, reliable=True)

    def _sighting_id(self, det) -> str | None:
        """Group detections made while sweeping into one sighting each, 15 m apart."""
        for pid, pts in self.detections.items():
            if pts and dist2((det.east_m, det.north_m), pts[-1]) <= self.cfg["detection_radius_m"]:
                return pid
        return f"{self.id}-X{len([k for k in self.detections if k.startswith(self.id)]) + 1}"

    def _on_inject(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        if ev.get("target") not in (self.id, "*"):
            return
        kind = ev.get("type")
        if kind == "battery":
            self.battery.pct = float(ev.get("pct", self.battery.pct))
            self.events.emit("BATTERY_SET", battery_pct=self.battery.pct)
        elif kind == "endurance":
            # Q8: the algorithm adapts to a different endurance without being rewritten.
            minutes = float(ev.get("minutes", self.battery.endurance_min))
            self.battery.endurance_min = minutes
            self.battery.hover = 100.0 / max(minutes, 0.1)
            self.events.emit("ENDURANCE_SET", endurance_min=minutes,
                             battery_pct=round(self.battery.pct, 1))


def main(args=None):
    rclpy.init(args=args)
    spin(Companion())


if __name__ == "__main__":
    main()
