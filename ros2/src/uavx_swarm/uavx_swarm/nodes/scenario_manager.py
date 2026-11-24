"""Scenario manager - plays the organisers' part (Levels 2-4).

* Mission clock: t = 0 when every UAV's companion is READY (pre-arm OK); published latched
  on /scenario/clock. Every `appear_s` / `at_s` in the scenario is relative to it.
* Targets: a `targets:` block generates the PoI field the way the organisers generate
  theirs - N sites drawn inside the search area, clear of the tree trunks and at least
  `min_separation_m` apart, each appearing at its own time inside `appear_window_s`
  (Q19/Q20: some are there from the start, the rest arrive as the mission runs). The draw
  is seeded, so `--seed N` replays a particular target field exactly, and the organisers'
  own fixed positions can be given instead under `pois:`.
* PoIs: at appear_s the red target is spawned in Gazebo (at its true position) and the
  PoI is announced on /scenario/poi with its *reported* position (what a field report
  would give - the UAV's camera finds the real one). PoIs with appear_s <= 0 are spawned
  as soon as Gazebo is up, so on-board plans (Level 2) find them.
* Disturbances (the hidden ones of Stage 2) are NEVER generated here. This node has no
  timeline, no random draw and no seeded plan: a disturbance exists only when the evaluator
  injects it from the control console (control_panel.py -> /scenario/inject_request).
  Requests that do not come from the console are refused.
      comm_outage / packet_loss / ber / jamming -> radio_net
      battery / endurance                       -> that UAV's companion
      uav_failure                               -> radio dies + MAVLink force-disarm: it falls
      new_poi                                   -> spawned and announced like any PoI
  Every one is executed by publishing on /scenario/inject and logged as DISTURBANCE.
"""
from __future__ import annotations

import json
import math
import os
import random
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..common import EVENT_QOS, LATCHED_QOS, EventBus, dumps, load_scenario, now_s, spin
from ..world_map import WorldMap

READY_STATES = {"READY", "ARMING", "TAKEOFF", "NAVIGATE", "SURVEY", "HOLD", "TO_COMM", "RETURN", "LANDING"}


class ScenarioManager(Node):
    def __init__(self):
        super().__init__("scenario_manager")
        self.declare_parameter("scenario", "level4")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("seed", -1)
        self.sc = load_scenario(self.get_parameter("scenario").value)
        from ament_index_python.packages import get_package_share_directory
        self.marker_sdf = os.path.join(get_package_share_directory("uavx_gazebo"), "models", "poi_marker", "model.sdf")
        self.world = self.sc["world"]
        self.events = EventBus(self, "scenario")
        self.t0 = None
        self.states: dict[str, str] = {}
        self.status: dict[str, dict] = {}
        self.removed: set[str] = set()
        self.appeared: dict[str, dict] = {}
        self.console_poi_count = 0
        self.run_dir = self.get_parameter("run_dir").value
        self.map = WorldMap.for_scenario(self.sc)
        seed = int(self.get_parameter("seed").value)
        if seed < 0:
            seed = self.sc.get("seed", 0)
            if seed == "random":
                seed = random.SystemRandom().randint(1, 99999)
        self.seed = int(seed)
        self.rng = random.Random(self.seed)
        self._draw_targets()
        self.pending_pois = sorted(self.sc["pois"], key=lambda p: p["appear_s"])
        for stale in ("disturbances", "hidden_disturbances"):
            if self.sc.get(stale):
                self.get_logger().error(f"scenario key '{stale}:' is ignored - disturbances can only be "
                                        f"injected from the control console")
        self.spawned: set[str] = set()
        self.instances = {u["id"]: u for u in self.sc["uavs"]}

        for u in self.sc["uavs"]:
            self.create_subscription(String, f"/{u['id']}/companion/status",
                                     lambda m, uid=u["id"]: self._on_status(uid, m), EVENT_QOS)
        self.create_subscription(String, "/scenario/inject_request", self._on_request, EVENT_QOS)
        self.poi_pub = self.create_publisher(String, "/scenario/poi", EVENT_QOS)
        self.inject_pub = self.create_publisher(String, "/scenario/inject", EVENT_QOS)
        self.clock_pub = self.create_publisher(String, "/scenario/clock", LATCHED_QOS)
        self.create_timer(0.5, self._tick)
        threading.Thread(target=self._spawn_initial, daemon=True).start()
        self.get_logger().info(f"scenario '{self.sc['name']}': {len(self.pending_pois)} PoIs, "
                               f"disturbances: control console only")

    # ---------------------------------------------------------------- target field
    def _draw_targets(self) -> None:
        """Expand a `targets:` block into PoIs, the way the organisers place theirs.

        Their generator draws each search-and-rescue site inside the ROI, keeps it clear of
        the tree trunks so the marker is visible from above, keeps sites `min_separation_m`
        apart, and gives each one a spawn time - most during the mission rather than at the
        start (Q19/Q20). The same rules here, from the run seed, so a target field can be
        replayed or re-drawn without editing the scenario. Explicit `pois:` are kept and
        counted towards the separation, which is how the organisers' own fixed positions go
        in when they publish them.
        """
        spec = self.sc.get("targets") or {}
        count = int(spec.get("count", 0))
        if count <= 0:
            return
        seed = spec.get("seed")
        rng = random.Random(int(seed)) if seed is not None else self.rng
        area = spec.get("region_m")
        if area is None and self.map is not None and self.map.mission_area:
            area = list(self.map.mission_area)
        if area is None:
            self.get_logger().error("targets: no region_m and the map has no mission_area - skipped")
            return
        x0, x1, y0, y1 = (float(v) for v in area)
        inset = float(spec.get("inset_m", 10.0))
        region = (x0 + inset, y0 + inset, x1 - inset, y1 - inset)
        sep = float(spec.get("min_separation_m", 15.0))
        min_tree = float(spec.get("min_tree_m", 6.0))
        window = spec.get("appear_window_s", [30.0, 900.0])
        at_start = int(spec.get("at_start", 2))
        pri = spec.get("priority", [1, 3])
        survey_s = float(spec.get("survey_s", 20.0))
        offset = spec.get("report_offset_m", [3.0, 8.0])
        prefix = str(spec.get("prefix", "T"))

        taken = [p["pos_m"] for p in self.sc["pois"]]
        made = []
        for i in range(count):
            pos = None
            if self.map is not None:
                pos = self.map.ground_free_point(rng, region, min_tree, taken, sep)
            if pos is None:
                pos = [round(rng.uniform(region[0], region[2]), 1), round(rng.uniform(region[1], region[3]), 1)]
            taken.append(pos)
            ang, off = rng.uniform(0, 6.283), rng.uniform(float(offset[0]), float(offset[1]))
            appear = 0.0 if i < at_start else round(rng.uniform(float(window[0]), float(window[1])), 1)
            made.append({"id": f"{prefix}{i + 1}", "pos_m": pos,
                         "reported_m": [round(pos[0] + off * math.cos(ang), 1),
                                        round(pos[1] + off * math.sin(ang), 1)],
                         "priority": rng.randint(int(pri[0]), int(pri[1])) if isinstance(pri, list) else int(pri),
                         "survey_s": survey_s, "appear_s": appear})
        made.sort(key=lambda p: p["appear_s"])
        self.sc["pois"].extend(made)
        path = os.path.join(self.run_dir, "target_field.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"seed": self.seed if seed is None else int(seed), "targets": made}, fh, indent=2)
        self.get_logger().info(f"target field: {len(made)} sites drawn "
                               f"({at_start} from t=0, the rest in {window[0]:.0f}-{window[1]:.0f} s) "
                               f"-> target_field.json")

    # ---------------------------------------------------------------- clock
    def _on_status(self, uid: str, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except ValueError:
            return
        self.states[uid] = body.get("state", "")
        self.status[uid] = body

    # ---------------------------------------------------------------- target selectors
    def _resolve_target(self, selector):
        """The console names a UAV (or '*' for every link); anything else is refused."""
        if selector in (None, "*") or selector in self.instances:
            return selector
        return None

    def _mission_t(self) -> float:
        return now_s(self) - self.t0

    def _tick(self) -> None:
        if now_s(self) <= 0.0:
            return
        if self.t0 is None:
            if all(self.states.get(u) in READY_STATES for u in self.instances):
                self.t0 = now_s(self)
                msg = String()
                msg.data = dumps({"t0": self.t0})
                self.clock_pub.publish(msg)
                self.events.emit("MISSION_CLOCK_START", t0=round(self.t0, 2))
            return
        mt = self._mission_t()
        while self.pending_pois and self.pending_pois[0]["appear_s"] <= mt:
            self._appear(self.pending_pois.pop(0))

    # ---------------------------------------------------------------- PoIs
    def _spawn_initial(self) -> None:
        for p in self.sc["pois"]:
            if p["appear_s"] <= 0.0:
                self._spawn_marker(p, retries=30)

    def _spawn_marker(self, poi: dict, retries: int = 5) -> None:
        if poi["id"] in self.spawned:
            return
        x, y = poi["pos_m"][:2]
        req = (f'sdf_filename: "{self.marker_sdf}", name: "poi_{poi["id"]}", '
               f'pose: {{position: {{x: {x}, y: {y}, z: 0.0}}}}')
        cmd = ["gz", "service", "-s", f"/world/{self.world}/create", "--reqtype", "gz.msgs.EntityFactory",
               "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req]
        for _ in range(retries):
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
            except subprocess.TimeoutExpired:
                out = ""
            if "true" in out:
                self.spawned.add(poi["id"])
                self.get_logger().info(f"PoI marker {poi['id']} spawned at ({x}, {y})")
                return
            time.sleep(2.0)
        self.get_logger().error(f"could not spawn PoI marker {poi['id']}")

    def _appear(self, poi: dict) -> None:
        if poi["id"] not in self.spawned:
            threading.Thread(target=self._spawn_marker, args=(poi,), daemon=True).start()
        body = {"id": poi["id"], "reported_m": poi.get("reported_m", poi["pos_m"]),
                "priority": poi["priority"], "survey_s": poi["survey_s"]}
        self.appeared[poi["id"]] = poi
        msg = String()
        msg.data = dumps(body)
        self.poi_pub.publish(msg)
        self.events.emit("POI_APPEARED", poi_id=poi["id"], true_pos=poi["pos_m"], reported=body["reported_m"],
                         priority=poi["priority"], mission_t=round(self._mission_t(), 1))

    # ---------------------------------------------------------------- disturbances
    def _on_request(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            self.get_logger().error(f"bad inject request: {msg.data}")
            return
        if ev.get("source") != "control_panel":
            self.get_logger().warn(f"refused '{ev.get('type')}': disturbances can only be injected "
                                   f"from the control console")
            return
        self._execute(ev)

    def _execute(self, ev: dict) -> None:
        kind = ev.get("type")
        ev["hidden"] = True            # the swarm is never told in advance
        if "target" in ev:
            selector = ev["target"]
            ev["target"] = self._resolve_target(selector)
            if ev["target"] is None:
                self.get_logger().warn(f"{kind}: no UAV matches '{selector}' now - skipped")
                return
            if selector != ev["target"]:
                ev["selector"] = selector
            if kind in ("uav_failure", "battery"):
                healthy = [u for u in self.instances if u not in self.removed and u != ev["target"]]
                if not healthy:
                    self.get_logger().warn(f"{kind}: {ev['target']} is the last healthy UAV - skipped")
                    return
                self.removed.add(ev["target"])
        self.events.emit("DISTURBANCE", kind=kind,
                         mission_t=round(self._mission_t(), 1) if self.t0 else None,
                         **{k: v for k, v in ev.items() if k != "type"})
        if kind == "new_poi":
            if not ev.get("id"):
                self.console_poi_count += 1
                ev["id"] = f"E{self.console_poi_count}"
            if not ev.get("reported_m"):
                # a field report is never exact: offset it like the scenario's own PoIs
                ang, off = self.rng.uniform(0, 6.283), self.rng.uniform(3.0, 8.0)
                ev["reported_m"] = [round(ev["pos_m"][0] + off * math.cos(ang), 1),
                                    round(ev["pos_m"][1] + off * math.sin(ang), 1)]
            poi = {"id": ev["id"], "pos_m": ev["pos_m"], "reported_m": ev["reported_m"],
                   "priority": int(ev.get("priority", 5)), "survey_s": float(ev.get("survey_s", 20.0)),
                   "appear_s": 0.0}
            self.sc["pois"].append(poi)
            self._appear(poi)
            return
        msg = String()
        msg.data = dumps(ev)
        self.inject_pub.publish(msg)
        if kind == "uav_failure":
            uav = self.instances.get(ev.get("target"))
            if uav:
                threading.Thread(target=self._force_disarm, args=(uav,), daemon=True).start()

    def _force_disarm(self, uav: dict) -> None:
        """Motor failure: emergency-stop the motors through MAVLink (MAVProxy output port)."""
        try:
            from pymavlink import mavutil
            port = 14550 + 10 * int(uav["instance"])
            mav = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=251)
            if mav.wait_heartbeat(timeout=15) is None:
                self.get_logger().error(f"uav_failure: no heartbeat from {uav['id']} on udp {port}")
                return
            for _ in range(5):
                mav.mav.command_long_send(uav["sysid"], 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                          0, 0, 21196, 0, 0, 0, 0, 0)
                hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    self.get_logger().warn(f"uav_failure: {uav['id']} motors stopped")
                    break
            mav.close()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"uav_failure: {exc}")


def main(args=None):
    rclpy.init(args=args)
    spin(ScenarioManager())


if __name__ == "__main__":
    main()
