"""Referee for Levels 1-4: logs everything, scores the run, ends it.

Uses ground truth (Gazebo odometry) and every node's events - information no UAV or the
GCS has - so the verdict does not depend on the autonomy grading itself.

Writes to the run directory
  events.jsonl     every event on /uavx/events (+ the referee's own)
  timeseries.csv   2 Hz per UAV: truth position, speed, state, battery, GCS link, separation
  summary.json     metrics + pass/fail per criterion of the scenario's `pass:` block

The run ends when every UAV is LANDED / FAILED / DONE (or at duration_s sim time).
Metric names follow the organisers' list (Stage 2 report template, section 10).
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ..common import EVENT_QOS, LATCHED_QOS, SENSOR_QOS, dist2, dist3, load_scenario, now_s
from ..world_map import WorldMap

FINAL = {"LANDED", "FAILED", "DONE"}
# Fallback for a scenario with no map; a map's own `geofence:` block wins (world_map.py).
GEOFENCE = {"x": (-200.0, 1100.0), "y": (-550.0, 550.0), "z_max": 100.0}


class MissionMonitor(Node):
    def __init__(self):
        super().__init__("mission_monitor")
        self.declare_parameter("scenario", "level1")
        self.declare_parameter("run_dir", ".")
        self.sc = load_scenario(self.get_parameter("scenario").value)
        self.run_dir = self.get_parameter("run_dir").value
        os.makedirs(self.run_dir, exist_ok=True)
        self.uavs = [u["id"] for u in self.sc["uavs"]]
        self.gcs_id = self.sc["gcs"].get("id", "gcs")
        self.gcs_pos = tuple(self.sc["gcs"]["position_m"])
        self.use_radio = "radio_net" in self.sc["components"]

        self.truth: dict[str, tuple] = {}
        self.prev_truth: dict[str, tuple] = {}
        self.status: dict[str, dict] = {}
        self.events: list[dict] = []
        self.links: dict[tuple, bool] = {}
        self.radio_stats: dict = {}
        self.max_speed = {u: 0.0 for u in self.uavs}
        self.max_alt = {u: 0.0 for u in self.uavs}
        self.airborne_s = {u: 0.0 for u in self.uavs}
        self.connected_s = {u: 0.0 for u in self.uavs}
        # multi-hop view (policy connected): a route to the GCS over links that are up,
        # time spent out on ferry sorties, and who was flying each PoI at the same moment
        self.route_s = {u: 0.0 for u in self.uavs}
        self.ferry_s = {u: 0.0 for u in self.uavs}
        self.ferry_route_s = {u: 0.0 for u in self.uavs}
        self.route_track: dict[str, list] = {u: [] for u in self.uavs}    # (t, route up)
        self.max_ferries = 0
        self.owner_conflicts: list[dict] = []
        self.min_sep = math.inf
        self.min_sep_pair = None
        self.collisions = 0
        self.fence_violations = 0
        self.fence_flags = set()
        self.t_first_airborne = None
        self.t_last_final = None
        self.t_start_wall = time.time()
        self.done = False
        self.exit_code = None
        self.map = WorldMap.for_scenario(self.sc)
        self.fence = ({"x": (self.map.geofence[0], self.map.geofence[1]),
                       "y": (self.map.geofence[2], self.map.geofence[3]),
                       "z_max": self.map.fence_alt_m} if self.map is not None else GEOFENCE)
        self.track: dict[str, list] = {u: [] for u in self.uavs}       # (t, x, y, z, gcs_link)
        self.obstacle_hits: set = set()
        self.min_clearance = {u: math.inf for u in self.uavs}
        # Q42 asks for the inter-UAV communication range, the energy used and the mission
        # start and completion times, so they are measured here rather than inferred later.
        self.uav_link_d: list[float] = []
        self.uav_link_up_d: list[float] = []
        self.max_hops = 0
        self.mission_t0 = None
        self.area_visited: set = set()
        self.area_cells = 0
        self.cell_m = 25.0

        self.ev_fh = open(os.path.join(self.run_dir, "events.jsonl"), "w", encoding="utf-8")
        self.ts_fh = open(os.path.join(self.run_dir, "timeseries.csv"), "w", newline="", encoding="utf-8")
        self.ts = csv.writer(self.ts_fh)
        self.ts.writerow(["t", "uav", "east_m", "north_m", "up_m", "speed_mps", "state", "battery_pct",
                          "gcs_link_up", "min_separation_m"])

        for u in self.uavs:
            self.create_subscription(Odometry, f"/{u}/odometry", lambda m, u=u: self._on_odom(u, m), SENSOR_QOS)
            self.create_subscription(String, f"/{u}/companion/status", lambda m, u=u: self._on_status(u, m), EVENT_QOS)
        self.create_subscription(String, "/uavx/events", self._on_event, EVENT_QOS)
        self.create_subscription(String, "/scenario/clock", self._on_clock, LATCHED_QOS)
        if self.use_radio:
            from uavx_interfaces.msg import NetworkState
            self.create_subscription(NetworkState, "/radio/network_state", self._on_network, EVENT_QOS)
            self.create_subscription(String, "/radio/stats", self._on_radio_stats, EVENT_QOS)
        if self.map is not None and self.map.mission_area:
            x0, x1, y0, y1 = self.map.mission_area
            self.area_cells = max(1, int((x1 - x0) / self.cell_m) * int((y1 - y0) / self.cell_m))
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"referee: level {self.sc['level']} '{self.sc['name']}', UAVs {self.uavs}, "
                               f"watchdog {self.sc['duration_s']:.0f} s sim")

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, uav: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.truth[uav] = (p.x, p.y, p.z, now_s(self))

    def _on_status(self, uav: str, msg: String) -> None:
        try:
            self.status[uav] = json.loads(msg.data)
        except ValueError:
            pass

    def _on_event(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        self.events.append(ev)
        self.ev_fh.write(json.dumps(ev) + "\n")
        self.ev_fh.flush()

    def _on_clock(self, msg: String) -> None:
        try:
            self.mission_t0 = json.loads(msg.data).get("t0")
        except ValueError:
            pass

    def _on_network(self, msg) -> None:
        self.links = {(l.a, l.b): l.up for l in msg.links}
        for l in msg.links:
            if l.a == self.gcs_id or l.b == self.gcs_id:
                continue
            self.uav_link_d.append(float(l.distance_m))      # Q42: inter-UAV range
            if l.up:
                self.uav_link_up_d.append(float(l.distance_m))

    def _on_radio_stats(self, msg: String) -> None:
        try:
            self.radio_stats = json.loads(msg.data)
        except ValueError:
            pass

    def _gcs_link(self, uav: str) -> bool:
        return self.links.get((self.gcs_id, uav), self.links.get((uav, self.gcs_id), False))

    def _gcs_reachable(self) -> set:
        """Every node with a multi-hop route to the GCS over links that are up (truth)."""
        adj: dict[str, set] = {}
        for (a, b), up in self.links.items():
            if up:
                adj.setdefault(a, set()).add(b)
                adj.setdefault(b, set()).add(a)
        seen, todo = {self.gcs_id}, [self.gcs_id]
        while todo:
            for n in adj.get(todo.pop(), ()):
                if n not in seen:
                    seen.add(n)
                    todo.append(n)
        return seen

    def _route_at(self, uav: str, t: float) -> bool:
        """Did uav have a multi-hop route to the GCS at sim time t?"""
        track = self.route_track.get(uav) or []
        best = min(track, key=lambda p: abs(p[0] - t), default=None)
        return bool(best and abs(best[0] - t) < 2.0 and best[1])

    # ---------------------------------------------------------------- periodic
    def _tick(self) -> None:
        t = now_s(self)
        if t <= 0.0 or self.done:
            return
        dt = 0.5
        reach = self._gcs_reachable() if self.use_radio else set(self.uavs)
        ferrying = 0
        holders: dict[str, list] = {}
        for u in self.uavs:
            st = self.status.get(u, {})
            if st.get("ferry") and st.get("state") not in FINAL:
                ferrying += 1
            if st.get("state") in ("NAVIGATE", "SURVEY") and st.get("task"):
                holders.setdefault(st["task"], []).append(u)
        self.max_ferries = max(self.max_ferries, ferrying)
        for pid, who in holders.items():
            if len(who) > 1 and not any(c["poi"] == pid and set(c["uavs"]) == set(who) for c in self.owner_conflicts):
                self.owner_conflicts.append({"poi": pid, "uavs": sorted(who), "t": round(t, 1)})
                if self.sc["pass"].get("single_owner"):
                    self._emit("POI_OWNER_CONFLICT", poi=pid, uavs=sorted(who))
        for u in self.uavs:
            if u not in self.truth:
                continue
            x, y, z, ts = self.truth[u]
            speed = 0.0
            if u in self.prev_truth:
                px, py, pz, pts = self.prev_truth[u]
                if ts > pts:
                    speed = math.hypot(x - px, y - py) / (ts - pts)
            self.prev_truth[u] = (x, y, z, ts)
            st = self.status.get(u, {})
            airborne = z > 1.0 and st.get("state") not in FINAL
            if airborne:
                self.airborne_s[u] += dt
                self.t_first_airborne = self.t_first_airborne or t
                self.max_speed[u] = max(self.max_speed[u], speed)
                self.max_alt[u] = max(self.max_alt[u], z)
                if self.use_radio and self._gcs_link(u):
                    self.connected_s[u] += dt
                if u in reach:
                    self.route_s[u] += dt
                if st.get("ferry"):
                    self.ferry_s[u] += dt
                    if u in reach:
                        self.ferry_route_s[u] += dt
            if airborne or z > 0.5:
                self.route_track[u].append((round(t, 1), u in reach))
            if airborne and self.area_cells and self.map.inside_area(x, y):
                self.area_visited.add((int(x // self.cell_m), int(y // self.cell_m)))
            outside = not (self.fence["x"][0] <= x <= self.fence["x"][1]
                           and self.fence["y"][0] <= y <= self.fence["y"][1]
                           and z <= self.fence["z_max"] + 2.0)
            if outside and u not in self.fence_flags:
                self.fence_violations += 1
                self.fence_flags.add(u)
                self._emit("GEOFENCE_VIOLATION", uav=u, pos=[round(x, 1), round(y, 1), round(z, 1)])
            elif not outside:
                self.fence_flags.discard(u)
            if airborne or z > 0.5:
                self.track[u].append((round(t, 1), round(x, 2), round(y, 2), round(z, 2),
                                      bool(self._gcs_link(u)) if self.use_radio else True))
            if self.map is not None and z > 2.0:
                self.min_clearance[u] = min(self.min_clearance[u], self.map.clearance(x, y, z))
                for o in self.map.obstacles:
                    if o.contains(x, y, z, 0.3) and (u, o.id) not in self.obstacle_hits:
                        self.obstacle_hits.add((u, o.id))
                        self._emit("OBSTACLE_COLLISION", uav=u, obstacle=o.id, pos=[round(x, 1), round(y, 1), round(z, 1)])
            sep = self._separation(u)
            self.ts.writerow([round(t, 2), u, round(x, 2), round(y, 2), round(z, 2), round(speed, 2),
                              st.get("state", ""), st.get("batt", ""), int(self._gcs_link(u)) if self.use_radio else "",
                              "" if sep is None else round(sep, 2)])
        self.ts_fh.flush()
        self._pair_checks()

        states = [self.status.get(u, {}).get("state") for u in self.uavs]
        all_final = all(s in FINAL for s in states)
        if all_final and self.t_first_airborne is not None:
            self.t_last_final = self.t_last_final or t
            if t - self.t_last_final > 3.0:
                self._finish("all UAVs finished")
        elif t > float(self.sc["duration_s"]):
            self._finish("watchdog: duration_s reached")

    def _separation(self, uav: str):
        if uav not in self.truth:
            return None
        best = None
        for other in self.uavs:
            if other != uav and other in self.truth:
                d = dist3(self.truth[uav][:3], self.truth[other][:3])
                best = d if best is None else min(best, d)
        return best

    def _pair_checks(self) -> None:
        for i, a in enumerate(self.uavs):
            for b in self.uavs[i + 1:]:
                if a not in self.truth or b not in self.truth:
                    continue
                d = dist3(self.truth[a][:3], self.truth[b][:3])
                if d < self.min_sep:
                    self.min_sep, self.min_sep_pair = d, (a, b)
                if d < 1.5 and (a, b) not in getattr(self, "_collided", set()):
                    self._collided = getattr(self, "_collided", set()) | {(a, b)}
                    self.collisions += 1
                    self._emit("COLLISION", pair=[a, b], distance_m=round(d, 2))

    def _emit(self, etype: str, **fields) -> None:
        ev = {"t": round(now_s(self), 3), "src": "referee", "type": etype, **fields}
        self.events.append(ev)
        self.ev_fh.write(json.dumps(ev) + "\n")
        self.get_logger().warn(f"[{etype}] {fields}")

    # ---------------------------------------------------------------- scoring
    def _ev(self, etype: str, src: str | None = None) -> list[dict]:
        return [e for e in self.events if e.get("type") == etype and (src is None or e.get("src") == src)]

    def _all_pois(self) -> list[dict]:
        """The scenario's PoIs plus every PoI that appeared during the run (hidden ones)."""
        pois = {p["id"]: {"id": p["id"], "pos_m": p["pos_m"], "priority": p["priority"], "hidden": False}
                for p in self.sc["pois"]}
        for e in self._ev("POI_APPEARED"):
            pois.setdefault(e["poi_id"], {"id": e["poi_id"], "pos_m": e["true_pos"],
                                          "priority": e.get("priority", 1), "hidden": True})
        return list(pois.values())

    def _metrics(self) -> dict:
        m: dict = {"sim_time_s": round(now_s(self), 1), "wall_time_s": round(time.time() - self.t_start_wall, 1)}
        m["real_time_factor"] = round(m["sim_time_s"] / max(m["wall_time_s"], 1e-3), 2)
        m["uavs"] = {}
        for u in self.uavs:
            landed = self._ev("LANDED", u)
            failed = self._ev("UAV_FAILED", u)
            st = self.status.get(u, {})
            m["uavs"][u] = {
                "final_state": st.get("state"),
                "battery_end_pct": st.get("batt"),
                "airborne_s": round(self.airborne_s[u], 1),
                "max_speed_mps": round(self.max_speed[u], 2),
                "max_alt_m": round(self.max_alt[u], 1),
                "pad_error_m": landed[-1].get("pad_error_m") if landed else None,
                "failed": bool(failed),
                "tasks_done": [e["task"] for e in self._ev("TASK_DONE", u)],
            }
            if self.use_radio:
                m["uavs"][u]["gcs_connectivity"] = round(self.connected_s[u] / self.airborne_s[u], 3) \
                    if self.airborne_s[u] > 0 else None
            # Q42: battery consumption and flight endurance, per UAV, from its own status
            endurance = st.get("endurance_min")
            used = None if st.get("batt") is None else round(100.0 - float(st["batt"]), 1)
            m["uavs"][u].update(
                endurance_min=endurance,
                battery_used_pct=used,
                battery_pct_per_min=round(used / (self.airborne_s[u] / 60.0), 2)
                if used is not None and self.airborne_s[u] > 30 else None,
                flight_time_s=st.get("flight_s"),
                endurance_margin_s=None if endurance is None or st.get("flight_s") is None
                else round(endurance * 60.0 - float(st["flight_s"]), 1),
                role=st.get("role"), recharges=st.get("recharges", 0),
                ferries=st.get("ferries", 0), frames_forwarded=st.get("forwarded", 0),
                sweep=st.get("sweep"))
        m["min_separation_m"] = None if self.min_sep is math.inf else round(self.min_sep, 2)
        m["collisions"] = self.collisions
        m["geofence_violations"] = self.fence_violations
        if self.map is not None:
            m["obstacle_collisions"] = len(self.obstacle_hits)
            m["min_obstacle_clearance_m"] = {u: (None if v is math.inf else round(v, 1))
                                             for u, v in self.min_clearance.items()}
        # PoIs: detection + geolocation error against the true positions (hidden ones included)
        truth = {p["id"]: p for p in self._all_pois()}
        pois = {}
        for p in truth.values():
            det = [e for e in self._ev("POI_DETECTED") if e.get("poi_id") == p["id"]]
            done = [e for e in self._ev("TASK_DONE") if e.get("task") == p["id"]]
            est = next((e.get("estimate") for e in reversed(done) if e.get("estimate")), None)
            if est is None and det:
                est = det[0].get("estimate")
            err = round(dist2(est, p["pos_m"]), 2) if est else None
            rx = [e for e in self._ev("POI_RECEIVED", self.gcs_id) if e.get("poi_id") == p["id"]]
            completed = [e for e in self._ev("POI_COMPLETED", self.gcs_id) if e.get("poi_id") == p["id"]]
            pois[p["id"]] = {
                "priority": p["priority"],
                "hidden": p.get("hidden", False),
                "true_pos_m": p["pos_m"],
                "detected_by": det[0]["src"] if det else None,
                "detected_t": det[0]["t"] if det else None,
                "geolocation_error_m": err,
                "surveyed": any(e.get("outcome") == "surveyed" for e in done),
                "report_at_gcs_t": rx[0]["t"] if rx else None,
                "report_latency_s": rx[0].get("latency_s") if rx else None,
                "completed_at_gcs": bool(completed),
                "image": det[0].get("image") if det else None,
            }
        m["pois"] = pois
        if truth:
            done_w = sum(truth[k]["priority"] for k, v in pois.items() if v["surveyed"] or v["completed_at_gcs"])
            m["mission_completion_rate"] = round(
                sum(1 for v in pois.values() if v["surveyed"] or v["completed_at_gcs"]) / len(pois), 3)
            m["priority_weighted_score"] = round(done_w / sum(p["priority"] for p in truth.values()), 3)
        done_events = self._ev("TASK_DONE")
        if done_events and self.t_first_airborne:
            m["mission_completion_time_s"] = round(max(e["t"] for e in done_events) - self.t_first_airborne, 1)
        # Q42: mission start and completion times, on the mission clock the scenario sets
        last_done = max((e["t"] for e in done_events), default=None)
        recall = next((e["t"] for e in self._ev("RECALL", self.gcs_id)), None)
        m["mission"] = {
            "clock_start_t": None if self.mission_t0 is None else round(self.mission_t0, 1),
            "first_airborne_t": None if self.t_first_airborne is None else round(self.t_first_airborne, 1),
            "last_task_done_t": None if last_done is None else round(last_done, 1),
            "recall_t": None if recall is None else round(recall, 1),
            "end_t": round(now_s(self), 1),
            "duration_s": None if self.mission_t0 is None else round(now_s(self) - self.mission_t0, 1),
            "work_duration_s": None if (self.mission_t0 is None or last_done is None)
            else round(last_done - self.mission_t0, 1)}
        m["images_saved"] = len([f for f in os.listdir(os.path.join(self.run_dir, "images"))]) \
            if os.path.isdir(os.path.join(self.run_dir, "images")) else 0
        m["coverage"] = self._coverage_metrics()
        if self.use_radio:
            m["communication"] = self._comm_metrics()
            m["autonomy"] = self._autonomy_metrics()
            m["relay"] = self._relay_metrics()
            m["network"] = self._network_metrics()
        return m

    def _network_metrics(self) -> dict:
        """The operating rules, measured: a multi-hop route to the GCS for every UAV off a
        ferry sortie, one ferry at a time, and no PoI flown by two UAVs at once."""
        air = {u: self.airborne_s[u] - self.ferry_s[u] for u in self.uavs}
        up = {u: self.route_s[u] - self.ferry_route_s[u] for u in self.uavs}
        total = sum(air.values())
        return {
            "connectivity_off_ferry": round(sum(up.values()) / total, 3) if total > 0 else None,
            "per_uav": {u: {"airborne_off_ferry_s": round(air[u], 1),
                            "route_up_s": round(up[u], 1),
                            "ferry_s": round(self.ferry_s[u], 1),
                            "connectivity": round(up[u] / air[u], 3) if air[u] > 0 else None}
                        for u in self.uavs},
            "max_concurrent_ferries": self.max_ferries,
            "poi_owner_conflicts": self.owner_conflicts,
        }

    def _coverage_metrics(self) -> dict:
        """How much of the organisers' search area the swarm actually flew over."""
        c: dict = {"sectors": {}, "area_cells": self.area_cells,
                   "cells_visited": len(self.area_visited)}
        if self.area_cells:
            c["area_fraction_overflown"] = round(len(self.area_visited) / self.area_cells, 3)
        plan = self._ev("COVERAGE_PLAN", self.gcs_id)
        if plan:
            c["plan"] = plan[-1].get("sectors")
            c["survey_alt_m"] = plan[-1].get("survey_alt_m")
            c["lane_spacing_m"] = plan[-1].get("lane_spacing_m")
        for e in self._ev("SECTOR_ASSIGNED", self.gcs_id):
            c["sectors"].setdefault(e["sector"], {})["assigned_to"] = e.get("uav")
        for e in self._ev("SECTOR_COVERED", self.gcs_id):
            c["sectors"].setdefault(e["sector"], {})["covered_t"] = round(e["t"], 1)
        c["sectors_covered"] = sum(1 for v in c["sectors"].values() if "covered_t" in v)
        c["sectors_planned"] = len(c.get("plan") or c["sectors"])
        for u in self.uavs:
            st = self.status.get(u, {}).get("sweep")
            if st:
                c["sectors"].setdefault(st["sector"], {})["progress"] = st.get("progress")
        return c

    def _relay_metrics(self) -> dict:
        """Multi-hop and data-ferry behaviour: the heart of the Stage 2 comms strategy."""
        a: dict = {}
        chains = self._ev("CHAIN_PLANNED", self.gcs_id)
        a["chain_plans"] = len(chains)
        if chains:
            last = chains[-1]
            a["final_chain"] = {k: last.get(k) for k in ("mode", "needed", "budget", "points", "tip", "handoff")}
            a["modes"] = {mode: sum(1 for e in chains if e.get("mode") == mode)
                          for mode in sorted({e.get("mode") for e in chains if e.get("mode")})}
            a["relays_needed_max"] = max((e.get("needed") or 0) for e in chains)
        a["relay_assignments"] = len(self._ev("RELAY_ASSIGNED"))
        a["relay_releases"] = len(self._ev("RELAY_RELEASED"))
        a["relays_on_station"] = [{"uav": e["src"], "t": round(e["t"], 1), "point": e.get("point")}
                                  for e in self._ev("RELAY_ON_STATION")]
        a["frames_forwarded"] = {u: self.status.get(u, {}).get("forwarded", 0) for u in self.uavs}
        a["frames_forwarded_total"] = sum(a["frames_forwarded"].values())
        ferries = self._ev("FERRY_START")
        done = self._ev("FERRY_DONE")
        a["ferries"] = len(ferries)
        a["ferry_legs_m"] = [e.get("leg_m") for e in ferries if e.get("leg_m") is not None]
        a["ferry_elapsed_s"] = [e.get("elapsed_s") for e in done if e.get("elapsed_s") is not None]
        a["ferries_skipped"] = len(self._ev("FERRY_SKIPPED"))
        a["route_changes"] = len(self._ev("ROUTE_CHANGED"))
        a["route_down_events"] = len(self._ev("ROUTE_DOWN"))
        hops = [e.get("hops") for e in self._ev("POI_RECEIVED", self.gcs_id) if e.get("hops") is not None]
        a["report_hops"] = {"max": max(hops) if hops else None,
                            "multi_hop_reports": sum(1 for h in hops if h > 1), "count": len(hops),
                            # `hops` counts forwards: 1 = it came home through one relay
                            "relayed_reports": sum(1 for h in hops if h >= 1)}
        a["radio_hops"] = (self.radio_stats.get("hops") or {})
        return a

    def _comm_metrics(self) -> dict:
        c: dict = {}
        stats = self.radio_stats.get("links", {})
        # keys look like "uav1->gcs|STATUS"
        tx = sum(v["tx"] for k, v in stats.items() if k.split("|")[0].endswith(f"->{self.gcs_id}") and k.endswith("|STATUS"))
        rx = sum(v["delivered"] for k, v in stats.items() if k.split("|")[0].endswith(f"->{self.gcs_id}") and k.endswith("|STATUS"))
        c["status_pdr_uav_to_gcs"] = round(rx / tx, 3) if tx else None
        all_tx = sum(v["tx"] for v in stats.values())
        all_rx = sum(v["delivered"] for v in stats.values())
        c["frame_pdr_all"] = round(all_rx / all_tx, 3) if all_tx else None
        lat = self.radio_stats.get("latency_ms", {})
        c["link_latency_ms_mean"] = lat.get("mean")
        c["link_latency_ms_p95"] = lat.get("p95")
        rep = [e.get("latency_s") for e in self._ev("POI_RECEIVED", self.gcs_id) if e.get("latency_s") is not None]
        c["poi_report_latency_s"] = {"mean": round(sum(rep) / len(rep), 2) if rep else None,
                                     "max": round(max(rep), 2) if rep else None,
                                     "within_10s": sum(1 for r in rep if r <= 10.0), "count": len(rep)}
        air = sum(self.airborne_s.values())
        c["connectivity_availability"] = round(sum(self.connected_s.values()) / air, 3) if air else None
        c["communication_downtime_s"] = {u: round(self.airborne_s[u] - self.connected_s[u], 1) for u in self.uavs}
        c["link_lost_events"] = len(self._ev("LINK_LOST"))
        # Q42: the communication range actually used between UAVs, and the organisers' limit
        if self.uav_link_d:
            c["inter_uav_range_m"] = {
                "max_sampled": round(max(self.uav_link_d), 1),
                "max_while_up": round(max(self.uav_link_up_d), 1) if self.uav_link_up_d else None,
                "mean_while_up": round(sum(self.uav_link_up_d) / len(self.uav_link_up_d), 1)
                if self.uav_link_up_d else None,
                "limit_m": float((self.sc.get("radio") or {}).get("max_range_m", 100.0)),
                "samples": len(self.uav_link_d)}
        c["impairments_at_end"] = (self.radio_stats.get("impairments") or {})
        c["backend"] = (self.radio_stats.get("backend") or {}).get("backend")
        return c

    def _autonomy_metrics(self) -> dict:
        a: dict = {}
        realloc = self._ev("REALLOCATED", self.gcs_id)
        a["reallocations"] = len(realloc)
        a["pois_moved"] = {k: v for e in realloc for k, v in (e.get("moved") or {}).items()}
        a["lost_declarations"] = [e.get("uav") for e in self._ev("UAV_DECLARED_LOST", self.gcs_id)]
        rec = [e.get("recovery_s") for e in self._ev("RECOVERED", self.gcs_id) if e.get("recovery_s") is not None]
        a["recovery_time_s"] = rec
        det = [e.get("detection_s") for e in self._ev("UAV_DECLARED_LOST", self.gcs_id) if e.get("detection_s") is not None]
        a["failure_detection_s"] = det
        a["preemptions"] = len(self._ev("PREEMPTED"))
        a["store_and_forward_deliveries"] = len([e for e in self._ev("POI_RECEIVED", self.gcs_id) if e.get("attempts", 1) > 1])
        return a

    # ---------------------------------------------------------------- disturbances
    def _frames(self) -> list[tuple]:
        """(t, src, dst, delivered, reason) for every radio frame, from radio_net's packets.csv."""
        path = os.path.join(self.run_dir, "packets.csv")
        if not os.path.isfile(path):
            return []
        rows = []
        with open(path, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                try:
                    rows.append((float(r["t"]), r["src"], r["dst"], r["delivered"] == "1", r["reason"]))
                except (KeyError, ValueError):
                    continue
        return rows

    @staticmethod
    def _pdr(frames, t0, t1, nodes=None) -> float | None:
        """Delivery ratio of frames sent over a usable link in [t0, t1). Frames that never had a
        link (out of range) or hit a dead radio (an outage / failed UAV - its own disturbance)
        are left out, so the number isolates loss and jamming."""
        sel = [f for f in frames if t0 <= f[0] < t1 and f[4] not in ("no_link", "unknown_node", "outage")
               and (nodes is None or f[1] in nodes or f[2] in nodes)]
        return round(sum(f[3] for f in sel) / len(sel), 3) if sel else None

    def _disturbance_report(self) -> list[dict]:
        """One row per disturbance that fired: how the swarm detected it and recovered
        (the Stage 2 report template's section 7 table)."""
        frames = self._frames()
        fail_timeout = float((self.sc.get("gcs_logic") or {}).get("fail_timeout_s", 6.0))
        rows = []
        for d in self._ev("DISTURBANCE"):
            t, kind, target = d["t"], d.get("kind"), d.get("target")
            dur = float(d.get("duration_s") or 0.0)
            row = {"t": t, "mission_t": d.get("mission_t"), "type": kind, "target": target,
                   "hidden": bool(d.get("hidden")), "handled": True}
            later = [e for e in self.events if e.get("t", 0) >= t]
            if kind == "uav_failure":
                lost = next((e for e in later if e["type"] == "UAV_DECLARED_LOST" and e.get("uav") == target), None)
                realloc = next((e for e in later if e["type"] == "REALLOCATED" and e.get("lost") == target), None)
                rec = [e for e in later if e["type"] == "RECOVERED" and e.get("lost") == target]
                row.update(detection_s=round(lost["t"] - t, 2) if lost else None,
                           orphans=lost.get("orphans") if lost else None,
                           moved=realloc.get("moved") if realloc else None,
                           recovery_s=round(max(e["t"] for e in rec) - t, 2) if rec else None,
                           response="declared lost by the GCS, PoIs re-allocated")
                row["handled"] = lost is not None and (not lost.get("orphans") or bool(realloc and realloc.get("moved")))
            elif kind == "battery":
                rth = next((e for e in later if e["type"] == "BATTERY_RTH" and e["src"] == target), None)
                landed = next((e for e in later if e["type"] == "LANDED" and e["src"] == target), None)
                realloc = next((e for e in later if e["type"] == "REALLOCATED" and e.get("lost") == target), None)
                row.update(detection_s=round(rth["t"] - t, 2) if rth else None,
                           released=rth.get("released") if rth else None,
                           moved=realloc.get("moved") if realloc else None,
                           landed_battery_pct=landed.get("battery_pct") if landed else None,
                           response="returned to recharge, released PoIs re-allocated")
                row["handled"] = rth is not None and landed is not None
            elif kind == "comm_outage":
                in_contact = self._link_at(target, t)
                window = [e for e in later if e["t"] <= t + dur + 15.0 and e["src"] == target]
                down = next((e for e in window if e["type"] == "LINK_LOST"), None)
                up = next((e for e in later if e["type"] == "LINK_RESTORED" and e["src"] == target
                           and (down is None or e["t"] >= down["t"])), None)
                false_lost = [e for e in later if e["type"] == "UAV_DECLARED_LOST" and e.get("uav") == target
                              and e["t"] <= t + dur + fail_timeout + 2]
                row.update(in_contact_when_hit=in_contact,
                           detection_s=round(down["t"] - t, 2) if down else None,
                           downtime_s=round(up["t"] - down["t"], 1) if down and up else None,
                           declared_lost=bool(false_lost), response="rode through (buffered, ARQ)")
                # handled = the link came back and nobody was written off; an outage that hits a
                # UAV already out of range changes nothing it could react to
                row["handled"] = not false_lost and (up is not None or not in_contact)
            elif kind in ("packet_loss", "jamming"):
                nodes = None if target in (None, "*") else {target}
                row.update(pdr_during=self._pdr(frames, t, t + dur, nodes),
                           pdr_before=self._pdr(frames, max(0.0, t - 60.0), t, nodes),
                           response="ARQ retransmission / store-and-forward")
                if kind == "jamming":
                    row.update(center_m=d.get("center_m"), radius_m=d.get("radius_m"), near_poi=d.get("near_poi"))
            elif kind == "new_poi":
                pid = d.get("id")
                assigned = next((e for e in later if e["type"] == "ASSIGNED" and e.get("poi_id") == pid), None)
                rx = next((e for e in later if e["type"] == "POI_RECEIVED" and e.get("poi_id") == pid), None)
                done = next((e for e in later if e["type"] == "POI_COMPLETED" and e.get("poi_id") == pid), None)
                row.update(poi=pid, priority=d.get("priority"), assigned_to=assigned.get("uav") if assigned else None,
                           assign_s=round(assigned["t"] - t, 2) if assigned else None,
                           report_s=round(rx["t"] - t, 1) if rx else None,
                           completed_s=round(done["t"] - t, 1) if done else None,
                           response="prioritised and re-planned")
                row["handled"] = done is not None
            rows.append(row)
        return rows

    def _link_at(self, uav: str, t: float) -> bool:
        """Did uav have a GCS link (radio_net ground truth) at sim time t?"""
        track = self.track.get(uav) or []
        best = min(track, key=lambda p: abs(p[0] - t), default=None)
        return bool(best and abs(best[0] - t) < 2.0 and best[4])

    def _expected_failures(self) -> set:
        """UAVs allowed to end FAILED: listed in the scenario, or 'injected' = whichever
        UAV a uav_failure disturbance actually hit (targets are chosen at run time)."""
        allowed = self.sc["pass"].get("allow_failure_of", [])
        if allowed == "injected":
            return {e.get("target") for e in self._ev("DISTURBANCE") if e.get("kind") == "uav_failure"}
        return set(allowed)

    def _criteria(self, m: dict) -> list[dict]:
        p = self.sc["pass"]
        out = []

        def check(name, ok, value):
            out.append({"check": name, "pass": bool(ok), "value": value})

        expected_failed = self._expected_failures()
        survivors = [u for u in self.uavs if u not in expected_failed]
        if p.get("all_landed", True):
            bad = [u for u in self.uavs if u not in expected_failed and m["uavs"][u]["final_state"] != "LANDED"]
            check("every UAV landed on its pad", not bad, "all" if not bad else f"not landed: {bad}")
        if "pad_error_m" in p:
            errs = [m["uavs"][u]["pad_error_m"] for u in survivors if m["uavs"][u]["pad_error_m"] is not None]
            check(f"landing error <= {p['pad_error_m']} m", errs and max(errs) <= p["pad_error_m"],
                  f"max {max(errs):.2f} m" if errs else "none")
        if "max_speed_mps" in p:
            v = max(m["uavs"][u]["max_speed_mps"] for u in self.uavs)
            check(f"speed <= {p['max_speed_mps']} m/s (truth)", v <= p["max_speed_mps"], f"{v:.2f} m/s")
        if "max_alt_m" in p:
            v = max(m["uavs"][u]["max_alt_m"] for u in self.uavs)
            check(f"height <= {p['max_alt_m']} m", v <= p["max_alt_m"], f"{v:.1f} m")
        check("no geofence violation", m["geofence_violations"] == 0, m["geofence_violations"])
        if len(self.uavs) > 1:
            check("no collision", m["collisions"] == 0, m["collisions"])
            if "min_separation_m" in p:
                v = m["min_separation_m"]
                check(f"min separation >= {p['min_separation_m']} m", v is not None and v >= p["min_separation_m"],
                      f"{v} m")
        if "waypoints_reached" in p:
            n = sum(1 for e in self._ev("TASK_DONE") if e.get("kind") == "waypoint")
            check("waypoints reached", n >= p["waypoints_reached"], f"{n}/{p['waypoints_reached']}")
        pois = m.get("pois", {})
        if p.get("pois_detected") == "all":
            n = sum(1 for v in pois.values() if v["detected_by"])
            check("every PoI found by camera", n == len(pois), f"{n}/{len(pois)}")
        if "geolocation_error_m" in p:
            errs = [v["geolocation_error_m"] for v in pois.values() if v["geolocation_error_m"] is not None]
            check(f"geolocation error <= {p['geolocation_error_m']} m", errs and max(errs) <= p["geolocation_error_m"],
                  f"max {max(errs):.2f} m, mean {sum(errs) / len(errs):.2f} m" if errs else "none")
        if p.get("images_saved"):
            check("survey imagery saved", m["images_saved"] >= p["images_saved"], m["images_saved"])
        if p.get("reports_at_gcs") == "all":
            n = sum(1 for v in pois.values() if v["report_at_gcs_t"] is not None)
            check("every PoI report reached the GCS", n == len(pois), f"{n}/{len(pois)}")
        if self.map is not None:
            check("no obstacle collision (3-D map)", m.get("obstacle_collisions", 0) == 0,
                  f"{m.get('obstacle_collisions', 0)}, min clearance "
                  f"{min((v for v in m['min_obstacle_clearance_m'].values() if v is not None), default=None)} m")
        if p.get("disturbances_handled"):
            rows = m.get("disturbances", [])
            bad = [f"{r['type']}@{r['mission_t']}" for r in rows if not r["handled"]]
            hidden = sum(1 for r in rows if r["hidden"])
            # Every disturbance comes from the operator's console, so a run can be clean:
            # with nothing injected there is nothing left unhandled.
            allow_none = p.get("disturbances_handled") in ("if_any", True)
            check(f"every disturbance handled ({len(rows)}, {hidden} hidden)",
                  (rows or allow_none) and not bad,
                  f"not handled: {bad}" if bad else (", ".join(f"{r['type']}@{r['mission_t']}" for r in rows)
                                                     or "none injected"))
        if "report_latency_s" in p:
            live = p.get("live_pois") or [k for k in pois if k not in set(p.get("ferry_pois", []))]
            lat = [pois[k]["report_latency_s"] for k in live if pois[k]["report_latency_s"] is not None]
            ok = len(lat) == len(live) and all(x <= p["report_latency_s"] for x in lat)
            check(f"in-range reports <= {p['report_latency_s']} s", ok,
                  f"max {max(lat):.2f} s over {len(lat)}" if lat else "none")
        if "live_report_latency_s" in p:
            # the organisers' 10 s rule, applied where it is physically possible: the UAV
            # that found the PoI had a GCS link at that moment (otherwise it is ferried)
            live, slow = [], []
            for pid, v in pois.items():
                if v["detected_t"] is None or v["report_latency_s"] is None:
                    continue
                if self._link_at(v["detected_by"], v["detected_t"]):
                    live.append(pid)
                    if v["report_latency_s"] > p["live_report_latency_s"]:
                        slow.append(f"{pid} {v['report_latency_s']} s")
            lat = [pois[k]["report_latency_s"] for k in live]
            check(f"reports found in contact <= {p['live_report_latency_s']} s", not slow,
                  f"{len(live)} live, max {max(lat):.2f} s; {len(pois) - len(live)} ferried" if lat else
                  f"none live; {len(pois)} ferried")
        if p.get("ferry_pois"):
            ferried = [k for k in p["ferry_pois"] if pois.get(k, {}).get("report_at_gcs_t") is not None]
            check("out-of-range report delivered (store-and-forward)", len(ferried) == len(p["ferry_pois"]),
                  f"{len(ferried)}/{len(p['ferry_pois'])}")
        if p.get("pois_completed") == "all":
            n = sum(1 for v in pois.values() if v["completed_at_gcs"] or v["surveyed"])
            check("every PoI surveyed", n == len(pois), f"{n}/{len(pois)}")
        if "reallocations_min" in p:
            n = m.get("autonomy", {}).get("reallocations", 0)
            moved = m.get("autonomy", {}).get("pois_moved", {})
            check(f"orphaned PoIs re-allocated (>= {p['reallocations_min']})",
                  n >= p["reallocations_min"] and bool(moved), f"{n} re-allocation(s), moved {moved}")
        if "max_lost_declarations" in p:
            lost = m.get("autonomy", {}).get("lost_declarations", [])
            false_alarms = [u for u in lost if u not in expected_failed]
            check("no false 'UAV lost' (outage ridden through)",
                  len(lost) <= p["max_lost_declarations"] and not false_alarms, f"declared lost: {lost}")
        if "failure_detection_max_s" in p:
            d = m.get("autonomy", {}).get("failure_detection_s", [])
            check(f"failure detected within {p['failure_detection_max_s']} s", d and max(d) <= p["failure_detection_max_s"],
                  d or "not detected")
        if "status_pdr_min" in p:
            v = m.get("communication", {}).get("status_pdr_uav_to_gcs")
            check(f"status PDR >= {p['status_pdr_min']}", v is not None and v >= p["status_pdr_min"], v)
        if p.get("emergency_poi"):
            e = pois.get(p["emergency_poi"], {})
            check(f"emergency PoI {p['emergency_poi']} surveyed", e.get("surveyed") or e.get("completed_at_gcs"),
                  e.get("detected_t"))
        # ---------------------------------------------------- area coverage
        cov = m.get("coverage", {})
        if "coverage_fraction_min" in p:
            v = cov.get("area_fraction_overflown")
            check(f"search area overflown >= {100 * p['coverage_fraction_min']:.0f} %",
                  v is not None and v >= p["coverage_fraction_min"],
                  f"{100 * v:.0f} % of {cov.get('area_cells')} cells" if v is not None else "not measured")
        if p.get("sectors_covered") == "all":
            n, total = cov.get("sectors_covered", 0), cov.get("sectors_planned", 0)
            check("every sector swept", total and n >= total, f"{n}/{total}")
        # ---------------------------------------------------- multi-hop and ferry
        rel = m.get("relay", {})
        if "multi_hop_reports_min" in p:
            n = (rel.get("report_hops") or {}).get("multi_hop_reports", 0)
            check(f"reports delivered through a relay (>= {p['multi_hop_reports_min']})",
                  n >= p["multi_hop_reports_min"],
                  f"{n} of {(rel.get('report_hops') or {}).get('count', 0)}, "
                  f"max {(rel.get('report_hops') or {}).get('max')} hops")
        if "relays_on_station_min" in p:
            n = len(rel.get("relays_on_station") or [])
            check(f"relays took station (>= {p['relays_on_station_min']})", n >= p["relays_on_station_min"], n)
        if p.get("ferry_delivered"):
            n = rel.get("ferries", 0)
            legs = rel.get("ferry_legs_m") or []
            undelivered = sum(1 for v in pois.values() if v["report_at_gcs_t"] is None)
            check("ferried reports reached the GCS", n > 0 and undelivered == 0,
                  f"{n} ferry trip(s), mean leg {sum(legs) / len(legs):.0f} m, "
                  f"{undelivered} report(s) never arrived" if legs else f"{n} trips")
        if "max_report_hops" in p:
            v = (rel.get("report_hops") or {}).get("max")
            check(f"reports took <= {p['max_report_hops']} hops", v is None or v <= p["max_report_hops"], v)
        # ---------------------------------------------------- connected-chain rules (policy connected)
        if "relayed_reports_min" in p:
            rh = rel.get("report_hops") or {}
            n = rh.get("relayed_reports", 0)
            check(f"reports came home through a relay (>= {p['relayed_reports_min']})",
                  n >= p["relayed_reports_min"], f"{n} of {rh.get('count', 0)}, max {rh.get('max')} forwards")
        net = m.get("network", {})
        if "network_connectivity_min" in p:
            v = net.get("connectivity_off_ferry")
            check(f"multi-hop route to the GCS off ferry sorties >= {100 * p['network_connectivity_min']:.0f} %",
                  v is not None and v >= p["network_connectivity_min"],
                  f"{100 * v:.1f} % of airborne time" if v is not None else "not measured")
        if "max_concurrent_ferries" in p:
            v = net.get("max_concurrent_ferries", 0)
            check(f"at most {p['max_concurrent_ferries']} UAV on a ferry sortie at a time",
                  v <= p["max_concurrent_ferries"], v)
        if p.get("single_owner"):
            bad = net.get("poi_owner_conflicts") or []
            check("no PoI flown by two UAVs at once", not bad,
                  "none" if not bad else ", ".join(f"{c['poi']}:{'+'.join(c['uavs'])}@{c['t']}" for c in bad))
        if "connected_report_latency_s" in p:
            # the 10 s rule wherever a multi-hop route existed at the moment of detection
            live, slow = [], []
            for pid, v in pois.items():
                if v["detected_t"] is None or v["report_latency_s"] is None:
                    continue
                if self._route_at(v["detected_by"], v["detected_t"]):
                    live.append(pid)
                    if v["report_latency_s"] > p["connected_report_latency_s"]:
                        slow.append(f"{pid} {v['report_latency_s']} s")
            lat = [pois[k]["report_latency_s"] for k in live]
            check(f"reports found on the network <= {p['connected_report_latency_s']} s", not slow,
                  (f"{len(live)} over the network, max {max(lat):.2f} s; {len(pois) - len(live)} ferried"
                   if lat else f"none over the network; {len(pois)} ferried")
                  + (f"; slow: {slow}" if slow else ""))
        # ---------------------------------------------------- organiser limits
        comm = m.get("communication", {})
        rng = comm.get("inter_uav_range_m") or {}
        if p.get("comm_range_limit") and rng:
            v, limit = rng.get("max_while_up"), rng.get("limit_m")
            check(f"no link used beyond {limit:.0f} m (organisers' range)",
                  v is None or v <= limit + 1.0, f"max {v} m while up")
        if p.get("endurance_respected"):
            over = [u for u in self.uavs
                    if (m["uavs"][u].get("endurance_margin_s") or 0) < 0]
            worst = min((m["uavs"][u].get("endurance_margin_s") for u in self.uavs
                         if m["uavs"][u].get("endurance_margin_s") is not None), default=None)
            check("no UAV flew beyond its endurance", not over,
                  f"tightest margin {worst} s" if worst is not None else "not measured")
        return out

    # ---------------------------------------------------------------- figures
    @staticmethod
    def _footprint(o, n: int = 16) -> list[tuple]:
        if o.shape == "cylinder":
            return [(o.cx + o.radius * math.cos(2 * math.pi * k / n), o.cy + o.radius * math.sin(2 * math.pi * k / n))
                    for k in range(n)]
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        hl, hw = o.length / 2, o.width / 2
        return [(o.cx + u * c - v * s, o.cy + u * s + v * c) for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]

    def _plot(self, m: dict, ok: bool) -> list[str]:
        """map3d.png (3-D view: map, flight paths, PoIs, failures, jamming) + map2d.png (top view)."""
        import types

        import matplotlib
        matplotlib.use("Agg")
        # A pip matplotlib next to Ubuntu's python3-matplotlib: the apt mpl_toolkits (a regular
        # package) would win over the pip one (a namespace package) and fail to import.
        # Bind mpl_toolkits to the copy that belongs to the matplotlib actually loaded.
        own = os.path.join(os.path.dirname(os.path.dirname(matplotlib.__file__)), "mpl_toolkits")
        if os.path.isdir(own) and "mpl_toolkits.mplot3d" not in sys.modules:
            pkg = types.ModuleType("mpl_toolkits")
            pkg.__path__ = [own]
            sys.modules["mpl_toolkits"] = pkg
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        colors = {u["id"]: tuple(u["color"][:3]) for u in self.sc["uavs"]}
        pois = m.get("pois", {})
        xs = [p[1] for tr in self.track.values() for p in tr] + [self.gcs_pos[0]] + [v["true_pos_m"][0] for v in pois.values()]
        ys = [p[2] for tr in self.track.values() for p in tr] + [self.gcs_pos[1]] + [v["true_pos_m"][1] for v in pois.values()]
        area = self.map.mission_area if self.map is not None else None
        if area:                      # always frame the organisers' search area, flown or not
            xs += [area[0], area[1]]
            ys += [area[2], area[3]]
        x0, x1, y0, y1 = min(xs) - 25, max(xs) + 25, min(ys) - 25, max(ys) + 25
        anchors = [e.get("point") for e in self._ev("RELAY_ON_STATION") if e.get("point")]
        zmax = max([p[3] for tr in self.track.values() for p in tr] + [o.height for o in (self.map.obstacles if self.map else [])]
                   + [20.0]) + 10
        obstacles = [o for o in (self.map.obstacles if self.map else [])
                     if x0 - 30 <= o.cx <= x1 + 30 and y0 - 30 <= o.cy <= y1 + 30]
        failures = {e["src"]: e["t"] for e in self._ev("UAV_FAILED")}
        jams = [r for r in m.get("disturbances", []) if r["type"] == "jamming" and isinstance(r.get("center_m"), list)]
        title = f"Level {self.sc['level']} {self.sc['name']} - {'PASS' if ok else 'FAIL'}"
        out = []

        def segments(track):
            seg, cur = [], None
            for p in track:
                if cur is None or p[4] != cur[0]:
                    if cur is not None:
                        seg.append(cur)
                    cur = (p[4], [p] if cur is None else [cur[1][-1], p])
                else:
                    cur[1].append(p)
            if cur is not None:
                seg.append(cur)
            return seg

        # ------------------------------------------------------------ 3-D
        fig = plt.figure(figsize=(13, 8.5))
        ax = fig.add_subplot(111, projection="3d")
        for o in obstacles:
            fp = self._footprint(o)
            top = [(x, y, o.height) for x, y in fp]
            bottom = [(x, y, 0.0) for x, y in fp]
            faces = [top] + [[bottom[i], bottom[(i + 1) % len(fp)], top[(i + 1) % len(fp)], top[i]] for i in range(len(fp))]
            ax.add_collection3d(Poly3DCollection(faces, facecolor=o.color, edgecolor=(0.25, 0.25, 0.25, 0.6),
                                                 linewidths=0.4, alpha=0.55))
            ax.text(o.cx, o.cy, o.height + 2, o.id, fontsize=7, color="0.25")
        if x0 < 75 < x1:
            ax.plot([75, 75], [max(y0, -500), min(y1, 500)], [0, 0], color=(0.9, 0.75, 0.1), lw=1.5)
        gx, gy, gz = self.gcs_pos
        ax.plot([gx, gx], [gy, gy], [0, gz], color="k", lw=2.5)
        ax.scatter([gx], [gy], [gz], marker="^", s=70, c="k")
        ax.text(gx, gy, gz + 3, "GCS", fontsize=8)
        for pid, v in pois.items():
            x, y = v["true_pos_m"][:2]
            ok_poi = v["surveyed"] or v["completed_at_gcs"]
            ax.scatter([x], [y], [0.5], marker="*" if v["hidden"] else "s", s=110 if v["hidden"] else 60,
                       c="magenta" if v["hidden"] else "red", edgecolors="k" if ok_poi else "none", depthshade=False)
            ax.text(x, y, 3, pid + ("" if ok_poi else " (missed)"), fontsize=8, color="darkred")
        for u, track in self.track.items():
            if not track:
                continue
            first = True
            for up, pts in segments(track):
                ax.plot([p[1] for p in pts], [p[2] for p in pts], [p[3] for p in pts], color=colors[u],
                        lw=1.8 if up else 1.2, ls="-" if up else ":", alpha=1.0 if up else 0.8,
                        label=f"{u} ({self.sc['uavs'][self.uavs.index(u)]['cruise_alt_m']:.0f} m)" if first else None)
                first = False
            if u in failures:
                last = track[-1]
                ax.scatter([last[1]], [last[2]], [max(last[3], 0.5)], marker="X", s=140, c="k")
                ax.text(last[1], last[2], last[3] + 4, f"{u} failed", fontsize=8)
        for j in jams:
            th = [2 * math.pi * k / 40 for k in range(41)]
            ax.plot([j["center_m"][0] + j["radius_m"] * math.cos(a) for a in th],
                    [j["center_m"][1] + j["radius_m"] * math.sin(a) for a in th], [0.3] * 41,
                    color="purple", ls="--", lw=1.2)
            ax.text(j["center_m"][0], j["center_m"][1], 2, "jamming", color="purple", fontsize=8)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_zlim(0, zmax)
        ax.set_box_aspect((x1 - x0, y1 - y0, 2.0 * zmax))
        ax.set_xlabel("East (m)", labelpad=8)
        ax.set_ylabel("North (m)", labelpad=8)
        ax.set_zlabel("Height (m)", labelpad=4)
        ax.view_init(elev=30, azim=-62)
        fig.subplots_adjust(left=0.0, right=0.94, bottom=0.02, top=0.95)
        ax.set_title(title + "   (solid: GCS link up, dotted: out of contact)", fontsize=10)
        ax.legend(loc="upper left", fontsize=8)
        path = os.path.join(self.run_dir, "map3d.png")
        fig.savefig(path, dpi=130, bbox_inches="tight")
        plt.close(fig)
        out.append(path)

        # ------------------------------------------------------------ top view
        fig, ax = plt.subplots(figsize=(10, 8))
        if area:
            ax.add_patch(plt.Rectangle((area[0], area[2]), area[1] - area[0], area[3] - area[2],
                                       fill=True, fc=(0.86, 0.91, 0.84), ec=(0.85, 0.72, 0.1),
                                       lw=1.4, zorder=0,
                                       label=f"search area {area[1] - area[0]:.0f} x {area[3] - area[2]:.0f} m"))
        if self.map is not None and self.map.canopy.trees:
            step = max(1, self.map.canopy.count // 400)      # a readable sample of the forest
            tx = [self.map.canopy.trees[i][0] for i in range(0, self.map.canopy.count, step)]
            ty = [self.map.canopy.trees[i][1] for i in range(0, self.map.canopy.count, step)]
            ax.scatter(tx, ty, s=6, c=[(0.25, 0.45, 0.25)], alpha=0.5, zorder=1,
                       label=f"canopy ({self.map.canopy.count} trees, 1 in {step} drawn)")
        for p in anchors:
            ax.plot([p[0]], [p[1]], marker="s", ms=8, mfc="none", mec="tab:blue", mew=1.4, zorder=5)
            ax.annotate("relay", (p[0], p[1]), textcoords="offset points", xytext=(7, 5),
                        fontsize=7, color="tab:blue")
        for o in obstacles:
            fp = self._footprint(o)
            ax.fill([p[0] for p in fp], [p[1] for p in fp], color=o.color, alpha=0.8, ec="0.3", lw=0.5)
            ax.text(o.cx, o.cy, f"{o.id}\n{o.height:.0f} m", ha="center", va="center", fontsize=7)
        th = [2 * math.pi * k / 90 for k in range(91)]
        ax.plot([gx + 100 * math.cos(a) for a in th], [gy + 100 * math.sin(a) for a in th], color="0.5", ls="--",
                lw=0.8, label="100 m radio range (GCS)")
        ax.plot([gx], [gy], "k^", ms=9)
        ax.text(gx + 2, gy + 3, "GCS", fontsize=8)
        for u, track in self.track.items():
            first = True
            for up, pts in segments(track):
                ax.plot([p[1] for p in pts], [p[2] for p in pts], color=colors[u], lw=1.6 if up else 1.0,
                        ls="-" if up else ":", label=u if first else None)
                first = False
            if u in failures and track:
                ax.plot([track[-1][1]], [track[-1][2]], "kX", ms=11)
        for pid, v in pois.items():
            x, y = v["true_pos_m"][:2]
            ax.plot([x], [y], "m*" if v["hidden"] else "rs", ms=12 if v["hidden"] else 8)
            ax.text(x + 3, y + 3, pid, fontsize=8, color="darkred")
        for j in jams:
            ax.add_patch(plt.Circle(j["center_m"][:2], j["radius_m"], color="purple", fill=False, ls="--"))
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
        ax.set_title(title + " - top view", fontsize=10)
        path = os.path.join(self.run_dir, "map2d.png")
        fig.savefig(path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        out.append(path)
        return out

    def _finish(self, reason: str) -> None:
        self.done = True
        self._emit("RUN_END", reason=reason)
        m = self._metrics()
        if self._ev("DISTURBANCE"):
            m["disturbances"] = self._disturbance_report()
        criteria = self._criteria(m)
        ok = all(c["pass"] for c in criteria)
        summary = {"level": self.sc["level"], "name": self.sc["name"], "pass": ok, "end_reason": reason,
                   "map": self.sc.get("map"), "criteria": criteria, "metrics": m}
        try:
            summary["figures"] = self._plot(m, ok)
        except Exception as exc:  # a figure must never cost the verdict
            self.get_logger().error(f"figure failed: {type(exc).__name__}: {exc}")
        with open(os.path.join(self.run_dir, "summary.json"), "w", encoding="utf-8") as fh:
            json.dump(summary, fh, indent=2)
        lines = ["", "=" * 76, f" LEVEL {self.sc['level']} {self.sc['name']}: {'PASS' if ok else 'FAIL'}   ({reason})",
                 f" sim {m['sim_time_s']} s, wall {m['wall_time_s']} s, real-time factor {m['real_time_factor']}",
                 "=" * 76]
        for c in criteria:
            lines.append(f"  {'PASS' if c['pass'] else 'FAIL'}  {c['check']:<46} {c['value']}")
        for r in m.get("disturbances", []):
            extra = {k: v for k, v in r.items() if k not in ("t", "type", "target", "hidden", "handled", "response")
                     and v is not None}
            lines.append(f"  {'ok ' if r['handled'] else 'NOT'} {'hidden ' if r['hidden'] else 'planned'} "
                         f"{r['type']:<12} -> {r['target'] or '-':<6} {json.dumps(extra)[:110]}")
        lines.append("=" * 76)
        self.get_logger().info("\n".join(lines))
        self.ev_fh.close()
        self.ts_fh.close()
        self.exit_code = 0 if ok else 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    code = node.exit_code if node.exit_code is not None else 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
