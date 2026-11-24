"""Shared helpers: scenario files, geodesy, QoS profiles and the event bus.

World frame: ENU metres (x East, y North, z Up). WHERE the origin is belongs to the map,
not to this file: `quake_town` puts it at the GCS, as the Stage 1 Python simulator does,
and the organisers' `forest_rescue` puts it at the centre of the 500 m x 500 m search area
with the GCS 365 m west. Either way the Gazebo world is pinned to a geographic origin in
its `<spherical_coordinates>`, and that same origin is what converts a UAV's GPS fix into
the world frame - and, more to the point, converts a position goal back into the latitude
and longitude ArduPilot is commanded with.

So the origin is NOT a constant. `load_scenario` sets it from the scenario's map (its
`frame:` block) or from an explicit `origin:` in the scenario, and falls back to CMAC for
the early levels that have neither. Every node calls `load_scenario` before it talks to an
autopilot, so one scenario always means one frame; getting this wrong does not degrade the
mission, it flies the swarm to the wrong continent.
"""
from __future__ import annotations

import copy
import json
import math
import os
from typing import Any

import yaml

try:
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    HAVE_RCLPY = True
except ImportError:          # no ROS 2 here: the scenario, map and geodesy helpers still load,
    HAVE_RCLPY = False       # which is what lets the offline test suite run with plain pytest

    class _Inert:            # stands in for the QoS enums and profiles; nothing reads them
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, name):
            return 0

    DurabilityPolicy = HistoryPolicy = ReliabilityPolicy = _Inert()
    QoSProfile = _Inert
    qos_profile_sensor_data = _Inert()

# ---------------------------------------------------------------- geodesy
DEFAULT_ORIGIN = {"latitude_deg": -35.363261,    # CMAC: SITL's default home, and the
                  "longitude_deg": 149.165230,   # origin uavx_disaster.sdf is pinned to
                  "elevation_m": 584.0}
EARTH_RADIUS_M = 6378137.0      # same scaling ArduPilot's Location uses

ORIGIN_LAT_DEG = DEFAULT_ORIGIN["latitude_deg"]
ORIGIN_LON_DEG = DEFAULT_ORIGIN["longitude_deg"]
ORIGIN_ALT_M = DEFAULT_ORIGIN["elevation_m"]
_COS_LAT0 = math.cos(math.radians(ORIGIN_LAT_DEG))


def set_origin(latitude_deg: float, longitude_deg: float, elevation_m: float | None = None) -> None:
    """Pin the ENU frame to a geographic origin. Must match the Gazebo world's own."""
    global ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_ALT_M, _COS_LAT0
    ORIGIN_LAT_DEG = float(latitude_deg)
    ORIGIN_LON_DEG = float(longitude_deg)
    if elevation_m is not None:
        ORIGIN_ALT_M = float(elevation_m)
    _COS_LAT0 = math.cos(math.radians(ORIGIN_LAT_DEG))


def origin() -> dict:
    return {"latitude_deg": ORIGIN_LAT_DEG, "longitude_deg": ORIGIN_LON_DEG,
            "elevation_m": ORIGIN_ALT_M}


def enu_to_latlon(east_m: float, north_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT_DEG + math.degrees(north_m / EARTH_RADIUS_M)
    lon = ORIGIN_LON_DEG + math.degrees(east_m / (EARTH_RADIUS_M * _COS_LAT0))
    return lat, lon


def latlon_to_enu(lat_deg: float, lon_deg: float) -> tuple[float, float]:
    north = math.radians(lat_deg - ORIGIN_LAT_DEG) * EARTH_RADIUS_M
    east = math.radians(lon_deg - ORIGIN_LON_DEG) * EARTH_RADIUS_M * _COS_LAT0
    return east, north


def yaw_from_quaternion(q) -> float:
    """ENU yaw (rad, 0 = East, counter-clockwise) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def dist2(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist3(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


# ---------------------------------------------------------------- QoS
SENSOR_QOS = qos_profile_sensor_data          # best effort: matches any publisher
COMMAND_QOS = QoSProfile(                     # reliable: accepted by any subscriber
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
RADIO_QOS = QoSProfile(                       # the emulator, not DDS, decides what is lost
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=200)
EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=500)
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ---------------------------------------------------------------- scenarios
_SCENARIO_DEFAULTS: dict[str, Any] = {
    "level": 0,
    "name": "",
    "description": "",
    "world": "uavx_disaster",
    "duration_s": 900.0,            # sim-time watchdog: the referee ends the run here
    "gcs": {"id": "gcs", "position_m": [0.0, 0.0, 10.0]},
    "uavs": [],
    "components": [],               # which uavx_swarm nodes the level runs
    "map": None,                    # uavx_gazebo/maps/<name>.yaml: obstacles, area, pads, canopy
    "mission": {"mode": "local", "waypoints_m": []},
    "pois": [],                     # fixed Points of Interest
    "targets": {},                  # or a generated PoI field (scenario_manager._draw_targets)
    "companion": {},
    "gcs_logic": {},
    "routing": {},                  # routing.RouteParams
    "relay": {},                    # relay_plan.RelayParams
    "radio": {},                    # overrides config/radio.yaml
    "radio_backend": "internal",    # radio_backend.py: "internal" | "ns3"
    "ap_params": {},                # extra ArduPilot parameters for this scenario
    "panel_port": 8089,             # control_panel.py
    "pass": {},
}

_UAV_DEFAULTS: dict[str, Any] = {
    "cruise_alt_m": 30.0,
    "color": [0.1, 0.6, 1.0],
    "camera": True,
}

_PAD_SLOTS = [(-25.0, -12.5), (-25.0, 12.5), (-25.0, -37.5),
              (-25.0, 37.5), (-25.0, -62.5), (-25.0, 62.5)]


def share_dir(package: str = "uavx_swarm") -> str:
    """The package's installed share directory, or its source tree when ROS 2 is absent.

    The fallback is what lets the whole offline test suite - maps, planners, radio, routing,
    scenarios - run with plain pytest on a machine with no ROS 2 install, which is where
    most of the checking actually happens.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package)
    except Exception:
        src = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        path = os.path.join(src, package)
        if os.path.isdir(path):
            return path
        raise


def resolve_scenario(name_or_path: str) -> str:
    """'level4' -> <share>/uavx_swarm/config/level4.yaml; a real path is returned as is."""
    if os.path.isfile(name_or_path):
        return os.path.abspath(name_or_path)
    name = name_or_path if name_or_path.endswith(".yaml") else f"{name_or_path}.yaml"
    path = os.path.join(share_dir(), "config", name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"scenario not found: {name_or_path} (looked for {path})")
    return path


def load_scenario(name_or_path: str) -> dict[str, Any]:
    with open(resolve_scenario(name_or_path), encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    sc = copy.deepcopy(_SCENARIO_DEFAULTS)
    for key, value in raw.items():
        if isinstance(value, dict) and isinstance(sc.get(key), dict):
            sc[key].update(value)
        else:
            sc[key] = value
    uavs = []
    for i, u in enumerate(sc["uavs"]):
        full = copy.deepcopy(_UAV_DEFAULTS)
        full.update(u)
        full.setdefault("id", f"uav{i + 1}")
        full.setdefault("sysid", i + 1)
        full.setdefault("instance", i)
        full.setdefault("pad_m", list(_PAD_SLOTS[i % len(_PAD_SLOTS)]))
        uavs.append(full)
    sc["uavs"] = uavs
    for i, p in enumerate(sc["pois"]):
        p.setdefault("id", f"P{i + 1}")
        p.setdefault("priority", 1)
        p.setdefault("appear_s", 0.0)
        p.setdefault("survey_s", 20.0)
    _apply_origin(sc)
    return sc


def _apply_origin(sc: dict) -> None:
    """Pin the ENU frame to this scenario's world (see the module docstring).

    Order: an explicit `origin:` in the scenario, then the map's `frame:` block, then CMAC.
    Done here because every node loads the scenario before it commands an autopilot, and a
    frame that disagrees with the Gazebo world sends position goals to the wrong place.
    """
    spec = sc.get("origin")
    if not spec and sc.get("map"):
        try:
            from .world_map import WorldMap
            spec = (WorldMap.load(sc["map"]).data.get("frame") or None)
        except Exception:                 # no map on disk (a unit test, a trimmed install)
            spec = None
    if not spec:
        return
    if "latitude_deg" in spec and "longitude_deg" in spec:
        set_origin(spec["latitude_deg"], spec["longitude_deg"], spec.get("elevation_m"))


def uav_by_id(scenario: dict, uav_id: str) -> dict:
    for u in scenario["uavs"]:
        if u["id"] == uav_id:
            return u
    raise KeyError(f"{uav_id} is not in the scenario")


# ---------------------------------------------------------------- node lifecycle
def spin(node, on_exit=None) -> None:
    """rclpy.spin that ends quietly when the launch shuts the run down (Ctrl-C / SIGINT)."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():           # a real error, not the context going away under the executor
            raise
    finally:
        if on_exit:
            on_exit()
        node.destroy_node()
        rclpy.try_shutdown()


# ---------------------------------------------------------------- time / events
def now_s(node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


class EventBus:
    """Publishes structured events on /uavx/events; the referee logs them all."""

    def __init__(self, node, source: str):
        from std_msgs.msg import String
        self._node = node
        self._source = source
        self._msg_type = String
        self._pub = node.create_publisher(String, "/uavx/events", EVENT_QOS)

    def emit(self, event_type: str, **fields: Any) -> None:
        body = {"t": round(now_s(self._node), 3), "src": self._source, "type": event_type}
        body.update(fields)
        msg = self._msg_type()
        msg.data = json.dumps(body, default=_json_default)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[{event_type}] " + json.dumps(fields, default=_json_default))


def _json_default(obj: Any) -> Any:
    if hasattr(obj, "tolist"):
        return obj.tolist()
    return str(obj)


def dumps(obj: Any) -> str:
    return json.dumps(obj, default=_json_default, separators=(",", ":"))
