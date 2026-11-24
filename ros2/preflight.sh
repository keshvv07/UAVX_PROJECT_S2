#!/usr/bin/env bash
# Is this machine ready to fly a UAV-X scenario in Gazebo?  Checks everything that can be
# checked before the simulator starts, so a missing package or a busy port is reported in
# two seconds instead of as a hang three minutes into a run.
#
#   bash ros2/preflight.sh            # the Grand Challenge scenario, gc1
#   bash ros2/preflight.sh level5     # any scenario name or .yaml path
#
# Exit status 0 = ready, 1 = at least one FAIL. WARN lines never stop a run.
# run_level.sh calls this automatically; UAVX_SKIP_PREFLIGHT=1 turns that off.
set -uo pipefail

REPO_ROS2="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENARIO="${1:-gc1}"
WS="${UAVX_WS:-$HOME/uavx_ws}"
ARDU_ENV="${ARDU_ENV:-$HOME/ardu_ws/env.sh}"
FAIL=0

ok()   { printf '  PASS  %s\n' "$1"; }
warn() { printf '  WARN  %s\n' "$1"; }
bad()  { printf '  FAIL  %s\n' "$1"; FAIL=1; }

echo "UAV-X preflight for '$SCENARIO'"

echo "[1] environment"
if [[ -f "$ARDU_ENV" ]]; then ok "ArduPilot/Gazebo/ROS 2 environment: $ARDU_ENV"
else bad "$ARDU_ENV not found - build ~/ardu_ws first (see ros2/README.md)"; fi
set +u
# shellcheck disable=SC1090
[[ -f "$ARDU_ENV" ]] && source "$ARDU_ENV"
if [[ -f "$WS/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"; ok "workspace built: $WS"
else bad "$WS/install/setup.bash missing - run: bash ros2/setup_ws.sh"; fi
set -u

echo "[2] programs"
for cmd in ros2 gz arducopter micro_ros_agent mavproxy.py; do
  if command -v "$cmd" >/dev/null 2>&1; then ok "$cmd"; else bad "$cmd not on PATH"; fi
done
if command -v ros2 >/dev/null 2>&1; then
  PKGS="$(ros2 pkg list 2>/dev/null)"
  for pkg in ardupilot_gz_bringup ardupilot_sitl ros_gz_sim ros_gz_bridge topic_tools \
             uavx_interfaces uavx_gazebo uavx_swarm; do
    if grep -qx "$pkg" <<<"$PKGS"; then ok "ROS package $pkg"; else bad "ROS package $pkg not found"; fi
  done
fi

echo "[3] scenario, map and world"
RES="$(python3 - "$SCENARIO" "$REPO_ROS2" <<'PY'
import os, sys, tempfile
scenario, repo = sys.argv[1], sys.argv[2]
sys.path.insert(0, os.path.join(repo, "src", "uavx_swarm"))
out = []
try:
    from uavx_swarm.common import load_scenario, resolve_scenario
    from uavx_swarm.world_map import WorldMap
    sc = load_scenario(resolve_scenario(scenario))
    out.append(("PASS", f"scenario '{sc['name']}': {len(sc['uavs'])} UAVs, components {', '.join(sc['components'])}"))
    ids = [u["id"] for u in sc["uavs"]]
    inst = [u["instance"] for u in sc["uavs"]]
    sysid = [u["sysid"] for u in sc["uavs"]]
    if len(set(ids)) == len(ids) and len(set(inst)) == len(inst) and len(set(sysid)) == len(sysid):
        out.append(("PASS", "UAV ids, SITL instances and system ids are all distinct"))
    else:
        out.append(("FAIL", "two UAVs share an id, a SITL instance or a system id"))
    if 0 not in inst:
        out.append(("FAIL", "no UAV on SITL instance 0 - the shared micro-ROS agent starts with instance 0"))
    wm = None
    if sc.get("map"):
        wm = WorldMap.load(sc["map"])
        out.append(("PASS", f"map '{sc['map']}': {len(wm.obstacles)} obstacles, {len(wm.canopy.trees)} trees"))
        bad_pads = [u["id"] for u in sc["uavs"]
                    if wm.blocked(u["pad_m"][0], u["pad_m"][1], 0.5, 1.0)
                    or not wm.inside_fence(u["pad_m"][0], u["pad_m"][1], 0.5)]
        if bad_pads:
            out.append(("FAIL", f"pad of {', '.join(bad_pads)} is inside an obstacle or outside the geofence"))
        else:
            out.append(("PASS", "every UAV pad is clear of obstacles and inside the geofence"))
    base = os.path.join(repo, "src", "uavx_gazebo", "worlds", f"{sc['world']}.sdf")
    if not os.path.exists(base):
        out.append(("FAIL", f"world file {base} missing"))
    else:
        path = os.path.join(tempfile.gettempdir(), f"uavx_preflight_{sc['world']}.sdf")
        if wm is not None:
            wm.write_world(base, path)
        else:
            path = base
        import xml.dom.minidom as md
        doc = md.parse(path)
        name = doc.getElementsByTagName("world")[0].getAttribute("name")
        if name != sc["world"]:
            out.append(("FAIL", f"world file is named '{name}' but the scenario says '{sc['world']}'"))
        else:
            out.append(("PASS", f"world '{name}' generates and parses ({os.path.getsize(path) // 1024} KiB)"))
        print("WORLD=" + path)
except Exception as exc:                                        # noqa: BLE001
    out.append(("FAIL", f"{type(exc).__name__}: {exc}"))
for level, msg in out:
    print(f"{level}|{msg}")
PY
)"
WORLD_FILE=""
while IFS= read -r line; do
  case "$line" in
    PASS\|*) ok "${line#PASS|}" ;;
    WARN\|*) warn "${line#WARN|}" ;;
    FAIL\|*) bad "${line#FAIL|}" ;;
    WORLD=*) WORLD_FILE="${line#WORLD=}" ;;
  esac
done <<<"$RES"
if [[ -n "$WORLD_FILE" ]] && command -v gz >/dev/null 2>&1; then
  if gz sdf -k "$WORLD_FILE" >/tmp/uavx_sdf_check.log 2>&1; then ok "gz sdf accepts the world file"
  else bad "gz sdf rejects the world file - see /tmp/uavx_sdf_check.log"; fi
fi
if command -v ros2 >/dev/null 2>&1; then
  SHARE="$(ros2 pkg prefix uavx_gazebo 2>/dev/null)/share/uavx_gazebo"
  for f in worlds maps models/uavx_quad/model.sdf models/poi_marker/model.sdf config/uavx_quad.parm tools; do
    [[ -e "$SHARE/$f" ]] && ok "installed: uavx_gazebo/$f" || bad "uavx_gazebo/$f not installed - rerun bash ros2/setup_ws.sh"
  done
  case ":${GZ_SIM_RESOURCE_PATH:-}:" in
    *uavx_gazebo*) ok "GZ_SIM_RESOURCE_PATH includes uavx_gazebo" ;;
    *) bad "GZ_SIM_RESOURCE_PATH does not include uavx_gazebo (source $WS/install/setup.bash)" ;;
  esac
fi

echo "[4] ports"
python3 - "$SCENARIO" "$REPO_ROS2" <<'PY'
import os, socket, sys
sys.path.insert(0, os.path.join(sys.argv[2], "src", "uavx_swarm"))
try:
    from uavx_swarm.common import load_scenario, resolve_scenario
    sc = load_scenario(resolve_scenario(sys.argv[1]))
except Exception:
    sys.exit(0)
checks = []
for u in sc["uavs"]:
    i = int(u["instance"])
    checks += [("tcp", 5760 + 10 * i, f"{u['id']} SITL console"), ("udp", 9002 + 10 * i, f"{u['id']} JSON FDM")]
checks.append(("udp", 2019, "micro-ROS agent"))
if "control_panel" in sc["components"]:
    checks.append(("tcp", int(sc.get("panel_port", 8089)), "control panel"))
busy = []
for proto, port, label in checks:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM if proto == "tcp" else socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", port))
    except OSError:
        busy.append(f"{proto}/{port} ({label})")
    finally:
        s.close()
if busy:
    print("  FAIL  port(s) in use - an earlier run may still be alive (pkill -x arducopter; pkill -f 'gz sim'): " + ", ".join(busy))
    sys.exit(1)
print(f"  PASS  all {len(checks)} ports the run needs are free")
PY
[[ $? -ne 0 ]] && FAIL=1

echo "[5] machine"
CORES="$(nproc 2>/dev/null || echo 1)"
MEM_MB="$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo 2>/dev/null || echo 0)"
FREE_MB="$(df -Pm "$HOME" 2>/dev/null | awk 'NR==2 {print $4}')"
(( CORES >= 4 )) && ok "$CORES CPU cores" || warn "$CORES CPU cores - five SITL instances plus Gazebo want 6 or more; expect a low real-time factor"
(( MEM_MB >= 6000 )) && ok "${MEM_MB} MB memory available" || warn "${MEM_MB} MB memory available - under 6 GB is tight for five UAVs"
(( ${FREE_MB:-0} >= 2000 )) && ok "${FREE_MB} MB free disk" || warn "${FREE_MB:-?} MB free disk - each run writes images and logs"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  warn "no display: run without --gui, or launch with headless_rendering:=true"
fi

echo
if (( FAIL )); then echo "NOT READY - fix the FAIL lines above."; exit 1; fi
echo "READY - bash ros2/run_level.sh $SCENARIO"
