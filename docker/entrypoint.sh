#!/usr/bin/env bash
set -eo pipefail
source "$HOME/ardu_ws/env.sh"
source "$HOME/uavx_ws/install/setup.bash"
export SDF_PATH="${GZ_SIM_RESOURCE_PATH:-}:${SDF_PATH:-}"
cd "$HOME/project"
case "${1:-preflight}" in
  preflight)
    shift
    exec bash ros2/preflight.sh "${1:-level0}"
    ;;
  test)
    shift
    cd ros2/src/uavx_swarm
    exec python3 -m pytest -q test "$@"
    ;;
  run)
    shift
    if [[ $# -lt 1 ]]; then echo "usage: run <0-5|gc1|demo3d> [--seed N]" >&2; exit 2; fi
    # Keep the repository's logging, cleanup and referee exit code.
    # Xvfb provides an X display for camera rendering even on a server.
    exec xvfb-run -a -s "-screen 0 1280x720x24" bash ros2/run_level.sh "$@" --no-pov
    ;;
  *)
    exec "$@"
    ;;
esac
