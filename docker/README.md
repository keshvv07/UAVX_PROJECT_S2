# UAV-X Stage 2 Docker (initial integration)
 
This configuration targets linux/amd64, ROS 2 Humble on Ubuntu Jammy, Gazebo Harmonic,
ArduPilot SITL and the existing UAV-X workspace. No mission algorithms or launch
files are modified. The first build needs internet and substantial disk and RAM.
Docker Engine/Desktop and Compose v2 are required on the host.

From the repository root:

    docker compose build
    docker compose run --rm sim preflight level0
    docker compose run --rm sim test
    docker compose run --rm sim run 0
    docker compose run --rm sim run 1
    docker compose run --rm --service-ports sim run 5
    docker compose run --rm --service-ports sim run gc1 --seed 42

Proceed only after the smaller levels pass. A run uses the existing referee's
summary.json and exit status. The control console is at http://localhost:8089
when a scenario includes it. Only one service using that published port can run
at once. On a remote Linux server, forward the loopback port using
ssh -L 8089:127.0.0.1:8089 user@server from your laptop.

The default is software rendering via Xvfb (no physical display required).
Camera processing remains active; the separate POV window is disabled.
Software rendering may be slow, especially for five vehicles. GPU passthrough
and interactive Gazebo GUI are not validated or configured by this initial version.
Do not use this configuration for flight hardware.

Logs are retained in the Docker named volume 'runs'. Inspect or export them:

    docker compose run --rm sim bash -c 'ls -lt ~/uavx_runs'
    docker compose run --rm -T sim tar -C /home/uavx/uavx_runs -czf - . > uavx_runs.tar.gz

The export command is for a Bash-compatible host shell. Do not run compose down -v
unless you intend to erase results. To inspect the build environment:

    docker compose run --rm sim bash
    docker compose run --rm sim cat /home/uavx/ardu_ws/upstream.lock.repos

After editing source files, rebuild the image. The upstream manifest is currently
branch-based, not a validated immutable lock. The build records the resolved
commits in upstream.lock.repos. Once a full build and Level 0 pass, retain that
manifest and the image digest for reproducible builds. The image is not yet
built or runtime-tested by the author of this configuration.

For constrained machines use BUILD_JOBS=2 docker compose build (Bash).
Suggested starting host resources for five simulated vehicles: 8+ CPU cores,
16–32 GB RAM and 50+ GB free disk; actual performance depends on hardware.
