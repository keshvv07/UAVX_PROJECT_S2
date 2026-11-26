FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS=4
ARG GZ_REPOS_URL=https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos
ARG DDS_GEN_REF=v4.7.1
ENV GZ_VERSION=harmonic HOME=/home/uavx
ENV PATH=/home/uavx/.local/bin:/home/uavx/Micro-XRCE-DDS-Gen/scripts:${PATH}
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 MPLBACKEND=Agg
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo ca-certificates curl wget git build-essential cmake pkg-config \
    python3-pip python3-rosdep python3-vcstool python3-colcon-common-extensions \
    python3-numpy python3-opencv python3-yaml python3-matplotlib python3-pytest \
    python3-dev python3-setuptools python3-venv default-jdk \
    libgl1-mesa-dri libegl1 libglx-mesa0 mesa-utils xvfb xauth \
    ros-humble-topic-tools ros-humble-robot-state-publisher && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic && \
    (test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init) && \
    curl -fsSL https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -o /etc/ros/rosdep/sources.list.d/00-gazebo.list && \
    useradd -m -s /bin/bash uavx && echo 'uavx ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/uavx && chmod 0440 /etc/sudoers.d/uavx
USER uavx
WORKDIR /home/uavx/ardu_ws
RUN mkdir -p src && vcs import --recursive --input "$GZ_REPOS_URL" src
# Upstream SITL prerequisites, including Python/Waf dependencies.
RUN cd src/ardupilot && SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_EXT_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_ARM_ENV=1 \
    bash Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN python3 -m pip install --user MAVProxy pymavlink
RUN git clone --recurse-submodules --branch "$DDS_GEN_REF" https://github.com/ArduPilot/Micro-XRCE-DDS-Gen.git "$HOME/Micro-XRCE-DDS-Gen" && \
    cd "$HOME/Micro-XRCE-DDS-Gen" && ./gradlew assemble
RUN source /opt/ros/humble/setup.bash && rosdep update && \
    sudo apt-get update && rosdep install --from-paths src --ignore-src -y && \
    CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" MAKEFLAGS="-j$BUILD_JOBS" \
    colcon build --parallel-workers "$BUILD_JOBS" --packages-up-to ardupilot_gz_bringup ardupilot_gazebo ardupilot_sitl micro_ros_agent ros_gz_bridge ros_gz_sim \
    --event-handlers console_cohesion+
RUN vcs export --exact src > "$HOME/ardu_ws/upstream.lock.repos" && \
    printf '%s\n' 'source /opt/ros/humble/setup.bash' 'source "$HOME/ardu_ws/install/setup.bash"' \
    'export GZ_VERSION=harmonic' 'export PATH="$HOME/.local/bin:$HOME/Micro-XRCE-DDS-Gen/scripts:$PATH"' \
    'export SDF_PATH="${GZ_SIM_RESOURCE_PATH:-}:${SDF_PATH:-}"' > "$HOME/ardu_ws/env.sh"
WORKDIR /home/uavx/project
COPY --chown=uavx:uavx ros2/ ./ros2/
COPY --chown=uavx:uavx docker/entrypoint.sh /home/uavx/entrypoint.sh
RUN CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" MAKEFLAGS="-j$BUILD_JOBS" bash ros2/setup_ws.sh && \
    mkdir -p "$HOME/uavx_runs" && \
    printf '%s\n' 'source "$HOME/ardu_ws/env.sh"' 'source "$HOME/uavx_ws/install/setup.bash"' >> "$HOME/.bashrc"
USER root
RUN rm -f /etc/sudoers.d/uavx && rm -rf /var/lib/apt/lists/*
USER uavx
EXPOSE 8089
ENTRYPOINT ["/bin/bash", "/home/uavx/entrypoint.sh"]
CMD ["preflight", "level0"]
