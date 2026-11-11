FROM docker.io/library/ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=kilted \
    ROS_PYTHON_VERSION=3

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential ca-certificates cmake curl git ninja-build pkg-config \
       python3 python3-pip python3-venv wget libompl-dev libtaskflow-cpp-dev

RUN python3 -m pip install --break-system-packages -q \
       colcon-common-extensions vcstool rosdep

WORKDIR /ws

# Dependencies come first so that changes to this repository do not rebuild them.
COPY dependencies.repos /ws/dependencies.repos
RUN mkdir -p src && vcs import --recursive --input dependencies.repos src

# ompl, octomap, fcl and taskflow are installed above. ros-${ROS_DISTRO}-ompl is
# not available without the ROS apt repository, and the rest have no rosdep key.
RUN rosdep init \
    && rosdep update --rosdistro=$ROS_DISTRO \
    && apt-get update \
    && apt-get install -y --no-install-recommends liboctomap-dev libfcl-dev \
    && rosdep install --rosdistro=$ROS_DISTRO --from-paths src --ignore-src -r -y \
       --skip-keys "ompl octomap fcl taskflow osqp-eigen"

ENV COLCON_ARGS="--base-paths src --merge-install --packages-skip-build-finished --event-handlers console_direct- console_cohesion+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DTESSERACT_ENABLE_TESTING=OFF -DTESSERACT_ENABLE_EXAMPLES=OFF"

# Split so that a failure late in the build keeps the earlier layers.
RUN colcon build --packages-up-to robotraconteur_companion drekar_launch_process_cpp $COLCON_ARGS

RUN colcon build --packages-up-to tesseract $COLCON_ARGS

RUN colcon build --packages-up-to tesseract_planning $COLCON_ARGS && rm -rf build log

# This repository
COPY package.xml /ws/src/tesseract_robotraconteur/package.xml
RUN rosdep install --rosdistro=$ROS_DISTRO --from-paths src --ignore-src -r -y \
       --skip-keys "ompl octomap fcl taskflow osqp-eigen" \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/tesseract_robotraconteur
RUN colcon build --packages-select tesseract_robotraconteur $COLCON_ARGS \
    && rm -rf build log

COPY tools/docker/entrypoint.sh /entrypoint.sh

ENV URDF_FILE=/ws/install/share/tesseract/support/urdf/abb_irb2400.urdf \
    SRDF_FILE=/ws/install/share/tesseract/support/urdf/abb_irb2400.srdf \
    TASK_PLUGIN_CONFIG_FILE=/ws/install/share/tesseract_planning/task_composer/config/task_composer_plugins.yaml \
    TESSERACT_RESOURCE_PATH=/ws/install/share

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-c", "exec tesseract_robotraconteur_service --urdf-file=\"$URDF_FILE\" --srdf-file=\"$SRDF_FILE\" --task-plugin-config-file=\"$TASK_PLUGIN_CONFIG_FILE\""]
