FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

WORKDIR /ws

COPY . /ws/src/tesseract_robotraconteur

RUN apt update \
    && apt upgrade -y \
    && apt install -y cmake curl git python3 python3-distutils python3-pip software-properties-common bash \
    && python3 -m pip install vcstool -q \
    && python3 -m pip install colcon-common-extensions -q

SHELL ["/bin/bash", "-c"]

RUN add-apt-repository universe -y

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt-get update

RUN python3 -m pip install rosdep -q \
    && rosdep init \
    && rosdep update

RUN cd /ws/src \
    && vcs import --input tesseract_robotraconteur/dependencies_with_ext.repos --recursive

RUN rosdep install --from-paths src --ignore-src -y

RUN source /opt/ros/noetic/setup.bash \
    && colcon build --merge-install  \
      --event-handlers desktop_notification- status- terminal_title- console_cohesion+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ./tools/docker/entrypoint.sh /entrypoint.sh

RUN rm -rf src && rm -rf build && rm -rf log
RUN rm -rf /var/lib/apt/lists/*
RUN apt-get clean

ENV URDF_FILE=/ws/install/share/tesseract_support/urdf/abb_irb2400.urdf
ENV SRDF_FILE=/ws/install/share/tesseract_support/urdf/abb_irb2400.srdf
ENV TASK_PLUGIN_CONFIG_FILE=/ws/install/share/tesseract_task_composer/config/task_composer_plugins.yaml
ENV TESSERACT_RESOURCE_PATH=/ws/install/share

ENTRYPOINT ["/entrypoint.sh"]
CMD exec tesseract_robotraconteur_service --urdf-file=$URDF_FILE --srdf-file=$SRDF_FILE --task-plugin-config-file=$TASK_PLUGIN_CONFIG_FILE
