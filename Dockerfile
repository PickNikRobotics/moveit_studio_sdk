from osrf/ros:humble-desktop-full

# use cyclone DDS (default is fast DDS)
# hadolint ignore=DL3008
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI /root/.ros/cyclonedds.xml
COPY .docker/cyclonedds.xml $CYCLONEDDS_URI

# set up the MoveIt Studio SDK
ENV STUDIO_SDK_WS /opt/moveit_studio_sdk_ws
WORKDIR ${STUDIO_SDK_WS}/src
# TODO(adlarkin) instead of copying individual folders, clone the repo once it's public?
COPY moveit_studio_sdk_msgs/ moveit_studio_sdk_msgs/
COPY moveit_studio_py/ moveit_studio_py/
WORKDIR $STUDIO_SDK_WS
RUN apt-get update && apt-get upgrade -y && rosdep install --from-paths src -y --ignore-src && \
    rm -rf /var/lib/apt/lists/*
# hadolint ignore=SC1091
RUN . /opt/ros/humble/setup.sh && colcon build

# hadolint ignore=SC2016
RUN echo 'source "${STUDIO_SDK_WS}/install/setup.bash"' >> ~/.bashrc

ENV SHELL /bin/bash
CMD ["bash"]
