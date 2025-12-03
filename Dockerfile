FROM osrf/ros:humble-desktop-full

# Use Cyclone DDS (default is Fast DDS)
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp

# Set up the MoveIt Pro SDK workspace
ENV STUDIO_SDK_WS /opt/moveit_studio_sdk_ws
WORKDIR $STUDIO_SDK_WS
COPY moveit_studio_msgs/ src/moveit_studio_msgs/
COPY moveit_studio_py/ src/moveit_studio_py/
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    rosdep install --from-paths src -y --ignore-src

# hadolint ignore=SC1091
RUN . /opt/ros/humble/setup.sh && colcon build

ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI /root/.ros/cyclonedds.xml
COPY .docker/cyclonedds.xml $CYCLONEDDS_URI

# hadolint ignore=SC2016
RUN echo 'source "${STUDIO_SDK_WS}/install/setup.bash"' >> ~/.bashrc

ENV SHELL /bin/bash
CMD ["bash"]
