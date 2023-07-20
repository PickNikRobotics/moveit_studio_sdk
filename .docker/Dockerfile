from osrf/ros:humble-desktop-full

# use cyclone DDS (default is fast DDS)
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
ENV CYCLONEDDS_URI="/root/.ros/cyclonedds.xml"
COPY ./cyclonedds.xml $CYCLONEDDS_URI

# this is a workaround to the base image's entrypoint that sources ros (see https://github.com/osrf/docker_images/issues/114)
RUN echo 'source "/opt/ros/humble/setup.bash"' >> ~/.bashrc

WORKDIR /opt/sdk_ws
ENV SHELL /bin/bash
CMD ["bash"]
