FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-usb-cam \
    ros-humble-cv-bridge \
    ros-humble-rqt-image-view \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/project

RUN source /opt/ros/humble/setup.bash && \
    cd /ws/src/project && \
    colcon build || true

CMD source /opt/ros/humble/setup.bash && \
    cd /ws/src/project && \
    source install/setup.bash && \
    ros2 launch camera_controller camera_turtle.launch.py
