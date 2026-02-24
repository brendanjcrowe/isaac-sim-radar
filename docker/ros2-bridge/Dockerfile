FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-tf2-ros \
    ros-humble-slam-toolbox \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt scipy

COPY ros2_ws/ /app/ros2_ws/
COPY radar_analysis/ /app/radar_analysis/
COPY launch/ /app/launch/
COPY rviz/ /app/rviz/
COPY config/ /app/config/
COPY tests/ /app/tests/

WORKDIR /app/ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

COPY docker/ros2-bridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /app

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/app/launch/full_stack.launch.py"]
