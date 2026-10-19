FROM ros:humble-ros-base-jammy@sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN python3 -c 'import pytest' && command -v colcon
WORKDIR /workspace
COPY ros2_ws/src /workspace/ros2_ws/src
COPY scripts /workspace/scripts
RUN source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build
COPY docker/entrypoint.sh /project-entrypoint.sh
ENTRYPOINT ["bash", "/project-entrypoint.sh"]
CMD ["python3", "/workspace/scripts/check_ros_demo.py", "--output-root", "/artifacts"]
