"""Публикация времени физического решателя без замены часов ROS."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
header = root/'include/stonefish_ros2/ROS2SimulationManager.h'
source = root/'src/stonefish_ros2/ROS2SimulationManager.cpp'
h = header.read_text()
s = source.read_text()
anchor = '    std::shared_ptr<ROS2Interface> interface_;'
if anchor not in h:
    raise RuntimeError('Review upstream interface declaration')
h = h.replace(anchor, anchor+'\n    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr physicsTimePub_;')
anchor = '    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);'
if s.count(anchor) != 1:
    raise RuntimeError('Review upstream constructor')
s = s.replace(anchor, anchor+'\n    physicsTimePub_ = nh_->create_publisher<std_msgs::msg::Float64>("/stonefish/physics_time", 10);')
anchor = '    (void)timeStep; // Suppress warning'
if s.count(anchor) != 1:
    raise RuntimeError('Review upstream step callback')
s = s.replace(anchor, anchor+'\n    std_msgs::msg::Float64 physicsTime;\n    physicsTime.data = getSimulationTime();\n    physicsTimePub_->publish(physicsTime);')
header.write_text(h)
source.write_text(s)
