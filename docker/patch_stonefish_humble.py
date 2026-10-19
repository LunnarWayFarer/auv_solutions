from pathlib import Path
import sys

source = Path(sys.argv[1]) / "src/stonefish_ros2/ROS2Interface.cpp"
original = source.read_text()
before = "msg.header.stamp + rclcpp::Duration(0, abs(data[i*2+1]))"
after = "rclcpp::Time(msg.header.stamp) + rclcpp::Duration(0, abs(data[i*2+1]))"
if original.count(before) != 1:
    raise RuntimeError("Upstream changed: review Humble compatibility patch before applying")
source.write_text(original.replace(before, after))
