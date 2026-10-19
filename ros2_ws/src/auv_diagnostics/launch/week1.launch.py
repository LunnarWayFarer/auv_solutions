from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    run_id = f"ros2-{uuid4().hex[:12]}"
    params = {
        "run_id": ParameterValue(LaunchConfiguration("run_id"), value_type=str),
        "vehicle_id": ParameterValue(LaunchConfiguration("vehicle_id"), value_type=str),
        "count": ParameterValue(LaunchConfiguration("count"), value_type=int),
    }
    nodes = [Node(package="auv_diagnostics", executable=executable, output="screen",
                  parameters=[params])
             for executable in ("synthetic_source", "telemetry_adapter", "diagnostic_stub")]
    recorder = Node(package="auv_diagnostics", executable="report_recorder", output="screen",
                    parameters=[params, {"output_dir": LaunchConfiguration("output_dir")}])

    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason="Demo process exited"))],
    )) for node in [*nodes, recorder]]
    return LaunchDescription([
        DeclareLaunchArgument("run_id", default_value=run_id),
        DeclareLaunchArgument("vehicle_id", default_value="auv-01"),
        DeclareLaunchArgument("count", default_value="5"),
        DeclareLaunchArgument("output_dir", default_value=f"/tmp/{run_id}"),
        *handlers, *nodes, recorder,
        TimerAction(period=30.0, actions=[EmitEvent(event=Shutdown(reason="Demo deadline"))]),
    ])
