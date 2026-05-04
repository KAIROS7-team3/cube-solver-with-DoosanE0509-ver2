from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    service_timeout_arg = DeclareLaunchArgument(
        "service_timeout_sec", default_value="10.0"
    )
    action_timeout_arg = DeclareLaunchArgument(
        "action_timeout_sec", default_value="120.0"
    )
    perceive_action_timeout_arg = DeclareLaunchArgument(
        "perceive_action_timeout_sec", default_value="30.0"
    )

    orchestrator_node = Node(
        package="cube_orchestrator",
        executable="master_orchestrator_node",
        name="master_orchestrator_node",
        output="screen",
        parameters=[
            {
                "service_timeout_sec": LaunchConfiguration("service_timeout_sec"),
                "action_timeout_sec": LaunchConfiguration("action_timeout_sec"),
                "perceive_action_timeout_sec": LaunchConfiguration(
                    "perceive_action_timeout_sec"
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            service_timeout_arg,
            action_timeout_arg,
            perceive_action_timeout_arg,
            orchestrator_node,
        ]
    )
