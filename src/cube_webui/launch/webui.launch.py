from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument("host", default_value="0.0.0.0")
    port_arg = DeclareLaunchArgument("port", default_value="8080")
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic", default_value="/camera/camera/color/image_raw"
    )

    webui_node = Node(
        package="cube_webui",
        executable="webui_server",
        name="cube_webui",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "camera_topic": LaunchConfiguration("camera_topic"),
            }
        ],
    )

    return LaunchDescription([host_arg, port_arg, camera_topic_arg, webui_node])
