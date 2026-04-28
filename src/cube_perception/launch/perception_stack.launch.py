from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    use_realsense = LaunchConfiguration("use_realsense")
    detector_backend = LaunchConfiguration("detector_backend")
    color_image_topic = LaunchConfiguration("color_image_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        condition=IfCondition(use_realsense),
        launch_arguments={"align_depth.enable": "true"}.items(),
    )

    vla_node = Node(
        package="cube_perception",
        executable="vla_detection_node",
        output="screen",
        parameters=[
            {
                "detector_backend": detector_backend,
                "color_image_topic": color_image_topic,
                "depth_image_topic": depth_image_topic,
                "camera_info_topic": camera_info_topic,
            }
        ],
    )

    color_node = Node(
        package="cube_perception",
        executable="color_extraction_node",
        output="screen",
        parameters=[{"color_image_topic": color_image_topic}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_realsense", default_value="true"),
            DeclareLaunchArgument("detector_backend", default_value="gemini"),
            DeclareLaunchArgument(
                "color_image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera/color/camera_info"
            ),
            realsense_launch,
            vla_node,
            color_node,
        ]
    )
