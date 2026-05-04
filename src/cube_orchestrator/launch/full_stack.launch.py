"""Full-stack launch for the cube solver.

Brings up cube_perception, cube_robot_action and cube_orchestrator with the
namespace remaps the orchestrator expects:
  /cube_perception/{detect_cube_pose, extract_face, get_cube_state}
  /robot/{pickup_cube, place_on_jig, rotate_cube_for_face, execute_solve_token, go_home}

Optional flags (all default true):
  use_dsr        — launch dsr_bringup2 (Doosan robot driver)
  use_gripper    — launch rh_p12_rna_controller gripper_node
  use_realsense  — launch realsense2_camera

Example (skip robot driver if already running separately):
  ros2 launch cube_orchestrator full_stack.launch.py use_dsr:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_dsr = LaunchConfiguration("use_dsr")
    use_gripper = LaunchConfiguration("use_gripper")
    use_realsense = LaunchConfiguration("use_realsense")
    dsr_host = LaunchConfiguration("dsr_host")
    dsr_port = LaunchConfiguration("dsr_port")
    color_image_topic = LaunchConfiguration("color_image_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    depth_unit_scale = LaunchConfiguration("depth_unit_scale")
    depth_sample_radius_px = LaunchConfiguration("depth_sample_radius_px")
    service_timeout_sec = LaunchConfiguration("service_timeout_sec")
    action_timeout_sec = LaunchConfiguration("action_timeout_sec")
    perceive_action_timeout_sec = LaunchConfiguration("perceive_action_timeout_sec")

    # ① Doosan 드라이버 (doosan-robot2 저장소 별도 클론 필요)
    dsr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dsr_bringup2"),
                "launch",
                "dsr_bringup2_rviz.launch.py",
            )
        ),
        condition=IfCondition(use_dsr),
        launch_arguments={
            "mode": "real",
            "model": "e0509",
            "host": dsr_host,
            "port": dsr_port,
        }.items(),
    )

    # ② RH-P12-RN(A) 그리퍼 노드 → /gripper/safe_grasp 제공
    # gripper_service_node = GripperNode 상속 → SafeGrasp + GripperCommand 모두 제공
    # robot_ip: Doosan 컨트롤러 IP — dsr_host와 동일해야 함
    gripper_node = Node(
        package="rh_p12_rna_controller",
        executable="gripper_service_node",
        name="gripper_service_node",
        output="screen",
        condition=IfCondition(use_gripper),
        parameters=[{"robot_ip": dsr_host}],
    )

    # ③ RealSense D455 카메라
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
        name="vla_detection_node",
        output="screen",
        parameters=[
            {
                "color_image_topic": color_image_topic,
                "depth_image_topic": depth_image_topic,
                "camera_info_topic": camera_info_topic,
                "depth_unit_scale": depth_unit_scale,
                "depth_sample_radius_px": depth_sample_radius_px,
            }
        ],
        remappings=[
            ("detect_cube_pose", "/cube_perception/detect_cube_pose"),
        ],
    )

    color_node = Node(
        package="cube_perception",
        executable="color_extraction_node",
        name="color_extraction_node",
        output="screen",
        parameters=[{"color_image_topic": color_image_topic}],
        remappings=[
            ("~/extract_face", "/cube_perception/extract_face"),
            ("~/get_cube_state", "/cube_perception/get_cube_state"),
        ],
    )

    robot_action_node = Node(
        package="cube_robot_action",
        executable="robot_action_server_node",
        name="robot_action_server_node",
        output="screen",
        remappings=[
            ("cube_robot_action/pickup_cube", "/robot/pickup_cube"),
            ("cube_robot_action/place_on_jig", "/robot/place_on_jig"),
            ("cube_robot_action/rotate_cube_for_face", "/robot/rotate_cube_for_face"),
            ("cube_robot_action/execute_solve_token", "/robot/execute_solve_token"),
            ("cube_robot_action/go_home", "/robot/go_home"),
        ],
    )

    orchestrator_node = Node(
        package="cube_orchestrator",
        executable="master_orchestrator_node",
        name="master_orchestrator_node",
        output="screen",
        parameters=[
            {
                "service_timeout_sec": service_timeout_sec,
                "action_timeout_sec": action_timeout_sec,
                "perceive_action_timeout_sec": perceive_action_timeout_sec,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_dsr", default_value="true"),
            DeclareLaunchArgument("use_gripper", default_value="true"),
            DeclareLaunchArgument("use_realsense", default_value="true"),
            DeclareLaunchArgument("dsr_host", default_value="192.168.137.100"),
            DeclareLaunchArgument("dsr_port", default_value="12345"),
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
            DeclareLaunchArgument("depth_unit_scale", default_value="0.001"),
            DeclareLaunchArgument("depth_sample_radius_px", default_value="2"),
            DeclareLaunchArgument("service_timeout_sec", default_value="10.0"),
            DeclareLaunchArgument("action_timeout_sec", default_value="120.0"),
            DeclareLaunchArgument("perceive_action_timeout_sec", default_value="30.0"),
            dsr_launch,
            gripper_node,
            realsense_launch,
            vla_node,
            color_node,
            robot_action_node,
            orchestrator_node,
        ]
    )
