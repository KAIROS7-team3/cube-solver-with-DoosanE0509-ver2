cmake_minimum_required(VERSION 3.8)
project(cube_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # cube_robot_action actions
  "cube_robot_action/action/ExecuteSolveToken.action"
  "cube_robot_action/action/GoHome.action"
  "cube_robot_action/action/PickupCube.action"
  "cube_robot_action/action/PlaceOnJig.action"
  "cube_robot_action/action/RotateCubeForFace.action"
  # cube_orchestrator services
  "cube_orchestrator/srv/StartRun.srv"
  "cube_orchestrator/srv/StartScan.srv"
  "cube_orchestrator/srv/StartSolve.srv"
  # cube_perception services
  "cube_perception/srv/DetectCubePose.srv"
  "cube_perception/srv/ExtractFace.srv"
  "cube_perception/srv/GetCubeState.srv"
  # rh_p12_rna_controller interfaces
  "rh_p12_rna_controller/action/GripperCommand.action"
  "rh_p12_rna_controller/action/SafeGrasp.action"
  "rh_p12_rna_controller/msg/GripperState.msg"
  "rh_p12_rna_controller/srv/GetState.srv"
  "rh_p12_rna_controller/srv/SetPosition.srv"
  DEPENDENCIES geometry_msgs std_msgs action_msgs unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
