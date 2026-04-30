cmake_minimum_required(VERSION 3.8)
project(rh_p12_rna_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 의존성 ----
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rh_p12_rna_controller_interfaces REQUIRED)

# ---- Action 인터페이스 빌드 (.action → Python/C++ 코드 생성) ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GripperCommand.action"
  DEPENDENCIES std_msgs builtin_interfaces
)

# ---- Python 실행 파일 설치 ----
install(PROGRAMS
  rh_p12_rna_controller/gripper_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_node
)

install(PROGRAMS
  rh_p12_rna_controller/gripper_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_service_node
)

# Install python modules into the same site-packages package directory as
# rosidl-generated `rh_p12_rna_controller.action` so imports are consistent.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY rh_p12_rna_controller/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
