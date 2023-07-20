cmake_minimum_required(VERSION 3.22)
project(moveit_studio_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_studio_sdk_msgs REQUIRED)
find_package(rclpy REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_msgs
  moveit_studio_sdk_msgs
)

ament_python_install_package(
  moveit_studio_py
  PACKAGE_DIR moveit_studio_py
)

install(PROGRAMS
  examples/start_blocking.py
  examples/start_stop_async.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
