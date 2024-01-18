cmake_minimum_required(VERSION 3.22)
project(moveit_pro_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_pro_sdk_msgs REQUIRED)
find_package(rclpy REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_msgs
  moveit_pro_sdk_msgs
)

ament_python_install_package(
  moveit_pro_py
  PACKAGE_DIR moveit_pro_py
)

install(PROGRAMS
  examples/start_blocking.py
  examples/start_stop_async.py
  examples/move_to_waypoint.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
