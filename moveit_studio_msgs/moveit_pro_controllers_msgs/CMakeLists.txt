cmake_minimum_required(VERSION 3.8)
project(moveit_pro_controllers_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/FollowJointTrajectoryWithAdmittance.action"
  "msg/AdmittanceParameters.msg"
  "msg/CartesianSelectionVector.msg"
  "msg/VelocityForceCommand.msg"
  "srv/ConfigureVelocityForceController.srv"
  DEPENDENCIES
  action_msgs
  builtin_interfaces
  control_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs)

ament_package()
