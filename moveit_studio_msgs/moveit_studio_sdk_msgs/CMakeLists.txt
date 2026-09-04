cmake_minimum_required(VERSION 3.22)
project(moveit_studio_sdk_msgs)

# Disable Wredundant-decls warnings since rosidl generates redundant function
# declarations
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-redundant-decls)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
    "msg/BehaviorParameterDescription.msg" "msg/BehaviorParameter.msg"
    "msg/RequestStatus.msg" "msg/TeleoperationMode.msg" "msg/TreeUids.msg")

set(SRV_FILES "srv/CancelObjective.srv" "srv/ExecuteObjective.srv"
              "srv/StampTreeUids.srv" "srv/VerifyXML.srv")

set(ACTION_FILES "action/DoObjectiveSequence.action"
                 "action/DoTeleoperate.action")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
  action_msgs
  builtin_interfaces
  geometry_msgs
  moveit_msgs
  sensor_msgs
  std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
