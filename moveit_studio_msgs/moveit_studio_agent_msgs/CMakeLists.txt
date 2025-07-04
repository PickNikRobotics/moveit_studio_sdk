cmake_minimum_required(VERSION 3.22)
project(moveit_studio_agent_msgs)

# Disable Wredundant-decls warnings since rosidl generates redundant function declarations
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-redundant-decls)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_studio_sdk_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/AgentInfo.msg"
  "msg/ClientInfo.msg"
  "msg/EndEffector.msg"
  "msg/Event.msg"
  "msg/FaultStatus.msg"
  "msg/Json.msg"
  "msg/LicenseInfo.msg"
  "msg/Log.msg"
  "msg/ObjectiveServerStatus.msg"
  "msg/PlanningGroup.msg"
  "msg/RobotJointState.msg"
  "msg/Waypoint.msg"
)

set(srv_files
  "srv/AddTool.srv"
  "srv/AttachTool.srv"
  "srv/DetachOrRemoveTool.srv"
  "srv/ClearStoredParameters.srv"
  "srv/EditWaypoints.srv"
  "srv/GetAgentInfo.srv"
  "srv/GetClosestFrame.srv"
  "srv/GetPlanningGroups.srv"
  "srv/GetWaypoints.srv"
  "srv/RetrieveBehaviorParameter.srv"
  "srv/RetrieveJointState.srv"
  "srv/RetrievePose.srv"
  "srv/SavePose.srv"
  "srv/SendPointCloud2.srv"
  "srv/SetStringArray.srv"
  "srv/SetTransform.srv"
  "srv/StoreJointState.srv"
  "srv/StorePose.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    moveit_msgs
    moveit_studio_sdk_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
