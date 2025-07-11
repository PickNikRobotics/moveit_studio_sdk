cmake_minimum_required(VERSION 3.22)
project(moveit_studio_vision_msgs)

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
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(msg_files
  "msg/GraspInfo.msg"
  "msg/GraspableFace.msg"
  "msg/GraspableObject.msg"
  "msg/Mask2D.msg"
  "msg/Mask3D.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectDetectionArray.msg"
  "msg/ObjectSubframe.msg"
  "msg/PointCloudPCD.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    moveit_msgs
    moveit_studio_sdk_msgs
    sensor_msgs
    shape_msgs
    std_msgs
    vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
