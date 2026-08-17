cmake_minimum_required(VERSION 3.22)
project(moveit_studio_msgs)

# Disable Wredundant-decls warnings since rosidl generates redundant function
# declarations
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-redundant-decls)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
    "msg/CartesianConstraint.msg"
    "msg/JointRange.msg"
    "msg/JointRangeConstraint.msg"
    "msg/LinkOrientationConstraint.msg"
    "msg/OrientationConstraint.msg"
    "msg/ProRRTOptimizationParams.msg"
    "msg/TrajectoryTiming.msg")

rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
