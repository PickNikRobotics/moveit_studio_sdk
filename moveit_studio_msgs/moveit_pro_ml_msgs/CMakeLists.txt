cmake_minimum_required(VERSION 3.8)
project(moveit_pro_ml_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/GetActionChunk.srv"
                           DEPENDENCIES sensor_msgs std_msgs trajectory_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
