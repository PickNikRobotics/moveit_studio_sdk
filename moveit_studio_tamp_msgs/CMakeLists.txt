cmake_minimum_required(VERSION 3.22)
project(moveit_studio_tamp_msgs)

# Disable Wredundant-decls warnings since rosidl generates redundant function declarations
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-redundant-decls)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_studio_vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

set(srv_files
  "srv/GetGraspableObject.srv"
  "srv/SetGraspableObject.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_files}
  DEPENDENCIES
    moveit_studio_vision_msgs
    std_srvs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
