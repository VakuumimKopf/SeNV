cmake_minimum_required(VERSION 3.8)
project(senv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ConTask.action"
  "msg/Laser.msg"
  "msg/Pic.msg"
  "msg/Lane.msg"
  "msg/Move.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
