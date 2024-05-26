cmake_minimum_required(VERSION 3.5)
project(hdmap_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Point.msg"
  "msg/Curve.msg"
  "msg/Boundary.msg"
  "msg/Lane.msg"
  "msg/Section.msg"
  "msg/Road.msg"
  "msg/Map.msg"
  "srv/GetGlobalMap.srv"
)

set(dependencies
  builtin_interfaces
  std_msgs
  geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
