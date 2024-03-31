cmake_minimum_required(VERSION 3.5.1)
project(hdmap_server VERSION 1.0.0)

set(target_name ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

file(GLOB_RECURSE hdmap_server_srcs
  "src/*.cc"
)

set(${target_name}_dependencies
  rclcpp
)

add_executable(${target_name}
  ${hdmap_server_srcs}
)

ament_target_dependencies(${target_name}
  ${${target_name}_dependencies}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${target_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${target_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${target_name})
ament_export_dependencies(${${target_name}_dependencies})

ament_package()
