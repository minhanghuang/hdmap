cmake_minimum_required(VERSION 3.5.1)
project(hdmap_server VERSION 1.0.0)

set(target_name ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hdmap_common REQUIRED)
find_package(hdmap_msgs REQUIRED)
find_package(hdmap_engine REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE hdmap_server_srcs
  "src/*.cc"
)

set(${target_name}_dependencies
  rclcpp
  visualization_msgs
  geometry_msgs
  hdmap_common
  hdmap_msgs
  hdmap_engine
)

add_executable(${target_name}
  ${hdmap_server_srcs}
)

ament_target_dependencies(${target_name}
  ${${target_name}_dependencies}
)

target_link_libraries(${target_name}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
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

install(DIRECTORY launch conf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${target_name})
ament_export_dependencies(${${target_name}_dependencies})

ament_package()
