cmake_minimum_required(VERSION 3.5.1)
project(hdmap_engine VERSION 1.0.0)

set(target_name ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE hdmap_engine_srcs
  "include/hdmap_engine/3rd_party/*.c"
  "src/*.cc"
)

add_library(${target_name} SHARED
  ${hdmap_engine_srcs}
)

set(${target_name}_dependencies
  rclcpp
  tinyxml2_vendor
  TinyXML2
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
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

install(TARGETS ${target_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${target_name})
ament_export_dependencies(${${target_name}_dependencies})

ament_package()
