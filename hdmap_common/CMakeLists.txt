cmake_minimum_required(VERSION 3.5.1)
project(hdmap_common VERSION 1.0.0)

set(target_name ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE ${target_name}_srcs
  "src/*.cc"
)

add_library(${target_name} SHARED
  ${${target_name}_srcs}
)

set(${target_name}_dependencies
  rclcpp
)

ament_target_dependencies(${target_name}
  ${${target_name}_dependencies}
)

target_link_libraries(${target_name}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${target_name}_util_test
    test/util_test.cc
  )
  if(TARGET ${target_name}_util_test)
    target_link_libraries(${target_name}_util_test
      ${Boost_FILESYSTEM_LIBRARY}
      ${Boost_SYSTEM_LIBRARY}
      ${target_name}
    )
  endif()
endif()

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
