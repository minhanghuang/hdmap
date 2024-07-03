cmake_minimum_required(VERSION 3.5)
project(hdmap_rviz_plugins)

set(target_name ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hdmap_common REQUIRED)
find_package(hdmap_msgs REQUIRED)

ament_package_xml()

file(GLOB_RECURSE headers_to_moc
  "include/hdmap_rviz_plugins/*.h"
)

foreach(header "${headers_to_moc}")
  qt5_wrap_cpp(hdmap_rviz_plugins_moc_files "${header}")
endforeach()

file(GLOB_RECURSE hdmap_rviz_srcs
  "src/*.cc"
)

add_library(${target_name} SHARED
  ${hdmap_rviz_plugins_moc_files}
  ${hdmap_rviz_srcs}
)

set(${target_name}_dependencies
  geometry_msgs
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  geometry_msgs
  hdmap_common
  hdmap_msgs
)

ament_target_dependencies(${target_name} PUBLIC
  ${${target_name}_dependencies}
)

target_include_directories(${target_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
include_directories(${target_name}
  include
)

target_link_libraries(${target_name} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
)

target_compile_definitions(${target_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
target_compile_definitions(${target_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_targets(${target_name})
ament_export_dependencies(${${target_name}_dependencies})
ament_export_include_directories(include)
ament_export_libraries(${target_name})

install(TARGETS ${target_name}
  EXPORT ${target_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
