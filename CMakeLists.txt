cmake_minimum_required(VERSION 3.5.1)

project(opendrive-engine VERSION 1.0.1)

set(TARGET_NAME ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE Debug) # Debug Release

option(BUILD_SHARED_LIBS "Build opendrive-engine shared library" ON)
option(BUILD_OPENDRIVE_ENGINE_TEST "Build opendrive-engine unittest" OFF)
option(BUILD_OPENDRIVE_ENGINE_VIEWER "Build opendrive-engine tools viewer" OFF)

set(OPENDRIVE_ENGINE_SHARED_TYPE SHARED)
if (NOT BUILD_SHARED_LIBS)
  set(OPENDRIVE_ENGINE_SHARED_TYPE STATIC)
endif()

message("---- option shared type:${OPENDRIVE_ENGINE_SHARED_TYPE}")
message("---- option unittest:${BUILD_OPENDRIVE_ENGINE_TEST}")
message("---- option build type:${CMAKE_BUILD_TYPE}")
message("---- option view:${BUILD_OPENDRIVE_ENGINE_VIEWER}")

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Tinyxml2 REQUIRED tinyxml2)
pkg_check_modules(OpenDriveCpp REQUIRED opendrive-cpp>=1.1.0)
pkg_check_modules(NanoFlann REQUIRED nanoflann)
pkg_check_modules(Cactus REQUIRED cactus>=1.1.0)

include_directories(
  include
  ${Tinyxml2_INCLUDE_DIRS}
  ${OpenDriveCpp_INCLUDE_DIRS}
  ${NanoFlann_INCLUDE_DIRS}
  ${Cactus_INCLUDE_DIRS}
)

link_directories(
  ${Tinyxml2_LIBRARY_DIRS}
  ${OpenDriveCpp_LIBRARY_DIRS}
  ${Cactus_LIBRARY_DIRS}
)

file(GLOB OPENDRIVE_ENGINE_SRCS
  "src/*.cc"
  "src/common/*.cc"
  "src/math/*.cc"
  "src/geometry/*.cc"
  "src/algo/kdtree/*.cc"
)

add_library(${TARGET_NAME} ${OPENDRIVE_ENGINE_SHARED_TYPE}
  ${OPENDRIVE_ENGINE_SRCS}
)

target_link_libraries(${TARGET_NAME}
  ${Tinyxml2_LIBRARIES}
  ${OpenDriveCpp_LIBRARIES}
  ${Cactus_LIBRARIES}
)

if(BUILD_OPENDRIVE_ENGINE_TEST)
  add_subdirectory(tests)
endif()

if(BUILD_OPENDRIVE_ENGINE_VIEWER)
  add_subdirectory(viewer/backend)
endif()

# #################################################################################
# version
# #################################################################################
configure_file(include/opendrive-engine/version.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/include/opendrive-engine/version.h
)

# #################################################################################
# config
# #################################################################################
configure_file("cmake/${TARGET_NAME}.pc.in"
  "${PROJECT_BINARY_DIR}/pkgconfig/${TARGET_NAME}.pc" @ONLY)

configure_package_config_file(
  "cmake/${TARGET_NAME}-config.cmake.in"
  "${PROJECT_BINARY_DIR}/cmake/${TARGET_NAME}-config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${TARGET_NAME}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

write_basic_package_version_file(
  "cmake/${TARGET_NAME}-config-version.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${PROJECT_BINARY_DIR}/pkgconfig/${TARGET_NAME}.pc"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/pkgconfig"
)

install(EXPORT ${TARGET_NAME}-targets
  DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${TARGET_NAME}"
)

install(FILES
  "${PROJECT_BINARY_DIR}/cmake/${TARGET_NAME}-config.cmake"
  "${PROJECT_BINARY_DIR}/cmake/${TARGET_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${TARGET_NAME}"
)

install(DIRECTORY include/${TARGET_NAME}/
  DESTINATION include/${TARGET_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

install(TARGETS ${TARGET_NAME}
  EXPORT ${TARGET_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

