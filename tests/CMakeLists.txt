cmake_minimum_required(VERSION 3.5.1)
project(opendrive-cpp-test VERSION 0.0.0)

set(TARGET_NAME ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Gtest REQUIRED gtest)
pkg_check_modules(Tinyxml2 REQUIRED tinyxml2)
pkg_check_modules(OpenDriveCpp REQUIRED opendrive-cpp)
pkg_check_modules(OpenDriveEngine QUIET opendrive-engine)

if(${OpenDriveEngine_FOUND})
  set(OpenDriveEngineLibs ${OpenDriveEngine_LIBRARIES})
else()
  set(OpenDriveEngineLibs opendrive-engine)
endif(${OpenDriveEngine_FOUND})

include_directories(
  ${Gtest_INCLUDE_DIRS}
  ${Tinyxml2_INCLUDE_DIRS}
  ${OpenDriveCpp_INCLUDE_DIRS}
  ${OpenDriveEngine_INCLUDE_DIRS}
)

link_directories (
  ${Gtest_LIBRARY_DIRS}
  ${Tinyxml2_LIBRARY_DIRS}
  ${OpenDriveCpp_LIBRARY_DIRS}
  ${OpenDriveEngine_LIBRARY_DIRS}
)

SET(TEST_SOURCES
  engine_test
  kdtree_test
)

FOREACH(test_src ${TEST_SOURCES})
  add_executable(${test_src} ${test_src}.cc)
  target_link_libraries(${test_src}
    ${Gtest_LIBRARIES}
    ${Tinyxml2_LIBRARIES}
    ${OpenDriveCpp_LIBRARIES}
    ${OpenDriveEngineLibs}
    gtest
  )
ENDFOREACH(test_src)
