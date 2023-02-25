cmake_minimum_required(VERSION 3.5.1)

project(engine_server VERSION 1.0.0)

find_package(PkgConfig REQUIRED)
pkg_check_modules(NlohmannJson REQUIRED nlohmann_json)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)
pkg_check_modules(Typhoon REQUIRED typhoon)
pkg_check_modules(OpenDriveCpp REQUIRED opendrive-cpp)
pkg_check_modules(OpenDriveEngine QUIET opendrive-engine)
pkg_check_modules(Flann REQUIRED flann)
pkg_check_modules(LZ4 REQUIRED liblz4)
pkg_check_modules(Cactus REQUIRED cactus)

if(${OpenDriveEngine_FOUND})
  set(OpenDriveEngineLibs ${OpenDriveEngine_LIBRARIES})
else()
  set(OpenDriveEngineLibs opendrive-engine)
endif(${OpenDriveEngine_FOUND})

include_directories(
  ${NlohmannJson_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIRS}
  ${Typhoon_INCLUDE_DIRS}
  ${OpenDriveCpp_INCLUDE_DIRS}
  ${Flann_INCLUDE_DIRS}
  ${LZ4_INCLUDE_DIRS}
  ${Cactus_INCLUDE_DIRS}
)

link_directories (
  ${NlohmannJson_LIBRARY_DIRS}
  ${YamlCpp_LIBRARY_DIRS}
  ${Typhoon_LIBRARY_DIRS}
  ${OpenDriveCpp_LIBRARY_DIRS}
  ${Flann_LIBRARY_DIRS}
  ${LZ4_LIBRARY_DIRS}
  ${Cactus_LIBRARY_DIRS}
)

file(GLOB ENGINE_SERVER_SRCS
  "*.cc"
  "src/*.cc"
)

add_executable(${PROJECT_NAME}_runner ${ENGINE_SERVER_SRCS})

target_link_libraries(${PROJECT_NAME}_runner
  ${OpenDriveEngineLibs}
  ${NlohmannJson_LIBRARIES}
  ${YamlCpp_LIBRARIES}
  ${Typhoon_LIBRARIES}
  ${OpenDriveCpp_LIBRARIES}
  ${Flann_LIBRARIES}
  ${LZ4_LIBRARIES}
  ${Cactus_LIBRARIES}
)

file(COPY conf/engine_server.yaml
  DESTINATION 
  ${CMAKE_BINARY_DIR}/conf
)

