cmake_minimum_required(VERSION 3.5.1)

project(engine_server VERSION 1.0.0)

find_package(PkgConfig REQUIRED)
pkg_check_modules(NlohmannJson REQUIRED nlohmann_json)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)
pkg_check_modules(Cyclone REQUIRED cyclone)
pkg_check_modules(OpenDriveCpp REQUIRED opendrive-cpp)
pkg_check_modules(OpenDriveEngine QUIET opendrive-engine)
pkg_check_modules(NanoFlann REQUIRED nanoflann)
pkg_check_modules(Cactus REQUIRED cactus)

if(${OpenDriveEngine_FOUND})
  set(OpenDriveEngineLibs ${OpenDriveEngine_LIBRARIES})
else()
  set(OpenDriveEngineLibs opendrive-engine)
endif(${OpenDriveEngine_FOUND})

include_directories(
  ${NlohmannJson_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIRS}
  ${Cyclone_INCLUDE_DIRS}
  ${OpenDriveCpp_INCLUDE_DIRS}
  ${NanoFlann_INCLUDE_DIRS}
  ${Cactus_INCLUDE_DIRS}
)

link_directories (
  ${NlohmannJson_LIBRARY_DIRS}
  ${YamlCpp_LIBRARY_DIRS}
  ${Cyclone_LIBRARY_DIRS}
  ${OpenDriveCpp_LIBRARY_DIRS}
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
  ${Cyclone_LIBRARIES}
  ${OpenDriveCpp_LIBRARIES}
  ${Cactus_LIBRARIES}
)

file(COPY conf/engine_server.yaml
  DESTINATION 
  ${CMAKE_BINARY_DIR}/conf
)

