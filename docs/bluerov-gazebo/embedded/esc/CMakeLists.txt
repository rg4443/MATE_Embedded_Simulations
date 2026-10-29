cmake_minimum_required(VERSION 3.16)
project(mate_virtual_esc LANGUAGES CXX)

find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
gz_find_package(gz-sim8 REQUIRED)

add_library(MATE_VirtualESC SHARED VirtualESC.cc)
target_compile_features(MATE_VirtualESC PRIVATE cxx_std_17)
target_link_libraries(MATE_VirtualESC PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8)

set(CONNECTOR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.local/ardupilot_gazebo)

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/ArduPilotPlugin.cc)
  message(FATAL_ERROR "Run esc/build_connector.py before configuring the ESC build")
endif()

add_library(MATE_ArduPilotPlugin SHARED
  ${CMAKE_CURRENT_BINARY_DIR}/ArduPilotPlugin.cc
  ${CONNECTOR_ROOT}/src/SocketUDP.cc
  ${CONNECTOR_ROOT}/src/Util.cc)

target_compile_features(MATE_ArduPilotPlugin PRIVATE cxx_std_17)
target_include_directories(MATE_ArduPilotPlugin PRIVATE ${CONNECTOR_ROOT}/include)
target_link_libraries(MATE_ArduPilotPlugin PRIVATE gz-sim8::gz-sim8)