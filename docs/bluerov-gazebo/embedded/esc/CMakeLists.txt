cmake_minimum_required(VERSION 3.16)
project(Embedded LANGUAGES CXX)

find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
gz_find_package(gz-sim8 REQUIRED)

add_library(Embedded_VirtualESC SHARED VirtualESC.cc)
target_compile_features(Embedded_VirtualESC PRIVATE cxx_std_17)
target_link_libraries(Embedded_VirtualESC PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8)

set(CONNECTOR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.local/ardupilot_gazebo)

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/ArduPilotPlugin.cc)
  message(FATAL_ERROR "Run esc/build_connector.py before configuring the ESC build")
endif()

add_library(Embedded_ArduPilotPlugin SHARED
  ${CMAKE_CURRENT_BINARY_DIR}/ArduPilotPlugin.cc
  ${CONNECTOR_ROOT}/src/SocketUDP.cc
  ${CONNECTOR_ROOT}/src/Util.cc)

target_compile_features(Embedded_ArduPilotPlugin PRIVATE cxx_std_17)
target_include_directories(Embedded_ArduPilotPlugin PRIVATE ${CONNECTOR_ROOT}/include)
target_link_libraries(Embedded_ArduPilotPlugin PRIVATE gz-sim8::gz-sim8)