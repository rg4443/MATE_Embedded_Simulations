cmake_minimum_required(VERSION 3.16)
project(task_4_topside LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(repo_root "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(mavlink_root "${repo_root}/modules/mavlink")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(generated_header "${generated_dir}/ardupilotmega/mavlink.h")

add_custom_command(
    OUTPUT "${generated_header}"
    COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=${mavlink_root}"
            "${Python3_EXECUTABLE}" -m pymavlink.tools.mavgen
            --lang=C --wire-protocol=2.0 --output "${generated_dir}"
            "${mavlink_root}/message_definitions/v1.0/ardupilotmega.xml"
    DEPENDS
        "${mavlink_root}/message_definitions/v1.0/ardupilotmega.xml"
        "${mavlink_root}/message_definitions/v1.0/common.xml"
        "${mavlink_root}/message_definitions/v1.0/minimal.xml"
    VERBATIM
)

add_custom_target(task_4_mavlink_headers DEPENDS "${generated_header}")

add_executable(task_4_topside topside.cpp)
add_dependencies(task_4_topside task_4_mavlink_headers)
target_include_directories(task_4_topside PRIVATE "${generated_dir}")
target_compile_features(task_4_topside PRIVATE cxx_std_17)
