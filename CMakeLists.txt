cmake_minimum_required(VERSION 3.28)

set(AIS4104_PRAXIS_GIT_TAG "c0e70b6c6ecff8e98896192d3fba5a335ddd1e9e")
set(AIS4104_CATCH2_GIT_TAG "v3.15.3")
set(AIS4104_PREVIEW_FRAMES "0" CACHE STRING
    "How many frames a preview executable draws before it exits on its own; zero runs until the window is closed")
set(CMAKE_OSX_DEPLOYMENT_TARGET "13.4" CACHE STRING
    "The oldest macOS release this build may run on")

project(ais4104_praxis LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

string(LENGTH "${AIS4104_PRAXIS_GIT_TAG}" ais4104_praxis_tag_length)
if (NOT ais4104_praxis_tag_length EQUAL 40 OR NOT AIS4104_PRAXIS_GIT_TAG MATCHES "^[0-9a-f]+$")
    message(FATAL_ERROR "AIS4104_PRAXIS_GIT_TAG must be a full forty-character praxis commit hash in lowercase hexadecimal, and is \"${AIS4104_PRAXIS_GIT_TAG}\"")
endif ()

if (AIS4104_CATCH2_GIT_TAG STREQUAL "")
    message(FATAL_ERROR "AIS4104_CATCH2_GIT_TAG must name the Catch2 release to fetch")
endif ()

include(FetchContent)

set(CMAKE_FOLDER "Dependencies")

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG ${AIS4104_CATCH2_GIT_TAG}
)
FetchContent_MakeAvailable(Catch2)

FetchContent_Declare(
    praxis
    GIT_REPOSITORY https://github.com/skrede/praxis.git
    GIT_TAG ${AIS4104_PRAXIS_GIT_TAG}
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(praxis)

unset(CMAKE_FOLDER)

meios_declare_resource(
    NAME kuka_experimental
    URL https://github.com/ros-industrial/kuka_experimental/archive/8d9292b04a22628b1b78d989e2ddd3abb913bf92.tar.gz
    HASH SHA256=02f299d967868fb32022617429ddede3a197fb8f18e09b88f88732e11a78bf79
    STRIP_TOP_LEVEL
)

meios_declare_resource(
    NAME ur_description
    URL https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/archive/refs/tags/4.3.1.tar.gz
    HASH SHA256=3532a25c9942bedcdfe41c845be6d144bbbbcd76f179f14a53a3e7b6972ae72d
    STRIP_TOP_LEVEL
)

meios_declare_resource(
    NAME lbr_med14_r820_description
    URL https://github.com/lbr-stack/med14_r820_description/archive/refs/tags/v2.5.0.tar.gz
    HASH SHA256=edb596d3e2b7f07f5b8f66ef68cd3ccfef0212e98efe25742867cfbe653deb5a
    STRIP_TOP_LEVEL
)

meios_declare_resource(
    NAME course_licences
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/descriptions/licences"
)

meios_declare_resource(
    NAME course_models
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models"
)

list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
include(Catch)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/warnings.cmake")

enable_testing()

add_subdirectory(modules)
add_subdirectory(application)
add_subdirectory(assignments)
add_subdirectory(presets)
add_subdirectory(tests)
