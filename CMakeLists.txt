cmake_minimum_required(VERSION 3.28)

set(AIS4104_PRAXIS_GIT_TAG "f441ecd38063daf6336dfd4e48debb5f81576a41" CACHE STRING
    "The praxis revision this project builds against, written as the full forty-character commit hash because praxis publishes no tags")
set(AIS4104_CATCH2_GIT_TAG "v3.15.3" CACHE STRING
    "The Catch2 release this project fetches, which is the release praxis pins for its own suite")
set(AIS4104_PREVIEW_FRAMES "0" CACHE STRING
    "How many frames a preview executable draws before it exits on its own; zero runs until the window is closed")
set(CMAKE_OSX_DEPLOYMENT_TARGET "13.4" CACHE STRING
    "The oldest macOS release this build may run on; below 13.4 the standard library withholds the floating-point to_chars that praxis writes its configuration through")

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

set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

include(FetchContent)

# Every target a fetched dependency declares inherits this folder, so an IDE groups the whole of
# them under one node instead of listing them beside this project's own targets.
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

list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
include(Catch)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/warnings.cmake")

enable_testing()

add_subdirectory(modules)
add_subdirectory(application)
add_subdirectory(assignments)
add_subdirectory(presets)
add_subdirectory(tests)
