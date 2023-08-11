# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# ######################################################################################################################
# Preamble
# ######################################################################################################################
cmake_minimum_required(VERSION 3.25)

project(
    dsd_mission_profile
    LANGUAGES CXX
    VERSION 1.0.0
)
# ######################################################################################################################
# Project Wide Setup
# ######################################################################################################################
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(DeveloperSetup)
include(Dependencies)
# ######################################################################################################################
# Dependencies
# ######################################################################################################################
find_package(ament_cmake REQUIRED)
find_package(dsd_ros_messages REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    find_package(launch_testing_ament_cmake)
    add_googletest_submodule()
endif()
# ######################################################################################################################
# Main targets
# ######################################################################################################################

# Targets
add_library(dsd_mission_profile_lib)
add_library(dsd_mission_profile::dsd_mission_profile_lib ALIAS dsd_mission_profile_lib)
target_sources(dsd_mission_profile_lib PRIVATE src/MissionProfilePublisher.cpp)
target_include_directories(dsd_mission_profile_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(
    dsd_mission_profile_lib PUBLIC dsd_ros_messages::dsd_ros_messages rclcpp::rclcpp ${std_msgs_TARGETS}
                                   ${visualization_msgs_TARGETS}
)
ipm_target_dev_setup(dsd_mission_profile_lib)

add_executable(dsd_mission_profile)
target_sources(dsd_mission_profile PRIVATE src/MissionProfileMain.cpp)
target_link_libraries(dsd_mission_profile PRIVATE dsd_mission_profile::dsd_mission_profile_lib)
ipm_target_dev_setup(dsd_mission_profile)

# Install
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/" DESTINATION config/${PROJECT_NAME})
install(TARGETS dsd_mission_profile DESTINATION lib/${PROJECT_NAME})

# ######################################################################################################################
# Tests
# ######################################################################################################################
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory(tests)
endif()

# ######################################################################################################################
# Documentation
# ######################################################################################################################
add_subdirectory(docs)

# ######################################################################################################################
# Package
# ######################################################################################################################
ament_package()
