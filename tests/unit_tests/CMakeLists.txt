# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

add_executable(UnitTests MissionProfilePublisherTests.cpp)
target_include_directories(UnitTests PRIVATE include)
target_link_libraries(UnitTests PRIVATE GTest::gmock_main dsd_mission_profile::dsd_mission_profile_lib)

include(GoogleTest)
gtest_discover_tests(UnitTests PROPERTIES LABELS "UnitTests")
