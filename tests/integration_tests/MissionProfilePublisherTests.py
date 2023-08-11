# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: Apache-2.0

import os, sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import unittest
import pytest
from time import time

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertSequentialStdout
import rclpy

from sensors_for_rail_sw_interfaces.msg import MissionProfileStamped, ID
from helper.TestSubscriber import TestSubscriber


@pytest.mark.rostest
def generate_test_description():
    mission_profiler_node = Node(
        package='dsd_mission_profile',
        node_executable='dsd_mission_profile',
        output="screen",
        emulate_tty=True,  # Unbuffered input for test
        parameters=[{
            "mission_profile_topic": "/test_topic"
        }, {
            "map_provider": "DB"
        }, {
            "track_ids": [1, 10, 42, 103, 1372]
        }, {
            "track_uuids": ["1", "10", "42", "103", "1372"]
        }])

    launch_description = LaunchDescription(
        [mission_profiler_node, ReadyToTest()])

    context = {
        'mission_profiler_node': mission_profiler_node,
    }

    return (launch_description, context)


class TestRunningDataPublisher(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.subscriber = TestSubscriber(MissionProfileStamped, "/test_topic")

    def tearDown(self):
        self.subscriber.tearDown()
        rclpy.shutdown()

    def test_message_content(self):
        for i in range(3):
            message = self.subscriber.spin(timeoutSec=2)
            self.assertMessage(message)

    def assertMessage(self, message):
        self.assertIsNotNone(message)

        self.assertEqual(0, message.seq)  # Not used currently
        self.assertAlmostEqual(time(), message.stamp.sec, delta=2)
        self.assertFalse(
            message.mission_profile.track_duration)  # Not used currently
        self.assertEqual(
            [ID(id=1), ID(id=10),
             ID(id=42),
             ID(id=103),
             ID(id=1372)], message.mission_profile.track_ids)


@post_shutdown_test()
class TestMissionProfilePublisher(unittest.TestCase):

    def test_mission_profile_periodically_broadcasted(self, proc_output,
                                                      mission_profiler_node):
        # Sequential check currently not necessary, but the test can be easily modified in case the logging message changes in the future
        with assertSequentialStdout(proc_output, mission_profiler_node) as cm:
            for n in range(3):
                cm.assertInStdout("Publishing Mission Profile message")
