#!/usr/bin/env python3
import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_dir = os.path.dirname(os.path.abspath(__file__))

    static_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '1', '--y', '-2', '--z', '0.5',
                   '--frame-id', 'world', '--child-frame-id', 'static_frame'],
        output='screen',
    )

    test_proc = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'test-tf-ros2.l')],
        output='screen',
        name='test_proc',
    )

    return launch.LaunchDescription([
        static_tf,
        launch.actions.TimerAction(period=1.0, actions=[test_proc]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_proc': test_proc}


class TestTf(unittest.TestCase):
    def test_wait_for_shutdown(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(process=test_proc, timeout=55)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_proc)
