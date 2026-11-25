#!/usr/bin/env python3
import os
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


def roseus(script, name):
    # stdin is kept open: do-until-key of the action servers stops at EOF
    return launch.actions.ExecuteProcess(
        cmd=['bash', '-c', 'sleep infinity | exec roseus ' + script],
        output='screen', name=name)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    servers = [roseus(os.path.join(test_dir, 'multiply10-action-server.l'), 'multiply10'),
               roseus(os.path.join(test_dir, 'sub5-action-server.l'), 'sub5')]
    test_proc = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'test-async-join-state-machine-actionlib.l')],
        output='screen',
        name='test_proc',
    )

    return launch.LaunchDescription(servers + [
        launch.actions.TimerAction(period=5.0, actions=[test_proc]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_proc': test_proc}


class TestSmach(unittest.TestCase):
    def test_wait_for_shutdown(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(process=test_proc, timeout=100)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_proc)
