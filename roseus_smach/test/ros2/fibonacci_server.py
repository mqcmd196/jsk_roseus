#!/usr/bin/env python3
"""Fibonacci action server (example_interfaces) for the roseus_smach samples.

ROS 1 tests use actionlib_tutorials/fibonacci_server instead.
"""
import rclpy
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci


def main():
    rclpy.init()
    node = rclpy.create_node('fibonacci_server')

    def execute(goal_handle):
        seq = [0, 1]
        for i in range(1, goal_handle.request.order):
            seq.append(seq[i] + seq[i - 1])
        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)

    ActionServer(node, Fibonacci, 'fibonacci', execute)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
