#!/bin/bash
# Test wrapper: ros2 topic pub (background) + test-multi-queue.l (foreground)
set -e
TEST_DIR=$(cd "$(dirname "$0")" && pwd)
BG_PIDS=""
cleanup() { pkill -P $$ 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT

ros2 topic pub -r 4 /a std_msgs/msg/String "{data: 'from a'}" &
BG_PIDS="$!"
ros2 topic pub -r 4 /b std_msgs/msg/String "{data: 'from b'}" &
BG_PIDS="$BG_PIDS $!"
sleep 3

roseus "$TEST_DIR/test-multi-queue-ros2.l"
