#!/bin/bash
# Test wrapper: test-add-two-ints-server-groupname-ros2.l (foreground/test) + add-two-ints-client (background)
# In ROS 2, DDS discovery is slow, so start server first then client after delay
set -e
TEST_DIR=$(cd "$(dirname "$0")" && pwd)
cleanup() { pkill -P $$ 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT

# Start client after delay (background) to allow server to advertise first
(sleep 5 && roseus "$TEST_DIR/add-two-ints-client-ros2.l") &

# Run server test (foreground) - this is the actual test
roseus "$TEST_DIR/test-add-two-ints-server-groupname-ros2.l"
