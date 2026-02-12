#!/bin/bash
# Test wrapper: talker.l (background) + test-talker-listener-ros2.l (foreground)
set -e
TEST_DIR=$(cd "$(dirname "$0")" && pwd)
BG_PIDS=""
cleanup() { pkill -P $$ 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT

roseus "$TEST_DIR/talker.l" &
BG_PIDS="$!"
sleep 3

roseus "$TEST_DIR/test-talker-listener-ros2.l"
