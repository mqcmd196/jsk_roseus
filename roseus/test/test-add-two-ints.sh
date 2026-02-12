#!/bin/bash
# Test wrapper: add-two-ints-server.l (background) + test-add-two-ints.l (foreground)
# Note: Use pipe to keep stdin open for do-until-key in background process
set -e
TEST_DIR=$(cd "$(dirname "$0")" && pwd)
BG_PIDS=""
cleanup() { pkill -P $$ 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT

sleep infinity | roseus "$TEST_DIR/add-two-ints-server.l" &
BG_PIDS="$!"
sleep 3

roseus "$TEST_DIR/test-add-two-ints-ros2.l"
