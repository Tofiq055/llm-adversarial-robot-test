#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ur5e_ws/install/setup.bash 2>/dev/null || true
source /ws/install/setup.bash 2>/dev/null || true
exec "$@"
