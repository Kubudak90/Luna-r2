#!/bin/bash
set -e
source /opt/ros/humble/install/setup.bash
source /opt/luna_ws/install/setup.bash
exec "$@"
