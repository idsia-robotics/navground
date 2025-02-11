#!/bin/bash
set -e


source "/opt/ros/rolling/setup.bash" --
source "/opt/navground/setup.bash" --
source "/navground_venv/bin/activate" --
source "/ws/install/setup.bash" --
export NAVGROUND_PLUGINS_PREFIX=/ws/install:$NAVGROUND_PLUGINS_PREFIX
export CMAKE_PREFIX_PATH=/opt/ros/rolling:$CMAKE_PREFIX_PATH
exec "$@"