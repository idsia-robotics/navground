#!/bin/bash
set -e

source "/opt/navground/setup.bash" --
source "/navground_venv/bin/activate" --
source "/ws/install/setup.bash" --
exec "$@"