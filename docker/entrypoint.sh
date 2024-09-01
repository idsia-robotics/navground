#!/bin/bash
set -e

source "/ws/env/bin/activate" --
source "/ws/install/setup.bash" --
exec "$@"