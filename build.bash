#!/usr/bin/env bash
set -euo pipefail

# Keep one CMake generator across development machines. Existing build caches
# in this workspace were configured with Unix Makefiles.
unset CMAKE_GENERATOR

exec colcon build "$@" --cmake-args -G "Unix Makefiles"
