#!/bin/bash
# CyberDog2 real-robot runner.
#
# Reference: ``real_robot_migration_guide.md`` §13
#
# This is a thin convenience wrapper around
# ``code/cyberdog_race/launch.sh``.  It exists so you can drop the
# repository anywhere and run ``./run_real.sh`` from the project root.
#
# Usage:
#   ./run_real.sh                  # full race
#   ./run_real.sh segment 1        # test segment 1
#   ./run_real.sh --perception-only
#
# First-time setup on CyberDog2:
#   chmod +x run_real.sh
#   chmod +x code/cyberdog_race/launch.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/code/cyberdog_race/launch.sh" real "$@"