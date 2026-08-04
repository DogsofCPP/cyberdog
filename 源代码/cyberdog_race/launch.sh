#!/bin/bash
# CyberDog Race - Easy Launch Script
# Run this inside the Docker container
#
# Usage:
#   bash launch.sh [sim|real] [full|segment-1|segment-2|...] [extra args...]
#

set -e

MODE="${1:-sim}"
SEGMENT=""
PYTHON="python3"
EXTRA_ARGS="${*:3}"

echo "================================================"
echo "  CyberDog Race Controller - Xiaomi Cup 2026"
echo "  Mode: $MODE"
echo "================================================"

# Determine mode
if [ "$MODE" = "real" ]; then
    echo "Real mode: connecting to physical robot"
elif [ "$MODE" = "sim" ]; then
    echo "Sim mode: running in Gazebo simulation"
else
    echo "Unknown mode: $MODE"
    echo "Usage: bash launch.sh [sim|real]"
    exit 1
fi

# Source ROS2
if [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
fi

# Add loco_hl_example to path (where this script likely lives)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RACE_DIR="$SCRIPT_DIR"
Loco_DIR="$(dirname "$SCRIPT_DIR")/loco_hl_example"

# Check if running as module
if [ -d "$RACE_DIR" ]; then
    export PYTHONPATH="$RACE_DIR:$PYTHONPATH"
fi

# Parse segment argument
if [[ "$2" == segment-* ]]; then
    SEG_NUM="${2#segment-}"
    SEGMENT="--test-segment=$SEG_NUM"
elif [[ "$2" =~ ^[0-9]$ ]]; then
    SEGMENT="--test-segment=$2"
fi

echo "Race dir: $RACE_DIR"
echo "loco_hl_example dir: $Loco_DIR"
echo "Segment: $SEGMENT"
echo "Extra args: $EXTRA_ARGS"

# Run the race
cd "$RACE_DIR"
$PYTHON -m cyberdog_race.race_main --mode=$MODE $SEGMENT $EXTRA_ARGS
