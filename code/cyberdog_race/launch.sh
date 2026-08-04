#!/bin/bash
# CyberDog Race - Launch Script
#
# Reference: ``real_robot_migration_guide.md`` §13
#
# Usage:
#   bash launch.sh [sim|real] [full|segment-N] [extra args...]
#
# Real mode notes:
#   - Code should live under ~/dograce/cyberdog_race (see §1-§2 of migration guide)
#   - The script auto-detects whether it is running inside the Docker container
#     (sim) or on the real CyberDog hardware.
#   - For real mode you may also run the wrapper directly:
#       export PYTHONPATH=~/dograce:$PYTHONPATH
#       python3 -m cyberdog_race.race_main --mode=real
#
set -e

MODE="${1:-sim}"
SEGMENT=""
EXTRA_ARGS="${*:3}"

echo "================================================"
echo "  CyberDog Race Controller - Xiaomi Cup 2026"
echo "  Mode: $MODE"
echo "================================================"

# ---- Detect real vs sim ----
# If /opt/ros/galactic/setup.bash exists we source it regardless of mode
# (the real robot image also has ROS2).
if [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Real mode: set up PYTHONPATH for ~/dograce layout
if [ "$MODE" = "real" ]; then
    echo "[launch] Real mode: using ~/dograce layout"
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    RACE_DIR="$SCRIPT_DIR"
    if [ -d "$RACE_DIR" ]; then
        # Running from the copied ~/dograce/cyberdog_race directory
        DOGRACE_DIR="$(dirname "$RACE_DIR")"
        export PYTHONPATH="$DOGRACE_DIR:$PYTHONPATH"
        echo "[launch] PYTHONPATH = $PYTHONPATH"
    fi
    echo "[launch] Real mode: ensure the robot is at the course start point"
elif [ "$MODE" = "sim" ]; then
    echo "[launch] Sim mode: running in Gazebo"
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    RACE_DIR="$SCRIPT_DIR"
    if [ -d "$RACE_DIR" ]; then
        export PYTHONPATH="$RACE_DIR:$PYTHONPATH"
    fi
else
    echo "[launch] Unknown mode: $MODE"
    echo "Usage: bash launch.sh [sim|real]"
    exit 1
fi

# ---- Parse segment argument ----
if [[ "$2" == segment-* ]]; then
    SEG_NUM="${2#segment-}"
    SEGMENT="--test-segment=$SEG_NUM"
elif [[ "$2" =~ ^[0-9]$ ]]; then
    SEGMENT="--test-segment=$2"
fi

echo "[launch] Race dir  : $RACE_DIR"
echo "[launch] Segment   : ${SEGMENT:-full race}"
echo "[launch] Extra args: ${EXTRA_ARGS:-none}"

# ---- Run ----
cd "$RACE_DIR"
python3 -m cyberdog_race.race_main --mode=$MODE $SEGMENT $EXTRA_ARGS