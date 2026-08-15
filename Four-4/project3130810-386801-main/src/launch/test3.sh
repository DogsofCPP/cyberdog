source /opt/ros/galactic/setup.bash
source install/setup.bash

if ros2 pkg executables cyberdog_demo | grep -q '^cyberdog_demo test3$'; then
    ros2 run cyberdog_demo test3 -- --live
else
    echo "test3 executable not found in install space; running source file directly."
    python3 /home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/test3.py --live
fi
