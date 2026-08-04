"""
CyberDog Race Package - 2026 Xiaomi Cup
Complete race controller for the CyberDog robot.

Package structure:
    cyberdog_race/
        race_main.py         - Main entry point
        lcm_controller.py    - LCM communication wrapper
        lcm_types/           - LCM message type definitions
        perception/          - ROS2 perception (camera, LiDAR, IMU)
            detectors/       - Ball, object, boundary detectors
        navigation/          - Navigation logic
            state_machine.py - 6-segment state machine
            segments/        - Per-segment navigation strategies
        motion/              - High-level motion primitives
        utils/               - Helpers (speech, mock, etc.)
"""
__version__ = "1.0.0"
__author__ = "CyberDog Race Team"
