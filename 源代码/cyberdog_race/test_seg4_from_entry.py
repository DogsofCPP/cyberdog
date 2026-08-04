"""Test Segment 4 from Entry Point.

Navigate to (3.0, 7.0, 180.0), then execute segment 4.

Prerequisites:
    Terminal 1: Gazebo + Lidar
        cd /home/cyberdog_sim
        source /opt/ros/galactic/setup.bash
        source install/setup.bash
        ros2 launch cyberdog_gazebo race_gazebo.launch.py use_lidar:=true

    Terminal 2: CyberDog Control
        cd /home/cyberdog_sim
        source /opt/ros/galactic/setup.bash
        source install/setup.bash
        ros2 launch cyberdog_gazebo cyberdog_control_launch.py

Usage:
    cd /home
    source /opt/ros/galactic/setup.bash
    source /home/cyberdog_ws/install/setup.bash
    python3 test_seg4_from_entry.py --mode=sim
"""
import argparse
import math
import sys
import time


def main():
    parser = argparse.ArgumentParser(
        description="Test Segment 4 from Entry Point")
    parser.add_argument(
        '--mode', default='sim',
        choices=['sim', 'real'],
        help='Run in simulation or real mode')
    parser.add_argument(
        '--no-voice', action='store_true',
        help='Disable voice announcements')
    args = parser.parse_args()

    # ---- LCM Controller ----
    from cyberdog_race.lcm_controller import LCMController
    ctrl = LCMController()
    ctrl.start()
    print("[Test] LCM controller started")

    # ---- Voice ----
    speaker = None
    if not args.no_voice:
        try:
            from cyberdog_race.utils.speech import VoiceSpeaker
            speaker = VoiceSpeaker(rate=130, pitch=50, voice='zh+f3')
            print("[Test] Voice speaker initialized")
        except Exception as e:
            print(f"[Test] Voice init failed: {e}, continuing without voice")

    # ---- ROS2 Perception ----
    if args.mode == 'sim':
        import threading
        import rclpy
        from cyberdog_race.perception.ros2_perception import ROS2Perception

        rclpy.init()
        perception = ROS2Perception()
        print("[Test] Perception module: ROS2Perception")
        print("[Test] Waiting for simulation sensors...")
        ready = perception.wait_until_ready(timeout_s=25.0)
        status = perception.sensor_status()
        if not ready and status.get('scan', False) and status.get('odom', False):
            print(f"[Test] Sensor readiness partial: {status}")
            ready = True
        if not ready:
            print(f"[Test] Sensor readiness timeout: {status}")
            ctrl.shutdown()
            rclpy.shutdown()
            return
        print(f"[Test] Sensors ready: {status}")

        spin_thread = threading.Thread(
            target=_spin_node, args=(perception,), daemon=True)
        spin_thread.start()
    else:
        from cyberdog_race.utils.mock_perception import MockPerception
        perception = MockPerception()

    # ---- Stand up ----
    print("[Test] Standing up...")
    ctrl.stand_up_and_wait(height=0.22, wait_s=2.0)

    # ---- Navigate to Segment 4 Entry Point ----
    TARGET_X, TARGET_Y, TARGET_YAW = 3.0, 7.0, 180.0
    print(f"[Test] Navigating to segment 4 entry: ({TARGET_X}, {TARGET_Y}, {TARGET_YAW})")

    from cyberdog_race.navigation.navigator import Navigator
    nav = Navigator(perception)
    nav.plan_to(TARGET_X, TARGET_Y, TARGET_YAW)

    arrival_time = time.perf_counter()
    ARRIVAL_TIMEOUT_S = 60.0

    while not nav.is_arrived():
        if time.perf_counter() - arrival_time > ARRIVAL_TIMEOUT_S:
            print("[Test] Navigation timeout, proceeding anyway...")
            break
        current_pos = perception.position
        heading_deg = perception.heading
        dist = nav.dist_to_target()
        print(f"[Test] Current: ({current_pos[0]:.2f}, {current_pos[1]:.2f}), "
              f"heading: {heading_deg:.1f} deg, dist: {dist:.2f}m")
        # Use Navigator's update to drive
        nav.update(ctrl)
        time.sleep(0.05)

    # Ensure heading is aligned before entering segment 4
    ENTRY_ANGLE_TOL = 20.0
    arrival_heading = perception.heading
    angle_diff = abs((arrival_heading - TARGET_YAW + 180) % 360 - 180)
    print(f"[Test] Arrived at entry: {perception.position}, heading: {arrival_heading:.1f} deg, "
          f"angle_diff: {angle_diff:.1f} deg")

    if angle_diff > ENTRY_ANGLE_TOL:
        print(f"[Test] Heading not aligned ({angle_diff:.1f} deg > {ENTRY_ANGLE_TOL}), "
              "rotating in place...")
        align_start = time.perf_counter()
        ALIGN_TIMEOUT_S = 15.0
        while True:
            if time.perf_counter() - align_start > ALIGN_TIMEOUT_S:
                print("[Test] Heading alignment timeout, proceeding anyway...")
                break
            heading_deg = perception.heading
            diff = ((TARGET_YAW - heading_deg + 180) % 360) - 180
            if abs(diff) < ENTRY_ANGLE_TOL:
                print(f"[Test] Heading aligned: {heading_deg:.1f} deg")
                break
            steer_cmd = max(-0.4, min(0.4, math.radians(diff) * 2.0))
            ctrl.locomotion(
                gait_id=27,  # GAIT_TROT_SLOW
                vx=0.0, vy=0.0, wz=steer_cmd,
                step_h_max=0.05, step_h_min=0.03,
                duration_ms=0,
            )
            time.sleep(0.05)

    print(f"[Test] Final entry state: pos={perception.position}, "
          f"heading={perception.heading:.1f} deg")

    # ---- Execute Segment 4 ----
    print("[Test] Executing segment 4...")
    from cyberdog_race.navigation.segments.segment_4 import execute as seg4_execute
    try:
        seg4_execute(ctrl, perception, speaker, nav, announced=set())
    except Exception as e:
        print(f"[Test] Segment 4 error: {e}")
        import traceback
        traceback.print_exc()

    # ---- Cleanup ----
    ctrl.shutdown()
    if args.mode == 'sim':
        rclpy.shutdown()
    print("[Test] Done")


def _spin_node(node):
    """Spin a rclpy node in a background thread."""
    import rclpy
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
