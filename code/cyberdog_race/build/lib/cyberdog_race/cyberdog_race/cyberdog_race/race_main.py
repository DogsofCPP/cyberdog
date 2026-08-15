"""CyberDog Race - Main Entry Point.

Usage (sim):
    source /opt/ros/galactic/setup.bash
    python3 -m cyberdog_race.race_main --mode=sim

Usage (real):
    source /opt/ros/galactic/setup.bash
    export PYTHONPATH=~/dograce:$PYTHONPATH
    python3 -m cyberdog_race.race_main --mode=real

Reference: ``real_robot_migration_guide.md``
    - Section 4: real mode uses ``ROS2Perception`` (not ``MockPerception``)
    - Section 7: real mode skips Gazebo calibration, sets offset to (0,0,0)
    - Section 13: recommended to use a wrapper shell script instead of bare
      ``python3 -m cyberdog_race.race_main``
"""
import argparse
import sys
import time


def _spin_node(node):
    """Spin a rclpy node in a background thread."""
    import rclpy
    try:
        rclpy.spin(node)
    except Exception:
        pass


def _calibrate_position(perception, status):
    """Auto-calibrate position offset based on sensor status.

    In sim mode, the robot might spawn at a different position in Gazebo
    than (0,0) in course_map coordinates. This function detects the offset
    and sets the position_offset so perception.position returns course_map coords.
    """
    if status.get('gazebo', False):
        gazebo_model = status.get('gazebo_model', 'unknown')
        print(f"[Main] Gazebo model detected: {gazebo_model}")
        raw_x, raw_y, raw_yaw = perception.pose
        offset_x = -raw_x
        offset_y = -raw_y
        print(f"[Main] Raw Gazebo position: ({raw_x:.3f}, {raw_y:.3f})")
        print(f"[Main] Setting position offset to: ({offset_x:.3f}, {offset_y:.3f})")
        perception.set_position_offset(offset_x, offset_y, 0.0)
        print(f"[Main] Calibrated position: {perception.position}")
    else:
        print("[Main] No Gazebo model_states detected, using odom as-is")


def _init_ros2_perception(mode, rclpy_state):
    """Bring up ROS2Perception for sim or real mode.

    Args:
        mode: 'sim' or 'real'
        rclpy_state: dict with key 'started' set to True when rclpy.init()
                     was successfully called

    Returns:
        (perception, spin_thread, rclpy_state) on success.
        (None, None, rclpy_state) on timeout/error.
    """
    import threading
    import rclpy
    from cyberdog_race.perception.ros2_perception import ROS2Perception

    if not rclpy.ok():
        rclpy.init()
    rclpy_state['started'] = True

    perception = ROS2Perception()
    print(f"[Main] Perception module: {ROS2Perception.__module__}")

    if mode == 'real':
        # Migration guide §7: real robot has no Gazebo.  The robot must be
        # placed at the course origin; we force the offset to (0,0,0) and
        # warn the user.
        perception.set_position_offset(0.0, 0.0, 0.0)
        print("[Main] Real mode: Gazebo calibration skipped, "
              "position offset forced to (0,0,0)")
        print("[Main] Real mode: ensure the robot is placed at the course "
              "start with heading aligned to the course x-axis before "
              "launching this script")

    print(f"[Main] Readiness policy: require scan+odom; RGB/depth are optional")
    print("[Main] Waiting for sensors...")
    ready = perception.wait_until_ready(timeout_s=25.0)
    status = perception.sensor_status()
    if not ready and status.get('scan', False) and status.get('odom', False):
        print(f"[Main] Sensor readiness partial: {status}")
        print("[Main] Continuing with scan+odom; "
              "visual target segments may be degraded")
        ready = True

    if not ready:
        print(f"[Main] Sensor readiness timeout: {status}")
        try:
            topics = perception.get_topic_names_and_types()
            print("[Main] Available ROS topics:")
            for name, types in topics:
                print(f"  {name}: {', '.join(types)}")
        except Exception as e:
            print(f"[Main] Could not list ROS topics: {e}")
        return None, None, rclpy_state

    print(f"[Main] Sensors ready: {status}")
    if not status.get('rgb', False):
        print("[Main] Warning: RGB not ready; "
              "visual target segments may use fallback behavior")

    if mode == 'sim':
        _calibrate_position(perception, status)

    spin_thread = threading.Thread(
        target=_spin_node, args=(perception,), daemon=True)
    spin_thread.start()
    print("[Main] ROS2 perception started (spinning)")
    return perception, spin_thread, rclpy_state


def main():
    parser = argparse.ArgumentParser(
        description="CyberDog Race Controller - 2026 Xiaomi Cup")
    parser.add_argument(
        '--mode', default='real',
        choices=['sim', 'real'],
        help='Run in simulation or real mode (default: real)')
    parser.add_argument(
        '--perception-only', action='store_true',
        help='Run only the perception node')
    parser.add_argument(
        '--test-segment', type=int, default=None,
        help='Test a specific segment (1-6)')
    parser.add_argument(
        '--test-low-crawl-start', action='store_true',
        help='Temporarily test Segment 4 low-crawl from the current start pose')
    parser.add_argument(
        '--low-crawl-duration', type=float, default=18.0,
        help='Low-crawl test duration in seconds')
    parser.add_argument(
        '--low-crawl-speed', type=float, default=0.05,
        help='Low-crawl test forward speed in m/s')
    parser.add_argument(
        '--low-crawl-method', default='user-gait',
        choices=['user-gait', 'legacy-user-gait', 'user-gait-side', 'head-down',
                 'front-stretch-forward', 'front-stretch-back',
                 'pulse', 'pose'],
        help='Low-crawl test method')
    parser.add_argument(
        '--low-crawl-z', type=float, default=-0.16,
        help='User-gait body z command for low crawl (negative lowers body)')
    parser.add_argument(
        '--low-crawl-pitch', type=float, default=None,
        help='User-gait body pitch in radians for low crawl')
    parser.add_argument(
        '--low-crawl-cycles', type=int, default=None,
        help='Number of user-gait low-crawl cycles to run')
    parser.add_argument(
        '--low-crawl-lateral', type=float, default=0.04,
        help='Sideways velocity for user-gait-side low crawl')
    parser.add_argument(
        '--low-crawl-rest-every', type=int, default=1,
        help='Restabilize after this many low-crawl cycles')
    parser.add_argument(
        '--head-down-pitch', type=float, default=-0.20,
        help='Body pitch in radians for head-down walking test')
    parser.add_argument(
        '--timeout', type=float, default=900.0,
        help='Race timeout in seconds (default 900s = 15min)')
    parser.add_argument(
        '--no-voice', action='store_true',
        help='Disable voice announcements')
    args = parser.parse_args()

    if args.perception_only:
        run_perception()
        return

    if args.test_segment is not None:
        test_single_segment(args.test_segment,
                            mode=args.mode,
                            enable_voice=not args.no_voice)
        return

    if args.test_low_crawl_start:
        test_low_crawl_start(mode=args.mode,
                             duration_s=args.low_crawl_duration,
                             speed=args.low_crawl_speed,
                             method=args.low_crawl_method,
                             gait_z=args.low_crawl_z,
                             gait_pitch=args.low_crawl_pitch,
                             cycles=args.low_crawl_cycles,
                             lateral=args.low_crawl_lateral,
                             rest_every=args.low_crawl_rest_every,
                             head_down_pitch=args.head_down_pitch)
        return

    run_full_race(mode=args.mode,
                  timeout=args.timeout,
                  enable_voice=not args.no_voice)


def run_perception():
    """Run standalone perception node (used by --perception-only)."""
    import rclpy
    from cyberdog_race.perception.ros2_perception import ROS2Perception

    rclpy.init()
    node = ROS2Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_full_race(mode='sim', timeout=900.0, enable_voice=True):
    """Run the complete race with all segments."""
    print("=" * 60)
    print("  CyberDog Race Controller")
    print("  Mode:", mode)
    print("  Timeout:", timeout, "s")
    print("  Voice:", "enabled" if enable_voice else "disabled")
    print("=" * 60)

    # ---- LCM Controller ----
    from cyberdog_race.lcm_controller import LCMController
    ctrl = LCMController()
    ctrl.start()
    print("[Main] LCM controller started")

    # ---- Voice ----
    speaker = None
    if enable_voice:
        try:
            from cyberdog_race.utils.speech import VoiceSpeaker
            speaker = VoiceSpeaker(rate=130, pitch=50, voice='zh+f3')
            print("[Main] Voice speaker initialized")
        except Exception as e:
            print(f"[Main] Voice init failed: {e}, continuing without voice")

    # ---- ROS2 Perception ----
    rclpy_state = {'started': False}
    perception = None
    spin_thread = None

    try:
        perception, spin_thread, rclpy_state = _init_ros2_perception(
            mode, rclpy_state)
        if perception is None:
            ctrl.shutdown()
            if rclpy_state['started']:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            return
    except Exception as e:
        print(f"[Main] Perception init failed ({e}); exiting")
        ctrl.shutdown()
        return

    # ---- State Machine ----
    from cyberdog_race.navigation.state_machine import RaceStateMachine
    sm = RaceStateMachine(ctrl, perception, speaker)

    print("[Main] Starting race...")
    try:
        sm.run(timeout_s=timeout)
    except KeyboardInterrupt:
        print("[Main] Interrupted")
        from cyberdog_race.navigation.state_machine import RaceState
        sm.transition_to(RaceState.ERROR)
    finally:
        ctrl.shutdown()
        if rclpy_state['started']:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        print("[Main] Done")


def test_single_segment(seg_num, mode='sim', enable_voice=True):
    """Test a single segment in isolation."""
    print(f"[Main] Testing segment {seg_num}")
    print("[Main] Mode:", mode)

    from cyberdog_race.lcm_controller import LCMController
    ctrl = LCMController()
    ctrl.start()

    rclpy_state = {'started': False}
    perception = None
    spin_thread = None

    try:
        perception, spin_thread, rclpy_state = _init_ros2_perception(
            mode, rclpy_state)
        if perception is None:
            ctrl.shutdown()
            if rclpy_state['started']:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            return
    except Exception as e:
        print(f"[Test] Perception init failed ({e})")
        ctrl.shutdown()
        return

    speaker = None
    if enable_voice:
        try:
            from cyberdog_race.utils.speech import VoiceSpeaker
            speaker = VoiceSpeaker()
        except Exception as e:
            print(f"[Test] Voice init failed: {e}, continuing without voice")

    # Stand up first
    print("[Test] Standing up...")
    ctrl.stand_up_and_wait(height=0.22, wait_s=2.0)

    segment_map = {
        1: ('cyberdog_race.navigation.segments.segment_1', 'execute'),
        2: ('cyberdog_race.navigation.segments.segment_2', 'execute'),
        3: ('cyberdog_race.navigation.segments.segment_3', 'execute'),
        4: ('cyberdog_race.navigation.segments.segment_4', 'execute'),
        5: ('cyberdog_race.navigation.segments.segment_5', 'execute'),
        6: ('cyberdog_race.navigation.segments.segment_6', 'execute'),
    }

    if seg_num not in segment_map:
        print(f"[Test] Invalid segment {seg_num}")
        ctrl.shutdown()
        return

    module_name, func_name = segment_map[seg_num]
    import importlib
    mod = importlib.import_module(module_name)
    func = getattr(mod, func_name)

    # Create a Navigator for this segment
    from cyberdog_race.navigation.navigator import Navigator
    nav = Navigator(perception)

    try:
        if seg_num == 1:
            func(ctrl, perception, nav)
        elif seg_num == 2:
            func(ctrl, perception, speaker, nav)
        elif seg_num == 3:
            func(ctrl, perception, nav)
        elif seg_num == 4:
            func(ctrl, perception, speaker, nav, announced=set())
        elif seg_num == 5:
            func(ctrl, perception, nav)
        elif seg_num == 6:
            func(ctrl, perception, speaker, nav)
    except Exception as e:
        print(f"[Test] Segment {seg_num} error: {e}")

    ctrl.shutdown()
    if rclpy_state['started']:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    print("[Test] Done")


def test_low_crawl_start(mode='sim', duration_s=18.0, speed=0.05,
                         method='user-gait', gait_z=-0.17, gait_pitch=None,
                         cycles=None,
                         lateral=0.04, rest_every=6,
                         head_down_pitch=-0.20):
    """Run only the temporary Segment 4 low-crawl posture test."""
    print("[LowCrawlTest] Testing Segment 4 low-crawl from current start pose")
    print("[LowCrawlTest] Mode:", mode)

    from cyberdog_race.lcm_controller import LCMController
    ctrl = LCMController()
    ctrl.start()

    rclpy_state = {'started': False}
    perception = None
    spin_thread = None

    try:
        perception, spin_thread, rclpy_state = _init_ros2_perception(
            mode, rclpy_state)
        if perception is None:
            ctrl.shutdown()
            if rclpy_state['started']:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            return
    except Exception as e:
        print(f"[LowCrawlTest] Perception init failed ({e})")
        ctrl.shutdown()
        return

    try:
        print("[LowCrawlTest] Standing up...")
        ctrl.stand_up_and_wait(height=0.22, wait_s=2.0)

        from cyberdog_race.navigation.segments.segment_4 import (
            test_low_crawl_from_start,
        )
        test_low_crawl_from_start(
            ctrl, perception, duration_s=duration_s, speed=speed,
            method=method, gait_z=gait_z, gait_pitch=gait_pitch, cycles=cycles,
            lateral=lateral, rest_every=rest_every,
            head_down_pitch=head_down_pitch)
    except Exception as e:
        print(f"[LowCrawlTest] Error: {e}")
    finally:
        ctrl.shutdown()
        if rclpy_state['started']:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        print("[LowCrawlTest] Done")


if __name__ == '__main__':
    main()