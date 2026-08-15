"""Walk forward on a CyberDog in normal posture (sim or real).

Reference: ``real_robot_migration_guide.md``
    - Section 4 (real mode): use ``ROS2Perception`` (not ``MockPerception``)
    - Section 7 (real mode): disable Gazebo calibration, set offset to (0,0,0)
    - Section 11 (real mode): keep speeds conservative (0.05-0.12 m/s)

This script:
    1. Starts the LCM controller.
    2. In ``sim`` mode, brings up the full ``ROS2Perception`` (uses Gazebo if
       available, falls back to ``/odom`` otherwise).
       In ``real`` mode, brings up ``ROS2Perception`` without Gazebo
       calibration and assumes the robot is placed at the course origin.
    3. Stands up, waits for stabilization, prints the controller feedback.
    4. Walks straight at a fixed forward speed for ``--duration`` seconds.
    5. Stops and shuts everything down.

Usage (sim):
    source /opt/ros/galactic/setup.bash
    python3 cyberdog_race/test_low_crawl.py --mode=sim

Usage (real):
    source /opt/ros/galactic/setup.bash
    export PYTHONPATH=~/dograce:$PYTHONPATH
    python3 cyberdog_race/test_low_crawl.py --mode=real --duration=5

Optional flags:
    --mode      sim | real   default: real
    --duration  SECONDS      default: 5.0
    --speed     VX m/s       default: 0.10 (conservative for real robot)
    --height    METERS       default: 0.22 (normal standing height)
"""
import argparse
import sys
import os
import time


# Ensure parent directory is in path for imports
_parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)


def _spin_node(node):
    """Spin a rclpy node in a background thread (sim only)."""
    import rclpy
    try:
        rclpy.spin(node)
    except Exception:
        pass


def _init_perception(mode, rclpy_state):
    """Bring up ROS2Perception.

    Returns:
        (perception, spin_thread) where ``spin_thread`` may be ``None`` if
        rclpy was not started.  ``rclpy_state['started']`` is set to True
        whenever rclpy.init() was actually called.
    """
    import threading
    import rclpy
    from cyberdog_race.perception.ros2_perception import ROS2Perception

    if not rclpy.ok():
        rclpy.init()
    rclpy_state['started'] = True

    perception = ROS2Perception()
    print(f"[Test] Perception module: {ROS2Perception.__module__}")
    print(f"[Test] Mode: {mode}")

    if mode == 'real':
        # Real robot has no Gazebo.  The robot must be placed at the
        # course origin (see migration guide section 7).
        perception.set_position_offset(0.0, 0.0, 0.0)
        print("[Test] Real mode: Gazebo calibration skipped, "
              "position offset forced to (0,0,0)")
        print("[Test] Real mode: ensure the robot is placed at the course "
              "start before this script runs")

    print("[Test] Waiting for sensors...")
    ready = perception.wait_until_ready(timeout_s=25.0)
    status = perception.sensor_status()
    if not ready and status.get('odom', False):
        print(f"[Test] Sensor readiness partial: {status}")
        print("[Test] Continuing with odom; scan/rgb-dependent behavior "
              "may degrade")
        ready = True
    if not ready:
        print(f"[Test] Sensor readiness timeout: {status}")
        return None, None

    print(f"[Test] Sensors ready: {status}")

    spin_thread = threading.Thread(
        target=_spin_node, args=(perception,), daemon=True)
    spin_thread.start()
    return perception, spin_thread


def main():
    parser = argparse.ArgumentParser(
        description="Walk forward on a CyberDog in normal posture (sim/real)")
    parser.add_argument(
        '--mode', default='real', choices=['sim', 'real'],
        help='Run against the simulator or the real robot (default: real)')
    parser.add_argument(
        '--duration', type=float, default=5.0,
        help='Walk duration in seconds (default: 5.0)')
    parser.add_argument(
        '--speed', type=float, default=0.10,
        help='Forward speed in m/s (default: 0.10, conservative for real)')
    parser.add_argument(
        '--height', type=float, default=0.22,
        help='Body height during the walk in meters (default: 0.22, normal)')
    parser.add_argument(
        '--skip-perception', action='store_true',
        help='Skip ROS2 perception entirely (LCM-only, useful if rclpy / '
             'sensors are unavailable on the target)')
    args = parser.parse_args()

    # ---- LCM Controller ----
    from cyberdog_race.lcm_controller import LCMController
    from cyberdog_race.lcm_types import GAIT_TROT_SLOW

    ctrl = LCMController()
    ctrl.start()
    print("[Test] LCM controller started")

    # ---- Perception (real or sim) ----
    perception = None
    rclpy_state = {'started': False}

    if not args.skip_perception:
        try:
            perception, _ = _init_perception(args.mode, rclpy_state)
        except ImportError as e:
            print(f"[Test] ROS2 perception unavailable ({e}); falling back "
                  "to LCM-only mode")
            perception = None
        except Exception as e:
            print(f"[Test] Perception init failed ({e}); falling back to "
                  "LCM-only mode")
            perception = None

    if perception is None:
        # LCM-only path: do not require /odom or any sensor.
        # No position/heading printouts -- controller feedback only.
        print("[Test] Perception disabled.  Walk will run blind "
              "(only LCM feedback will be printed).")

    # ---- Stand up ----
    print("[Test] Standing up...")
    ctrl.stand_up_and_wait(height=args.height, wait_s=2.0)
    time.sleep(1.0)

    mode_resp, gait_resp, bar_resp = ctrl.get_response()
    print(f"[Test] After stand: resp=(mode={mode_resp}, "
          f"gait={gait_resp}, bar={bar_resp})")

    # ---- Walk forward for the requested duration ----
    print(f"[Test] ===== Walk Parameters =====")
    print(f"[Test] Mode         : {args.mode}")
    print(f"[Test] Duration     : {args.duration:.2f}s")
    print(f"[Test] Body height  : {args.height:.3f}m (normal)")
    print(f"[Test] Forward speed: {args.speed:.3f} m/s")
    print(f"[Test] Gait         : GAIT_TROT_SLOW ({GAIT_TROT_SLOW})")
    print(f"[Test] =============================")

    # Single continuous locomotion command. duration_ms=0 means "keep going
    # until I tell you to stop".  No body_pitch override -- normal posture.
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=args.speed, vy=0.0, wz=0.0,
        step_h_max=0.06, step_h_min=0.06,
        body_height=args.height,
        duration_ms=0)

    walk_start = time.perf_counter()
    last_status_print = 0.0
    last_pos_print = 0.0
    step_count = 0

    try:
        while time.perf_counter() - walk_start < args.duration:
            step_count += 1
            elapsed = time.perf_counter() - walk_start

            # Periodic status print (controller feedback only)
            if elapsed - last_status_print > 1.0:
                mode_fb, gait_fb, bar_fb = ctrl.get_response()
                print(f"[Test] t={elapsed:.2f}s tick={step_count} "
                      f"resp=(mode={mode_fb}, gait={gait_fb}, bar={bar_fb})")
                last_status_print = elapsed

            # Optional odometry print every 2s, only if perception is alive
            if (perception is not None
                    and elapsed - last_pos_print > 2.0):
                try:
                    px, py = perception.position
                    heading = perception.heading
                    print(f"[Test] t={elapsed:.2f}s odom=({px:.3f}, {py:.3f}) "
                          f"heading={heading:.1f} deg")
                except Exception:
                    pass
                last_pos_print = elapsed

            time.sleep(0.1)

        # ---- Stop ----
        print("[Test] Stopping...")
        ctrl.stop_moving(body_height=args.height)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("[Test] Interrupted by user")
        try:
            ctrl.stop_moving(body_height=args.height)
        except Exception:
            pass

    finally:
        elapsed_total = time.perf_counter() - walk_start
        try:
            ctrl.shutdown()
        except Exception:
            pass
        if rclpy_state['started']:
            try:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
        print(f"[Test] Done. Walked for {elapsed_total:.2f}s "
              f"({step_count} control ticks).")


if __name__ == '__main__':
    main()