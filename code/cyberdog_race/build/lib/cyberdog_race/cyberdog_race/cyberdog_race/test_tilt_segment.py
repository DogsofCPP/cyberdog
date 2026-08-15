"""Test Tilt Segment (Left-High / Right-Low) Direction 180.

This script tests the lateral-tilt stability control for Segment 5 legs
on the left-high / right-low slope. Parameters mirror segment_5.py:
  - speed: 0.08 m/s
  - step height: (0.05, 0.03) m
  - body height: 0.18 m
  - lateral offset: +0.04 m (shift COM right = downhill)
  - roll compensation via gait steps

Usage:
    cd /home
    source /opt/ros/galactic/setup.bash
    source /home/cyberdog_ws/install/setup.bash
    PYTHONPATH=/home:$PYTHONPATH python3 cyberdog_race/test_tilt_segment.py --mode=sim --duration=20

    # Real robot:
    python3 cyberdog_race/test_tilt_segment.py --mode=real --duration=20
"""
import argparse
import sys
import os
import time
import math

# Ensure parent directory is in path for imports
_parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

# ---- Segment-5 tilt constants (mirrored) ----
SPEED_TILT = 0.08
STEP_H_TILT_MAX = 0.05
STEP_H_TILT_MIN = 0.03
BODY_HEIGHT_TILT = 0.18
TILT_LATERAL_OFFSET = -0.04  # strafe left toward downhill (left side is high)

# Roll compensation (same as segment_5)
ROLL_GAIN = 1.2
ROLL_CLAMP_RAD = 0.20
ROLL_UPDATE_HZ = 12.0
GAIT_TROT_SLOW = 27  # from cyberdog_race.lcm_types

# Stabilize gait steps: single short step with roll bias
_STABILIZE_STEPS_TEMPLATE = [
    # [px, py, pz, roll, pitch, yaw, contact, height]
    [ 0.00, -0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.00],
    [ 0.00,  0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.00],
    [ 0.06,  0.00, 0.00,  0.00, 0.000, 0.0, 1, 0.05],
    [ 0.00,  0.00, 0.00,  0.00, 0.000, 0.0, 1, 0.00],
    [ 0.06, -0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.05],
    [ 0.06,  0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.05],
]


def _get_measured_roll(perception, safe_roll=0.0):
    try:
        rpy = getattr(perception, "rpy", None)
        if rpy is not None:
            roll, _, _ = rpy
            return float(roll)

        odom_pose = getattr(perception, "odom_pose", None)
        if odom_pose is not None and len(odom_pose) >= 6:
            roll, _, _ = odom_pose[3:6]
            return float(roll)
    except Exception:
        pass
    return safe_roll


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def _get_stabilize_steps(roll_rad=0.0):
    steps = []
    for s in _STABILIZE_STEPS_TEMPLATE:
        steps.append([
            s[0], s[1], s[2],
            roll_rad,   # roll bias
            s[4], s[5], s[6], s[7]
        ])
    return steps


def _apply_roll_via_gait_steps(ctrl, roll_rad):
    steps = _get_stabilize_steps(roll_rad=roll_rad)
    try:
        ctrl.execute_gait_steps(steps)
    except Exception:
        pass


def _safe_position_xy(perception):
    try:
        return perception.position[0], perception.position[1]
    except Exception:
        return 0.0, 0.0


def _safe_heading_deg(perception):
    try:
        return float(perception.heading)
    except Exception:
        return 0.0


def main():
    parser = argparse.ArgumentParser(description="Test Tilt Segment (Direction 180)")
    parser.add_argument(
        '--mode', default='sim',
        choices=['sim', 'real'],
        help='Run in simulation or real mode')
    parser.add_argument(
        '--duration', type=float, default=20.0,
        help='Duration to run in seconds (default: 20.0)')
    parser.add_argument(
        '--distance', type=float, default=2.0,
        help='Target distance to travel in meters (default: 2.0)')
    args = parser.parse_args()

    # ---- LCM Controller ----
    from cyberdog_race.lcm_controller import LCMController
    ctrl = LCMController()
    ctrl.start()
    print("[Test] LCM controller started")

    # ---- ROS2 Perception (sim and real) ----
    # Per migration guide §4 / §7: real mode also uses ROS2Perception but
    # forces offset to (0,0,0).  MockPerception is no longer reachable in
    # any mode.
    perception = None
    rclpy_started = False
    if args.mode in ('sim', 'real'):
        import threading
        import rclpy
        from cyberdog_race.perception.ros2_perception import ROS2Perception

        if not rclpy.ok():
            rclpy.init()
        rclpy_started = True

        perception = ROS2Perception()
        print("[Test] Perception module: ROS2Perception")

        if args.mode == 'real':
            perception.set_position_offset(0.0, 0.0, 0.0)
            print("[Test] Real mode: position offset forced to (0,0,0)")
        else:
            print("[Test] Waiting for simulation sensors...")

        ready = perception.wait_until_ready(timeout_s=25.0)
        status = perception.sensor_status()
        if not ready and status.get('scan', False) and status.get('odom', False):
            print(f"[Test] Sensor readiness partial: {status}")
            ready = True
        if not ready:
            print(f"[Test] Sensor readiness timeout: {status}")
            ctrl.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
            return
        print(f"[Test] Sensors ready: {status}")

        spin_thread = threading.Thread(
            target=_spin_node, args=(perception,), daemon=True)
        spin_thread.start()
    else:
        # No other modes are supported.
        print(f"[Test] Unknown mode: {args.mode}")
        ctrl.shutdown()
        return

    # ---- Stand up ----
    print("[Test] Standing up...")
    ctrl.stand_up_and_wait(height=0.22, wait_s=2.0)
    time.sleep(1.0)

    # ---- Rotate to heading 180 deg on slope ----
    print("[Test] Rotating to heading=180 deg (slope-aware rotation)...")
    from cyberdog_race.utils.nav_helper import turn_to_angle_on_slope
    ctrl.set_height(BODY_HEIGHT_TILT, duration_ms=300)
    time.sleep(0.4)
    reached = turn_to_angle_on_slope(
        ctrl, perception, target_deg=180.0,
        turn_rate=0.4,
        tol_deg=5.0,
        timeout_s=30.0,
        body_height=BODY_HEIGHT_TILT,
        lateral_offset=TILT_LATERAL_OFFSET,
        roll_gain=ROLL_GAIN,
        roll_clamp_rad=ROLL_CLAMP_RAD,
        roll_update_hz=ROLL_UPDATE_HZ,
    )
    if not reached:
        print("[Test] WARNING: failed to reach heading 180 deg, continuing anyway...")
    current_heading = _safe_heading_deg(perception)
    print(f"[Test] Heading after rotation: {current_heading:.1f} deg")

    # ---- Start tilt locomotion ----
    print(f"[Test] Starting tilt test at heading=180 deg")
    print(f"[Test] Parameters: speed={SPEED_TILT} m/s, "
          f"step=({STEP_H_TILT_MAX},{STEP_H_TILT_MIN}) m, "
          f"body_h={BODY_HEIGHT_TILT} m, vy_offset={TILT_LATERAL_OFFSET} m")

    start_pos = None
    start_heading = None
    if perception:
        try:
            start_pos = perception.position[:2]
            start_heading = perception.heading
            print(f"[Test] Start position: ({start_pos[0]:.3f}, {start_pos[1]:.3f}), "
                  f"heading: {start_heading:.1f} deg")
        except Exception as e:
            print(f"[Test] Could not read initial pose: {e}")

    # Track state
    last_pose_update = 0.0
    last_locomotion_update = 0.0
    last_debug_t = time.perf_counter()
    last_dist = None
    step_count = 0
    crawl_start = time.perf_counter()

    print("[Test] ===== Tilt Test Parameters =====")
    print(f"[Test] Speed: {SPEED_TILT} m/s")
    print(f"[Test] Step height: ({STEP_H_TILT_MAX}, {STEP_H_TILT_MIN}) m")
    print(f"[Test] Body height: {BODY_HEIGHT_TILT} m")
    print(f"[Test] Lateral offset (vy): {TILT_LATERAL_OFFSET} m (left = downhill, left is high)")
    print(f"[Test] Roll gain: {ROLL_GAIN}, clamp: +/-{ROLL_CLAMP_RAD} rad")
    print(f"[Test] Roll update rate: {ROLL_UPDATE_HZ} Hz")
    print("[Test] ================================")

    while time.perf_counter() - crawl_start < args.duration:
        step_count += 1
        now = time.perf_counter()
        elapsed = now - crawl_start

        # ---- Roll compensation ----
        if now - last_pose_update > 1.0 / max(1e-3, ROLL_UPDATE_HZ):
            measured_roll = _get_measured_roll(perception)
            cmd_roll = _clamp(-measured_roll * ROLL_GAIN, -ROLL_CLAMP_RAD, ROLL_CLAMP_RAD)
            _apply_roll_via_gait_steps(ctrl, cmd_roll)
            last_pose_update = now

        # ---- Locomotion command (every 0.5s) ----
        if elapsed - last_locomotion_update > 0.5:
            ctrl.locomotion(
                gait_id=GAIT_TROT_SLOW,
                vx=-SPEED_TILT,              # 180 deg: negative X = forward in heading frame
                vy=TILT_LATERAL_OFFSET,     # shift COM left = downhill (left is high)
                wz=0.0,
                step_h_max=STEP_H_TILT_MAX,
                step_h_min=STEP_H_TILT_MIN,
                body_height=BODY_HEIGHT_TILT,
                duration_ms=0,
            )
            last_locomotion_update = elapsed

        time.sleep(0.05)

        # ---- Debug output (every 0.5s) ----
        if now - last_debug_t >= 0.5:
            try:
                pos = perception.position[:2]
                heading = _safe_heading_deg(perception)
                body_height = getattr(perception, 'body_height', None)
                measured_roll = _get_measured_roll(perception)

                if start_pos:
                    dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
                else:
                    dist = 0.0

                height_str = f"{body_height:.3f}m" if body_height is not None else "N/A"
                roll_str = f"{measured_roll:.1f}°" if measured_roll is not None else "N/A"

                print(f"[Test] Step {step_count}: pos=({pos[0]:.2f},{pos[1]:.2f}) "
                      f"dist={dist:.2f}m heading={heading:.1f}deg "
                      f"height={height_str} roll={roll_str}")

                # Check progress
                if last_dist is not None and dist < last_dist - 0.05:
                    print(f"[Test] WARNING: regressing! dist={dist:.3f}m (prev={last_dist:.3f}m)")
                last_dist = dist

            except Exception as e:
                print(f"[Test] Step {step_count} (read error: {e})")
            last_debug_t = now

        # ---- Early stop if target distance reached ----
        if start_pos and perception:
            try:
                pos = perception.position[:2]
                dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
                if dist >= args.distance:
                    print(f"[Test] Target distance {args.distance}m reached, stopping.")
                    break
            except Exception:
                pass

    # ---- Restore normal posture ----
    print("[Test] Restoring normal posture...")
    ctrl.stop_moving()
    ctrl.set_height(0.22, duration_ms=500)
    time.sleep(0.8)

    # ---- Report results ----
    if perception and start_pos:
        try:
            end_pos = perception.position[:2]
            end_heading = _safe_heading_deg(perception)
            dist = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])
            elapsed = time.perf_counter() - crawl_start
            avg_speed = dist / elapsed if elapsed > 0 else 0

            print(f"\n[Test] ===== Results =====")
            print(f"[Test] Start: ({start_pos[0]:.3f}, {start_pos[1]:.3f}), "
                  f"heading: {start_heading:.1f} deg")
            print(f"[Test] End:   ({end_pos[0]:.3f}, {end_pos[1]:.3f}), "
                  f"heading: {end_heading:.1f} deg")
            print(f"[Test] Distance: {dist:.3f}m")
            print(f"[Test] Duration: {elapsed:.2f}s")
            print(f"[Test] Avg speed: {avg_speed:.3f}m/s")
            print(f"[Test] Steps: {step_count}")
            print(f"[Test] ====================\n")
        except Exception as e:
            print(f"[Test] Could not generate report: {e}")

    # ---- Cleanup ----
    ctrl.stop_moving()
    time.sleep(0.5)
    ctrl.shutdown()
    if rclpy_started:
        import rclpy as _rclpy
        if _rclpy.ok():
            _rclpy.shutdown()
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
