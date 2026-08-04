"""Test vy sign on flat ground.

Goal:
  - Determine whether positive vy makes the robot strafe left or right.

This test does NOT navigate, does NOT turn, and does NOT use slope logic.
It simply commands pure lateral motion twice: +vy then -vy, and prints the
observed position delta.

Usage (sim):
  cd /home
  source /opt/ros/galactic/setup.bash
  source /home/cyberdog_ws/install/setup.bash
  PYTHONPATH=/home:$PYTHONPATH python3 /home/cyberdog_race/test_vy.py --mode=sim --vy=0.03 --hold=2.0

Usage (real):
  PYTHONPATH=/home:$PYTHONPATH python3 /home/cyberdog_race/test_vy.py --mode=real --vy=0.03 --hold=2.0
"""

import argparse
import math
import os
import sys
import time

# Ensure parent directory is in path for imports
_parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)


def _safe_position_xy(perception):
    try:
        px, py = perception.position
        return float(px), float(py)
    except Exception:
        try:
            px, py, _ = perception.odom_pose
            return float(px), float(py)
        except Exception:
            return None


def _spin_node(node):
    import rclpy
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description="Test vy sign on flat ground")
    parser.add_argument('--mode', default='sim', choices=['sim', 'real'])
    parser.add_argument('--vy', type=float, default=0.03, help='lateral speed command (m/s)')
    parser.add_argument('--hold', type=float, default=2.0, help='hold time per command (s)')
    parser.add_argument('--settle', type=float, default=1.0, help='settle time after standup (s)')
    parser.add_argument('--gait', type=int, default=27, help='gait_id (default GAIT_TROT_SLOW=27)')
    parser.add_argument('--body-height', type=float, default=0.22, help='body height during test (m)')
    parser.add_argument('--step-h', type=float, default=0.03, help='step height max/min (m)')
    args = parser.parse_args()

    from cyberdog_race.lcm_controller import LCMController

    ctrl = LCMController()
    ctrl.start()
    print('[TestVy] LCM controller started')

    perception = None
    spin_thread = None
    if args.mode == 'sim':
        import threading
        import rclpy
        from cyberdog_race.perception.ros2_perception import ROS2Perception

        rclpy.init()
        perception = ROS2Perception()
        print('[TestVy] Perception module: ROS2Perception')
        print('[TestVy] Waiting for simulation sensors...')
        ready = perception.wait_until_ready(timeout_s=25.0)
        status = perception.sensor_status()
        if not ready and status.get('odom', False):
            ready = True
        if not ready:
            print(f'[TestVy] Sensor readiness timeout: {status}')
            ctrl.shutdown()
            rclpy.shutdown()
            return
        print(f'[TestVy] Sensors ready: {status}')

        spin_thread = threading.Thread(target=_spin_node, args=(perception,), daemon=True)
        spin_thread.start()
    else:
        from cyberdog_race.utils.mock_perception import MockPerception

        perception = MockPerception()
        print('[TestVy] Perception module: MockPerception')

    print('[TestVy] Standing up...')
    ctrl.stand_up_and_wait(height=args.body_height, wait_s=2.0)
    time.sleep(max(0.0, args.settle))

    p0 = _safe_position_xy(perception)
    if p0 is None:
        print('[TestVy] Warning: cannot read position; will run open-loop only.')
    else:
        print(f'[TestVy] Start pos=({p0[0]:.3f},{p0[1]:.3f})')

    def _run_once(vy_cmd, label):
        print(f'[TestVy] Command {label}: vx=0 vy={vy_cmd:+.3f} for {args.hold:.1f}s')
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < args.hold:
            ctrl.locomotion(
                gait_id=int(args.gait),
                vx=0.0,
                vy=float(vy_cmd),
                wz=0.0,
                step_h_max=float(args.step_h),
                step_h_min=float(args.step_h),
                body_height=float(args.body_height),
                duration_ms=0,
            )
            time.sleep(0.08)
        ctrl.stop_moving(body_height=float(args.body_height))
        time.sleep(0.5)

    _run_once(+abs(args.vy), label='+vy')
    p_plus = _safe_position_xy(perception)
    if p_plus is not None and p0 is not None:
        dx = p_plus[0] - p0[0]
        dy = p_plus[1] - p0[1]
        print(f'[TestVy] After +vy pos=({p_plus[0]:.3f},{p_plus[1]:.3f}) Δ=({dx:+.3f},{dy:+.3f})')

    _run_once(-abs(args.vy), label='-vy')
    p_minus = _safe_position_xy(perception)
    if p_minus is not None and p_plus is not None:
        dx = p_minus[0] - p_plus[0]
        dy = p_minus[1] - p_plus[1]
        print(f'[TestVy] After -vy pos=({p_minus[0]:.3f},{p_minus[1]:.3f}) Δ=({dx:+.3f},{dy:+.3f})')

    print('[TestVy] Done, restoring posture...')
    ctrl.stop_moving(body_height=float(args.body_height))
    time.sleep(0.2)
    ctrl.set_height(0.22, duration_ms=500)
    time.sleep(0.8)

    ctrl.shutdown()
    if args.mode == 'sim':
        import rclpy
        rclpy.shutdown()


if __name__ == '__main__':
    main()
