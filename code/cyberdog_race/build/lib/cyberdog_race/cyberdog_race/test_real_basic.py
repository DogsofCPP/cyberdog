"""Minimal motion test for the real CyberDog.

Reference: ``real_robot_migration_guide.md`` §9

This script tests the bare-minimum LCM control chain on a physical robot:
    1. Stand up
    2. Slow forward walk
    3. Stop
    4. Pure damper (lie down)

It intentionally does NOT bring up ROS2 perception (see §9 of the migration
guide: verify basic motion BEFORE adding perception).

If this script is unstable, do NOT proceed to segment tests.

Usage (real robot):
    source /opt/ros/galactic/setup.bash
    export PYTHONPATH=~/dograce:$PYTHONPATH
    python3 cyberdog_race/test_real_basic.py

Usage (sim):
    source /opt/ros/galactic/setup.bash
    python3 cyberdog_race/test_real_basic.py --mode=sim

Optional flags:
    --mode      sim | real   default: real
    --speed     VX m/s       default: 0.10 (conservative, per §11)
    --walk-s    SECONDS      forward walk duration; default: 2.0
"""
import argparse
import sys
import os
import time

_parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)


def main():
    parser = argparse.ArgumentParser(
        description="Minimal motion test for the real CyberDog")
    parser.add_argument(
        '--mode', default='real', choices=['sim', 'real'],
        help='Run against simulator or real robot (default: real)')
    parser.add_argument(
        '--speed', type=float, default=0.10,
        help='Forward speed in m/s (default: 0.10, conservative per §11)')
    parser.add_argument(
        '--walk-s', type=float, default=2.0,
        help='Forward walk duration in seconds (default: 2.0)')
    args = parser.parse_args()

    from cyberdog_race.lcm_controller import LCMController
    from cyberdog_race.lcm_types import GAIT_TROT_SLOW

    ctrl = LCMController()
    ctrl.start()
    print("[Test] LCM controller started")
    print(f"[Test] Mode: {args.mode}, speed={args.speed} m/s")

    try:
        # ---- Step 1: Stand up ----
        print("\n=== [1/4] Standing up ===")
        ok = ctrl.recovery_stand()
        mode, gait, bar = ctrl.get_response()
        print(f"[1/4] recovery_stand ok={ok}  resp=(mode={mode}, gait={gait}, bar={bar})")
        if not ok:
            print("[1/4] WARNING: recovery_stand did not acknowledge")
        time.sleep(1.5)

        mode, gait, bar = ctrl.get_response()
        print(f"[1/4] After settle: resp=(mode={mode}, gait={gait}, bar={bar})")

        # ---- Step 2: Slow forward walk ----
        print(f"\n=== [2/4] Walking forward ({args.walk_s}s @ {args.speed} m/s) ===")
        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=args.speed, vy=0.0, wz=0.0,
            step_h_max=0.06, step_h_min=0.06,
            body_height=0.22,
            duration_ms=0)

        walk_start = time.perf_counter()
        last_print = 0.0
        while time.perf_counter() - walk_start < args.walk_s:
            elapsed = time.perf_counter() - walk_start
            if elapsed - last_print > 0.5:
                mode, gait, bar = ctrl.get_response()
                print(f"[2/4] t={elapsed:.1f}s  resp=(mode={mode}, gait={gait}, bar={bar})")
                last_print = elapsed
            time.sleep(0.1)

        # ---- Step 3: Stop ----
        print("\n=== [3/4] Stopping ===")
        ctrl.stop_moving(body_height=0.22)
        mode, gait, bar = ctrl.get_response()
        print(f"[3/4] After stop: resp=(mode={mode}, gait={gait}, bar={bar})")
        time.sleep(0.5)

        # ---- Step 4: Pure damper (lie down) ----
        print("\n=== [4/4] Pure damper (lie down) ===")
        ok = ctrl.pure_damper()
        mode, gait, bar = ctrl.get_response()
        print(f"[4/4] pure_damper ok={ok}  resp=(mode={mode}, gait={gait}, bar={bar})")
        time.sleep(1.0)

        print("\n[OK] All 4 steps completed.")

    except KeyboardInterrupt:
        print("\n[Interrupted] Sending stop then damper...")
        try:
            ctrl.stop_moving(body_height=0.22)
        except Exception:
            pass
        try:
            ctrl.pure_damper()
        except Exception:
            pass

    finally:
        elapsed = time.perf_counter()
        ctrl.shutdown()
        print(f"[Done] Shutdown complete.")


if __name__ == '__main__':
    main()