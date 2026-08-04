"""Test Low-Crawl Gait Directly.

This script tests the low-crawl gait for height-bar passages.
It focuses solely on the body lowering and forward crawling motion.

Usage:
    cd /home
    source /opt/ros/galactic/setup.bash
    source /home/cyberdog_ws/install/setup.bash
    PYTHONPATH=/home:$PYTHONPATH python3 cyberdog_race/test_low_crawl.py --mode=sim

    # For real robot:
    python3 test_low_crawl.py --mode=real --duration=10
"""
import argparse
import sys
import os
import time

# Ensure parent directory is in path for imports
_parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)


def main():
    parser = argparse.ArgumentParser(description="Test Low-Crawl Gait")
    parser.add_argument(
        '--mode', default='sim',
        choices=['sim', 'real'],
        help='Run in simulation or real mode')
    parser.add_argument(
        '--duration', type=float, default=5.0,
        help='Duration to crawl in seconds (default: 5.0)')
    parser.add_argument(
        '--distance', type=float, default=1.0,
        help='Target distance to crawl in meters (default: 1.0)')
    parser.add_argument(
        '--height', type=float, default=0.10,
        help='Target body height in meters (default: 0.10)')
    parser.add_argument(
        '--pitch', type=float, default=-0.25,
        help='Body pitch angle in radians (default: -0.25, more negative=head lower)')
    parser.add_argument(
        '--speed', type=float, default=0.10,
        help='Forward speed in m/s (default: 0.10)')
    args = parser.parse_args()

    # ---- LCM Controller ----
    from cyberdog_race.lcm_controller import LCMController
    ctrl = LCMController()
    ctrl.start()
    print("[Test] LCM controller started")

    # ---- ROS2 Perception (simulation only) ----
    perception = None
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
    time.sleep(1.0)

    # ---- Pre-lower body ----
    print(f"[Test] Pre-lowering body to {args.height}m...")
    ctrl.set_height(args.height, duration_ms=500)
    time.sleep(0.8)

    # ---- Upload low-crawl gait ----
    print("[Test] Uploading low-crawl gait...")
    from cyberdog_race.navigation.segments.segment_4 import (
        _upload_low_crawl_gait,
        LOW_CRAWL_STEPS,
        _LOW_CRAWL_GAIT_UPLOADED,
    )

    # Reset the uploaded flag to force re-upload
    import cyberdog_race.navigation.segments.segment_4 as seg4
    seg4._LOW_CRAWL_GAIT_UPLOADED = False
    _upload_low_crawl_gait(ctrl)
    seg4._LOW_CRAWL_GAIT_UPLOADED = True

    # ---- Execute low-crawl gait ----
    print(f"[Test] Starting low-crawl for {args.duration}s (target: {args.distance}m)...")

    start_pos = None
    if perception:
        try:
            start_pos = perception.position[:2]
            print(f"[Test] Start position: {start_pos}")
        except:
            pass

    # ---- Send initial low-height posture before starting gait loop ----
    # Test: First set height, then use look_forward for pitch control
    from cyberdog_race.lcm_types import GAIT_TROT_SLOW
    print(f"[Test] Sending initial posture: height={args.height}, pitch={args.pitch} rad")
    
    # Set body height first
    ctrl.set_height(args.height, duration_ms=500)
    time.sleep(0.6)
    
    # Use look_forward for better pitch control (uses MODE_POSITION_INTERP)
    ctrl.look_forward(args.pitch, duration_ms=500)
    time.sleep(0.6)
    
    # Then send continuous locomotion command
    print(f"[Test] Sending continuous locomotion command...")
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=args.speed, vy=0.0, wz=0.0,
        step_h_max=0.018, step_h_min=0.018,
        body_height=args.height,
        body_pitch=args.pitch,
        duration_ms=0)
    time.sleep(0.3)

    step_count = 0
    crawl_start = time.perf_counter()
    last_pitch_update = 0
    last_locomotion_update = 0
    print("[Test] ===== Low-Crawl Parameters =====")
    print(f"[Test] Target body height: {args.height}m")
    print(f"[Test] Target pitch: {args.pitch} rad ({args.pitch*180/3.14:.1f} deg)")
    print(f"[Test] Step height: 0.018m (18mm), forward speed: {args.speed} m/s")
    print("[Test] Using continuous locomotion + periodic pitch updates")
    print("[Test] ===============================")
    
    while time.perf_counter() - crawl_start < args.duration:
        step_count += 1
        elapsed = time.perf_counter() - crawl_start
        
        # Re-send locomotion command every 0.5s to maintain forward motion + low posture
        if elapsed - last_locomotion_update > 0.5:
            ctrl.locomotion(
                gait_id=GAIT_TROT_SLOW,
                vx=args.speed, vy=0.0, wz=0.0,
                step_h_max=0.018, step_h_min=0.018,
                body_height=args.height,
                body_pitch=args.pitch,
                duration_ms=0)
            last_locomotion_update = elapsed
        
        # Re-send pitch command every 0.3s to maintain angle
        if elapsed - last_pitch_update > 0.3:
            ctrl.look_forward(args.pitch, duration_ms=200)
            last_pitch_update = elapsed
        
        time.sleep(0.1)

        # Print real-time parameters
        try:
            pos = perception.position[:2]
            heading = perception.heading
            body_height = perception.body_height
            body_pitch = perception.body_pitch
            mode, gait, bar = ctrl.get_response()
            elapsed = time.perf_counter() - crawl_start
            
            if start_pos:
                dist = ((pos[0] - start_pos[0])**2 + (pos[1] - start_pos[1])**2)**0.5
            else:
                dist = 0.0
            
            # Format output
            height_str = f"{body_height:.3f}m" if body_height is not None else "N/A"
            pitch_str = f"{body_pitch:.1f}°" if body_pitch is not None else "N/A"
            
            # Debug info
            debug_info = []
            odom_full = getattr(perception, '_odom_full', None)
            imu_data = getattr(perception, 'imu_data', None)
            if odom_full:
                debug_info.append(f"odom_z={odom_full[2]:.3f}")
            if imu_data:
                debug_info.append(f"imu_quat=({imu_data.orientation.x:.2f},{imu_data.orientation.y:.2f},{imu_data.orientation.z:.2f},{imu_data.orientation.w:.2f})")
            
            debug_str = " [" + ", ".join(debug_info) + "]" if debug_info else ""
            
            print(f"[Test] Step {step_count}: pos=({pos[0]:.2f}, {pos[1]:.2f}), "
                  f"dist={dist:.2f}m, height={height_str}, pitch={pitch_str}{debug_str}")
        except Exception as e:
            print(f"[Test] Step {step_count} (error: {e})")

    # ---- Restore normal posture ----
    print("[Test] Restoring normal posture...")
    ctrl.set_height(0.22, duration_ms=500)
    time.sleep(0.8)

    # ---- Report results ----
    if perception and start_pos:
        try:
            end_pos = perception.position[:2]
            dist = ((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)**0.5
            elapsed = time.perf_counter() - crawl_start
            speed = dist / elapsed if elapsed > 0 else 0
            print(f"\n[Test] ===== Results =====")
            print(f"[Test] Start: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
            print(f"[Test] End:   ({end_pos[0]:.3f}, {end_pos[1]:.3f})")
            print(f"[Test] Distance: {dist:.3f}m")
            print(f"[Test] Duration: {elapsed:.2f}s")
            print(f"[Test] Avg speed: {speed:.3f}m/s")
            print(f"[Test] Steps: {step_count}")
            print(f"[Test] ====================\n")
        except:
            pass

    # ---- Cleanup ----
    ctrl.stop_moving()
    time.sleep(0.5)
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
