"""
Segment 1: Stone Path (石径探路)

Global world-frame coordinate navigation using the course_map and Navigator.
Robot starts at (0, 0) facing -X (backwards). Crosses stone slabs backwards to the S1-S2 connector,
then turns and enters S2.

World-frame waypoints (from course_map.S1_WAYPOINTS):
  (2.7,  0.0,  180.0) - end of stone slab straight (facing -X, walking backwards)
  (2.7,  0.0, 90.0) - turn to face +Y (S2 direction)
  (3.0,  0.0, 90.0) - S2 entry

The Navigator provides:
  - World-frame go-to with proportional heading correction
  - Stuck detection and recovery (backup + rotate)
  - Deviation detection and plan recovery
"""
import time
import math
from ...lcm_types import GAIT_TROT_SLOW
from ...utils.nav_helper import turn_to_angle, lidar_corridor_strafe
from .. import course_map


SEGMENT_TIMEOUT_S = 120.0

# Stone slab crossing parameters (backwards walking)
WALK_SPEED        = -0.20   # Negative for backwards walking
TURN_WZ          = 0.34
MAX_STEER        = 0.16
MAX_STRAFE       = 0.035
YAW_GAIN         = 0.90
LATERAL_GAIN     = 0.12

STONE_STEP_H_MAX = 0.45
STONE_STEP_H_MIN = 0.40


def execute(ctrl, perception, navigator):
    """
    Execute segment 1 using global world-frame coordinate navigation (backwards).

    Args:
        ctrl:        LCMController instance
        perception:  ROS2Perception / MockPerception
        navigator:    Navigator instance for world-frame reactive navigation

    Phase plan:
      1. Turn to face -X (180 degrees) to prepare for backwards walking
      2. Walk backwards across stones to end of slab area at (2.7, 0.0)
      3. Continue along -X to the S2 entry x position at (3.0, 0.0)
      4. Turn left 90° to face +Y (S2 entry direction)
    """
    print("[Seg1] Starting Stone Path (backwards, global world-frame navigation)")
    print(f"[Seg1] Start position={perception.position}, heading={perception.heading:.1f} deg")
    print(f"[Seg1] Walking backwards from (0.0, 0.0) to (2.7, 0.0)")
    print(f"[Seg1] Waypoints: {course_map.S1_WAYPOINTS}")

    seg_start = time.perf_counter()

    # Phase 0: Turn to face -X (180 degrees) for backwards walking
    target_heading_backward = 180.0
    current_heading = perception.heading
    print(f"[Seg1] Phase 0: turning to face -X (backwards) at {target_heading_backward:.1f} deg (from {current_heading:.1f})")
    if not turn_to_angle(ctrl, perception, target_heading_backward, tol_deg=6.0, timeout_s=30.0):
        print("[Seg1] Turn to backwards timeout, continuing anyway")
    ctrl.stop_moving()
    time.sleep(0.3)

    # Phase 1: walk backwards across stones to end of slab area
    # Navigator drives toward (2.7, 0.0) with heading correction + LiDAR centering
    # Target heading: 180° (facing -X, moving in +X direction while facing backwards)
    print(f"[Seg1] Phase 1: navigating backwards to stone end at (2.7, 0.0)")
    debug_count = 0
    while time.perf_counter() - seg_start < SEGMENT_TIMEOUT_S:
        px, py = perception.position
        # Check if X >= 2.7 (has passed the stone area backwards), allow generous Y tolerance
        if _reached_waypoint_xy(perception, (2.7, 0.0), tol_x=0.10, tol_y=0.60):
            print(f"[Seg1] Stone area cleared at {perception.position}")
            break
        _walk_backwards_with_stone_correction(ctrl, perception)
        time.sleep(0.05)
        # Debug every 2 seconds
        debug_count += 1
        if debug_count % 40 == 0:
            dist = math.sqrt((2.7 - px)**2 + (0.0 - py)**2)
            print(f"[Seg1] Current: ({px:.3f}, {py:.3f}), dist_to_waypoint={dist:.3f}m, heading={perception.heading:.1f}°")
    else:
        print("[Seg1] Timeout on phase 1")
        ctrl.stop_moving()
        return True

    # Phase 2: continue along -X (backwards) to S2 entry before turning.
    print(f"[Seg1] Phase 2: navigating backwards to S2 entry x at (3.0, 0.0)")
    s2_start = time.perf_counter()
    while time.perf_counter() - s2_start < 12.0:
        px, py = perception.position
        if _reached_waypoint_xy(perception, (3.0, 0.0), tol_x=0.12, tol_y=0.45):
            print(f"[Seg1] S2 entry x reached at {perception.position}")
            ctrl.stop_moving()
            break
        # Walk backwards with heading correction while still facing -X.
        # Target heading is 180° (facing -X)
        _, _, yaw = perception.odom_pose
        yaw_error = _wrap_angle(yaw - math.pi)  # Compare to 180 degrees (pi radians)
        steer = _clamp(-yaw_error * 0.6, -0.12, 0.12)
        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=-0.12, vy=0.0, wz=steer,  # Negative vx for backwards
            step_h_max=STONE_STEP_H_MAX, step_h_min=STONE_STEP_H_MIN,
            duration_ms=0,
        )
        time.sleep(0.05)
    else:
        print("[Seg1] Phase 2 timeout, turning to S2 direction anyway")
        ctrl.stop_moving()

    # Phase 3: turn left 90° to face +Y (S2 direction)
    target_heading = 90.0
    current_heading = perception.heading
    print(f"[Seg1] Phase 3: turning to heading {target_heading:.1f} deg (from {current_heading:.1f})")
    if not turn_to_angle(ctrl, perception, target_heading, tol_deg=6.0, timeout_s=30.0):
        print("[Seg1] Turn timeout, continuing anyway")
    ctrl.stop_moving()
    return True


# ------------------------------------------------------------------
# Navigation primitives (preserved from original for stone correction)
# ------------------------------------------------------------------

def _reached_waypoint(perception, waypoint_xy, tol=0.15):
    """Return True if robot is within tol meters of waypoint (world frame)."""
    return _reached_waypoint_xy(perception, waypoint_xy, tol_x=tol, tol_y=tol)


def _reached_waypoint_xy(perception, waypoint_xy, tol_x=0.15, tol_y=0.15):
    """Return True if robot is within tol of waypoint, with separate X/Y tolerances."""
    px, py = perception.position
    dx = abs(waypoint_xy[0] - px)
    dy = abs(waypoint_xy[1] - py)
    return dx < tol_x and dy < tol_y


def _walk_backwards_with_stone_correction(ctrl, perception):
    """
    Walk backwards with proportional heading correction + LiDAR-assisted
    lateral centering (keeps robot between stone slabs).

    Target heading: 180° (facing -X direction).
    The robot moves in +X direction while facing backwards (-X).
    """
    rx, ry, yaw = perception.odom_pose
    # Target heading is 180° (pi radians), compare current yaw to target
    yaw_error = _wrap_angle(yaw - math.pi)
    steer = _clamp(-yaw_error * YAW_GAIN, -MAX_STEER, MAX_STEER)
    strafe = lidar_corridor_strafe(perception, lateral_gain=LATERAL_GAIN, max_strafe=MAX_STRAFE)
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=WALK_SPEED, vy=strafe, wz=steer,  # WALK_SPEED is negative (-0.20) for backwards
        step_h_max=STONE_STEP_H_MAX, step_h_min=STONE_STEP_H_MIN,
        duration_ms=0,
    )


def _walk_with_stone_correction(ctrl, perception):
    """
    Walk forward with proportional heading correction + LiDAR-assisted
    lateral centering (keeps robot between stone slabs).
    """
    rx, ry, yaw = perception.odom_pose
    yaw_error = _wrap_angle(yaw - 0.0)
    steer = _clamp(-yaw_error * YAW_GAIN, -MAX_STEER, MAX_STEER)
    strafe = lidar_corridor_strafe(perception, lateral_gain=LATERAL_GAIN, max_strafe=MAX_STRAFE)
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=WALK_SPEED, vy=strafe, wz=steer,
        step_h_max=STONE_STEP_H_MAX, step_h_min=STONE_STEP_H_MIN,
        duration_ms=0,
    )


def _walk_forward_light(ctrl, perception):
    """Walk straight with light heading correction."""
    _, _, yaw = perception.odom_pose
    yaw_error = _wrap_angle(yaw - 0.0)
    steer = _clamp(-yaw_error * 0.5, -0.10, 0.10)
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=0.12, vy=0.0, wz=steer,
        step_h_max=STONE_STEP_H_MAX, step_h_min=STONE_STEP_H_MIN,
        duration_ms=0,
    )


# ------------------------------------------------------------------
# Navigation helpers
# ------------------------------------------------------------------


def _wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))
