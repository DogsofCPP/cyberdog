"""
Navigation helpers for hardcoded coordinate-based waypoint navigation.
Inspired by the 初赛 demo.py approach: wait for robot to reach a waypoint,
then turn to a heading, then proceed.
"""
import math
import time
import numpy as np


# ------------------------------------------------------------------
# Tolerance constants (can be overridden from config)
# ------------------------------------------------------------------
WAYPOINT_TOL_M = 0.10      # how close to a waypoint is "close enough"
HEADING_TOL_DEG = 5.0      # how close to a heading is "close enough"
TURN_RATE = 0.8            # rad/s when turning in place

# ------------------------------------------------------------------
# LiDAR lateral correction parameters (for corridor/aisle centering)
# ------------------------------------------------------------------
LATERAL_GAIN     = 0.12   # P gain for lateral error correction
MAX_STRAFE       = 0.035  # max lateral speed (m/s)
LIDAR_LEFT_ANGLE_MIN = 0.35  # radians - left side detection range min
LIDAR_LEFT_ANGLE_MAX = 0.65  # radians - left side detection range max
LIDAR_RIGHT_ANGLE_MIN = -0.65  # radians - right side detection range min
LIDAR_RIGHT_ANGLE_MAX = -0.35  # radians - right side detection range max


# ------------------------------------------------------------------
# Core navigation primitives
# ------------------------------------------------------------------

def go_to_waypoint(ctrl, perception, waypoint_xy,
                    speed=0.20, tol_m=None, max_vx=None, max_vy=None, max_wz=None,
                    use_lidar_strafe=False, lateral_gain=None, max_strafe=None,
                    gait_id=None, step_h_max=None, step_h_min=None,
                    body_height=None, body_pitch=None):
    """
    Drive toward a waypoint (x, y) in world frame until within tolerance.

    Args:
        ctrl:        LCMController instance
        perception:  ROS2Perception / MockPerception with position & heading
        waypoint_xy: (target_x, target_y) in world frame
        speed:       forward speed m/s
        tol_m:       distance tolerance (default WAYPOINT_TOL_M)
        max_vx:      max forward speed override
        max_vy:      max lateral speed override
        max_wz:      max angular speed override
        use_lidar_strafe: enable LiDAR-assisted lateral centering (default False)
        lateral_gain: P gain for lateral correction (default LATERAL_GAIN)
        max_strafe:  max lateral speed for LiDAR correction (default MAX_STRAFE)
        gait_id:     optional gait id passed through to ctrl.locomotion
        step_h_max:  step height max override (default None = controller default)
        step_h_min:  step height min override (default None = controller default)
        body_height: body centroid height in meters during locomotion (default None = 0.22)
        body_pitch:  body pitch in radians during locomotion (negative = head down, default None = 0)

    Returns:
        True when waypoint reached, False if timeout / perception unavailable
    """
    if tol_m is None:
        tol_m = WAYPOINT_TOL_M
    if max_vx is None:
        max_vx = speed
    if max_vy is None:
        max_vy = speed * 0.6
    if max_wz is None:
        max_wz = 0.8
    if lateral_gain is None:
        lateral_gain = LATERAL_GAIN
    if max_strafe is None:
        max_strafe = MAX_STRAFE

    tol_sq = tol_m * tol_m

    while True:
        try:
            px, py = perception.position
            heading_deg = perception.heading
        except (AttributeError, TypeError):
            # Fallback to odom_pose if position/heading not available
            px, py, _ = perception.odom_pose
            heading_deg = perception.odom_pose[2] * 180.0 / math.pi % 360.0

        dx = waypoint_xy[0] - px
        dy = waypoint_xy[1] - py
        dist_sq = dx * dx + dy * dy

        if dist_sq < tol_sq:
            ctrl.stop_moving()
            return True

        # Compute lateral correction if enabled
        strafe = 0.0
        if use_lidar_strafe:
            strafe = lidar_corridor_strafe(
                perception, lateral_gain=lateral_gain, max_strafe=max_strafe)

        _navigate_toward(ctrl, px, py, heading_deg, dx, dy,
                         max_vx, max_vy, max_wz,
                         extra_vy=strafe,
                         gait_id=gait_id,
                         step_h_max=step_h_max, step_h_min=step_h_min,
                         body_height=body_height, body_pitch=body_pitch)
        time.sleep(0.05)


def turn_to_angle(ctrl, perception, target_deg,
                   turn_rate=None, tol_deg=None, timeout_s=30.0):
    """
    Rotate in place until robot heading matches target_deg.

    Args:
        ctrl:        LCMController instance
        perception:  ROS2Perception / MockPerception with heading
        target_deg:  target heading in degrees [0, 360)
        turn_rate:   angular rate rad/s (default TURN_RATE)
        tol_deg:     heading tolerance degrees (default HEADING_TOL_DEG)
        timeout_s:   give up after this long

    Returns:
        True when heading reached, False on timeout
    """
    if turn_rate is None:
        turn_rate = TURN_RATE
    if tol_deg is None:
        tol_deg = HEADING_TOL_DEG

    start = time.perf_counter()
    while time.perf_counter() - start < timeout_s:
        try:
            heading_deg = perception.heading
        except (AttributeError, TypeError):
            heading_deg = perception.odom_pose[2] * 180.0 / math.pi % 360.0

        diff = _angle_diff(target_deg, heading_deg)

        if abs(diff) <= tol_deg:
            ctrl.stop_moving()
            return True

        # Choose shortest rotation direction
        wz = turn_rate if diff > 0 else -turn_rate
        ctrl.locomotion(vx=0.0, vy=0.0, wz=wz)
        time.sleep(0.05)

    ctrl.stop_moving()
    return False


def turn_to_angle_on_slope(ctrl, perception, target_deg,
                           turn_rate=None, tol_deg=None, timeout_s=30.0,
                           body_height=0.18, lateral_offset=0.04,
                           roll_gain=1.2, roll_clamp_rad=0.20, roll_update_hz=12.0):
    """
    Rotate in place on a tilted surface while maintaining stability.

    Stability strategy:
    - Lower body center-of-mass by reducing body_height
    - Shift COM toward downhill side via lateral vy offset
    - Compensate measured roll via gait steps
    - Use slower turn rate than flat-ground rotation

    Args:
        ctrl:           LCMController instance
        perception:     ROS2Perception / MockPerception with heading and roll
        target_deg:     target heading in degrees [0, 360)
        turn_rate:      angular rate rad/s (default 0.4 rad/s, slower than flat)
        tol_deg:        heading tolerance degrees (default 5.0)
        timeout_s:      give up after this long
        body_height:    body height during rotation (default 0.18 m)
        lateral_offset: vy offset toward downhill side (default 0.04 m)
        roll_gain:      P gain for roll compensation
        roll_clamp_rad: max roll compensation angle
        roll_update_hz: how fast to update roll compensation

    Returns:
        True when heading reached, False on timeout
    """
    if turn_rate is None:
        turn_rate = 0.4
    if tol_deg is None:
        tol_deg = HEADING_TOL_DEG

    ctrl.set_height(body_height, duration_ms=300)
    time.sleep(0.4)

    start = time.perf_counter()
    last_pose_update = 0.0

    while time.perf_counter() - start < timeout_s:
        try:
            heading_deg = perception.heading
        except (AttributeError, TypeError):
            heading_deg = perception.odom_pose[2] * 180.0 / math.pi % 360.0

        diff = _angle_diff(target_deg, heading_deg)

        if abs(diff) <= tol_deg:
            ctrl.stop_moving()
            return True

        # ---- Roll compensation (every cycle) ----
        now = time.perf_counter()
        if now - last_pose_update > 1.0 / max(1e-3, roll_update_hz):
            roll = _get_measured_roll(perception)
            cmd_roll = _clamp(-roll * roll_gain, -roll_clamp_rad, roll_clamp_rad)
            _apply_roll_via_gait_steps(ctrl, cmd_roll)
            last_pose_update = now

        # Choose shortest rotation direction, slower on slope
        wz = turn_rate if diff > 0 else -turn_rate
        ctrl.locomotion(
            gait_id=27,  # GAIT_TROT_SLOW
            vx=0.0,
            vy=lateral_offset,   # shift COM toward downhill side
            wz=wz,
            step_h_max=0.03,
            step_h_min=0.02,
            body_height=body_height,
            duration_ms=0,
        )
        time.sleep(0.05)

    ctrl.stop_moving()
    return False


def _get_measured_roll(perception, safe_roll=0.0):
    try:
        rpy = getattr(perception, "rpy", None)
        if rpy is not None:
            return float(rpy[0])
        odom_pose = getattr(perception, "odom_pose", None)
        if odom_pose is not None and len(odom_pose) >= 6:
            return float(odom_pose[3])
    except Exception:
        pass
    return safe_roll


def _apply_roll_via_gait_steps(ctrl, roll_rad):
    steps = _get_stabilize_steps(roll_rad=roll_rad)
    try:
        ctrl.execute_gait_steps(steps)
    except Exception:
        pass


def _get_stabilize_steps(roll_rad=0.0):
    template = [
        [ 0.00, -0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.00],
        [ 0.00,  0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.00],
        [ 0.06,  0.00, 0.00,  0.00, 0.000, 0.0, 1, 0.05],
        [ 0.00,  0.00, 0.00,  0.00, 0.000, 0.0, 1, 0.00],
        [ 0.06, -0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.05],
        [ 0.06,  0.08, 0.00,  0.00, 0.000, 0.0, 1, 0.05],
    ]
    steps = []
    for s in template:
        steps.append([s[0], s[1], s[2], roll_rad, s[4], s[5], s[6], s[7]])
    return steps


def go_to_waypoint_on_slope(ctrl, perception, target_xy,
                            speed=0.06, tol_m=None,
                            body_height=0.10, body_pitch=-0.14,
                            lateral_offset=-0.04,
                            roll_gain=1.2, roll_clamp_rad=0.20,
                            roll_update_hz=12.0, step_h=0.015,
                            timeout_s=55.0):
    """
    Navigate toward a world-frame waypoint on a tilted surface using low-crawl posture.

    Combines go_to_waypoint direction calculation with segment_4 low-crawl stability:
    - Low body (height=0.10, pitch=-0.14) for slope stability
    - Slow speed (0.06 m/s) for controlled movement
    - Lateral vy offset toward downhill side for COM balance
    - Roll compensation via gait steps

    Args:
        ctrl:            LCMController instance
        perception:      ROS2Perception / MockPerception
        target_xy:       (x, y) in world frame
        speed:           forward speed m/s (default 0.06)
        tol_m:           distance tolerance (default WAYPOINT_TOL_M)
        body_height:     body height during tilt (default 0.10 m)
        body_pitch:      body pitch during tilt (default -0.14 rad, nose-down)
        lateral_offset:  vy toward downhill side (default -0.04 m, left = downhill)
        roll_gain:       P gain for roll compensation
        roll_clamp_rad:  max roll compensation angle
        roll_update_hz:  roll compensation update rate
        step_h:          step height during tilt
        timeout_s:       max duration

    Returns:
        True when waypoint reached, False on timeout
    """
    if tol_m is None:
        tol_m = WAYPOINT_TOL_M

    start_t = time.perf_counter()
    last_pose_update = 0.0
    last_move_update = 0.0
    last_debug_t = start_t

    while time.perf_counter() - start_t < timeout_s:
        px, py = _safe_position_xy(perception)
        heading_deg = _safe_heading_deg(perception)
        dx = target_xy[0] - px
        dy = target_xy[1] - py
        dist = math.hypot(dx, dy)
        now = time.perf_counter()

        if now - last_debug_t >= 0.5:
            roll = _get_measured_roll(perception)
            print(f"[SlopeNav] pos=({px:.3f},{py:.3f}) heading={heading_deg:.1f}deg "
                  f"dist={dist:.3f}m roll={roll:.2f}rad")
            last_debug_t = now

        if dist <= tol_m:
            return True

        # ---- Roll compensation (every cycle) ----
        if now - last_pose_update > 1.0 / max(1e-3, roll_update_hz):
            roll = _get_measured_roll(perception)
            cmd_roll = _clamp(-roll * roll_gain, -roll_clamp_rad, roll_clamp_rad)
            _apply_roll_via_gait_steps(ctrl, cmd_roll)

            if hasattr(ctrl, 'set_body_pose'):
                ctrl.set_body_pose(height_m=body_height,
                                   pitch_rad=body_pitch,
                                   duration_ms=120)
            last_pose_update = now

        # ---- Navigation (every 0.4s) ----
        if now - last_move_update > 0.40:
            _navigate_toward(
                ctrl, px, py, heading_deg, dx, dy,
                speed, lateral_offset, 0.0,     # vx=speed, vy=lateral_offset, wz=0
                step_h_max=step_h, step_h_min=step_h,
                body_height=body_height,
                body_pitch=body_pitch,
            )
            last_move_update = now

        time.sleep(0.1)

    print(f"[SlopeNav] timeout at dist={dist:.3f}m")
    return False


def go_to_waypoint_then_turn(ctrl, perception, waypoint_xy, target_heading_deg,
                               speed=0.20, waypoint_tol_m=None,
                               heading_tol_deg=None, turn_rate=None,
                               waypoint_timeout_s=60.0, turn_timeout_s=30.0):
    """
    Combine go_to_waypoint and turn_to_angle into one call.

    Returns (waypoint_reached, heading_reached).
    """
    reached = go_to_waypoint(
        ctrl, perception, waypoint_xy,
        speed=speed, tol_m=waypoint_tol_m,
        max_wz=turn_rate if turn_rate else None,
        max_vx=speed,
    )
    turned = turn_to_angle(
        ctrl, perception, target_heading_deg,
        turn_rate=turn_rate, tol_deg=heading_tol_deg,
        timeout_s=turn_timeout_s,
    )
    return reached, turned


# ------------------------------------------------------------------
# Cardinal-direction navigation (四方形移动)
# ------------------------------------------------------------------

CARDINAL_HEADINGS = {0: 0.0, 90: math.pi / 2.0, 180: math.pi, 270: -math.pi / 2.0}


def go_to_point_with_heading(ctrl, perception, target_x, target_y, target_heading_deg,
                              pos_tol=0.20, heading_tol=15.0, timeout_s=30.0,
                              forward_speed=0.15, turn_wz=0.5,
                              use_lidar_strafe=False, lateral_gain=None, max_strafe=None):
    """
    Navigate to a target point and then face a specific heading.

    Phase 1: Turn to face the target direction.
    Phase 2: Move straight toward the target (approach angle re-computed each step).
    Phase 3: Turn to the final target heading.

    Args:
        ctrl:              LCM controller instance
        perception:        Perception object with position and heading
        target_x, target_y: Target world coordinates
        target_heading_deg: Final heading in degrees (0=+X, 90=+Y, 180=-X, 270=-Y)
        pos_tol:           Position tolerance in meters
        heading_tol:       Heading tolerance in degrees
        timeout_s:         Maximum total time
        forward_speed:     Forward speed during movement phase
        turn_wz:           Turn rate during heading adjustments
        use_lidar_strafe: enable LiDAR-assisted lateral centering (default False)
        lateral_gain: P gain for lateral correction (default LATERAL_GAIN)
        max_strafe:  max lateral speed for LiDAR correction (default MAX_STRAFE)

    Returns:
        True  - reached target position and final heading within tolerances
        False - timeout before arriving
    """
    if lateral_gain is None:
        lateral_gain = LATERAL_GAIN
    if max_strafe is None:
        max_strafe = MAX_STRAFE

    start_time = time.perf_counter()
    seg_start = start_time

    # Phase 1: turn to face the target direction
    print(f"[go_to_point_with_heading] Phase 1: turn to approach angle for ({target_x}, {target_y})")
    while time.perf_counter() - start_time < timeout_s:
        px, py = perception.position
        dx = target_x - px
        dy = target_y - py
        approach_rad = math.atan2(dy, dx)
        approach_deg = math.degrees(approach_rad)

        _, _, yaw = perception.odom_pose
        yaw_err = _wrap_angle(yaw - approach_rad)
        if abs(yaw_err) < math.radians(heading_tol):
            print(f"[go_to_point_with_heading] Phase 1 done, facing {math.degrees(yaw):.1f} deg")
            ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
            break
        wz = turn_wz if yaw_err > 0 else -turn_wz
        ctrl.locomotion(vx=0.0, vy=0.0, wz=wz)
        time.sleep(0.05)
    else:
        ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
        print(f"[go_to_point_with_heading] Phase 1 timeout")
        return False

    # Phase 2: move straight toward the target, re-compute approach angle each step
    print(f"[go_to_point_with_heading] Phase 2: move to ({target_x}, {target_y})")
    debug_count = 0
    while time.perf_counter() - start_time < timeout_s:
        px, py = perception.position
        dx = target_x - px
        dy = target_y - py
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < pos_tol:
            ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
            elapsed = time.perf_counter() - seg_start
            print(f"[go_to_point_with_heading] Phase 2 reached ({px:.3f}, {py:.3f}), dist={dist:.3f}m, heading={perception.heading:.1f} deg, elapsed={elapsed:.1f}s")
            break

        # Recompute approach angle from current position (dynamic correction)
        approach_rad = math.atan2(dy, dx)
        _, _, yaw = perception.odom_pose
        yaw_err = _wrap_angle(yaw - approach_rad)
        steer = _clamp(-yaw_err * 0.6, -0.12, 0.12)
        vx = forward_speed

        # LiDAR lateral correction
        strafe = 0.0
        if use_lidar_strafe:
            strafe = lidar_corridor_strafe(perception, lateral_gain=lateral_gain, max_strafe=max_strafe)

        ctrl.locomotion(vx=vx, vy=strafe, wz=steer)
        time.sleep(0.05)

        debug_count += 1
        if debug_count % 40 == 0:
            elapsed = time.perf_counter() - seg_start
            print(f"[go_to_point_with_heading] Phase 2: current=({px:.3f}, {py:.3f}), dist={dist:.3f}m, heading={perception.heading:.1f} deg, elapsed={elapsed:.1f}s")
    else:
        ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
        px, py = perception.position
        print(f"[go_to_point_with_heading] Phase 2 timeout at ({px:.3f}, {py:.3f})")
        return False

    # Phase 3: turn to the final target heading
    print(f"[go_to_point_with_heading] Phase 3: turn to final heading {target_heading_deg} deg")
    final_reached = turn_to_angle(
        ctrl, perception, target_heading_deg,
        turn_rate=turn_wz, tol_deg=heading_tol, timeout_s=timeout_s)

    px, py = perception.position
    print(f"[go_to_point_with_heading] Done at ({px:.3f}, {py:.3f}), heading={perception.heading:.1f} deg")
    return final_reached


def go_to_point_cardinal(ctrl, perception, target_x, target_y,
                          pos_tol=0.20, heading_tol=15.0, timeout_s=30.0,
                          forward_speed=0.15, turn_wz=0.5,
                          use_lidar_strafe=False, lateral_gain=None, max_strafe=None):
    """
    Navigate to a target point using only cardinal directions (四方形移动).

    The approach direction is snapped to the nearest cardinal axis.
    Phase 1: turn to face that cardinal direction.
    Phase 2: move straight, re-computing approach angle each step.
    Phase 3: turn to face the cardinal that best points toward the target.

    Args:
        ctrl:              LCM controller instance
        perception:        Perception object with position and heading
        target_x, target_y: Target world coordinates
        pos_tol:           Position tolerance in meters
        heading_tol:        Heading tolerance in degrees
        timeout_s:          Maximum total time
        forward_speed:      Forward speed during movement phase
        turn_wz:           Turn rate during heading adjustments
        use_lidar_strafe: enable LiDAR-assisted lateral centering (default False)
        lateral_gain: P gain for lateral correction (default LATERAL_GAIN)
        max_strafe:  max lateral speed for LiDAR correction (default MAX_STRAFE)

    Returns:
        True  - reached target position within tolerance
        False - timeout before arriving
    """
    if lateral_gain is None:
        lateral_gain = LATERAL_GAIN
    if max_strafe is None:
        max_strafe = MAX_STRAFE

    start_time = time.perf_counter()
    seg_start = start_time

    px, py = perception.position
    dx = target_x - px
    dy = target_y - py
    raw_angle = math.degrees(math.atan2(dy, dx))
    approach_deg = _snap_to_cardinal(raw_angle)
    approach_rad = math.radians(approach_deg)
    print(f"[go_to_point_cardinal] Target=({target_x}, {target_y}), "
          f"raw={raw_angle:.1f} deg -> cardinal={approach_deg:.1f} deg")

    # Phase 1: turn to face the snapped cardinal direction
    print(f"[go_to_point_cardinal] Phase 1: turn to {approach_deg:.1f} deg")
    while time.perf_counter() - start_time < timeout_s:
        _, _, yaw = perception.odom_pose
        yaw_err = _wrap_angle(yaw - approach_rad)
        if abs(yaw_err) < math.radians(heading_tol):
            print(f"[go_to_point_cardinal] Phase 1 done, facing {math.degrees(yaw):.1f} deg")
            ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
            break
        wz = turn_wz if yaw_err > 0 else -turn_wz
        ctrl.locomotion(vx=0.0, vy=0.0, wz=wz)
        time.sleep(0.05)
    else:
        ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
        print(f"[go_to_point_cardinal] Phase 1 timeout")
        return False

    # Phase 2: move straight toward the target, re-compute approach angle each step
    print(f"[go_to_point_cardinal] Phase 2: move to ({target_x}, {target_y})")
    debug_count = 0
    while time.perf_counter() - start_time < timeout_s:
        px, py = perception.position
        dx = target_x - px
        dy = target_y - py
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < pos_tol:
            ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
            elapsed = time.perf_counter() - seg_start
            print(f"[go_to_point_cardinal] Phase 2 reached ({px:.3f}, {py:.3f}), dist={dist:.3f}m, heading={perception.heading:.1f} deg, elapsed={elapsed:.1f}s")
            break

        # Recompute approach angle from current position
        approach_rad = math.atan2(dy, dx)
        _, _, yaw = perception.odom_pose
        yaw_err = _wrap_angle(yaw - approach_rad)
        steer = _clamp(-yaw_err * 0.6, -0.12, 0.12)
        vx = forward_speed

        # LiDAR lateral correction
        strafe = 0.0
        if use_lidar_strafe:
            strafe = lidar_corridor_strafe(perception, lateral_gain=lateral_gain, max_strafe=max_strafe)

        ctrl.locomotion(vx=vx, vy=strafe, wz=steer)
        time.sleep(0.05)

        debug_count += 1
        if debug_count % 40 == 0:
            elapsed = time.perf_counter() - seg_start
            print(f"[go_to_point_cardinal] Phase 2: current=({px:.3f}, {py:.3f}), dist={dist:.3f}m, heading={perception.heading:.1f} deg, elapsed={elapsed:.1f}s")
    else:
        ctrl.locomotion(vx=0.0, vy=0.0, wz=0.0)
        px, py = perception.position
        print(f"[go_to_point_cardinal] Phase 2 timeout at ({px:.3f}, {py:.3f})")
        return False

    # Phase 3: turn to face the cardinal that points toward the target
    dx = target_x - px
    dy = target_y - py
    if abs(dx) >= abs(dy):
        final_deg = 0.0 if dx >= 0 else 180.0
    else:
        final_deg = 90.0 if dy >= 0 else 270.0
    final_deg = _snap_to_cardinal(final_deg)
    print(f"[go_to_point_cardinal] Phase 3: turn to {final_deg:.1f} deg for next leg")

    turn_to_angle(ctrl, perception, final_deg,
                  turn_rate=turn_wz, tol_deg=heading_tol, timeout_s=timeout_s)

    px, py = perception.position
    print(f"[go_to_point_cardinal] Done at ({px:.3f}, {py:.3f}), heading={perception.heading:.1f} deg")
    return True


def _snap_to_cardinal(angle_deg):
    """Snap an angle in degrees to the nearest cardinal direction (0/90/180/270)."""
    candidates = {0.0, 90.0, 180.0, 270.0}
    angle_deg = angle_deg % 360.0
    best = min(candidates, key=lambda c: _angular_distance(angle_deg, c))
    return best


def _angular_distance(a, b):
    """Minimum angular distance between two angles in degrees."""
    diff = abs((a - b + 180.0) % 360.0 - 180.0)
    return diff


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------

def _navigate_toward(ctrl, px, py, heading_deg, dx, dy,
                     max_vx, max_vy, max_wz, extra_vy=0.0,
                     gait_id=None, step_h_max=None, step_h_min=None,
                     body_height=None, body_pitch=None):
    """
    Send a locomotion command to approach (dx, dy) from current position.
    Pure proportional control on heading error.

    Args:
        extra_vy: additional lateral velocity from LiDAR correction
        gait_id: optional gait id passed through to ctrl.locomotion
        step_h_max: step height max override (None = controller default)
        step_h_min: step height min override (None = controller default)
        body_height: body centroid height override in meters (None = default ~0.22)
    """
    heading_rad = math.radians(heading_deg)

    # Rotate error into body frame
    cos_h = math.cos(-heading_rad)
    sin_h = math.sin(-heading_rad)
    bx = dx * cos_h - dy * sin_h
    by = dx * sin_h + dy * cos_h

    # Proportional heading correction
    heading_error = math.atan2(by, max(0.05, bx))
    wz = _clamp(heading_error * 2.0, -max_wz, max_wz)

    # Forward speed proportional to distance
    dist = math.sqrt(dx * dx + dy * dy)
    vx = _clamp(dist * 0.8, 0.05, max_vx)

    # Lateral correction (odometry-based + LiDAR correction)
    vy = _clamp(by * 1.0 + extra_vy, -max_vy, max_vy)

    kwargs = {
        "vx": vx,
        "vy": vy,
        "wz": wz,
        "step_h_max": step_h_max,
        "step_h_min": step_h_min,
        "body_height": body_height,
        "body_pitch": body_pitch,
    }
    if gait_id is not None:
        kwargs["gait_id"] = gait_id
    ctrl.locomotion(**kwargs)


def _angle_diff(target, current):
    """Shortest signed angular difference target - current in degrees.
    
    Returns positive for counter-clockwise rotation, negative for clockwise.
    Always returns value in range (-180, 180].
    """
    diff = (target - current + 180.0) % 360.0 - 180.0
    return diff


def _wrap_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ------------------------------------------------------------------
# LiDAR-assisted lateral correction (通用化自 segment_1 的 corridor_strafe)
# ------------------------------------------------------------------

def lidar_corridor_strafe(perception, lateral_gain=None, max_strafe=None):
    """
    Light lateral centering using LiDAR (not for physical walls).
    Uses left/right side detection to compute lateral error and correct it.

    Args:
        perception:  ROS2Perception / MockPerception with LiDAR data
        lateral_gain: P gain for lateral correction (default LATERAL_GAIN)
        max_strafe:  max lateral speed (default MAX_STRAFE)

    Returns:
        Lateral velocity command (positive = strafe right, negative = strafe left)
    """
    if lateral_gain is None:
        lateral_gain = LATERAL_GAIN
    if max_strafe is None:
        max_strafe = MAX_STRAFE

    ranges = perception.get_lidar_ranges()
    angles = perception.get_lidar_angles()
    if len(ranges) == 0:
        return 0.0

    valid = np.isfinite(ranges)

    # Left side: angles between 0.35π and 0.65π (roughly 63° to 117°)
    left_ranges = ranges[(angles > math.pi * LIDAR_LEFT_ANGLE_MIN) &
                        (angles < math.pi * LIDAR_LEFT_ANGLE_MAX) & valid]

    # Right side: angles between -0.65π and -0.35π (roughly -117° to -63°)
    right_ranges = ranges[(angles < -math.pi * LIDAR_LEFT_ANGLE_MIN) &
                          (angles > -math.pi * LIDAR_LEFT_ANGLE_MAX) & valid]

    left_mean  = _mean_safe_np(left_ranges)
    right_mean = _mean_safe_np(right_ranges)

    if left_mean is None or right_mean is None:
        return 0.0

    # Positive lateral_error means robot is left of center (right side is closer)
    # We want to strafe toward center: if right < left, move left (negative)
    lateral_error = right_mean - left_mean
    strafe = _clamp(-lateral_error * lateral_gain, -max_strafe, max_strafe)

    return strafe


def _mean_safe_np(arr):
    """Safe mean computation that returns None for empty arrays."""
    if len(arr) == 0:
        return None
    return float(np.mean(arr))
