"""\
Segment 5: Five-Part Ramp/Tilt/Flat + Jump (五段坡/斜坡/平路+跳下)

Track description (world frame):
  1) Uphill (yaw=90) from (3.0, 7.7) to (3.0, 12.35), then left turn to yaw=180
     Height ramps from h=5cm to h=20cm
  2) Lateral tilt: left higher than right (left 20cm, right 10cm, width 50cm)
     Move (yaw=180) from (3.0, 12.35) to (-0.3, 12.35), then right turn to yaw=90
  3) Same lateral tilt
     Move (yaw=90) from (-0.3, 12.35) to (-0.3, 15.3), then right turn to yaw=0
  4) Same lateral tilt
     Move (yaw=0) from (-0.3, 15.3) to (3.0, 15.3), then right turn to yaw=270 (-90)
  5) Flat
     Move (yaw=270) from (3.0, 15.3) to (3.0, 13.44), then right turn to yaw=180
     Jump down, landing near (2.4, 13.44) facing yaw=180

Control strategy:
  - Uses world-coordinate waypoint navigation (go_to_waypoint + turn_to_angle).
  - Uphill segment ramps body height gradually.
  - Lateral-tilt segments apply IMU roll compensation (negate measured roll, clamp).
  - Safety: per-segment timeout and no-progress detection.

Notes:
  - "h" in the description is interpreted as commanded body height (m).
  - If the controller does not support direct body pose roll commands, roll
    compensation is applied via small USER_DEFINED gait steps (gait_id=110).
"""

import math
import time

from ...lcm_types import (
    robot_control_cmd_lcmt,
    GAIT_TROT_SLOW,
    MODE_LOCOMOTION,
)
from ...utils.nav_helper import go_to_waypoint, turn_to_angle, turn_to_angle_on_slope


def _safe_heading_deg(perception):
    try:
        return float(perception.heading)
    except (AttributeError, TypeError, ValueError):
        try:
            return float(perception.odom_pose[2]) * 180.0 / math.pi % 360.0
        except Exception:
            return 0.0


# -----------------------------------------------------------------------------
# Tunables
# -----------------------------------------------------------------------------
SEGMENT_TIMEOUT_S = 420.0  # Counted after reaching Segment-5 start pose.

# Waypoint tolerances
POS_TOL_M = 0.15
TILT_POS_TOL_M = 0.25  # looser tolerance on cross-slope (tilt) legs

# Fine approach tolerance for leg5 (bridge end): keep X tighter so we actually reach 3.05
LEG5_FINE_TOL_M = 0.08

# Uphill leg acceptance: require tight Y alignment to avoid stopping short.
UPHILL_Y_TOL_M = 0.04
UPHILL_SAFE_TURN_MARGIN_M = 0.10
UPHILL_SAFE_TURN_EPS_M = 0.015

TURN_TOL_DEG = 3.0
TURN_RATE = 0.5

# Speeds
SPEED_UPHILL = 0.18  # Increased for faster uphill traversal
SPEED_TILT = 0.14  # faster on cross-slope; rely on vy/roll control to stay on track
SPEED_FLAT = 0.22
SPEED_POST_JUMP_CORRECT = 0.12

# Step heights
STEP_H_UPHILL = (0.22, 0.15)  # Increased for better step clearance (max=0.22m, min=0.15m)
STEP_H_TILT = (0.045, 0.030)  # larger stride on cross-slope (tune to avoid slip)
STEP_H_FLAT = (0.06, 0.03)

# Body heights
H_UPHILL_START_M = 0.22  # Normal height for smooth transitions
H_UPHILL_END_M = 0.22    # Normal height to avoid crouch on stop

# Pitch control (nose-down "俯冲" for step clearance)
PITCH_START_RAD = -0.12   # nose-down at approach (rad)
PITCH_END_RAD   =  0.05   # slight nose-up at top for stability
PITCH_TRANSITION_RATIO = 0.35  # when to start returning pitch (35% of distance)

# Tilt walking with roll+vy compensation.
TILT_BODY_HEIGHT_M = 0.22
TILT_BODY_PITCH_RAD = -0.06
TILT_HIGH_SIDE_WORLD_Y_MPS = 0.0
TILT_HIGH_SIDE_WORLD_X_MPS = 0.0
TILT_VY_COMP_MPS = 0.070
TILT_LINE_KP = 0.45
TILT_VY_MAX_MPS = 0.24
TILT_VY_MIN_CORRECT_MPS = 0.0
TILT_LINE_DEADBAND_M = 0.08
TILT_HIGH_SIDE_OFFSET_M = 0.0
TILT_ROLL_VY_KP = 0.18
TILT_ROLL_DEADBAND_RAD = 0.015
TILT_ROLL_SIGN = 1.0
TILT_ROLL_BIAS_RAD = -0.25
# Tilt heading correction: keep this small on cross-slope to avoid side-slip
TILT_WZ_MAX = 0.22

# Tilt turning strategy
# Prefer curved arc turning to keep a forward support polygon.
TILT_TURN_USE_LEGACY_LOW_CRAWL = False
TILT_TURN_LEGACY_VY_MPS = 0.06
TILT_TURN_LEGACY_FWD_MPS = 0.010

ROLL_GAIN = 2.4  # stronger roll compensation
ROLL_CLAMP_RAD = 0.60  # allow larger lean command on steep cross-slope
ROLL_UPDATE_HZ = 12.0  # Increased for faster roll updates

# Curved turn parameters (forward arc turn for slope stability)
# Instead of spinning in place, use forward motion + yaw to maintain support polygon
# Turn radius: vx/wz ratio for ~0.5m turn radius
CURVED_TURN_VX_DEFAULT = 0.06    # forward speed
CURVED_TURN_WZ_DEFAULT = 0.15    # yaw rate rad/s (~10.5s for 90°, turn radius ~0.4m)
CURVED_TURN_RPY = [0.0, 0.0, 0.0]  # baseline posture
CURVED_TURN_POS_DES = [0.0, 0.0, 0.22]  # normal body height
CURVED_TURN_STEP_H = [0.06, 0.03]   # normal step height

# Legacy low-crawl settings for the cross-slope bridge sections.
LEGACY_TILT_BODY_Z = -0.035
LEGACY_TILT_STEP_H = (0.025, 0.018)
LEGACY_TURN_RATE = 0.45
LEGACY_TURN_FORWARD = 0.015
LEGACY_FIRST_TURN_FORWARD = 0.0
LEGACY_FIRST_TURN_VY = 0.0
LEGACY_FIRST_TURN_ROLL = 0.0

# Turn roll compensation (for cross-slope / early tilt)
TURN_ROLL_GAIN = 3.0
TURN_ROLL_CLAMP_RAD = 0.55

# Speed degradation table for protection fallback
# (trigger_count -> (vx, wz, body_z))
PROTECTION_FALLBACK = [
    (0.05, 0.25, 0.12),   # 0 triggers: default
    (0.03, 0.18, 0.11),   # 1 trigger: slower
    (0.02, 0.12, 0.10),   # 2 triggers: even slower
]


# -----------------------------------------------------------------------------
# Track definition
# -----------------------------------------------------------------------------
S5_START = (3.1, 7.2, 90.0)
S5_LEGS = [
    # (target_x, target_y, after_turn_yaw_deg, kind)
    # 1) 到(3.0,12.35)后左转 -> 180
    (3.1, 12.25, 180.0, "uphill"),
    # 2) 到(-0.3,12.35)后右转 -> 90
    (-0.0, 12.30, 90.0, "tilt"),
    # 3) 到(-0.3,15.3)后右转 -> 0
    (-0.4, 15.0, 10.0, "tilt"),
    # 4) 到(3.0,15.3)后右转 -> 270(-90)
    (3.1, 15.3, 270.0, "tilt"),
    # 5) 到(3.0,13.44)后右转 -> 180 (随后跳下)
    (3.0, 13.5, 180.0, "flat"),
]

JUMP_LAND_XY = (2.4, 13.44)
JUMP_LAND_YAW = 180.0
SEG5_IMPL_VERSION = "seg5_tilt_trot_v3"


def execute(ctrl, perception, navigator=None):
    print(f"[Seg5] Starting Segment 5: 5-part ramp/tilt/flat + jump ({SEG5_IMPL_VERSION})")
    _log_pose(perception, prefix="[Seg5] initial")

    seg_start = time.perf_counter()
    next_pose_log_t = seg_start + 0.5

    # First, navigate to S5_START position and heading
    sx, sy, start_yaw = S5_START
    print(f"[Seg5] Moving to start position ({sx:.2f}, {sy:.2f})")

    # Move to start with debug logging every 0.5s
    # Use longer timeout since distance from origin is ~7.8m
    start_nav_t = time.perf_counter()
    last_debug_t = start_nav_t
    while True:
        px, py = _safe_position_xy(perception)
        dist = math.hypot(sx - px, sy - py)
        now = time.perf_counter()
        if now - last_debug_t >= 0.5:
            heading = _safe_heading_deg(perception)
            print(f"[Seg5] pos=({px:.3f},{py:.3f}) heading={heading:.1f}deg dist={dist:.3f}m")
            last_debug_t = now

        # Check if we're close enough (use generous tolerance for start)
        if dist <= 0.30:
            break

        # Timeout after 60s for the long journey from origin
        if now - start_nav_t > 60.0:
            print(f"[Seg5] Start position timeout at pos=({px:.3f},{py:.3f})")
            break

        # Use flat leg with longer timeout
        _run_flat_leg(ctrl, perception, (sx, sy), speed=0.15, tol_m=0.30, timeout_s=55.0)

    _log_pose(perception, prefix="[Seg5] start pos reached")

    # Turn to the expected start heading (yaw=90 for uphill)
    turn_to_angle(ctrl, perception, start_yaw,
                  turn_rate=TURN_RATE, tol_deg=TURN_TOL_DEG, timeout_s=15.0)
    _log_pose(perception, prefix="[Seg5] start heading set")
    seg_start = time.perf_counter()
    next_pose_log_t = seg_start + 0.5

    for idx, (tx, ty, after_yaw, kind) in enumerate(S5_LEGS, start=1):
        now_global = time.perf_counter()
        if now_global >= next_pose_log_t:
            _log_pose(perception, prefix="[Seg5] pose")
            next_pose_log_t += 0.5

        if now_global - seg_start > SEGMENT_TIMEOUT_S:
            print(f"[Seg5] Timeout before leg {idx}")
            ctrl.stop_moving(body_height=0.22)
            return True

        print(f"[Seg5] Leg {idx}/5 kind={kind} -> target=({tx:.2f},{ty:.2f}), after_yaw={after_yaw:.1f}")

        if kind == "uphill":
            ok = _run_uphill_leg(ctrl, perception, (tx, ty),
                                 height_start=H_UPHILL_START_M,
                                 height_end=H_UPHILL_END_M,
                                 speed=SPEED_UPHILL,
                                 tol_m=POS_TOL_M,
                                 timeout_s=None,
                                 keep_x=3.0,
                                 y_tol_m=UPHILL_Y_TOL_M)  # Keep X near bridge centerline
        elif kind == "tilt":
            ok = _run_lateral_tilt_leg(ctrl, perception, (tx, ty),
                                       speed=SPEED_TILT,
                                       tol_m=TILT_POS_TOL_M,
                                       timeout_s=360.0,
                                       vy_extra_bias=(0.02 if idx == 4 else 0.0))
        else:
            ok = _run_flat_leg(ctrl, perception, (tx, ty),
                               speed=SPEED_FLAT,
                               tol_m=(LEG5_FINE_TOL_M if idx == 5 else POS_TOL_M),
                               timeout_s=45.0)

        _log_pose(perception, prefix=f"[Seg5] leg{idx} reached")

        if not ok:
            print(f"[Seg5] Warning: leg {idx} not confirmed reached")
            if idx == 1:
                print("[Seg5] Retrying uphill crest approach before turning")
                ok = _run_uphill_leg(ctrl, perception, (tx, ty),
                                     height_start=H_UPHILL_END_M,
                                     height_end=H_UPHILL_END_M,
                                     speed=0.08,
                                     tol_m=0.12,
                                     timeout_s=28.0,
                                     keep_x=3.0,
                                     y_tol_m=0.06,
                                     safe_y_accept=float(ty) - float(UPHILL_SAFE_TURN_MARGIN_M))
                _log_pose(perception, prefix="[Seg5] leg1 retry")
                if not ok:
                    px, py = _safe_position_xy(perception)
                    safe_y = float(ty) - float(UPHILL_SAFE_TURN_MARGIN_M)
                    if py + float(UPHILL_SAFE_TURN_EPS_M) >= safe_y:
                        print(f"[Seg5] Accepting near-crest turn position: "
                              f"pos=({px:.3f},{py:.3f}), safe_y={safe_y:.2f}")
                    else:
                        print(f"[Seg5] Abort Segment 5: not safely on crest for turn "
                              f"(pos_y={py:.3f}, safe_y={safe_y:.2f})")
                        ctrl.stop_moving(body_height=0.22)
                        return True
            else:
                if kind == "tilt":
                    print(f"[Seg5] Abort Segment 5: tilt leg {idx} not reached, skip unsafe turn")
                    ctrl.stop_moving(body_height=0.22)
                    return True
                print(f"[Seg5] Continuing after leg {idx} warning")

        # After each leg, turn to required yaw.
        # On cross-slope we prefer arc turn; if it fails (protection/timeout),
        # fall back to low-crawl turning which keeps COM lower.
        print(f"[Seg5] Turning to {after_yaw:.1f} deg after leg {idx}")

        if idx == 1:
            turned = _curved_turn_to_angle(
                ctrl, perception, after_yaw,
                vx=0.035,
                vy=0.0,
                wz=0.28,
                body_z=0.22,
                rpy_des=[0.0, 0.0, 0.0],
                pos_des=[0.0, 0.0, 0.22],
                step_height=[0.04, 0.025],
                tol_deg=4.0,
                timeout_s=80.0,
                label=f"Seg5 crest arc turn after leg{idx}",
            )

        elif kind == "tilt":
            if TILT_TURN_USE_LEGACY_LOW_CRAWL:
                turned = _legacy_low_crawl_turn_to_absolute(
                    ctrl, perception, after_yaw,
                    turn_rate=LEGACY_TURN_RATE,
                    forward_speed=TILT_TURN_LEGACY_FWD_MPS,
                    lateral_speed=(0.0 if idx == 4 else TILT_TURN_LEGACY_VY_MPS),
                    roll_rad=TILT_ROLL_BIAS_RAD,
                    timeout_s=80.0,
                    restore_height=False,
                    label=f"Seg5 legacy low-crawl turn (tilt) after leg{idx}",
                )
            else:
                if idx in (2, 3):
                    turned = _curved_turn_to_angle(
                        ctrl, perception, after_yaw,
                        vx=0.058,
                        vy=TILT_VY_COMP_MPS,
                        wz=0.22,
                        body_z=0.22,
                        rpy_des=[TILT_ROLL_BIAS_RAD, 0.0, 0.0],
                        pos_des=[0.0, 0.0, 0.22],
                        step_height=STEP_H_TILT,
                        tol_deg=5.0,
                        timeout_s=80.0,
                        label=f"Seg5 curved turn (tilt) after leg{idx}",
                        enable_roll_comp=True,
                        roll_bias_rad=TILT_ROLL_BIAS_RAD,
                        vy_end_zero_deg=12.0,
                        vy_min_scale=0.45,
                    )
                    # After arc turn (rad-based), let controller settle.
                    time.sleep(0.1)
                else:
                    if idx == 4:
                        turned = _curved_turn_to_angle(
                            ctrl, perception, after_yaw,
                            vx=0.058,
                            vy=0.0,
                            wz=0.22,
                            body_z=0.22,
                            rpy_des=[TILT_ROLL_BIAS_RAD, 0.0, 0.0],
                            pos_des=[0.0, 0.0, 0.22],
                            step_height=STEP_H_TILT,
                            tol_deg=5.0,
                            timeout_s=80.0,
                            label=f"Seg5 curved turn (tilt) after leg{idx}",
                            enable_roll_comp=True,
                            roll_bias_rad=TILT_ROLL_BIAS_RAD,
                        )
                    else:
                        turned = _curved_turn_to_angle(
                            ctrl, perception, after_yaw,
                            vx=0.058,
                            vy=TILT_VY_COMP_MPS,
                            wz=0.22,
                            body_z=0.22,
                            rpy_des=[TILT_ROLL_BIAS_RAD, 0.0, 0.0],
                            pos_des=[0.0, 0.0, 0.22],
                            step_height=STEP_H_TILT,
                            tol_deg=5.0,
                            timeout_s=80.0,
                            label=f"Seg5 curved turn (tilt) after leg{idx}",
                            enable_roll_comp=True,
                            roll_bias_rad=TILT_ROLL_BIAS_RAD,
                        )

        else:
            # For leg5 (bridge end before jumping), use in-place spin to align heading.
            if idx == 5:
                turned = turn_to_angle(
                    ctrl, perception, after_yaw,
                    turn_rate=TURN_RATE, tol_deg=TURN_TOL_DEG, timeout_s=30.0)
            else:
                turned = _curved_turn_to_angle(
                    ctrl, perception, after_yaw,
                    vx=CURVED_TURN_VX_DEFAULT,
                    wz=CURVED_TURN_WZ_DEFAULT,
                    body_z=CURVED_TURN_POS_DES[2],
                    rpy_des=CURVED_TURN_RPY,
                    pos_des=CURVED_TURN_POS_DES,
                    step_height=CURVED_TURN_STEP_H,
                    tol_deg=3.0,
                    timeout_s=80.0,
                    label=f"Seg5 curved turn after leg{idx}",
                )

        if not turned:
            print(f"[Seg5] Warning: turn after leg {idx} timeout")

        # After leg 5, execute jump and optional landing correction
        if idx == 5:
            print("[Seg5] Preparing to jump down")
            # After leg5 turn, let posture settle then trigger jump.
            time.sleep(0.2)
            _execute_jump(ctrl)
            time.sleep(0.6)

            # Try to correct to landing point softly (if already close, returns quickly)
            print(f"[Seg5] Post-jump correction to ({JUMP_LAND_XY[0]:.2f},{JUMP_LAND_XY[1]:.2f})")
            _run_flat_leg(ctrl, perception, JUMP_LAND_XY,
                          speed=SPEED_POST_JUMP_CORRECT,
                          tol_m=0.30,
                          timeout_s=10.0)
            turn_to_angle(ctrl, perception, JUMP_LAND_YAW,
                          turn_rate=TURN_RATE, tol_deg=TURN_TOL_DEG, timeout_s=8.0)
            _log_pose(perception, prefix="[Seg5] final")
            ctrl.stop_moving(body_height=0.22)
            return True

    ctrl.stop_moving()
    return True


# -----------------------------------------------------------------------------
# Leg runners
# -----------------------------------------------------------------------------

def _run_uphill_leg(ctrl, perception, target_xy,
                    height_start, height_end,
                    speed, tol_m, timeout_s=None,
                    keep_x=None,
                    y_tol_m=UPHILL_Y_TOL_M,
                    safe_y_accept=None):
    """Run uphill leg with optional X position keeping.

    Args:
        keep_x: If provided, keep the robot's X position near this value
                by adding lateral correction (useful for staying on narrow bridges)
    """
    start_t = time.perf_counter()

    # If timeout is not provided, use a conservative default.
    if timeout_s is None:
        timeout_s = 55.0

    start_xy = _safe_position_xy(perception)
    total_dist = max(0.20, math.hypot(target_xy[0] - start_xy[0], target_xy[1] - start_xy[1]))

    last_progress_t = start_t
    last_dist = None
    next_height_update = 0.0
    last_debug_t = start_t

    # X correction parameters
    X_GAIN = 0.8   # P gain for X correction
    MAX_VY_CORRECT = 0.08  # max lateral correction speed

    # Stage A: coarse approach with larger tolerance
    while time.perf_counter() - start_t < timeout_s:
        px, py = _safe_position_xy(perception)
        dist = math.hypot(target_xy[0] - px, target_xy[1] - py)
        now = time.perf_counter()

        if now - last_debug_t >= 0.5:
            heading = _safe_heading_deg(perception)
            print(f"[Seg5] pos=({px:.3f},{py:.3f}) heading={heading:.1f}deg dist={dist:.3f}m")
            last_debug_t = now

        if dist <= max(tol_m, 0.30):
            break

        if last_dist is None or dist < last_dist - 0.01:
            last_dist = dist
            last_progress_t = time.perf_counter()
        if now - last_progress_t > 18.0:
            print(f"[Seg5] uphill no progress: dist={dist:.3f}m")
            return False

        traveled = math.hypot(px - start_xy[0], py - start_xy[1])
        ratio = max(0.0, min(1.0, traveled / total_dist))
        height = height_start + (height_end - height_start) * ratio

        # Compute pitch: nose-down at start, gradually return to slight nose-up
        if ratio < PITCH_TRANSITION_RATIO:
            pitch = PITCH_START_RAD
        else:
            t = (ratio - PITCH_TRANSITION_RATIO) / (1.0 - PITCH_TRANSITION_RATIO)
            pitch = PITCH_START_RAD + (PITCH_END_RAD - PITCH_START_RAD) * t

        if now >= next_height_update:
            ctrl.set_height(height, duration_ms=220)
            next_height_update = now + 0.25

        # Near the end of the uphill, slow down to reduce slip at the crest.
        speed_scale = 1.0
        if ratio >= 0.85:
            speed_scale = 0.55
        elif ratio >= 0.70:
            speed_scale = 0.75
        vx_cmd = float(speed) * speed_scale

        go_to_waypoint(
            ctrl, perception, target_xy,
            speed=vx_cmd,
            tol_m=0.30,
            step_h_max=STEP_H_UPHILL[0], step_h_min=STEP_H_UPHILL[1],
            body_height=height,
            body_pitch=pitch,
        )

        # Compute lateral correction for X keeping
        vy_correction = 0.0
        if keep_x is not None:
            x_err = px - keep_x
            vy_correction = _clamp(-x_err * X_GAIN, -MAX_VY_CORRECT, MAX_VY_CORRECT)

        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=_clamp(vx_cmd, 0.04, float(speed)),
            vy=vy_correction,
            wz=0.0,
            step_h_max=STEP_H_UPHILL[0], step_h_min=STEP_H_UPHILL[1],
            body_height=height,
            body_pitch=pitch,
            duration_ms=0,
        )
        time.sleep(0.05)

    # Stage B: fine approach with smaller tolerance and strict heading control.
    # Do not report success unless the crest/corner Y is really reached; otherwise
    # the robot starts the cross-slope turn with one side still near the edge.
    fine_start = time.perf_counter()
    last_debug_t = fine_start
    fine_last_progress_t = fine_start
    fine_last_progress_dist = None
    last_px, last_py, last_dist_seen, last_y_err = start_xy[0], start_xy[1], float("inf"), float("inf")
    while time.perf_counter() - fine_start < 18.0:
        px, py = _safe_position_xy(perception)
        dist = math.hypot(target_xy[0] - px, target_xy[1] - py)
        last_px, last_py, last_dist_seen = px, py, dist
        now = time.perf_counter()

        if now - last_debug_t >= 0.5:
            heading = _safe_heading_deg(perception)
            print(f"[Seg5] pos=({px:.3f},{py:.3f}) heading={heading:.1f}deg dist={dist:.3f}m")
            last_debug_t = now

        y_err = target_xy[1] - py
        last_y_err = y_err
        if safe_y_accept is not None and py + float(UPHILL_SAFE_TURN_EPS_M) >= float(safe_y_accept):
            ctrl.stop_moving(body_height=height_end)
            print(f"[Seg5] uphill safe-y reached: pos=({px:.3f},{py:.3f}), "
                  f"safe_y={float(safe_y_accept):.2f}")
            return True
        if dist <= tol_m and abs(y_err) <= y_tol_m:
            ctrl.stop_moving(body_height=height_end)
            return True

        if fine_last_progress_dist is None or dist < fine_last_progress_dist - 0.01:
            fine_last_progress_dist = dist
            fine_last_progress_t = now
        if now - fine_last_progress_t > 10.0:
            print(f"[Seg5] uphill no progress: dist={dist:.3f}m")
            return False

        traveled = math.hypot(px - start_xy[0], py - start_xy[1])
        ratio = max(0.0, min(1.0, traveled / total_dist))
        height = height_start + (height_end - height_start) * ratio

        # Compute pitch: nose-down at start, gradually return to slight nose-up
        if ratio < PITCH_TRANSITION_RATIO:
            pitch = PITCH_START_RAD
        else:
            t = (ratio - PITCH_TRANSITION_RATIO) / (1.0 - PITCH_TRANSITION_RATIO)
            pitch = PITCH_START_RAD + (PITCH_END_RAD - PITCH_START_RAD) * t

        if now >= next_height_update:
            ctrl.set_height(height, duration_ms=220)
            next_height_update = now + 0.25

        # Near the end of the uphill, slow down to reduce slip at the crest.
        speed_scale = 1.0
        if ratio >= 0.85:
            speed_scale = 0.55
        elif ratio >= 0.70:
            speed_scale = 0.75
        vx_cmd = 0.10 * speed_scale

        go_to_waypoint(
            ctrl, perception, target_xy,
            speed=vx_cmd,
            tol_m=tol_m,
            max_wz=TURN_RATE,
            step_h_max=STEP_H_UPHILL[0], step_h_min=STEP_H_UPHILL[1],
            body_height=height,
            body_pitch=pitch,
        )

        # Compute lateral correction for X keeping
        vy_correction = 0.0
        if keep_x is not None:
            x_err = px - keep_x
            vy_correction = _clamp(-x_err * X_GAIN, -MAX_VY_CORRECT, MAX_VY_CORRECT)

        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=_clamp(vx_cmd, 0.04, 0.10),
            vy=vy_correction,
            wz=0.0,
            step_h_max=STEP_H_UPHILL[0], step_h_min=STEP_H_UPHILL[1],
            body_height=height,
            body_pitch=pitch,
            duration_ms=0,
        )
        time.sleep(0.05)

    ctrl.stop_moving()
    print(f"[Seg5] uphill fine approach timeout: pos=({last_px:.3f},{last_py:.3f}), "
          f"target=({target_xy[0]:.2f},{target_xy[1]:.2f}), "
          f"dist={last_dist_seen:.3f}m, y_err={last_y_err:.3f}m")
    return False


def _set_tilt_body_pose(ctrl, height_m, pitch_rad, duration_ms=200):
    """Set body height and pitch for tilt stability."""
    if hasattr(ctrl, 'set_body_pose'):
        ctrl.set_body_pose(height_m=height_m, pitch_rad=pitch_rad,
                           duration_ms=duration_ms)
    else:
        ctrl.set_height(height_m, duration_ms=duration_ms)
        ctrl.look_forward(pitch_rad, duration_ms=duration_ms)


def _get_measured_roll(perception, safe_roll=0.0):
    """Return roll in radians.

    Tries multiple sources because perception fields differ between sim/real.
    """
    try:
        # 1) direct field
        roll = getattr(perception, "roll", None)
        if roll is not None:
            return float(roll)

        # 2) body_pose if it includes rpy
        body_pose = getattr(perception, "body_pose", None)
        if body_pose is not None:
            try:
                pose = body_pose  # could be property returning tuple
                if pose is not None and len(pose) >= 6:
                    return float(pose[3])
            except Exception:
                pass

        # 3) Odom full pose if odom_pose carries rpy (some implementations do)
        odom_pose = getattr(perception, "odom_pose", None)
        if odom_pose is not None and len(odom_pose) >= 6:
            roll, _, _ = odom_pose[3:6]
            return float(roll)

        # 4) IMU quaternion (sensor_msgs/Imu)
        imu_data = getattr(perception, "imu_data", None)
        if imu_data is not None:
            q = getattr(imu_data, "orientation", None)
            if q is not None:
                # roll (x-axis rotation) from quaternion
                sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
                cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                return float(math.atan2(sinr_cosp, cosr_cosp))

    except (AttributeError, TypeError, ValueError):
        pass

    return float(safe_roll)


def _compute_target_wz_on_slope(perception, dx, dy, *, kp=0.8, wz_max=TILT_WZ_MAX):
    """Compute a small wz to bias heading toward target.

    Uses odom yaw if present; clamps to keep turns gentle on cross-slope.
    """
    try:
        yaw_rad = float(getattr(perception, "odom_pose")[2])
        target_yaw = math.atan2(float(dy), float(dx))
        yaw_err = ((yaw_rad - target_yaw + math.pi) % (2.0 * math.pi)) - math.pi
        return float(_clamp(-yaw_err * float(kp), -float(wz_max), float(wz_max)))
    except Exception:
        return 0.0


def _run_lateral_tilt_leg(ctrl, perception, target_xy, speed, tol_m, timeout_s, *, vy_extra_bias=0.0):
    """Traverse a left-high / right-low slope using slow trot + line hold.

    Key ideas:
      - Use built-in slow trot; legacy low-crawl was slipping in place here.
      - Apply lateral velocity to hold the current leg centerline.
      - Apply a fixed roll bias; IMU roll on this terrain is often too small.

    Notes:
      - The vy command is closed-loop in world coordinates, then projected into
        the robot lateral axis. This avoids sign mistakes at yaw=0/90/180/270.
    """
    start_t = time.perf_counter()
    last_debug_t = start_t

    cmd_roll = float(TILT_ROLL_BIAS_RAD)

    # Progress / slip detection: if distance doesn't decrease, degrade.
    last_dist = None
    last_progress_t = start_t

    # Start-up ramp: first moments after a turn are the most unstable.
    # Keep speed conservative briefly, then let line hold take over.
    RAMP_S = 0.6
    RAMP_MIN_SCALE = 0.70

    while time.perf_counter() - start_t < timeout_s:
        px, py = _safe_position_xy(perception)
        dx = target_xy[0] - px
        # Track a line slightly toward the high side instead of the geometric
        # center. This prevents oscillating across the centerline on a slippery
        # cross-slope and keeps the feet away from the low/right edge.
        desired_y = float(target_xy[1]) - float(TILT_HIGH_SIDE_OFFSET_M)
        dy = desired_y - py
        dist = math.hypot(dx, dy)
        now = time.perf_counter()

        if dist <= tol_m:
            return True

        if last_dist is None or dist < last_dist - 0.01:
            last_dist = dist
            last_progress_t = now
        if now - last_progress_t > 12.0:
            print(f"[Seg5] tilt no progress: dist={dist:.3f}m")
            return False

        wz = _compute_target_wz_on_slope(perception, dx, dy)
        # Hold the leg centerline in world coordinates, then project that desired
        # correction onto robot-left (body +Y). At yaw=180, body +Y is world -Y,
        # so a world -Y correction must become positive body vy.
        heading_rad = math.radians(_safe_heading_deg(perception))
        lateral_world_x = -math.sin(heading_rad)
        lateral_world_y = math.cos(heading_rad)
        measured_roll = _get_measured_roll(perception)
        line_corr_y = 0.0
        if abs(dy) > TILT_LINE_DEADBAND_M:
            line_corr_y = float(TILT_LINE_KP) * float(dy)
        world_corr_y = _clamp(float(TILT_HIGH_SIDE_WORLD_Y_MPS) + line_corr_y,
                              -float(TILT_VY_MAX_MPS), float(TILT_VY_MAX_MPS))
        if abs(dy) > TILT_LINE_DEADBAND_M and abs(world_corr_y) < TILT_VY_MIN_CORRECT_MPS:
            world_corr_y = math.copysign(float(TILT_VY_MIN_CORRECT_MPS), world_corr_y)
        roll_corr_y = 0.0
        if abs(measured_roll) > TILT_ROLL_DEADBAND_RAD:
            roll_corr_y = _clamp(
                -float(TILT_ROLL_VY_KP) * float(measured_roll),
                -float(TILT_VY_MAX_MPS),
                float(TILT_VY_MAX_MPS),
            )
        world_corr_y = _clamp(world_corr_y + roll_corr_y,
                              -float(TILT_VY_MAX_MPS), float(TILT_VY_MAX_MPS))
        # Project world-Y + world-X compensation onto robot lateral axis (body +Y / robot left)
        # lateral_world_y = cos(heading): world +Y -> robot left projection
        # lateral_world_x = -sin(heading): world +X -> robot left projection
        world_corr_x = float(TILT_HIGH_SIDE_WORLD_X_MPS)
        high_side_proj = world_corr_y * lateral_world_y + world_corr_x * lateral_world_x
        vy_cmd = _clamp(high_side_proj + float(TILT_VY_COMP_MPS) + float(vy_extra_bias),
                        -float(TILT_VY_MAX_MPS), float(TILT_VY_MAX_MPS))
        line_err = abs(float(dy))
        speed_scale = 1.0
        if line_err > 0.15:
            speed_scale = 0.75
        elif line_err > 0.08:
            speed_scale = 0.90

        # Startup ramp: limit speed right after entering this leg.
        ramp_t = _clamp((now - start_t) / max(1e-3, float(RAMP_S)), 0.0, 1.0)
        ramp_scale = float(RAMP_MIN_SCALE) + (1.0 - float(RAMP_MIN_SCALE)) * float(ramp_t)

        vx_cmd = float(speed) * speed_scale * ramp_scale

        if now - last_debug_t >= 0.5:
            heading = _safe_heading_deg(perception)
            y_err = desired_y - py
            print(f"[Seg5] tilt-trot pos=({px:.3f},{py:.3f}) heading={heading:.1f}deg "
                  f"dist={dist:.3f}m y_err={y_err:.3f} roll={measured_roll:.3f}rad "
                  f"cmd_roll={cmd_roll:.3f} vx_cmd={vx_cmd:.3f} "
                  f"line_corr_y={line_corr_y:.3f} roll_corr_y={roll_corr_y:.3f} "
                  f"world_corr_y={world_corr_y:.3f} world_corr_x={world_corr_x:.3f} "
                  f"high_side_proj={high_side_proj:.3f} "
                  f"vy_cmd={vy_cmd:.3f} wz={wz:.3f}")
            last_debug_t = now

        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=float(vx_cmd),
            vy=float(vy_cmd),
            wz=float(wz),
            step_h_max=float(STEP_H_TILT[0]),
            step_h_min=float(STEP_H_TILT[1]),
            body_height=float(TILT_BODY_HEIGHT_M),
            body_pitch=float(TILT_BODY_PITCH_RAD),
            body_roll=float(cmd_roll),
            duration_ms=0,
        )

        time.sleep(0.05)

    print(f"[Seg5] tilt timeout: target={target_xy}, dist={dist:.3f}m")
    return False


def _upload_legacy_tilt_gait(ctrl):
    """Upload the Segment-4 legacy under-bar gait for low-COM tilt walking."""
    try:
        from .segment_4 import _upload_legacy_under_bar_gait
        _upload_legacy_under_bar_gait(ctrl)
        time.sleep(0.2)
    except Exception as e:
        print(f"[Seg5] Warning: legacy gait upload failed: {e}")


def _send_legacy_user_gait(ctrl, vx=0.0, vy=0.0, wz=0.0,
                           body_z=LEGACY_TILT_BODY_Z,
                           pitch_rad=0.0,
                           roll_rad=TILT_ROLL_BIAS_RAD,
                           duration_ms=700):
    """Send legacy user gait with roll support, fallback for older controller."""
    try:
        ctrl.user_gait_locomotion(
            vx=vx,
            vy=vy,
            wz=wz,
            body_z=body_z,
            pitch_rad=pitch_rad,
            roll_rad=roll_rad,
            step_height=LEGACY_TILT_STEP_H,
            contact=1,
            value=1,
            duration_ms=duration_ms,
        )
    except TypeError:
        ctrl.user_gait_locomotion(
            vx=vx,
            vy=vy,
            wz=wz,
            body_z=body_z,
            pitch_rad=pitch_rad,
            step_height=LEGACY_TILT_STEP_H,
            contact=1,
            value=1,
            duration_ms=duration_ms,
        )
        _apply_roll_via_gait_steps(ctrl, roll_rad)


def _legacy_low_crawl_turn_to_absolute(ctrl, perception, target_yaw_deg,
                                       turn_rate=0.55,
                                       forward_speed=0.025,
                                       lateral_speed=TILT_VY_COMP_MPS,
                                       roll_rad=TILT_ROLL_BIAS_RAD,
                                       timeout_s=24.0,
                                       restore_height=False,
                                       label="Seg5 legacy low-crawl turn"):
    """Turn to an absolute heading using legacy low-crawl gait."""
    _upload_legacy_tilt_gait(ctrl)

    target_yaw_deg = float(target_yaw_deg) % 360.0
    turn_rate = max(0.03, min(abs(turn_rate), 0.80))
    forward_speed = max(0.0, min(forward_speed, 0.05))
    lateral_speed = _clamp(float(lateral_speed), -0.08, 0.08)
    start = time.perf_counter()
    last_debug = start
    last_err = None

    print(f"[Seg5] {label}: target={target_yaw_deg:.1f}deg, "
          f"turn_rate={turn_rate:.2f}, forward={forward_speed:.3f}, "
          f"lateral={lateral_speed:.3f}, "
          f"roll={roll_rad:.3f}, timeout={timeout_s:.1f}s")

    while time.perf_counter() - start < timeout_s:
        yaw = _safe_heading_deg(perception)
        err = _angle_diff(target_yaw_deg, yaw)
        last_err = err
        if abs(err) <= 5.0:
            if restore_height:
                ctrl.stop_moving(body_height=0.22)
            else:
                _send_legacy_user_gait(
                    ctrl,
                    vx=0.0, vy=0.0, wz=0.0,
                    body_z=LEGACY_TILT_BODY_Z,
                    pitch_rad=0.0,
                    roll_rad=roll_rad,
                    duration_ms=600,
                )
            print(f"[Seg5] {label}: reached yaw={yaw:.1f}, err={err:.1f}")
            return True

        wz = turn_rate if err > 0.0 else -turn_rate
        _send_legacy_user_gait(
            ctrl,
            vx=forward_speed,
            vy=lateral_speed,
            wz=wz,
            body_z=LEGACY_TILT_BODY_Z,
            pitch_rad=0.0,
            roll_rad=roll_rad,
            duration_ms=700,
        )

        now = time.perf_counter()
        if now - last_debug >= 0.5:
            px, py = _safe_position_xy(perception)
            print(f"[Seg5] {label}: pos=({px:.3f},{py:.3f}) "
                  f"yaw={yaw:.1f} target={target_yaw_deg:.1f} err={err:.1f} "
                  f"vx={forward_speed:.3f} vy={lateral_speed:.3f} wz={wz:.3f}")
            last_debug = now
        time.sleep(0.10)

    _send_legacy_user_gait(
        ctrl,
        vx=0.0, vy=0.0, wz=0.0,
        body_z=LEGACY_TILT_BODY_Z,
        pitch_rad=0.0,
        roll_rad=roll_rad,
        duration_ms=600,
    )
    yaw = _safe_heading_deg(perception)
    print(f"[Seg5] {label}: timeout yaw={yaw:.1f}, "
          f"target={target_yaw_deg:.1f}, err={last_err:.1f}")
    return False

# -----------------------------------------------------------------------------
# Curved arc turn (forward + yaw for slope stability)
# -----------------------------------------------------------------------------

# Optional post-turn settle to avoid "turn-end -> straight-start" instability on cross-slope.
# Enabled by default; tune per sim/hardware.
POST_TURN_SETTLE_ENABLE = True
POST_TURN_SETTLE_S = 0.6
POST_TURN_SETTLE_VX = 0.04
POST_TURN_SETTLE_WZ = 0.0
POST_TURN_SETTLE_VY_MIN_SCALE = 0.45


def _curved_turn_to_angle(ctrl, perception, target_yaw_deg,
                          vx=None, vy=0.0, wz=None,
                          body_z=None,
                          rpy_des=None,
                          pos_des=None,
                          step_height=None,
                          tol_deg=TURN_TOL_DEG,
                          timeout_s=30.0,
                          label="Seg5 curved arc turn",
                          enable_roll_comp=False,
                          roll_sign=None,
                          roll_gain=ROLL_GAIN,
                          roll_clamp_rad=ROLL_CLAMP_RAD,
                          roll_update_hz=ROLL_UPDATE_HZ,
                          roll_bias_rad=0.0,
                          debug_roll=False,
                          wz_init_scale=1.6,
                          wz_min_scale=0.55,
                          vy_end_zero_deg=18.0,
                          vy_min_scale=0.0):
    """Turn using forward motion + yaw arc instead of stationary spin.

    This approach maintains a forward support polygon which is much more stable
    on slopes than spinning in place around the center of mass.

    If enable_roll_comp is True, we continuously update msg.rpy_des[0] based on
    measured roll to keep the body leaning into the high side on cross-slope.

    roll_bias_rad can be used to add a constant roll offset (lean) during turning,
    useful when sensors under-report roll but the terrain is known (e.g. left-high).

    Strategy:
      - Give small forward velocity (vel_des[0]) to maintain forward support
      - Simultaneously apply yaw rate (vel_des[2]) to complete the turn
      - Monitor switch_status for protection triggers and degrade speed if needed
      - duration=0, loop continuously at ~20Hz
    """
    if vx is None:
        vx = CURVED_TURN_VX_DEFAULT
    if wz is None:
        wz = CURVED_TURN_WZ_DEFAULT

    vy = float(vy)
    if body_z is None:
        body_z = CURVED_TURN_POS_DES[2]
    if rpy_des is None:
        rpy_des = CURVED_TURN_RPY
    if pos_des is None:
        pos_des = CURVED_TURN_POS_DES
    if step_height is None:
        step_height = CURVED_TURN_STEP_H

    if roll_sign is None:
        roll_sign = float(TILT_ROLL_SIGN)

    wz_abs = float(wz)
    vx_abs = float(vx)
    vy_abs = float(vy)
    tol_deg = max(1.0, float(tol_deg))
    timeout_s = max(1.0, float(timeout_s))
    target_yaw_deg = float(target_yaw_deg)

    roll_bias_rad = float(roll_bias_rad)

    wz_init_scale = float(wz_init_scale)
    wz_min_scale = float(wz_min_scale)
    if wz_init_scale < 1.0:
        wz_init_scale = 1.0
    wz_min_scale = _clamp(wz_min_scale, 0.20, 1.0)

    start_t = time.perf_counter()
    last_debug_t = start_t
    protection_count = 0

    last_roll_update_t = 0.0
    roll_cmd = 0.0
    measured_roll_dbg = 0.0

    vy_end_zero_deg = max(0.0, float(vy_end_zero_deg))
    vy_min_scale = _clamp(float(vy_min_scale), 0.0, 1.0)

    print(f"[Seg5] {label}: target={target_yaw_deg:.1f}deg, "
          f"vx={vx_abs:.3f}, vy={vy_abs:.3f}, wz={wz_abs:.3f}, timeout={timeout_s:.1f}s")

    # Build the continuous command (duration=0)
    msg = robot_control_cmd_lcmt()
    msg.mode = MODE_LOCOMOTION
    msg.gait_id = GAIT_TROT_SLOW
    msg.contact = 15  # all feet
    msg.vel_des = [vx_abs, vy_abs, wz_abs]
    msg.rpy_des = list(rpy_des)
    msg.pos_des = [pos_des[0], pos_des[1], body_z]
    msg.step_height = list(step_height)
    msg.value = 0
    msg.duration = 0  # continuous control
    msg.life_count = 0

    def _send_curved_cmd():
        """Send the curved turn command by updating the controller's shared command."""
        ctrl._increment_life()
        ctrl._send_lock.acquire()
        try:
            ctrl._cmd.mode = msg.mode
            ctrl._cmd.gait_id = msg.gait_id
            ctrl._cmd.contact = msg.contact
            for i in range(3):
                ctrl._cmd.vel_des[i] = msg.vel_des[i]
            for i in range(3):
                ctrl._cmd.rpy_des[i] = msg.rpy_des[i]
            for i in range(3):
                ctrl._cmd.pos_des[i] = msg.pos_des[i]
            for i in range(2):
                ctrl._cmd.step_height[i] = msg.step_height[i]
            ctrl._cmd.value = msg.value
            ctrl._cmd.duration = msg.duration
            ctrl._delay_cnt = 15
        finally:
            ctrl._send_lock.release()

    def _apply_fallback(trigger_count):
        """Degrade speed parameters based on protection trigger count."""
        idx = min(trigger_count, len(PROTECTION_FALLBACK) - 1)
        vx_fb, wz_fb, body_z_fb = PROTECTION_FALLBACK[idx]
        msg.vel_des[0] = vx_fb
        msg.vel_des[2] = wz_fb
        # Reduce lateral push together with forward/yaw to avoid drifting off track.
        msg.vel_des[1] = msg.vel_des[1] * (0.65 ** float(idx))
        msg.pos_des[2] = body_z_fb
        print(f"[Seg5] {label}: protection trigger #{trigger_count}, "
              f"degrading to vx={vx_fb:.3f}, vy={msg.vel_des[1]:.3f}, wz={wz_fb:.3f}, z={body_z_fb:.3f}")

    while time.perf_counter() - start_t < timeout_s:
        now_loop = time.perf_counter()
        if enable_roll_comp and (now_loop - last_roll_update_t) >= 1.0 / max(1e-3, float(roll_update_hz)):
            measured_roll_dbg = float(_get_measured_roll(perception))
            roll_cmd = _clamp(
                ((-measured_roll_dbg * float(roll_gain)) * float(roll_sign)) + roll_bias_rad,
                -float(roll_clamp_rad),
                float(roll_clamp_rad),
            )
            msg.rpy_des[0] = float(roll_cmd)
            last_roll_update_t = now_loop

        # Check heading
        yaw = _safe_heading_deg(perception)
        err = ((target_yaw_deg - yaw + 540.0) % 360.0) - 180.0

        if abs(err) <= tol_deg:
            # No stop/sleep handoff: set a straight-start posture and return.
            # The next leg will immediately override this continuous command.
            if enable_roll_comp:
                measured_roll_dbg = float(_get_measured_roll(perception))
                roll_cmd = _clamp(
                    ((-measured_roll_dbg * float(roll_gain)) * float(roll_sign)) + roll_bias_rad,
                    -float(roll_clamp_rad),
                    float(roll_clamp_rad),
                )
                msg.rpy_des[0] = float(roll_cmd)

            handoff_vy = 0.0
            if vy_abs != 0.0:
                vy_scale = max(float(POST_TURN_SETTLE_VY_MIN_SCALE), float(vy_min_scale))
                handoff_vy = float(vy_abs) * float(vy_scale)

            msg.vel_des[0] = float(POST_TURN_SETTLE_VX)
            msg.vel_des[1] = float(handoff_vy)
            msg.vel_des[2] = float(POST_TURN_SETTLE_WZ)
            _send_curved_cmd()

            print(f"[Seg5] {label}: reached target={target_yaw_deg:.1f}deg "
                  f"(yaw={yaw:.1f}, err={err:.1f})")
            return True

        # Determine yaw direction
        err_sign = 1.0 if err > 0.0 else -1.0

        # Yaw rate scheduling: keep fast for the first ~80% of the turn, then
        # drop sharply for the last ~20% to avoid side-slip near the end.
        # Map error range [tol_deg, 180deg] -> progress in [0, 1].
        err_abs = abs(err)
        progress = _clamp((180.0 - err_abs) / max(1.0, (180.0 - tol_deg)), 0.0, 1.0)
        if progress < 0.80:
            wz_scale = wz_init_scale
        else:
            # progress in [0.8,1] -> t in [0,1]
            t = (progress - 0.80) / 0.20
            # ease-out drop
            t = _clamp(t, 0.0, 1.0)
            wz_scale = wz_init_scale + (wz_min_scale - wz_init_scale) * t
        msg.vel_des[2] = (wz_abs * wz_scale) * err_sign

        # Lateral push scheduling: keep anti-slip vy early, then fade to 0 near the end
        # to avoid drifting out of the narrow tilt lane.
        vy_scale = 1.0
        if vy_end_zero_deg > 0.0:
            vy_scale = _clamp(err_abs / max(1.0, vy_end_zero_deg), float(vy_min_scale), 1.0)
        msg.vel_des[1] = vy_abs * vy_scale

        switch_status = 0
        try:
            switch_status = ctrl._resp.switch_status
        except AttributeError:
            pass

        if switch_status not in [0, 1]:
            protection_count += 1
            _apply_fallback(protection_count)
            if protection_count >= len(PROTECTION_FALLBACK):
                print(f"[Seg5] {label}: protection max ({switch_status}), aborting")
                ctrl.stop_moving(body_height=0.22)
                ctrl.set_height(0.22, duration_ms=500)
                time.sleep(0.5)
                return False

        _send_curved_cmd()

        now = time.perf_counter()
        if now - last_debug_t >= 0.5:
            roll_dbg = ""
            if enable_roll_comp:
                if debug_roll:
                    roll_dbg = (
                        f" measured_roll={measured_roll_dbg:.3f}"
                        f" roll_cmd={roll_cmd:.3f} bias={roll_bias_rad:.3f}"
                    )
                else:
                    roll_dbg = f" roll_cmd={roll_cmd:.3f}"
            print(f"[Seg5] {label}: yaw={yaw:.1f}deg target={target_yaw_deg:.1f}deg "
                  f"err={err:.1f}deg vx={msg.vel_des[0]:.3f} vy={msg.vel_des[1]:.3f} wz={msg.vel_des[2]:.3f} "
                  f"prot={switch_status} triggers={protection_count}{roll_dbg}")
            last_debug_t = now

        time.sleep(0.05)

    ctrl.stop_moving(body_height=body_z)
    ctrl.set_height(body_z, duration_ms=300)
    time.sleep(0.3)
    yaw_final = _safe_heading_deg(perception)
    err_final = ((target_yaw_deg - yaw_final + 540.0) % 360.0) - 180.0
    print(f"[Seg5] {label}: timeout target={target_yaw_deg:.1f}deg "
          f"(yaw={yaw_final:.1f}, err={err_final:.1f})")
    return False


def _run_flat_leg(ctrl, perception, target_xy, speed, tol_m, timeout_s):
    start_t = time.perf_counter()
    last_progress_t = start_t
    last_dist = None
    last_debug_t = start_t

    # Stage A: coarse approach with larger tolerance
    while time.perf_counter() - start_t < timeout_s:
        px, py = _safe_position_xy(perception)
        dist = math.hypot(target_xy[0] - px, target_xy[1] - py)
        now = time.perf_counter()

        if now - last_debug_t >= 0.5:
            heading = _safe_heading_deg(perception)
            print(f"[Seg5] pos=({px:.3f},{py:.3f}) heading={heading:.1f}deg dist={dist:.3f}m")
            last_debug_t = now

        if dist <= max(tol_m, 0.30):
            break

        if last_dist is None or dist < last_dist - 0.03:
            last_dist = dist
            last_progress_t = time.perf_counter()
        if now - last_progress_t > 10.0:
            print(f"[Seg5] flat no progress: dist={dist:.3f}m")
            ctrl.stop_moving(body_height=0.22)
            return False

        go_to_waypoint(
            ctrl, perception, target_xy,
            speed=speed,
            tol_m=0.30,
            step_h_max=STEP_H_FLAT[0], step_h_min=STEP_H_FLAT[1],
        )
        time.sleep(0.05)

    # Stage B: fine approach with smaller tolerance and strict heading control
    fine_start = time.perf_counter()
    last_debug_t = fine_start
    while time.perf_counter() - fine_start < 8.0:
        px, py = _safe_position_xy(perception)
        dist = math.hypot(target_xy[0] - px, target_xy[1] - py)
        now = time.perf_counter()

        if now - last_debug_t >= 0.5:
            heading = _safe_heading_deg(perception)
            print(f"[Seg5] pos=({px:.3f},{py:.3f}) heading={heading:.1f}deg dist={dist:.3f}m")
            last_debug_t = now

        if dist <= tol_m:
            ctrl.stop_moving(body_height=0.22)
            return True

        if last_dist is None or dist < last_dist - 0.03:
            last_dist = dist
            last_progress_t = time.perf_counter()
        if now - last_progress_t > 8.0:
            print(f"[Seg5] flat no progress: dist={dist:.3f}m")
            ctrl.stop_moving(body_height=0.22)
            return False

        go_to_waypoint(
            ctrl, perception, target_xy,
            speed=0.10,
            tol_m=tol_m,
            max_wz=TURN_RATE,
            step_h_max=STEP_H_FLAT[0], step_h_min=STEP_H_FLAT[1],
        )
        time.sleep(0.05)

    ctrl.stop_moving()
    return True


# -----------------------------------------------------------------------------
# Jump
# -----------------------------------------------------------------------------

def _execute_jump(ctrl):
    """Execute jump off the bridge using JUMP3D standing long jump."""
    print("[Seg5] Executing JUMP3D long jump off bridge")

    ctrl.stop_moving(body_height=0.22)
    time.sleep(0.5)

    ctrl.jump3d(
        gait_id=1,
        duration_ms=1500,
        timeout_s=3.0,
    )

    time.sleep(0.5)

    # Softly re-take control after landing.
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=0.05, vy=0.0, wz=0.0,
        step_h_max=0.04, step_h_min=0.03,
        body_height=0.22,
        duration_ms=0,
    )
    time.sleep(0.5)

    ctrl.stop_moving(body_height=0.22)
    print("[Seg5] Jump landed")


# -----------------------------------------------------------------------------
# Roll helpers
# -----------------------------------------------------------------------------

def _get_measured_roll(perception, safe_roll=0.0):
    """Return measured body roll in radians.

    Perception sources vary by runtime:
    - ROS2Perception: IMU message is sensor_msgs/Imu (quaternion only), and may also
      provide `body_pose` from Gazebo/odom with (x,y,z,roll,pitch,yaw).
    - MockPerception: may expose `rpy` directly.

    This helper tries the most explicit sources first and gracefully falls back.
    """
    try:
        # 1) Direct rpy tuple if available
        rpy = getattr(perception, "rpy", None)
        if rpy is not None:
            roll, _, _ = rpy
            return float(roll)

        # 2) Full body pose if provided: (x, y, z, roll, pitch, yaw)
        body_pose = getattr(perception, "body_pose", None)
        if body_pose is not None:
            try:
                pose = body_pose  # could be property returning tuple
                if pose is not None and len(pose) >= 6:
                    return float(pose[3])
            except Exception:
                pass

        # 3) Odom full pose if odom_pose carries rpy (some implementations do)
        odom_pose = getattr(perception, "odom_pose", None)
        if odom_pose is not None and len(odom_pose) >= 6:
            roll, _, _ = odom_pose[3:6]
            return float(roll)

        # 4) IMU quaternion (sensor_msgs/Imu)
        imu_data = getattr(perception, "imu_data", None)
        if imu_data is not None:
            q = getattr(imu_data, "orientation", None)
            if q is not None:
                # roll (x-axis rotation) from quaternion
                sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
                cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                return float(math.atan2(sinr_cosp, cosr_cosp))

    except (AttributeError, TypeError, ValueError):
        pass

    return float(safe_roll)


def _apply_roll_via_gait_steps(ctrl, roll_rad):
    # Keep this very light: just bias rpy_des roll.
    steps = _get_stabilize_steps(roll_rad=roll_rad)
    try:
        ctrl.execute_gait_steps(steps)
    except Exception:
        pass


def _get_stabilize_steps(roll_rad=0.0):
    # Single short step to push pose command into WBC.
    roll_rad = _clamp(roll_rad, -ROLL_CLAMP_RAD, ROLL_CLAMP_RAD)
    return [
        dict(mode=11, gait_id=110, contact=15,
             vel_des=[0.0, 0.0, 0.0, 0.0],
             rpy_des=[roll_rad, 0.0, 0.0],
             pos_des=[0.0, 0.0, -0.10],
             acc_des=[0.8, 0.8, 0.8, 1.0, 1.0, 1.4],
             ctrl_point=[0.0, 0.0, 0.70],
             foot_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             step_height=[26026.0, 26026.0],
             value=0, duration=180),
    ]


# -----------------------------------------------------------------------------
# Misc helpers
# -----------------------------------------------------------------------------

def _safe_position_xy(perception):
    try:
        px, py = perception.position
        return float(px), float(py)
    except (AttributeError, TypeError, ValueError):
        px, py, _ = perception.odom_pose
        return float(px), float(py)


def _log_pose(perception, prefix="[Seg5]"):
    try:
        px, py = perception.position
        yaw = perception.heading
    except (AttributeError, TypeError):
        px, py, yaw = perception.odom_pose[:3]
        yaw = float(yaw) * 180.0 / math.pi % 360.0
    print(f"{prefix} pos=({px:.3f},{py:.3f}) heading={yaw:.1f}deg")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# -----------------------------------------------------------------------------
# Low-crawl turn (relative angle delta)
# -----------------------------------------------------------------------------

def _angle_diff(target, current):
    """Return shortest angular difference from current to target (degrees)."""
    diff = ((float(target) - float(current) + 540.0) % 360.0) - 180.0
    return diff


def low_crawl_turn(ctrl, perception,
                   target_delta_deg=90.0,
                   turn_rate=0.10,
                   forward_speed=0.0,
                   timeout_s=14.0,
                   restore_height=True):
    """Low-crawl turn from current pose by target_delta_deg.

    Args:
        ctrl: Controller instance
        perception: Perception instance for position/heading feedback
        target_delta_deg: Relative turn angle in degrees (positive = counterclockwise)
        turn_rate: Angular rate limit in rad/s (clamped to [0.03, 0.80])
        forward_speed: Forward velocity in m/s (clamped to [0.0, 0.05])
        timeout_s: Maximum time allowed for the turn
        restore_height: If True, restore normal height after turn; if False, keep low posture

    Returns:
        bool: True if target heading reached within tolerance (5 deg), False otherwise
    """
    from .segment_4 import _upload_legacy_under_bar_gait, _body_pose_debug

    print("[Seg5] Low-crawl turn test")
    print("[Seg5] Uploading legacy under-bar gait files...")
    _upload_legacy_under_bar_gait(ctrl)
    time.sleep(0.5)

    try:
        start_pos = perception.position[:2]
        start_heading = perception.heading
    except (AttributeError, TypeError):
        start_pos = None
        start_heading = 0.0

    target_heading = (float(start_heading) + target_delta_deg) % 360.0
    turn_rate = max(0.03, min(abs(turn_rate), 0.80))
    forward_speed = max(0.0, min(forward_speed, 0.05))

    print(f"[Seg5] Start: position={start_pos}, heading={start_heading}")
    print(f"[Seg5] Target: heading={target_heading:.1f} deg "
          f"(delta={target_delta_deg:.1f}), turn_rate={turn_rate:.2f}, "
          f"forward_speed={forward_speed:.2f}, timeout={timeout_s:.1f}s")

    start = time.perf_counter()
    last_debug = 0.0
    reached = False
    last_err = None

    while time.perf_counter() - start < timeout_s:
        try:
            current_heading = perception.heading
            pos = perception.position[:2]
        except (AttributeError, TypeError):
            current_heading = 0.0
            pos = None

        err = _angle_diff(target_heading, current_heading)
        last_err = err
        if abs(err) <= 3.0:
            reached = True
            break

        wz = turn_rate if err > 0.0 else -turn_rate
        ctrl.user_gait_locomotion(
            vx=forward_speed,
            vy=0.0,
            wz=wz,
            body_z=-0.035,
            pitch_rad=0.0,
            step_height=(0.03, 0.03),
            contact=1,
            value=1,
            duration_ms=220,
        )

        now = time.perf_counter()
        if now - last_debug > 0.5:
            dist = None
            if start_pos is not None and pos is not None:
                dist = math.hypot(pos[0] - start_pos[0],
                                  pos[1] - start_pos[1])
            dist_str = "dist=N/A" if dist is None else f"dist={dist:.3f}m"
            print(f"[Seg5] Turning: pos={pos}, heading={current_heading:.1f}, "
                  f"err={err:.1f}, wz={wz:.2f}, {dist_str}, "
                  f"{_body_pose_debug(perception)}")
            last_debug = now
        time.sleep(0.12)

    if restore_height:
        ctrl.stop_moving(body_height=0.22)
        ctrl.set_height(0.22, duration_ms=500)
        time.sleep(0.5)
    else:
        ctrl.user_gait_locomotion(
            vx=0.0,
            vy=0.0,
            wz=0.0,
            body_z=-0.035,
            pitch_rad=0.0,
            step_height=(0.03, 0.03),
            contact=1,
            value=1,
            duration_ms=1000,
        )
        time.sleep(0.2)

    try:
        end_pos = perception.position[:2]
        end_heading = perception.heading
    except (AttributeError, TypeError):
        end_pos = None
        end_heading = None

    dist = None
    if start_pos is not None and end_pos is not None:
        dist = math.hypot(end_pos[0] - start_pos[0],
                          end_pos[1] - start_pos[1])
    dist_str = "dist=N/A" if dist is None else f"dist={dist:.3f}m"
    err_str = "err=N/A" if last_err is None else f"err={last_err:.1f}deg"
    print(f"[Seg5] Low-crawl turn {'reached' if reached else 'stopped'}: "
          f"end_pos={end_pos}, heading={end_heading}, {err_str}, {dist_str}")
    return reached