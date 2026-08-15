"""
Segment 6 fixed-start test: wide-gait pass-through, narrow-gait ball carry.

This follows the proven strategy in project3130810-374577-main-src/race/6.py:
  1. Calibrate current pose as (2.40, 13.50, 180deg).
  2. Walk to an approach point behind the known ball.
  3. Use wide gait (gait_id=27, value=0) to pass over the ball.
  4. Use narrow gait (gait_id=5, value=0) to carry the ball to the exit.
  5. Lie down near (3.16, 12.90, 0deg).
"""
import math
import os
import sys
import time

from ...utils.nav_helper import go_to_waypoint, turn_to_angle


SEG6_IMPL_VERSION = "seg6_fixed_start_carry_ball_v1"

YAML_KIND_VEC_X_DOUBLE = 3
SEG6_RUNTIME_GAIT_PARAMS = {
    "des_roll_pitch_height": [0.0, 0.0, 0.255],
    "des_roll_pitch_height_motion": [0.0, 0.0, 0.255],
    "des_roll_pitch_height_stair": [0.0, 0.0, 0.255],
    "y_offset_trot_10_4": [0.015, -0.015, 0.065, -0.065],
    "y_offset_trot": [-0.05, 0.05, -0.05, 0.05],
}
DEFAULT_RUNTIME_GAIT_PARAMS = {
    "des_roll_pitch_height": [0.0, 0.0, 0.35],
    "des_roll_pitch_height_motion": [0.0, 0.0, 0.35],
    "des_roll_pitch_height_stair": [0.0, 0.0, 0.35],
    "y_offset_trot_10_4": [0.025, -0.035, 0.025, -0.035],
    "y_offset_trot": [0.04, -0.04, 0.04, -0.04],
}

FIXED_START_X = 2.40
FIXED_START_Y = 13.50
FIXED_START_YAW_DEG = 180.0

BALL_X = 0.40 # 0.30 14.70
BALL_Y = 14.70
BALL_APPROACH_TOL_M = 0.05

PRE_BALL_APPROACH_X = 1.2
PRE_BALL_APPROACH_Y = 14.225
PRE_BALL_APPROACH_TOL_M = 0.06

STAGE6_X_MIN = 0.13
STAGE6_X_MAX = 3.13
STAGE6_Y_MIN = 13.20
STAGE6_Y_MAX = 15.70

GAIT_WIDE = 27
GAIT_NARROW = 5
GAIT_STAND = 31

VALUE_HIGH_POSTURE = 0
STEP_H_WIDE = 0.08
STEP_H_NARROW = 0.04

NAV_TOL = 0.14
NAV_TIMEOUT_S = 70.0
TURN_TIMEOUT_S = 40.0
TURN_TOL_DEG = 1.0
FINE_TURN_TOL_DEG = 1.0

WALKTHROUGH_CLOSE_DIST = 0.18
WALKTHROUGH_XY_TOL = 0.010
WALKTHROUGH_TIMEOUT_S = 50.0






CARRY_MID_X = 2.4
CARRY_MID_Y = 13.10

CARRY_MID_TIMEOUT_S = 90.0

CARRY_FINAL_X = 2.90
CARRY_FINAL_Y = 13.00
CARRY_FINAL_TIMEOUT_S = 60.0
CARRY_FINAL_X_TOL = 0.025
CARRY_FINAL_Y_TOL = 0.025

FINAL_X = 3.10
FINAL_Y = 13.05
FINAL_YAW_DEG = 0.0


def execute(ctrl, perception, speaker, navigator=None):
    del navigator
    print(f"[Seg6] Starting Segment 6 ({SEG6_IMPL_VERSION})")
    _publish_gait_params(perception, SEG6_RUNTIME_GAIT_PARAMS, "apply seg6 gait params")
    try:
        _calibrate_fixed_start(perception)
        _log_pose(perception, "[Seg6] initial")
        print(f"[Seg6] known ball=({BALL_X:.2f},{BALL_Y:.2f}) final=({FINAL_X:.2f},{FINAL_Y:.2f},0)")
        print("[Seg6] strategy: wide gait through ball -> narrow gait carry -> lie down")

        if speaker:
            try:
                speaker.announce("football")
            except Exception:
                pass

        _stand(ctrl, settle_s=0.4)

        if not _walk_through_ball(ctrl, perception):
            print("[Seg6] Warning: walkthrough not fully confirmed; continuing to carry phase")

        _carry_ball_to_exit(ctrl, perception)
        _finish(ctrl, perception)
        return True
    finally:
        _publish_gait_params(perception, DEFAULT_RUNTIME_GAIT_PARAMS, "restore default gait params")


def _walk_through_ball(ctrl, perception):
    approach_x = PRE_BALL_APPROACH_X
    approach_y = PRE_BALL_APPROACH_Y

    px, py = _safe_position(perception)
    approach_yaw = math.degrees(math.atan2(approach_y - py, approach_x - px)) % 360.0
    _debug_turn_start(perception, approach_yaw, "pre-approach")
    turn_to_angle(ctrl, perception, approach_yaw, tol_deg=TURN_TOL_DEG, timeout_s=TURN_TIMEOUT_S)
    _debug_turn_done(perception, "pre-approach")
    _stand(ctrl, settle_s=0.3)

    print(f"[Seg6] Step1 wide-through: pre-ball approach=({approach_x:.3f},{approach_y:.3f}) "
          f"ball=({BALL_X:.2f},{BALL_Y:.2f})")

    ok = go_to_waypoint(
        ctrl,
        perception,
        (approach_x, approach_y),
        speed=0.22,
        tol_m=PRE_BALL_APPROACH_TOL_M,
        max_wz=0.6,
    )
    if not ok:
        px, py = _safe_position(perception)
        print(f"[Seg6] pre-ball approach failed at pos=({px:.3f},{py:.3f}); abort walkthrough")
        _stand(ctrl, settle_s=0.2)
        return False

    _debug_reached(perception, approach_x, approach_y, "pre-approach")

    px, py = _safe_position(perception)
    print(f"[Seg6] pre-ball approach reached pos=({px:.3f},{py:.3f})")
    walk_yaw = math.atan2(BALL_Y - py, BALL_X - px)
    walk_yaw_deg = math.degrees(walk_yaw) % 360.0
    _debug_turn_start(perception, walk_yaw_deg, "to-ball")
    turn_to_angle(ctrl, perception, walk_yaw_deg, tol_deg=TURN_TOL_DEG, timeout_s=TURN_TIMEOUT_S)
    _debug_turn_done(perception, "to-ball")

    print("[Seg6] wide gait active: gait_id=27 value=0 step_h=0.08")
    ok_ball = go_to_waypoint(
        ctrl,
        perception,
        (BALL_X, BALL_Y),
        speed=0.22,
        tol_m=BALL_APPROACH_TOL_M,
        max_wz=0.18,
        gait_id=GAIT_WIDE,
        step_h_max=STEP_H_WIDE,
        step_h_min=STEP_H_WIDE,
    )
    if ok_ball:
        _stand(ctrl, settle_s=0.2)
        _log_pose(perception, "[Seg6] ball waypoint reached")
        _debug_reached(perception, BALL_X, BALL_Y, "ball")
        px, py = _safe_position(perception)
        print(f"[Seg6] ball reached pos=({px:.3f},{py:.3f})")
        return True

    px, py = _safe_position(perception)
    print(f"[Seg6] ball waypoint failed at pos=({px:.3f},{py:.3f}); continue with close lock")

    start = time.perf_counter()
    step = 0
    x_locked = False
    y_locked = False
    last_debug_t = start

    while time.perf_counter() - start < WALKTHROUGH_TIMEOUT_S:
        px, py = _safe_position(perception)
        heading = _safe_heading_deg(perception)
        dist = math.hypot(BALL_X - px, BALL_Y - py)

        if dist <= WALKTHROUGH_CLOSE_DIST:
            x_err = BALL_X - px
            y_err = BALL_Y - py
            if abs(x_err) <= WALKTHROUGH_XY_TOL:
                x_locked = True
            if abs(y_err) <= WALKTHROUGH_XY_TOL:
                y_locked = True
            if x_locked and y_locked:
                print(f"[Seg6] walkthrough centered pos=({px:.3f},{py:.3f}) dist={dist:.3f}")
                break

            world_vx = 0.0 if x_locked else _clamp(1.4 * x_err, -0.08, 0.08)
            world_vy = 0.0 if y_locked else _clamp(1.4 * y_err, -0.08, 0.08)
            vx, vy = _world_to_body(world_vx, world_vy, heading)
        else:
            vx = 0.18 if dist < 0.30 else 0.50 if dist < 0.60 else 0.65
            vy = 0.0

        yaw_ref = math.degrees(math.atan2(BALL_Y - py, BALL_X - px)) % 360.0
        if dist <= WALKTHROUGH_CLOSE_DIST:
            yaw_ref = walk_yaw_deg
        yaw_err = _angle_diff(yaw_ref, heading)
        wz = _clamp(math.radians(yaw_err) * 1.2, -0.18, 0.18)
        _send_wide(ctrl, vx=vx, vy=vy, wz=wz)

        step += 1
        now = time.perf_counter()
        if now - last_debug_t >= 1.0:
            print(f"[Seg6] walkthrough pos=({px:.3f},{py:.3f}) heading={heading:.1f} "
                  f"dist_ball={dist:.3f} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
            last_debug_t = now
        time.sleep(0.10)

    _stand(ctrl, settle_s=0.3)
    _log_pose(perception, "[Seg6] walkthrough done")
    return True


def _carry_ball_to_exit(ctrl, perception):
    print("[Seg6] Step2 narrow carry: turn toward mid exit path")
    px, py = _safe_position(perception)
    mid_yaw_deg = math.degrees(math.atan2(CARRY_MID_Y - py, CARRY_MID_X - px)) % 360.0
    _debug_turn_start(perception, mid_yaw_deg, "to-carry-mid")
    turn_to_angle(ctrl, perception, mid_yaw_deg, tol_deg=TURN_TOL_DEG, timeout_s=TURN_TIMEOUT_S)
    _debug_turn_done(perception, "to-carry-mid")
    _stand(ctrl, settle_s=0.25)
    _preheat_narrow(ctrl)

    print(f"[Seg6] narrow gait carry mid: target=({CARRY_MID_X:.2f},{CARRY_MID_Y:.2f})")
    ok_mid = go_to_waypoint(
        ctrl,
        perception,
        (CARRY_MID_X, CARRY_MID_Y),
        speed=0.26,
        tol_m=0.05,
        max_wz=0.35,
        gait_id=GAIT_NARROW,
        step_h_max=STEP_H_NARROW,
        step_h_min=STEP_H_NARROW,
    )
    if not ok_mid:
        px, py = _safe_position(perception)
        print(f"[Seg6] Warning: carry-mid waypoint timeout at pos=({px:.3f},{py:.3f}); continuing")
    else:
        _debug_reached(perception, CARRY_MID_X, CARRY_MID_Y, "carry-mid")

    _stand(ctrl, settle_s=0.2)

    print("[Seg6] Step3 narrow carry: turn +X and move to exit mouth")
    _debug_turn_start(perception, 0.0, "to-exit")
    turn_to_angle(ctrl, perception, 0.0, tol_deg=TURN_TOL_DEG, timeout_s=TURN_TIMEOUT_S)
    _debug_turn_done(perception, "to-exit")
    _debug_turn_start(perception, 0.0, "to-exit-fine")
    turn_to_angle(ctrl, perception, 0.0, tol_deg=FINE_TURN_TOL_DEG, timeout_s=6.0)
    _debug_turn_done(perception, "to-exit-fine")
    _preheat_narrow(ctrl)
    _carry_final(ctrl, perception)

    print("[Seg6] Wide gait forward 15cm to release ball into exit")
    px, py = _safe_position(perception)
    release_target = (px + 0.15, py)
    _debug_turn_start(perception, 0.0, "to-release")
    turn_to_angle(ctrl, perception, 0.0, tol_deg=5.0, timeout_s=10.0)
    _debug_turn_done(perception, "to-release")
    go_to_waypoint(
        ctrl,
        perception,
        release_target,
        speed=0.22,
        tol_m=0.06,
        max_wz=0.35,
        gait_id=GAIT_NARROW,
        step_h_max=STEP_H_NARROW,
        step_h_min=STEP_H_NARROW,
    )
    _debug_reached(perception, release_target[0], release_target[1], "release")
    _stand(ctrl, settle_s=0.3)


def _carry_final(ctrl, perception):
    print(f"[Seg6] narrow carry final: target=({CARRY_FINAL_X:.2f},{CARRY_FINAL_Y:.2f})")
    start = time.perf_counter()
    step = 0
    x_locked = False
    y_locked = False
    last_debug_t = start
    while time.perf_counter() - start < CARRY_FINAL_TIMEOUT_S:
        px, py = _safe_position(perception)
        heading = _safe_heading_deg(perception)
        x_err = CARRY_FINAL_X - px
        y_err = CARRY_FINAL_Y - py
        if abs(x_err) <= CARRY_FINAL_X_TOL:
            x_locked = True
        if abs(y_err) <= CARRY_FINAL_Y_TOL:
            y_locked = True
        if x_locked and y_locked:
            print(f"[Seg6] carry final reached pos=({px:.3f},{py:.3f})")
            break

        vx = 0.0 if x_locked else _clamp(1.0 * x_err, -0.18, 0.24)
        vy = 0.0 if y_locked else _clamp(3.0 * y_err, -0.16, 0.16)
        yaw_err = _angle_diff(0.0, heading)
        wz = _clamp(math.radians(yaw_err) * 1.8, -0.35, 0.35)
        _send_narrow(ctrl, vx=vx, vy=vy, wz=wz)

        step += 1
        now = time.perf_counter()
        if now - last_debug_t >= 1.0:
            print(f"[Seg6] carry-final pos=({px:.3f},{py:.3f}) heading={heading:.1f} "
                  f"x_err={x_err:.3f} y_err={y_err:.3f} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
            last_debug_t = now
        time.sleep(0.10)
    _debug_reached(perception, CARRY_FINAL_X, CARRY_FINAL_Y, "carry-final")
    _stand(ctrl, settle_s=0.2)


def _drive_to_xy(ctrl, perception, target_x, target_y, tol, timeout_s, label):
    start = time.perf_counter()

    # Phase 1: Turn toward target direction
    px, py = _safe_position(perception)
    heading = _safe_heading_deg(perception)
    dx = target_x - px
    dy = target_y - py
    target_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
    yaw_err = _angle_diff(target_yaw, heading)
    print(f"[Seg6] {label} turn first: target_yaw={target_yaw:.1f} cur_heading={heading:.1f} yaw_err={yaw_err:.1f}")

    if abs(yaw_err) > 5.0:
        turn_ok = turn_to_angle(ctrl, perception, target_yaw, tol_deg=5.0, timeout_s=TURN_TIMEOUT_S)
        if not turn_ok:
            print(f"[Seg6] {label} turn timeout, continuing anyway")
        _stand(ctrl, settle_s=0.2)
    _debug_turn_done(perception, label)
    _debug_reached(perception, target_x, target_y, label)

    # Phase 2: Walk to target
    step = 0
    last_debug_t = start
    while time.perf_counter() - start < timeout_s:
        px, py = _safe_position(perception)
        heading = _safe_heading_deg(perception)
        dx = target_x - px
        dy = target_y - py
        dist = math.hypot(dx, dy)
        if dist <= tol:
            _stand(ctrl, settle_s=0.1)
            _debug_reached(perception, target_x, target_y, label)
            return True

        target_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
        yaw_err = _angle_diff(target_yaw, heading)
        world_speed = 0.28 if dist > 0.60 else 0.14
        world_vx = world_speed * dx / max(dist, 1e-6)
        world_vy = world_speed * dy / max(dist, 1e-6)
        vx, vy = _world_to_body(world_vx, world_vy, heading)
        vy = _clamp(vy, -0.14, 0.14)
        wz = _clamp(math.radians(yaw_err) * 1.4, -0.40, 0.40)

        _send_wide(ctrl, vx=vx, vy=vy, wz=wz)

        step += 1
        now = time.perf_counter()
        if now - last_debug_t >= 1.0:
            print(f"[Seg6] {label} pos=({px:.3f},{py:.3f}) heading={heading:.1f} "
                  f"target=({target_x:.2f},{target_y:.2f}) dist={dist:.3f} "
                  f"yaw_err={yaw_err:.1f} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
            last_debug_t = now
        time.sleep(0.10)

    _stand(ctrl, settle_s=0.1)
    px, py = _safe_position(perception)
    print(f"[Seg6] {label} timeout pos=({px:.3f},{py:.3f}) "
          f"target=({target_x:.2f},{target_y:.2f})")
    return False


def _relock_ball_center(ctrl, perception, timeout_s=4.0):
    """Briefly re-center the robot over the ball using the same XY lock logic as walkthrough."""
    start = time.perf_counter()
    x_locked = False
    y_locked = False
    while time.perf_counter() - start < timeout_s:
        px, py = _safe_position(perception)
        heading = _safe_heading_deg(perception)
        dist = math.hypot(BALL_X - px, BALL_Y - py)

        if dist > max(WALKTHROUGH_CLOSE_DIST, 0.22):
            return False

        x_err = BALL_X - px
        y_err = BALL_Y - py

        if abs(x_err) <= WALKTHROUGH_XY_TOL:
            x_locked = True
        if abs(y_err) <= WALKTHROUGH_XY_TOL:
            y_locked = True

        if x_locked and y_locked:
            _stand(ctrl, settle_s=0.1)
            return True

        world_vx = 0.0 if x_locked else _clamp(1.4 * x_err, -0.06, 0.06)
        world_vy = 0.0 if y_locked else _clamp(1.4 * y_err, -0.06, 0.06)
        vx, vy = _world_to_body(world_vx, world_vy, heading)
        wz = _clamp(math.radians(_angle_diff(0.0, heading)) * 0.6, -0.12, 0.12)
        _send_wide(ctrl, vx=vx, vy=vy, wz=wz)
        time.sleep(0.10)

    _stand(ctrl, settle_s=0.1)
    return False


def _turn_to_heading_segmented_with_relock(ctrl, perception, target_deg, chunk_deg=35.0,
                                          tol_deg=TURN_TOL_DEG, timeout_s=TURN_TIMEOUT_S,
                                          label="seg-turn"):
    del ctrl, perception, target_deg, chunk_deg, tol_deg, timeout_s, label
    raise NotImplementedError("segmented relock turning removed; use turn_to_angle")


def _stand(ctrl, settle_s=0.25):
    ctrl.locomotion(
        gait_id=GAIT_STAND,
        vx=0.0,
        vy=0.0,
        wz=0.0,
        step_h_max=STEP_H_WIDE,
        step_h_min=STEP_H_WIDE,
        value=VALUE_HIGH_POSTURE,
        duration_ms=0,
    )
    time.sleep(settle_s)


def _preheat_narrow(ctrl):
    print("[Seg6] preheat narrow gait: 15 zero-speed commands")
    for _ in range(15):
        _send_narrow(ctrl, vx=0.0, vy=0.0, wz=0.0)
        time.sleep(0.08)
    _stand(ctrl, settle_s=0.15)


def _send_wide(ctrl, vx=0.0, vy=0.0, wz=0.0):
    ctrl.locomotion(
        gait_id=GAIT_WIDE,
        vx=float(vx),
        vy=float(vy),
        wz=float(wz),
        step_h_max=STEP_H_WIDE,
        step_h_min=STEP_H_WIDE,
        value=VALUE_HIGH_POSTURE,
        duration_ms=0,
    )


def _send_narrow(ctrl, vx=0.0, vy=0.0, wz=0.0):
    ctrl.locomotion(
        gait_id=GAIT_NARROW,
        vx=float(vx),
        vy=float(vy),
        wz=float(wz),
        step_h_max=STEP_H_NARROW,
        step_h_min=STEP_H_NARROW,
        value=VALUE_HIGH_POSTURE,
        duration_ms=0,
    )


def _finish(ctrl, perception):
    _drive_to_xy(ctrl, perception, FINAL_X, FINAL_Y, 0.18, 14.0, "final point")
    _debug_turn_start(perception, FINAL_YAW_DEG, "final-yaw")
    turn_to_angle(ctrl, perception, FINAL_YAW_DEG, tol_deg=8.0, timeout_s=6.0)
    _debug_turn_done(perception, "final-yaw")
    _stand(ctrl, settle_s=0.5)
    ctrl.pure_damper()
    time.sleep(1.2)
    _log_pose(perception, "[Seg6] finished")
    print("[Seg6] Robot lying down, segment 6 complete")


def _publish_gait_params(perception, params, label):
    YamlParam = _load_yaml_param_msg()
    if YamlParam is None:
        print(f"[Seg6] {label}: cyberdog_msg unavailable, skip runtime gait params")
        return False

    try:
        publisher = perception.create_publisher(YamlParam, "/yaml_parameter", 10)
    except Exception as exc:
        print(f"[Seg6] {label}: cannot create /yaml_parameter publisher ({exc})")
        return False

    print(f"[Seg6] {label}: {', '.join(params.keys())}")
    for _ in range(4):
        for name, values in params.items():
            msg = YamlParam()
            msg.name = name
            msg.kind = YAML_KIND_VEC_X_DOUBLE
            msg.is_user = 1
            padded = [0.0] * 12
            for i, value in enumerate(values[:12]):
                padded[i] = float(value)
            msg.vecxd_value = padded
            publisher.publish(msg)
        time.sleep(0.05)
    time.sleep(0.25)
    return True


def _load_yaml_param_msg():
    try:
        from cyberdog_msg.msg import YamlParam
        return YamlParam
    except Exception:
        pass

    for base in (
        "/home/cyberdog_sim/install/cyberdog_msg/lib/python3.8/site-packages",
        "/home/cyberdog_sim/install/cyberdog_msg/local/lib/python3.8/dist-packages",
        "/home/cyberdog_sim/install/lib/python3.8/site-packages",
        "/home/cyberdog_sim/install/local/lib/python3.8/dist-packages",
        "/home/cyberdog_sim/install/cyberdog_msg/lib/python3.10/site-packages",
        "/home/cyberdog_sim/install/cyberdog_msg/local/lib/python3.10/dist-packages",
        "/home/cyberdog_sim/install/lib/python3.10/site-packages",
        "/home/cyberdog_sim/install/local/lib/python3.10/dist-packages",
    ):
        if os.path.isdir(base) and base not in sys.path:
            sys.path.insert(0, base)
    try:
        from cyberdog_msg.msg import YamlParam
        return YamlParam
    except Exception as exc:
        print(f"[Seg6] cyberdog_msg import failed after path fallback: {exc}")
        return None


def _calibrate_fixed_start(perception):
    try:
        raw_x, raw_y = perception.position
        raw_heading = _safe_heading_deg(perception)
        offset_x = FIXED_START_X - float(raw_x)
        offset_y = FIXED_START_Y - float(raw_y)
        offset_yaw = math.radians(_angle_diff(FIXED_START_YAW_DEG, raw_heading))
        perception.set_position_offset(offset_x, offset_y, offset_yaw)
        print(f"[Seg6] fixed-start calibration: raw=({raw_x:.3f},{raw_y:.3f}) "
              f"heading={raw_heading:.1f} offset=({offset_x:.3f},{offset_y:.3f},"
              f"{math.degrees(offset_yaw):.1f}deg)")
    except Exception as exc:
        print(f"[Seg6] fixed-start calibration skipped: {exc}")


def _safe_position(perception):
    try:
        px, py = perception.position
        return float(px), float(py)
    except Exception:
        odom = getattr(perception, "odom_pose", (0.0, 0.0, 0.0))
        return float(odom[0]), float(odom[1])


def _safe_heading_deg(perception):
    try:
        return float(perception.heading) % 360.0
    except Exception:
        odom = getattr(perception, "odom_pose", (0.0, 0.0, 0.0))
        return math.degrees(float(odom[2])) % 360.0


def _log_pose(perception, prefix):
    px, py = _safe_position(perception)
    heading = _safe_heading_deg(perception)
    print(f"{prefix} pos=({px:.3f},{py:.3f}) heading={heading:.1f}")


def _debug_pose(perception, label=""):
    """Print current position and heading with optional label."""
    px, py = _safe_position(perception)
    heading = _safe_heading_deg(perception)
    print(f"[Seg6] {label} pos=({px:.3f},{py:.3f}) heading={heading:.1f}")


def _debug_turn_start(perception, target_deg, label):
    """Print turn start info with position, current heading, and target heading."""
    px, py = _safe_position(perception)
    heading = _safe_heading_deg(perception)
    yaw_err = _angle_diff(target_deg, heading)
    print(f"[Seg6] TURN START {label}: target={target_deg:.1f} cur={heading:.1f} "
          f"pos=({px:.3f},{py:.3f}) err={yaw_err:.1f}")


def _debug_turn_done(perception, label):
    """Print turn done info with position and final heading."""
    px, py = _safe_position(perception)
    heading = _safe_heading_deg(perception)
    print(f"[Seg6] TURN DONE {label}: pos=({px:.3f},{py:.3f}) heading={heading:.1f}")


def _debug_reached(perception, target_x, target_y, label):
    """Print waypoint reached info with position, target, and heading."""
    px, py = _safe_position(perception)
    heading = _safe_heading_deg(perception)
    print(f"[Seg6] REACHED {label}: pos=({px:.3f},{py:.3f}) heading={heading:.1f} "
          f"target=({target_x:.3f},{target_y:.3f})")


def _world_to_body(world_vx, world_vy, heading_deg):
    h = math.radians(heading_deg)
    vx = math.cos(h) * world_vx + math.sin(h) * world_vy
    vy = -math.sin(h) * world_vx + math.cos(h) * world_vy
    return vx, vy


def _angle_diff(target_deg, current_deg):
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def _clamp(value, low, high):
    return max(low, min(high, value))