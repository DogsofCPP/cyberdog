"""
Segment 4: Deep Tunnel (深隧寻珍)
World-coordinate based navigation with systematic object interaction.
"""
import os
import time
import math
from ...lcm_types import GAIT_TROT_SLOW, GAIT_TROT_FAST, GAIT_LOW_CRAWL
from ...perception.detectors.ball_detector import BallDetector
from ...perception.detectors.object_detector import ObjectDetector
from ...utils.nav_helper import go_to_waypoint, turn_to_angle


SEGMENT_TIMEOUT_S = 240.0
TARGET_OBJECTS = {'coke', 'orange_ball', 'football'}

# World-frame target positions for Segment 4 objects
TARGET_POSITIONS = {
    'coke':        (-0.10, 11.0),
    'orange_ball': ( 0.95, 11.0),
    'football':    ( 2.1,  10.8),
}

# Snake path through deep tunnel.
# Objects are at: coke (-0.1, 11.1), orange_ball (0.95, 11.1), football (2.1, 10.8)
S4_WAYPOINTS = [
    ( 3.0,  7.2, 180.0),    # 0: entry/start (facing west)
    (-0.10, 7.2, 90.0),    # 1: left end of row 1
    (-0.10, 8.6, 90.0),     #new_position
    (-0.10,11.1, 270.0),     # 2: coke bottle (facing south, Y-down)
    (-0.10, 10.0, 270.0),    # 3: post-coke staging point
    (-0.10, 7.2, 0.0),   # 4: back up row 1 (facing north)
    ( 0.95, 7.2, 68.5),    # 5: middle transfer point
    ( 1.8,  8.9, 120.3),    # 6: detour to avoid obstacle near x=0.95 column
    ( 0.95, 9.0,  90.0),    # 7: approach x=0.9 5 column below obstacle zone
    ( 0.95,11.0,  270.0),    # 8: orange ball  (facing south, Y-down)
    ( 0.95, 8.9,  0.0),   # 9: back up row 2 (facing north)
    ( 2.2,  8.9,  90.0),    # 10: right to row 3
    ( 2.1, 10.95,  270.0),    # 11: football  (facing south, Y-down)
    ( 2.1,  7.2,  0.0),   # 12: back up row 3 (facing north)
    ( 3.15,  7.2,  90.0),    # 13: return to entry (facing east)
]

# Waypoint indices that must use low-crawl.
# Grouped by which height bar they pass through.
LOW_POSITIONS_1 = {3, 4, 5, 6}       # height bar at Y=9.27
LOW_POSITIONS_2 = {12, 13}         # height bar at Y=10.6

ANNOUNCEMENT_MAP = {
    'coke': '识别到可乐瓶',
    'orange_ball': '识别到橙色小球',
    'football': '识别到足球',
    'height_bar': '识别到限高杆',
    'obstacle': '识别到无法跨越障碍',
}


# ---------------------------------------------------------------------------
# Low-crawl gait for height-bar passages (body height = 0.10m)
# Smooth diagonal-trot with optimized parameters.
# step_height encoding: [fr_mm + fl_mm*1e4, rr_mm + rl_mm*1e4]
# Step height reduced to 18mm for stability during low-crawl
# ---------------------------------------------------------------------------
_LC_STEP_H = 18018.0    # all 4 legs 18mm (encoded: 18 + 18*1e4)

# Low-pitch for bar clearance. Keep this moderate; too much pitch + low z
# can drive WBC torque to its limits and make the legs look stuck.
_LOW_CRAWL_PITCH = -0.14
_LOW_CRAWL_HEIGHT = 0.10
_LOW_CRAWL_GAIT_Z = -0.16

# Smooth low-crawl — alternating diagonal pairs (FR+RL vs FL+RR)
# 2-phase cycle: "forward" step (push body) + "backward" step (prepare next push)
# vel_des=[0.10,0,0,0] = forward velocity for crawling through height bar
LOW_CRAWL_STEPS = [
    # Phase 1: FR+RL lift forward, FL+RR support
    dict(mode=11, gait_id=110, contact=10,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[ 0.10, 0.0, 0.40],
         foot_pose=[ 0.10, 0.0, -0.03,  0.0,  0.0, -0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=300),
    dict(mode=11, gait_id=110, contact=10,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[ 0.10, 0.0, 0.40],
         foot_pose=[ 0.10, 0.0, -0.03,  0.0,  0.0, -0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=200),
    # Phase 1 return: FR+RL back, FL+RR support
    dict(mode=11, gait_id=110, contact=10,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[-0.10, 0.0, 0.40],
         foot_pose=[-0.03, 0.0, -0.03,  0.0,  0.0, -0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=300),
    dict(mode=11, gait_id=110, contact=10,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[-0.10, 0.0, 0.40],
         foot_pose=[-0.03, 0.0, -0.03,  0.0,  0.0, -0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=200),
    # Phase 2: FL+RR lift forward, FR+RL support
    dict(mode=11, gait_id=110, contact=5,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[ 0.10, 0.0, 0.40],
         foot_pose=[ 0.0, 0.0,  0.03,  0.0, -0.03,  0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=300),
    dict(mode=11, gait_id=110, contact=5,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[ 0.10, 0.0, 0.40],
         foot_pose=[ 0.0, 0.0,  0.03,  0.0, -0.03,  0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=200),
    # Phase 2 return: FL+RR back, FR+RL support
    dict(mode=11, gait_id=110, contact=5,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[-0.10, 0.0, 0.40],
         foot_pose=[ 0.0, 0.0, -0.03,  0.0, -0.03, -0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=300),
    dict(mode=11, gait_id=110, contact=5,
         vel_des=[0.10, 0.0, 0.0, 0.0],
         rpy_des=[0.0, _LOW_CRAWL_PITCH, 0.0],
         pos_des=[0.0, 0.0, _LOW_CRAWL_GAIT_Z],
         acc_des=[1.0, 1.0, 1.0, 1.5, 1.5, 2.0],
         ctrl_point=[-0.10, 0.0, 0.40],
         foot_pose=[ 0.0, 0.0, -0.03,  0.0, -0.03, -0.03],
         step_height=[_LC_STEP_H, _LC_STEP_H],
         value=0, duration=200),
]


def _announce(speaker, obj_type):
    msg = ANNOUNCEMENT_MAP.get(obj_type, obj_type)
    if speaker:
        try:
            speaker.announce(obj_type)
            return
        except Exception:
            pass
    print(f"[Seg4] Announce: {msg}")


def execute(ctrl, perception, speaker, navigator=None, announced=None):
    """Execute segment 4 deep tunnel navigation.

    Snake path: entry at (3.0, 7.0) facing -X (yaw=180°).
    Robot weaves left-to-right through the tunnel, visiting each object row.

    Path:
      (3,7) -> (-0.1,7) -> (-0.1,11.1) -> (-0.1,7) -> (0.95,7) ->
      (0.95,11.1) -> (0.95,7) -> (2.1,7) -> (2.1,10.8) ->
      (2.1,7) -> (3,7)

    Objects: coke at (-0.1, 11.1), orange_ball at (0.95, 11.1),
    football at (2.1, 10.8).

    Args:
        ctrl:       LCMController instance
        perception: ROS2Perception / MockPerception
        speaker:    VoiceSpeaker instance
        navigator:  Navigator instance (unused, kept for API compat)
        announced:  Set of already-announced object types
    """
    announced = announced or set()
    print("[Seg4] Starting Deep Tunnel")
    print(f"[Seg4] Start position={perception.position}, heading={perception.heading:.1f} deg")
    print(f"[Seg4] Waypoints: {S4_WAYPOINTS}")

    ball_det = BallDetector(ball_height_m=0.60)
    obj_det = ObjectDetector()

    seg_start = time.perf_counter()
    obj_handled = set()

    # Background object detection while at entry
    for _ in range(8):
        elapsed = time.perf_counter() - seg_start
        if elapsed > SEGMENT_TIMEOUT_S:
            ctrl.stop_moving()
            return True
        objects, orange_balls = _detect_scene(perception, obj_det, ball_det)
        _handle_scene_objects(ctrl, perception, speaker, announced, obj_handled,
                             objects, orange_balls, ball_det)
        time.sleep(0.05)

    # Main navigation loop: iterate through waypoints 1..N
    for idx in range(1, len(S4_WAYPOINTS)):
        wx, wy, wyaw = S4_WAYPOINTS[idx]
        print(f"[Seg4] WP{idx}: going to ({wx}, {wy})")

        # Legs passing through y≈9 (height-bar zone): go down AND come back up both have bars.
        # Only the middle lane (WP4→WP5 / WP6←WP5) is clear.
        # Determine which height bar applies based on waypoint index.
        low_leg_1 = idx in LOW_POSITIONS_1
        low_leg_2 = idx in LOW_POSITIONS_2
        low_leg = low_leg_1 or low_leg_2
        bar_id = 1 if low_leg_1 else (2 if low_leg_2 else None)
        if low_leg:
            reached = _go_to_waypoint_low(ctrl, perception, (wx, wy), wyaw,
                                          speed=0.18, tol_m=0.20, bar_id=bar_id)
        else:
            reached = go_to_waypoint(
                ctrl, perception, (wx, wy),
                speed=0.24, tol_m=0.10,
            )
        print(f"[Seg4] WP{idx}: reached ({perception.position[:2]}), "
              f"heading={perception.heading:.1f} deg")

        # Turn to explicit heading if specified (precision matching segment_2)
        if wyaw is not None:
            turn_to_angle(ctrl, perception, wyaw,
                          turn_rate=0.65, tol_deg=3.0, timeout_s=7.0)
            print(f"[Seg4] WP{idx}: turned to heading={wyaw:.1f} deg")

        # Re-apply low height after turning (stop_moving resets height)
        if low_leg:
            ctrl.set_height(0.10, duration_ms=300)
            time.sleep(0.4)

        # Object interaction if needed
        obj_at_wp = _get_object_at_waypoint(idx)
        if obj_at_wp and obj_at_wp not in obj_handled:
            print(f"[Seg4] Interacting with {obj_at_wp}")
            _interact_object(ctrl, perception, obj_at_wp)
            obj_handled.add(obj_at_wp)

    # All waypoints done
    print(f"[Seg4] All waypoints done: {perception.position}, "
          f"heading={perception.heading:.1f} deg, "
          f"handled={sorted(obj_handled)}")
    ctrl.stop_moving()
    return True


def _go_to_waypoint_precise(ctrl, perception, waypoint_xy,
                            coarse_speed=0.18, coarse_tol_m=0.30,
                            use_lidar_strafe=True):
    """Drive to a waypoint using the coarse arrival threshold only."""
    go_to_waypoint(ctrl, perception, waypoint_xy,
                   speed=coarse_speed, tol_m=coarse_tol_m,
                   use_lidar_strafe=use_lidar_strafe)
    ctrl.stop_moving()
    return True


def test_low_crawl_from_start(ctrl, perception, duration_s=18.0, speed=0.06,
                              method='user-gait', gait_z=_LOW_CRAWL_GAIT_Z,
                              gait_pitch=None, cycles=None, lateral=0.04,
                              rest_every=1, head_down_pitch=-0.20):
    """Temporary Segment-4 low-crawl posture test.

    This starts from the current robot position, then tests a selectable
    low-crawl method while printing measured body height and pitch.
    """
    low_height = _LOW_CRAWL_HEIGHT
    low_pitch = _LOW_CRAWL_PITCH if gait_pitch is None else gait_pitch
    low_step_height = 0.015
    crawl_speed = max(0.0, min(speed, 0.08))

    try:
        start_pos = perception.position[:2]
        start_heading = perception.heading
    except (AttributeError, TypeError):
        start_pos = None
        start_heading = None

    print("[Seg4-Test] Starting low-crawl-from-start test")
    print(f"[Seg4-Test] Target height={low_height:.2f}m, "
          f"pitch={low_pitch:.2f}rad ({math.degrees(low_pitch):.1f}deg), "
          f"speed={crawl_speed:.2f}m/s")
    print(f"[Seg4-Test] Method={method}, user_gait_z={gait_z:.2f}, "
          f"gait_pitch={low_pitch:.2f}, cycles={cycles}, "
          f"lateral={lateral:.2f}, rest_every={rest_every}, "
          f"head_down_pitch={head_down_pitch:.2f}")
    print(f"[Seg4-Test] Start position={start_pos}, heading={start_heading}")

    if method == 'user-gait':
        return _test_low_crawl_user_gait(
            ctrl, perception, duration_s, crawl_speed, gait_z, low_pitch,
            cycles=cycles, rest_every=rest_every)
    if method == 'legacy-user-gait':
        return _test_legacy_low_crawl_user_gait(
            ctrl, perception, duration_s, crawl_speed)
    if method == 'user-gait-side':
        return _test_low_crawl_user_gait(
            ctrl, perception, duration_s, crawl_speed, gait_z, low_pitch,
            cycles=cycles, lateral=lateral, rest_every=rest_every)
    if method == 'head-down':
        return _test_head_down_walk(
            ctrl, perception, duration_s, crawl_speed, head_down_pitch,
            cycles=cycles)
    if method == 'front-stretch-back':
        return _test_front_stretch_back(
            ctrl, perception, duration_s, gait_z, cycles=cycles,
            forward=False)
    if method == 'front-stretch-forward':
        return _test_front_stretch_back(
            ctrl, perception, duration_s, gait_z, cycles=cycles,
            forward=True)
    if method == 'pose':
        return _test_low_crawl_pose_hold(
            ctrl, perception, duration_s, low_height, low_pitch)
    if method != 'pulse':
        print(f"[Seg4-Test] Unknown method '{method}', using pulse")

    try:
        print("[Seg4-Test] Lowering body...")
        _set_low_body_pose(ctrl, low_height, low_pitch, duration_ms=700)

        # Hold low pose with zero velocity first, so it is easy to see whether
        # the posture command itself takes effect before forward motion starts.
        print("[Seg4-Test] Holding low pose for 3.0s before walking...")
        hold_start = time.perf_counter()
        last_debug = 0.0
        while time.perf_counter() - hold_start < 3.0:
            _set_low_body_pose(ctrl, low_height, low_pitch, duration_ms=180)
            now = time.perf_counter()
            if now - last_debug > 0.5:
                print(f"[Seg4-Test] Holding low pose: {_body_pose_debug(perception)}")
                last_debug = now
            time.sleep(0.05)

        print(f"[Seg4-Test] Crawling forward for {duration_s:.1f}s...")
        crawl_start = time.perf_counter()
        last_debug = 0.0
        last_pose_update = 0.0
        last_move_update = 0.0
        while time.perf_counter() - crawl_start < duration_s:
            now = time.perf_counter()
            elapsed = now - crawl_start

            if elapsed - last_pose_update > 0.30:
                _set_low_body_pose(ctrl, low_height, low_pitch, duration_ms=120)
                last_pose_update = elapsed

            if elapsed - last_move_update > 0.40:
                ctrl.locomotion(
                    gait_id=GAIT_TROT_SLOW,
                    vx=crawl_speed, vy=0.0, wz=0.0,
                    step_h_max=low_step_height, step_h_min=low_step_height,
                    body_height=low_height,
                    body_pitch=low_pitch,
                    duration_ms=180)
                last_move_update = elapsed

            if now - last_debug > 0.5:
                try:
                    pos = perception.position[:2]
                    heading = perception.heading
                except (AttributeError, TypeError):
                    pos = None
                    heading = None
                dist = None
                if start_pos is not None and pos is not None:
                    dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
                dist_str = "dist=N/A" if dist is None else f"dist={dist:.3f}m"
                print(f"[Seg4-Test] Moving: pos={pos}, heading={heading}, "
                      f"{dist_str}, {_body_pose_debug(perception)}")
                last_debug = now
            time.sleep(0.1)

    finally:
        print("[Seg4-Test] Stopping and restoring normal height...")
        ctrl.stop_moving(body_height=low_height)
        ctrl.set_height(0.22, duration_ms=500)
        time.sleep(0.5)
        print(f"[Seg4-Test] Done, final {_body_pose_debug(perception)}")

    return True


def _test_front_stretch_back(ctrl, perception, duration_s, gait_z, cycles=None,
                             forward=False):
    """Lower the front by stretching front feet forward, then creep."""
    direction = "forward" if forward else "backward"
    print(f"[Seg4-Test] Front-stretch + {direction} creep test")
    print("[Seg4-Test] This is exploratory; keep an eye on balance and torque logs.")

    try:
        start_pos = perception.position[:2]
    except (AttributeError, TypeError):
        start_pos = None

    _upload_low_crawl_gait(ctrl)
    time.sleep(0.5)

    # Pre-shape: front feet forward, mild body pitch.  The six foot_pose values
    # map to front-pair x/y, rear-pair x/y, plus paired z offsets.
    ctrl.execute_gait_steps([
        dict(mode=11, gait_id=110, contact=15,
             vel_des=[0.0, 0.0, 0.0, 0.0],
             rpy_des=[0.0, -0.12, 0.0],
             pos_des=[0.0, 0.0, -0.10],
             acc_des=[1.0, 1.0, 1.0, 1.1, 1.1, 1.6],
             ctrl_point=[0.12, 0.0, 0.70],
             foot_pose=[0.12, 0.0, -0.025, -0.03, 0.0, 0.005],
             step_height=[18018.0, 18018.0],
             value=0, duration=900),
    ])

    cycle = _get_front_stretch_steps(gait_z=gait_z, forward=forward)
    cycle_duration_s = sum(step['duration'] for step in cycle) / 1000.0
    max_cycles = cycles if cycles is not None else max(1, math.ceil(duration_s / cycle_duration_s))
    max_cycles = min(max_cycles, 8)
    print(f"[Seg4-Test] Running {max_cycles} front-stretch-{direction} cycles "
          f"(cycle≈{cycle_duration_s:.2f}s)")

    for idx in range(1, max_cycles + 1):
        ctrl.execute_gait_steps(cycle)
        try:
            pos = perception.position[:2]
            heading = perception.heading
        except (AttributeError, TypeError):
            pos = None
            heading = None
        dist = None
        if start_pos is not None and pos is not None:
            dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
        dist_str = "dist=N/A" if dist is None else f"dist={dist:.3f}m"
        print(f"[Seg4-Test] Front-stretch-{direction} {idx}/{max_cycles}: pos={pos}, "
              f"heading={heading}, {dist_str}, {_body_pose_debug(perception)}")

    print("[Seg4-Test] Restoring normal height...")
    ctrl.stop_moving(body_height=0.18)
    ctrl.set_height(0.22, duration_ms=500)
    time.sleep(0.5)
    return True


def _get_front_stretch_steps(gait_z=-0.14, forward=True):
    gait_z = max(-0.18, min(-0.08, gait_z))
    step_h = 18018.0
    weight = [1.0, 1.0, 1.0, 1.1, 1.1, 1.6]
    front_x = 0.12
    sign = 1.0 if forward else -1.0
    rear_push = sign * 0.055
    rear_set = -sign * 0.020
    vx_push = sign * 0.035
    vx_set = sign * 0.025
    return [
        dict(mode=11, gait_id=110, contact=10,
             vel_des=[vx_push, 0.0, 0.0, 0.0],
             rpy_des=[0.0, -0.12, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[rear_push, 0.0, 0.70],
             foot_pose=[front_x, 0.0, -0.025, rear_push, 0.0, 0.005],
             step_height=[step_h, step_h],
             value=0, duration=500),
        dict(mode=11, gait_id=110, contact=10,
             vel_des=[vx_set, 0.0, 0.0, 0.0],
             rpy_des=[0.0, -0.12, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[rear_set, 0.0, 0.70],
             foot_pose=[front_x, 0.0, -0.025, rear_set, 0.0, 0.005],
             step_height=[step_h, step_h],
             value=0, duration=450),
        dict(mode=11, gait_id=110, contact=5,
             vel_des=[vx_push, 0.0, 0.0, 0.0],
             rpy_des=[0.0, -0.12, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[rear_push, 0.0, 0.70],
             foot_pose=[front_x, 0.0, -0.025, rear_push, 0.0, 0.005],
             step_height=[step_h, step_h],
             value=0, duration=500),
        dict(mode=11, gait_id=110, contact=5,
             vel_des=[vx_set, 0.0, 0.0, 0.0],
             rpy_des=[0.0, -0.12, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[rear_set, 0.0, 0.70],
             foot_pose=[front_x, 0.0, -0.025, rear_set, 0.0, 0.005],
             step_height=[step_h, step_h],
             value=0, duration=450),
    ]


def _test_head_down_walk(ctrl, perception, duration_s, speed, pitch, cycles=None):
    """Walk at normal height while pitching the body down."""
    normal_height = 0.22
    speed = max(0.02, min(speed, 0.08))
    pitch = max(-0.35, min(0.0, pitch))

    try:
        start_pos = perception.position[:2]
    except (AttributeError, TypeError):
        start_pos = None

    print("[Seg4-Test] Head-down normal-height walking test")
    print(f"[Seg4-Test] height={normal_height:.2f}m, pitch={pitch:.2f}rad "
          f"({math.degrees(pitch):.1f}deg), speed={speed:.2f}m/s")

    if hasattr(ctrl, 'set_body_pose'):
        ctrl.set_body_pose(height_m=normal_height, pitch_rad=pitch,
                           duration_ms=500)
    else:
        ctrl.look_forward(pitch, duration_ms=500)

    cycle_s = 0.5
    max_cycles = cycles if cycles is not None else max(1, math.ceil(duration_s / cycle_s))
    for idx in range(1, max_cycles + 1):
        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=speed, vy=0.0, wz=0.0,
            step_h_max=0.035, step_h_min=0.025,
            body_height=normal_height,
            body_pitch=pitch,
            duration_ms=300)
        time.sleep(0.2)
        if idx % 2 == 0:
            try:
                pos = perception.position[:2]
                heading = perception.heading
            except (AttributeError, TypeError):
                pos = None
                heading = None
            dist = None
            if start_pos is not None and pos is not None:
                dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
            dist_str = "dist=N/A" if dist is None else f"dist={dist:.3f}m"
            print(f"[Seg4-Test] Head-down {idx}/{max_cycles}: pos={pos}, "
                  f"heading={heading}, {dist_str}, {_body_pose_debug(perception)}")

    print("[Seg4-Test] Restoring normal pose...")
    ctrl.stop_moving(body_height=normal_height)
    ctrl.set_height(normal_height, duration_ms=500)
    time.sleep(0.5)
    return True


def _test_low_crawl_pose_hold(ctrl, perception, duration_s, low_height, low_pitch):
    print(f"[Seg4-Test] Pose-hold only for {duration_s:.1f}s")
    start = time.perf_counter()
    last_debug = 0.0
    while time.perf_counter() - start < duration_s:
        _set_low_body_pose(ctrl, low_height, low_pitch, duration_ms=180)
        now = time.perf_counter()
        if now - last_debug > 0.5:
            print(f"[Seg4-Test] Pose-hold: {_body_pose_debug(perception)}")
            last_debug = now
        time.sleep(0.05)
    print("[Seg4-Test] Restoring normal height...")
    ctrl.set_height(0.22, duration_ms=500)
    time.sleep(0.5)
    return True


def _test_legacy_low_crawl_user_gait(ctrl, perception, duration_s, speed):
    """Test the previous-year height-bar user gait (legacy)."""
    print("[Seg4-Test] Legacy user-gait low-crawl test")
    print("[Seg4-Test] Uploading legacy under-bar gait files...")
    _upload_low_height_gait(ctrl)  # Use Four-4 strategy
    time.sleep(0.5)

    try:
        start_pos = perception.position[:2]
    except (AttributeError, TypeError):
        start_pos = None

    vx = max(0.03, min(speed, 0.10))
    run_s = max(1.0, duration_s)
    print(f"[Seg4-Test] Triggering legacy gait: vx={vx:.2f}m/s, "
          f"body_z=-0.035, duration={run_s:.1f}s")

    if hasattr(ctrl, 'user_gait_locomotion'):
        ctrl.user_gait_locomotion(
            vx=vx,
            body_z=-0.035,
            pitch_rad=0.0,
            step_height=(0.03, 0.03),
            contact=1,
            value=1,
            duration_ms=int(run_s * 1000))
    else:
        ctrl.execute_gait_steps([
            dict(mode=11, gait_id=110, contact=1,
                 vel_des=[vx, 0.0, 0.0, 0.0],
                 rpy_des=[0.0, 0.0, 0.0],
                 pos_des=[0.0, 0.0, -0.035],
                 acc_des=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 ctrl_point=[0.0, 0.0, 0.0],
                 foot_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 step_height=[0.03, 0.03],
                 value=1, duration=int(run_s * 1000)),
        ])

    test_start = time.perf_counter()
    last_debug = 0.0
    while time.perf_counter() - test_start < run_s:
        now = time.perf_counter()
        if now - last_debug > 0.7:
            try:
                pos = perception.position[:2]
                heading = perception.heading
            except (AttributeError, TypeError):
                pos = None
                heading = None
            dist = None
            if start_pos is not None and pos is not None:
                dist = math.hypot(pos[0] - start_pos[0],
                                  pos[1] - start_pos[1])
            dist_str = "dist=N/A" if dist is None else f"dist={dist:.3f}m"
            print(f"[Seg4-Test] Legacy moving: pos={pos}, heading={heading}, "
                  f"{dist_str}, {_body_pose_debug(perception)}")
            last_debug = now
        time.sleep(0.05)

    print("[Seg4-Test] Stopping and restoring normal height...")
    ctrl.stop_moving(body_height=0.22)
    ctrl.set_height(0.22, duration_ms=500)
    time.sleep(0.5)
    return True


def _test_low_crawl_user_gait(ctrl, perception, duration_s, speed,
                              gait_z, low_pitch, cycles=None, lateral=0.0,
                              rest_every=1):
    print("[Seg4-Test] User-gait low-crawl test")
    print("[Seg4-Test] Uploading low-crawl gait files...")
    _upload_low_crawl_gait(ctrl)
    time.sleep(0.5)

    try:
        start_pos = perception.position[:2]
    except (AttributeError, TypeError):
        start_pos = None

    start = time.perf_counter()
    last_debug = 0.0
    cycle = _get_low_crawl_steps(
        speed=speed, gait_z=gait_z, pitch=low_pitch, lateral=lateral)
    cycle_duration_s = sum(step['duration'] for step in cycle) / 1000.0
    max_cycles = cycles if cycles is not None else max(1, math.ceil(duration_s / cycle_duration_s))
    if rest_every is None:
        rest_every = 1
    print(f"[Seg4-Test] Running {max_cycles} low-crawl cycles "
          f"(cycle≈{cycle_duration_s:.2f}s, lateral={lateral:.2f}, "
          f"burst={rest_every if rest_every else 'off'} cycles)")

    _low_crawl_rest(ctrl, gait_z, low_pitch, reason="Pre-shaping low pose")
    burst_start_pos = None
    previous_pos = None

    for cycle_idx in range(1, max_cycles + 1):
        if rest_every and (cycle_idx - 1) % rest_every == 0:
            try:
                burst_start_pos = perception.position[:2]
            except (AttributeError, TypeError):
                burst_start_pos = None

        ctrl.execute_gait_steps(cycle)
        now = time.perf_counter()
        try:
            pos = perception.position[:2]
            heading = perception.heading
        except (AttributeError, TypeError):
            pos = None
            heading = None
        dist = None
        if start_pos is not None and pos is not None:
            dist = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
        step_dist = None
        if previous_pos is not None and pos is not None:
            step_dist = math.hypot(pos[0] - previous_pos[0],
                                   pos[1] - previous_pos[1])
        group_dist = None
        if burst_start_pos is not None and pos is not None:
            group_dist = math.hypot(pos[0] - burst_start_pos[0],
                                    pos[1] - burst_start_pos[1])
        dist_str = "dist=N/A" if dist is None else f"dist={dist:.3f}m"
        step_str = "" if step_dist is None else f", step_dist={step_dist:.3f}m"
        group_str = "" if group_dist is None else f", burst_dist={group_dist:.3f}m"
        print(f"[Seg4-Test] Cycle {cycle_idx}/{max_cycles}: pos={pos}, "
              f"heading={heading}, {dist_str}{step_str}{group_str}, "
              f"{_body_pose_debug(perception)}")
        last_debug = now
        if pos is not None:
            previous_pos = pos
        if step_dist is not None and step_dist < 0.006 and cycle_idx < max_cycles:
            _low_crawl_unjam(ctrl, gait_z, low_pitch)
        if rest_every and cycle_idx < max_cycles and cycle_idx % rest_every == 0:
            _low_crawl_rest(ctrl, gait_z, low_pitch)

    print("[Seg4-Test] Stopping and restoring normal height...")
    ctrl.stop_moving(body_height=_LOW_CRAWL_HEIGHT)
    ctrl.set_height(0.22, duration_ms=500)
    time.sleep(0.5)
    return True


def _low_crawl_rest(ctrl, gait_z, pitch, reason="Restabilizing low pose"):
    print(f"[Seg4-Test] {reason}...")
    relax_z = min(gait_z + 0.05, -0.09)
    relax_pitch = max(pitch + 0.06, -0.06)
    ctrl.execute_gait_steps([
        dict(mode=11, gait_id=110, contact=15,
             vel_des=[0.0, 0.0, 0.0, 0.0],
             rpy_des=[0.0, relax_pitch, 0.0],
             pos_des=[0.0, 0.0, relax_z],
             acc_des=[0.8, 0.8, 0.8, 1.0, 1.0, 1.4],
             ctrl_point=[0.0, 0.0, 0.70],
             foot_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             step_height=[26026.0, 26026.0],
             value=0, duration=420),
        dict(mode=11, gait_id=110, contact=15,
             vel_des=[0.0, 0.0, 0.0, 0.0],
             rpy_des=[0.0, pitch, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=[0.9, 0.9, 0.9, 1.1, 1.1, 1.6],
             ctrl_point=[0.0, 0.0, 0.70],
             foot_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             step_height=[26026.0, 26026.0],
             value=0, duration=360),
    ])


def _low_crawl_unjam(ctrl, gait_z, pitch):
    print("[Seg4-Test] Low progress detected; unloading legs...")
    unload_z = min(gait_z + 0.07, -0.08)
    unload_pitch = max(pitch + 0.08, -0.04)
    ctrl.execute_gait_steps([
        dict(mode=11, gait_id=110, contact=15,
             vel_des=[0.0, 0.0, 0.0, 0.0],
             rpy_des=[0.0, unload_pitch, 0.0],
             pos_des=[0.0, 0.0, unload_z],
             acc_des=[0.7, 0.7, 0.7, 0.9, 0.9, 1.2],
             ctrl_point=[0.0, 0.0, 0.80],
             foot_pose=[0.0, 0.0, 0.012, 0.0, 0.0, 0.012],
             step_height=[36036.0, 36036.0],
             value=0, duration=550),
        dict(mode=11, gait_id=110, contact=15,
             vel_des=[0.0, 0.0, 0.0, 0.0],
             rpy_des=[0.0, pitch, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=[0.8, 0.8, 0.8, 1.0, 1.0, 1.45],
             ctrl_point=[0.0, 0.0, 0.75],
             foot_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             step_height=[32032.0, 32032.0],
             value=0, duration=450),
    ])


def _get_low_crawl_steps(speed=0.05, gait_z=_LOW_CRAWL_GAIT_Z, pitch=None,
                         lateral=0.0):
    pitch = _LOW_CRAWL_PITCH if pitch is None else pitch
    step_h = 36036.0
    speed = max(0.02, min(speed, 0.045))
    lateral = max(-0.05, min(lateral, 0.05))
    vx = 0.0 if abs(lateral) > 1e-4 else speed
    vy = lateral
    lift_x = 0.055
    set_x = 0.0
    lift_y = 0.035 if lateral >= 0.0 else -0.035
    set_y = -0.012 if lateral >= 0.0 else 0.012
    duration_lift = 520
    duration_set = 420
    weight = [0.75, 0.75, 0.75, 0.95, 0.95, 1.35]
    return [
        dict(mode=11, gait_id=110, contact=10,
             vel_des=[vx, vy, 0.0, 0.0],
             rpy_des=[0.0, pitch, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[lift_x if vx else 0.0, lift_y if vy else 0.0, 0.70],
             foot_pose=[lift_x if vx else 0.0, lift_y if vy else 0.0, 0.0,
                        0.0, 0.0, 0.0],
             step_height=[step_h, step_h],
             value=0, duration=duration_lift),
        dict(mode=11, gait_id=110, contact=10,
             vel_des=[vx, vy, 0.0, 0.0],
             rpy_des=[0.0, pitch, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[set_x if vx else 0.0, set_y if vy else 0.0, 0.70],
             foot_pose=[set_x if vx else 0.0, set_y if vy else 0.0, 0.0,
                        0.0, 0.0, 0.0],
             step_height=[step_h, step_h],
             value=0, duration=duration_set),
        dict(mode=11, gait_id=110, contact=5,
             vel_des=[vx, vy, 0.0, 0.0],
             rpy_des=[0.0, pitch, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[lift_x if vx else 0.0, lift_y if vy else 0.0, 0.70],
             foot_pose=[0.0, 0.0, 0.0,
                        0.0, lift_y if vy else 0.0, 0.0],
             step_height=[step_h, step_h],
             value=0, duration=duration_lift),
        dict(mode=11, gait_id=110, contact=5,
             vel_des=[vx, vy, 0.0, 0.0],
             rpy_des=[0.0, pitch, 0.0],
             pos_des=[0.0, 0.0, gait_z],
             acc_des=weight,
             ctrl_point=[set_x if vx else 0.0, set_y if vy else 0.0, 0.70],
             foot_pose=[0.0, 0.0, 0.0,
                        0.0, set_y if vy else 0.0, 0.0],
             step_height=[step_h, step_h],
             value=0, duration=duration_set),
    ]


def _set_low_body_pose(ctrl, height_m, pitch_rad, duration_ms=200):
    if hasattr(ctrl, 'set_body_pose'):
        ctrl.set_body_pose(height_m=height_m, pitch_rad=pitch_rad,
                           duration_ms=duration_ms)
    else:
        ctrl.set_height(height_m, duration_ms=duration_ms)
        ctrl.look_forward(pitch_rad, duration_ms=duration_ms)


def _interact_object(ctrl, perception, obj_type, duration_s=1.0):
    """Execute a brief forward motion to interact with an object."""
    start = time.perf_counter()
    if obj_type == 'football':
        gait_id = GAIT_TROT_FAST
        speed = 0.30
    else:
        gait_id = GAIT_TROT_SLOW
        speed = 0.18
    while time.perf_counter() - start < duration_s:
        ctrl.locomotion(
            gait_id=gait_id,
            vx=speed, vy=0.0, wz=0.0,
            step_h_max=0.06, step_h_min=0.03,
            duration_ms=0,
        )
        time.sleep(0.05)


def _drive_until_y(ctrl, perception, target_y, speed, tol_m=0.08,
                   timeout_s=20.0, label="drive"):
    """Move along current heading until reaching a target Y coordinate."""
    try:
        _, start_y = perception.position[:2]
    except (AttributeError, TypeError):
        _, start_y, _ = perception.odom_pose[:2]
    direction = 1.0 if target_y >= start_y else -1.0
    vx = abs(speed)
    start = time.perf_counter()
    last_progress_t = start
    last_err_abs = None
    debug_count = 0
    while True:
        try:
            px, py = perception.position[:2]
        except (AttributeError, TypeError):
            px, py, _ = perception.odom_pose[:2]
        err_y = target_y - py
        if abs(err_y) <= tol_m or err_y * direction <= 0.0:
            ctrl.stop_moving(body_height=0.22)
            return True
        err_abs = abs(err_y)
        if last_err_abs is None or err_abs < last_err_abs - 0.03:
            last_err_abs = err_abs
            last_progress_t = time.perf_counter()
        if time.perf_counter() - start > timeout_s:
            print(f"[Seg4] {label} timeout but not reached at pos=({px:.3f}, {py:.3f}), "
                  f"target_y={target_y:.2f}")
            ctrl.stop_moving(body_height=0.22)
            return False
        if time.perf_counter() - last_progress_t > 8.0:
            print(f"[Seg4] {label} no progress at pos=({px:.3f}, {py:.3f}), "
                  f"target_y={target_y:.2f}, err_y={err_y:.3f}")
            ctrl.stop_moving(body_height=0.22)
            return False
        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=vx,
            vy=0.0,
            wz=0.0,
            step_h_max=0.06,
            step_h_min=0.03,
            body_height=0.22,
            duration_ms=0)
        debug_count += 1
        if debug_count % 40 == 0:
            print(f"[Seg4] {label}: pos=({px:.3f}, {py:.3f}), "
                  f"target_y={target_y:.2f}, err_y={err_y:.3f}")
        time.sleep(0.05)
    ctrl.stop_moving(body_height=0.22)
    return False


def _legacy_low_crawl_to_target_xy(ctrl, perception, target_xy,
                                speed=0.10,
                                tol_m=0.15,
                                body_z=-0.055,
                                timeout_s=30.0,
                                label="legacy low-crawl"):
    """Drive toward a target XY using the legacy under-bar USER_DEFINED gait.

    This is extracted so other segments can reuse the same legacy gait behavior
    (upload + trigger + progress/timeout monitoring) without inheriting Segment 4
    height-bar specific geometry logic.

    Notes:
      - This uses the controller's `user_gait_locomotion` if available; otherwise
        falls back to `execute_gait_steps` with an equivalent single command.
      - It does not attempt to manage heading; callers can turn before/after.

    Returns:
      True if target is reached within tol_m, otherwise False.
    """
    # Clamp to conservative ranges used elsewhere in Segment 4.
    vx = max(0.05, min(float(speed), 0.12))
    tol_m = max(0.05, float(tol_m))
    body_z = max(-0.10, min(float(body_z), -0.02))
    timeout_s = max(2.0, float(timeout_s))

    # Use Four-4's low-height gait strategy
    _upload_low_height_gait(ctrl)
    time.sleep(0.5)

    print(f"[Seg4] {label} started (Four-4): speed={vx:.2f}m/s, body_z={body_z:.3f}, "
          f"target_xy={target_xy}, tol={tol_m:.2f}m, timeout={timeout_s:.1f}s")

    reached = False
    for attempt in range(1, 4):
        print(f"[Seg4] {label} attempt {attempt}/3")
        if hasattr(ctrl, 'user_gait_locomotion'):
            ctrl.user_gait_locomotion(
                vx=vx,
                body_z=body_z,
                pitch_rad=0.0,
                step_height=(0.03, 0.03),
                contact=1,
                value=1,
                duration_ms=int(timeout_s * 1000))
        else:
            ctrl.execute_gait_steps([
                dict(mode=11, gait_id=110, contact=1,
                     vel_des=[vx, 0.0, 0.0, 0.0],
                     rpy_des=[0.0, 0.0, 0.0],
                     pos_des=[0.0, 0.0, body_z],
                     acc_des=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     ctrl_point=[0.0, 0.0, 0.0],
                     foot_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     step_height=[0.03, 0.03],
                     value=1, duration=int(timeout_s * 1000)),
            ])

        start_t = time.perf_counter()
        last_progress_t = start_t
        last_dist = None
        debug_count = 0

        while True:
            try:
                px, py = perception.position[:2]
            except (AttributeError, TypeError):
                px, py = perception.odom_pose[:2]

            dist = math.hypot(target_xy[0] - px, target_xy[1] - py)
            if dist <= tol_m:
                reached = True
                break

            if last_dist is None or dist < last_dist - 0.03:
                last_dist = dist
                last_progress_t = time.perf_counter()

            if time.perf_counter() - start_t > timeout_s:
                print(f"[Seg4] {label} timeout: pos=({px:.3f}, {py:.3f}), "
                      f"target={target_xy}, dist={dist:.3f}m")
                break

            if time.perf_counter() - last_progress_t > 8.0:
                print(f"[Seg4] {label} no progress: pos=({px:.3f}, {py:.3f}), "
                      f"target={target_xy}, dist={dist:.3f}m, "
                      f"{_body_pose_debug(perception)}")
                break

            debug_count += 1
            if debug_count % 40 == 0:
                print(f"[Seg4] {label} moving: pos=({px:.3f}, {py:.3f}), "
                      f"target={target_xy}, dist={dist:.3f}m")
            time.sleep(0.05)

        if reached:
            break

        ctrl.stop_moving(body_height=0.22)
        time.sleep(0.5)

    ctrl.stop_moving(body_height=0.22)
    ctrl.set_height(0.22, duration_ms=400)
    time.sleep(0.5)
    return reached


def _go_to_waypoint_low(ctrl, perception, waypoint_xy, target_heading, speed=0.10, tol_m=0.30, bar_id=1):
    """Drive toward a waypoint in low-crawl posture (for height-bar passages).

    Uses the previous-year under-bar USER_DEFINED gait.  This gait keeps the
    body only mildly lowered (z=-0.035) and relies on the uploaded contact
    sequence/MPC trajectory instead of forcing an extreme body-height command.

    Args:
        bar_id: 1 for height bar at Y=9.27, 2 for height bar at Y=10.6
    """
    BAR_POSITIONS = {1: 9.65, 2: 10.6}
    BAR_CENTER_Y = BAR_POSITIONS.get(bar_id, 9.27)
    # Lower body more to improve clearance under height bar.
    # Keep within a conservative range to avoid belly contact/torque spikes.
    LEGACY_BODY_Z = -0.055
    LEGACY_SPEED = max(0.05, min(speed, 0.12))
    # Increased margins so robot starts low-crawl farther from bar center,
    # giving more room to transition into low posture before reaching the bar.
    if bar_id == 2:
        BAR_APPROACH_MARGIN = 0.30
        BAR_EXIT_MARGIN = 0.10
    else:
        BAR_APPROACH_MARGIN = 0.30
        BAR_EXIT_MARGIN = 0.20
    normal_speed = 0.20

    # Use explicit heading if provided, otherwise compute from direction
    try:
        px, py = perception.position[:2]
        heading_deg = perception.heading
    except (AttributeError, TypeError):
        px, py, _ = perception.odom_pose[:2]
        heading_deg = perception.odom_pose[2] * 180.0 / math.pi % 360.0

    if target_heading is not None:
        final_heading = target_heading
    else:
        dx = waypoint_xy[0] - px
        dy = waypoint_xy[1] - py
        final_heading = math.degrees(math.atan2(dy, dx)) % 360.0

    moving_positive_y = waypoint_xy[1] >= py
    crawl_start_y = BAR_CENTER_Y - BAR_APPROACH_MARGIN if moving_positive_y else BAR_CENTER_Y + BAR_APPROACH_MARGIN
    crawl_end_y = BAR_CENTER_Y + BAR_EXIT_MARGIN if moving_positive_y else BAR_CENTER_Y - BAR_EXIT_MARGIN

    route_min_y = min(py, waypoint_xy[1])
    route_max_y = max(py, waypoint_xy[1])
    crosses_bar = route_min_y <= BAR_CENTER_Y <= route_max_y

    if not crosses_bar:
        print("[Seg4] Low-crawl route does not cross height bar; using normal waypoint nav")
        go_to_waypoint(ctrl, perception, waypoint_xy,
                       speed=normal_speed, tol_m=tol_m)
        if target_heading is not None:
            turn_to_angle(ctrl, perception, target_heading,
                          turn_rate=0.5, tol_deg=3.0, timeout_s=8.0)
        return True

    if route_min_y < crawl_start_y < route_max_y:
        print(f"[Seg4] Normal approach before bar: current_y={py:.2f}, "
              f"crawl_start_y={crawl_start_y:.2f}")
        reached_approach = False
        for attempt in range(1, 4):
            reached_approach = _drive_until_y(
                ctrl, perception, crawl_start_y, normal_speed,
                tol_m=0.08, timeout_s=45.0, label=f"normal approach {attempt}")
            if reached_approach:
                break
        if not reached_approach:
            print("[Seg4] Approach point not reached; skip low-crawl for safety")
            if target_heading is not None:
                turn_to_angle(ctrl, perception, target_heading,
                              turn_rate=0.5, tol_deg=3.0, timeout_s=8.0)
            return False

    low_distance = abs(crawl_end_y - crawl_start_y)
    timeout_s = max(10.0, min(24.0, low_distance / max(LEGACY_SPEED, 0.01) * 2.8 + 6.0))

    # Use Four-4's low-height gait strategy
    _upload_low_height_gait(ctrl)
    time.sleep(0.5)
    print(f"[Seg4] Low-height gait (Four-4) started: "
          f"y {crawl_start_y:.2f}->{crawl_end_y:.2f}, "
          f"timeout={timeout_s:.1f}s")
    reached_low = False

    # Calculate number of gait cycles based on distance
    # Each cycle takes ~0.84s (from Four-4's implementation)
    num_cycles = max(1, int(low_distance / LEGACY_SPEED / 0.84))

    for attempt in range(1, 4):
        print(f"[Seg4] Low-height gait attempt {attempt}/3, cycles={num_cycles}")
        for _ in range(num_cycles):
            ctrl.user_gait_execute(duration_ms=840)
            time.sleep(0.02)  # Small gap between cycles

        crawl_start = time.perf_counter()
        last_progress_t = crawl_start
        last_err_abs = None
        debug_count = 0

        while True:
            try:
                px, py = perception.position[:2]
            except (AttributeError, TypeError):
                px, py, _ = perception.odom_pose[:2]

            err_y = crawl_end_y - py
            if abs(err_y) <= 0.10 or err_y * (1.0 if moving_positive_y else -1.0) <= 0.0:
                reached_low = True
                break
            err_abs = abs(err_y)
            if last_err_abs is None or err_abs < last_err_abs - 0.03:
                last_err_abs = err_abs
                last_progress_t = time.perf_counter()
            if time.perf_counter() - crawl_start > timeout_s:
                print(f"[Seg4] Legacy low-crawl timeout but not reached at "
                      f"pos=({px:.3f}, {py:.3f}), target_y={crawl_end_y:.2f}")
                break
            if time.perf_counter() - last_progress_t > 8.0:
                print(f"[Seg4] Legacy low-crawl no progress at "
                      f"pos=({px:.3f}, {py:.3f}), target_y={crawl_end_y:.2f}, "
                      f"err_y={err_y:.3f}")
                break

            debug_count += 1
            if debug_count % 40 == 0:
                print(f"[Seg4] Legacy low-crawl moving: pos=({px:.3f}, {py:.3f}), "
                      f"target_y={crawl_end_y:.2f}, err_y={err_y:.3f}, "
                      f"{_body_pose_debug(perception)}")
            time.sleep(0.05)

        if reached_low:
            break
        ctrl.stop_moving(body_height=0.22)
        time.sleep(0.5)

    if not reached_low:
        print("[Seg4] Low-crawl exit point not reached; keep normal posture and stop this leg")
        ctrl.stop_moving(body_height=0.22)
        ctrl.set_height(0.22, duration_ms=400)
        time.sleep(0.5)
        return False

    ctrl.stop_moving(body_height=0.22)
    ctrl.set_height(0.22, duration_ms=400)
    time.sleep(0.5)

    print(f"[Seg4] Normal travel after bar to waypoint {waypoint_xy}")
    go_to_waypoint(ctrl, perception, waypoint_xy,
                   speed=normal_speed, tol_m=tol_m)
    if target_heading is not None:
        turn_to_angle(ctrl, perception, target_heading,
                      turn_rate=0.5, tol_deg=3.0, timeout_s=8.0)
    return True


def _body_pose_debug(perception):
    height = getattr(perception, 'body_height', None)
    pitch = getattr(perception, 'body_pitch', None)
    h = "height=N/A" if height is None else f"height={height:.3f}m"
    p = "pitch=N/A" if pitch is None else f"pitch={pitch:.1f}deg"
    return f"{h}, {p}"


def _legacy_under_bar_sections(repeats=34):
    sections = [
        ([1, 1, 1, 1], 10),
    ]
    for _ in range(repeats):
        sections.extend([
            ([0, 1, 1, 0], 10),
            ([1, 1, 1, 1], 2),
            ([1, 0, 0, 1], 10),
            ([1, 1, 1, 1], 2),
        ])
    return sections


def _format_legacy_gait_def():
    lines = ["# Gait Def for legacy under-bar crawl\n"]
    for contact, duration in _legacy_under_bar_sections():
        lines.append("\n[[section]]\n")
        lines.append(f"contact  = {contact}\n")
        lines.append(f"duration = {duration}\n")
    return ''.join(lines)


def _legacy_under_bar_step_templates(repeats=34):
    common = {
        'mode': 11,
        'gait_id': 110,
        'vel_des': [0.1, 0.0, 0.0],
        'rpy_des': [0.0, 0.0, 0.0],
        'pos_des': [0.0, 0.0, -0.035],
        'acc_des': [10.0, 10.0, 10.0, 50.0, 50.0, 10.0],
        'value': 1,
        'contact': 10,
        'mu': 0.40,
    }

    def make(foot_pose, step_height, duration, ctrl_point_xy=None):
        ctrl_point = [
            foot_pose[9] if ctrl_point_xy is None else ctrl_point_xy[0],
            foot_pose[10] if ctrl_point_xy is None else ctrl_point_xy[1],
            common['mu'],
        ]
        return dict(
            mode=common['mode'],
            gait_id=common['gait_id'],
            contact=common['contact'],
            vel_des=list(common['vel_des']),
            rpy_des=list(common['rpy_des']),
            pos_des=list(common['pos_des']),
            acc_des=list(common['acc_des']),
            ctrl_point=ctrl_point,
            foot_pose=[
                foot_pose[0], foot_pose[1],
                foot_pose[3], foot_pose[4],
                foot_pose[6], foot_pose[7],
            ],
            step_height=[
                math.ceil(step_height[0] * 1000.0)
                + math.ceil(step_height[1] * 1000.0) * 1000.0,
                math.ceil(step_height[2] * 1000.0)
                + math.ceil(step_height[3] * 1000.0) * 1000.0,
            ],
            value=common['value'],
            duration=duration,
        )

    steps = [
        make(
            [0.0] * 12,
            [0.0, 0.0, 0.0, 0.0],
            400,
            ctrl_point_xy=(0.0, 0.0)),
    ]
    for repeat_idx in range(repeats):
        last = repeat_idx == repeats - 1
        steps.extend([
            make(
                [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
                [0.03, 0.0, 0.0, 0.03],
                1000),
            make(
                [0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                200),
            make(
                [0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.03, 0.03, 0.0],
                1000),
            make(
                [0.0] * 12 if last else
                [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                200),
        ])
    return steps


def _format_toml_array(values):
    return "[" + ", ".join(f"{v:.6g}" if isinstance(v, float) else str(v)
                           for v in values) + "]"


def _format_legacy_gait_params():
    lines = ["# Gait Params\n"]
    for idx, step in enumerate(_legacy_under_bar_step_templates()):
        lines.append("\n[[step]]\n")
        lines.append(f"mode = {step['mode']}\n")
        lines.append(f"gait_id = {step['gait_id']}\n")
        lines.append(f"contact = {step['contact']}\n")
        lines.append(f"life_count = {idx}\n")
        lines.append(f"vel_des = {_format_toml_array(step['vel_des'])}\n")
        lines.append(f"rpy_des = {_format_toml_array(step['rpy_des'])}\n")
        lines.append(f"pos_des = {_format_toml_array(step['pos_des'])}\n")
        lines.append(f"acc_des = {_format_toml_array(step['acc_des'])}\n")
        lines.append(f"ctrl_point = {_format_toml_array(step['ctrl_point'])}\n")
        lines.append(f"foot_pose = {_format_toml_array(step['foot_pose'])}\n")
        lines.append(f"step_height = {_format_toml_array(step['step_height'])}\n")
        lines.append(f"value = {step['value']}\n")
        lines.append(f"duration = {step['duration']}\n")
    return ''.join(lines)


def _upload_low_height_gait(ctrl):
    """Upload low-height gait using Four-4 strategy.

    Loads TOML files from config/ directory and uploads them via LCM.
    This uses Four-4's proven low posture gait for height-bar passages.
    """
    seg_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    config_dir = os.path.join(seg_dir, 'config')

    def_path = os.path.join(config_dir, 'gait_def_low_height.toml')
    full_path = os.path.join(config_dir, 'gait_params_low_height_full.toml')
    params_path = os.path.join(config_dir, 'gait_params_low_height.toml')

    try:
        ctrl.change_gait(def_path, full_path, params_path)
        print("[Seg4] Low-height gait (Four-4 strategy) uploaded")
    except Exception as e:
        print(f"[Seg4] Warning: Low-height gait upload failed: {e}")


def _upload_low_height_re_gait(ctrl):
    """Upload low-height reverse gait for exiting height-bar passages.

    Uses the low_height_re TOML files (backward walking).
    """
    seg_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    config_dir = os.path.join(seg_dir, 'config')

    def_path = os.path.join(config_dir, 'gait_def_low_height_re.toml')
    full_path = os.path.join(config_dir, 'gait_params_low_height_re_full.toml')
    params_path = os.path.join(config_dir, 'gait_params_low_height_re.toml')

    try:
        ctrl.change_gait(def_path, full_path, params_path)
        print("[Seg4] Low-height reverse gait uploaded")
    except Exception as e:
        print(f"[Seg4] Warning: Low-height reverse gait upload failed: {e}")


def _upload_low_crawl_gait(ctrl):
    """Upload low-crawl gait definition and parameters to the motion controller.

    Reads TOML files from config/ directory and sends them to the motion controller
    via LCM. gait_id=110 (USER_DEFINED) is used for the custom gait.
    
    Note: This is called every time low-crawl is needed, ensuring the correct
    gait parameters are loaded even if another custom gait was uploaded before.
    """
    # Resolve config directory relative to this file
    seg_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    config_dir = os.path.join(seg_dir, 'config')

    gait_def_path = os.path.join(config_dir, 'gait_def_low_crawl.toml')
    gait_params_path = os.path.join(config_dir, 'gait_params_low_crawl.toml')

    try:
        with open(gait_def_path, 'r') as f:
            gait_def = f.read()
        with open(gait_params_path, 'r') as f:
            gait_params = f.read()

        ctrl.send_gait_file(gait_def)
        time.sleep(0.15)
        ctrl.send_gait_file(gait_params)
        time.sleep(0.15)
        print("[Seg4] Low-crawl gait (id=110) uploaded")
    except Exception as e:
        print(f"[Seg4] Warning: Low-crawl gait upload failed: {e}")


def _get_object_at_waypoint(waypoint_idx):
    """Return object type at given waypoint index, or None."""
    obj_mapping = {
        4: 'coke',        # (-0.1, 11.0)
        9: 'orange_ball', # ( 0.95, 11.1)
        12: 'football',   # ( 2.1, 10.8)
    }
    return obj_mapping.get(waypoint_idx)


def _detect_scene(perception, obj_det, ball_det):
    """Detect objects and orange balls in current view."""
    rgb = perception.latest_rgb
    if rgb is None:
        return [], []
    try:
        objects = obj_det.detect_all(rgb)
        balls = ball_det.detect_all_balls(rgb)
        orange_balls = [b for b in balls if b['color'] == 'orange']
        return objects, orange_balls
    except Exception:
        return [], []


def _handle_scene_objects(ctrl, perception, speaker, announced, obj_handled,
                          objects, orange_balls, ball_det):
    """React to detected objects in the current frame."""
    for obj in sorted(objects, key=lambda o: o.get('area', 0), reverse=True):
        obj_type = obj['type']
        if obj_type not in announced:
            announced.add(obj_type)
            _announce(speaker, obj_type)
            time.sleep(0.2)

        if obj_type in obj_handled:
            continue

        if obj_type == 'coke':
            print("[Seg4] Knocking coke bottle (reactive)")
            _approach_by_pixel(ctrl, perception, obj, speed=0.18, duration_s=0.8)
            obj_handled.add('coke')
        elif obj_type == 'football':
            print("[Seg4] Driving football toward goal (reactive)")
            _approach_by_pixel(ctrl, perception, obj, speed=0.16, duration_s=0.4)
            ctrl.locomotion(
                gait_id=GAIT_TROT_FAST,
                vx=0.32, vy=0.0, wz=0.0,
                step_h_max=0.08, step_h_min=0.05,
                duration_ms=0,
            )
            time.sleep(0.8)
            obj_handled.add('football')
        elif obj_type == 'height_bar':
            print("[Seg4] Passing under height bar")
            _announce(speaker, 'height_bar')
            _pass_under_height_bar(ctrl)
            obj_handled.add('height_bar')
        elif obj_type == 'obstacle':
            print("[Seg4] Avoiding block obstacle")
            _avoid_obstacle(ctrl, obj)
            obj_handled.add('obstacle')

    if orange_balls:
        if 'orange_ball' not in announced:
            announced.add('orange_ball')
            _announce(speaker, 'orange_ball')
            time.sleep(0.2)
        if 'orange_ball' not in obj_handled:
            closest = min(orange_balls, key=ball_det.estimate_ball_distance)
            dist = ball_det.estimate_ball_distance(closest)
            if dist < 1.2:
                print(f"[Seg4] Hitting orange ball at {dist:.2f}m (reactive)")
                _approach_by_pixel(ctrl, perception, closest, speed=0.18, duration_s=0.8)
                obj_handled.add('orange_ball')


def _pass_under_height_bar(ctrl):
    """Pass under a height bar with low-crawl posture and low-crawl return.

    Uses Four-4's low_height gait (forward) and low_height_re gait (backward)
    instead of turning around, for smoother operation.
    """
    # Lower body first
    ctrl.set_height(0.10, duration_ms=400)
    time.sleep(0.5)
    ctrl.look_forward(-0.25, duration_ms=300)
    time.sleep(0.4)

    # Forward pass through the bar using low_height gait
    _upload_low_height_gait(ctrl)
    time.sleep(0.5)
    print("[Seg4] Low-height forward pass through height bar")
    for _ in range(8):
        ctrl.user_gait_execute(duration_ms=840)
        time.sleep(0.02)

    # Wait a moment, then use low_height_re for backward return (no turning)
    time.sleep(0.3)
    print("[Seg4] Low-height reverse pass (no turn)")
    _upload_low_height_re_gait(ctrl)
    time.sleep(0.5)
    for _ in range(6):
        ctrl.user_gait_execute(duration_ms=840)
        time.sleep(0.02)

    # Restore normal height
    ctrl.set_height(0.22, duration_ms=400)
    time.sleep(0.5)


def _avoid_obstacle(ctrl, obj):
    u = obj.get('u', 320.0)
    strafe_vy = 0.16 if u < 320.0 else -0.16
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=0.0, vy=strafe_vy, wz=0.0,
        step_h_max=0.05, step_h_min=0.03,
        duration_ms=0,
    )
    time.sleep(0.8)
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=0.14, vy=0.0, wz=0.0,
        step_h_max=0.05, step_h_min=0.03,
        duration_ms=0,
    )
    time.sleep(1.0)
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=0.0, vy=-strafe_vy, wz=0.0,
        step_h_max=0.05, step_h_min=0.03,
        duration_ms=0,
    )
    time.sleep(0.7)


def _approach_by_pixel(ctrl, perception, target, speed=0.18, duration_s=0.5):
    """Walk toward a target detected in the image, aligned by pixel position."""
    u = target.get('u', 320.0)
    du = u - 320.0
    angle = math.atan2(du, perception.fx)
    steer = max(-0.5, min(0.5, -angle * 2.5))
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=speed, vy=0.0, wz=steer,
        step_h_max=0.05, step_h_min=0.03,
        duration_ms=0,
    )
    time.sleep(duration_s)
