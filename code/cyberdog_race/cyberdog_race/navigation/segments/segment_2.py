"""
Segment 2: Wild Ball Search (荒野寻珠)

Primary behavior uses RGB orange-ball detection.  If the simulator has no
published RGB frames, fall back to a deterministic grid sweep based on the
course map so the robot still performs useful actions instead of walking
straight through the segment.
"""
import time
import math
from ...lcm_types import GAIT_TROT_SLOW
from ...perception.detectors.ball_detector import BallDetector
from ...perception.detectors.boundary_detector import BoundaryDetector
from ...utils.nav_helper import (
    go_to_point_with_heading,
    go_to_point_cardinal,
    turn_to_angle as _turn_to_angle,
)
from .. import course_map


BALL_HEIGHT_M = 0.20
MIN_HIT_DIST_M = 0.75
HIT_APPROACH_SPEED = 0.12
SEARCH_SPEED = 0.16
SEGMENT_TIMEOUT_S = 260.0
TARGET_ORANGE_BALLS = 4
EXIT_DRIVE_TIME_S = 3.0
RGB_WAIT_S = 2.0
BLUE_DANGER_DIST_M = 1.35
BLUE_CENTER_PX = 190.0
BOUNDARY_GAIN = 0.8
SEARCH_SWEEP_WZ = 0.22
ORANGE_APPROACH_DIST_M = 1.45
ORANGE_EXIT_APPROACH_DIST_M = 1.25
BLUE_BLOCK_U_PX = 90.0
WAYPOINT_TOL_M = 0.18
NAV_VX_MAX = 0.16
NAV_VY_MAX = 0.12
NAV_WZ_MAX = 0.35
HIT_DIST_M = 1.15
HIT_CENTER_PX = 130.0
HIT_TIMEOUT_S = 10.0
HIT_LOST_MARK_DIST_M = 1.30


# ---------------------------------------------------------------------------
# Ball interaction parameters
# ---------------------------------------------------------------------------


class BallTracker:
    def __init__(self):
        self._hit_set = set()

    def mark_hit(self, ball_key):
        self._hit_set.add(ball_key)

    def is_hit(self, ball_key):
        return ball_key in self._hit_set

    @property
    def total_hit(self):
        return len(self._hit_set)

    @property
    def all_found(self):
        return self.total_hit >= TARGET_ORANGE_BALLS


def execute(ctrl, perception, speaker=None, navigator=None):
    print("[Seg2] Starting Wild Ball Search")
    print(f"[Seg2] Start position={perception.position}, heading={perception.heading:.1f} deg")

    # Visual detection: prefers real RGB orange ball; falls back to geometry
    # sweep if RGB is unavailable.
    ball_detector = BallDetector(ball_height_m=BALL_HEIGHT_M)
    boundary_detector = BoundaryDetector()
    tracker = BallTracker()

    if _wait_for_rgb(perception, RGB_WAIT_S):
        return _execute_visual_search(
            ctrl, perception, speaker, ball_detector, boundary_detector, tracker, navigator)

    print("[Seg2] RGB not available within %.1fs, falling back to blind sweep" % RGB_WAIT_S)
    return _execute_geometry_sweep(ctrl, perception, speaker)


def _execute_visual_search(ctrl, perception, speaker, ball_detector,
                           boundary_detector, tracker, navigator=None):
    print("[Seg2] Visual waypoint mode - snake pattern with auto-hit")
    print(f"[Seg2] World-frame waypoints: {course_map.S2_WAYPOINTS}")
    print(f"[Seg2] Ball hit waypoints: {course_map.S2_BALL_HIT_WAYPOINTS}")
    print(f"[Seg2] Exit waypoints: {course_map.S2_EXIT_WAYPOINTS}")

    _walk(ctrl, vx=0.12, duration_s=0.5)

    seg_start = time.perf_counter()

    # Use world-frame waypoints from course_map
    waypoints = list(course_map.S2_WAYPOINTS)
    ball_hit_waypoints = {wp[:2]: wp[2:] for wp in course_map.S2_BALL_HIT_WAYPOINTS}
    exit_waypoints = list(course_map.S2_EXIT_WAYPOINTS)

    state = 'navigate'
    if waypoints:
        active_wp = waypoints.pop(0)
        # Normalize to 3 elements
        if len(active_wp) == 2:
            active_wp = (active_wp[0], active_wp[1], None)
    else:
        active_wp = None
    return_wp = active_wp
    return_state = 'navigate'
    hit_start = None
    hit_best_dist = float('inf')
    exit_started = False
    timeout_announced = False
    last_debug_log = 0.0

    while True:
        elapsed = time.perf_counter() - seg_start
        if elapsed > SEGMENT_TIMEOUT_S:
            if not timeout_announced:
                print(f"[Seg2] Timeout after hitting {tracker.total_hit}/4, exiting by waypoint")
                timeout_announced = True
            if state != 'exit':
                state = 'exit'
                exit_started = True
                if exit_waypoints:
                    active_wp = exit_waypoints.pop(0)

        orange_balls, blue_balls = _detect_balls(perception, ball_detector)
        if elapsed - last_debug_log > 1.0:
            _log_detection_state(perception, ball_detector, orange_balls, blue_balls)
            print(f"[Seg2] World pos={perception.position}, heading={perception.heading:.1f}, state={state}, hit={tracker.total_hit}/4")
            last_debug_log = elapsed

        closest_orange = None
        if orange_balls:
            unseen = [
                b for b in orange_balls
                if not tracker.is_hit(_ball_key(b, perception, ball_detector))
            ]
            candidates = unseen or orange_balls
            closest_orange = min(candidates, key=ball_detector.estimate_ball_distance)

        if tracker.all_found and state not in ('exit', 'done'):
            print("[Seg2] All 4 orange balls handled, switching to exit waypoints")
            state = 'exit'
            exit_started = False  # Reset flag so exit state will pop properly

        if state == 'navigate':
            # Check if we're at a ball hit waypoint
            px, py = perception.position
            for (bx, by), (target_heading, ball_num) in ball_hit_waypoints.items():
                dist_to_ball = math.sqrt((bx - px)**2 + (by - py)**2)
                if dist_to_ball < 0.5 and not tracker.is_hit(('planned', bx, by)):
                    print(f"[Seg2] At ball #{ball_num} position ({bx}, {by}), auto-hit!")
                    state = 'hit_orange'
                    hit_start = time.perf_counter()
                    hit_best_dist = dist_to_ball
                    break

            if state == 'navigate':
                if closest_orange is not None and _should_hit_orange(
                        perception, ball_detector, closest_orange, blue_balls):
                    dist = ball_detector.estimate_ball_distance(closest_orange)
                    print(f"[Seg2] Leaving route to hit orange at {dist:.2f}m")
                    return_wp = active_wp
                    return_state = 'navigate'
                    state = 'hit_orange'
                    hit_start = time.perf_counter()
                    hit_best_dist = dist
                    continue

            if state == 'navigate':
                wp_xy = (active_wp[0], active_wp[1]) if active_wp else None
                if _go_to_waypoint(ctrl, perception, active_wp):
                    if waypoints:
                        active_wp = waypoints.pop(0)
                        if len(active_wp) == 2:
                            active_wp = (active_wp[0], active_wp[1], None)
                        print(f"[Seg2] Next route waypoint: {active_wp}")
                    else:
                        print(f"[Seg2] Route complete with hits={tracker.total_hit}/4, exiting")
                        state = 'exit'
                        exit_started = False
                        active_wp = exit_waypoints.pop(0) if exit_waypoints else None

        elif state == 'hit_orange':
            px, py = perception.position
            
            # Check if this is a planned ball position
            planned_ball = None
            for (bx, by), (target_heading, ball_num) in ball_hit_waypoints.items():
                if not tracker.is_hit(('planned', bx, by)):
                    dist = math.sqrt((bx - px)**2 + (by - py)**2)
                    if dist < 0.5:
                        planned_ball = (bx, by, target_heading, ball_num)
                        break
            
            if planned_ball:
                bx, by, target_heading, ball_num = planned_ball
                print(f"[Seg2] Auto-hitting planned ball #{ball_num} at ({bx}, {by})")
                # Drive forward and bump
                _cmd(ctrl, vx=0.15)
                time.sleep(1.5)
                _cmd(ctrl, vx=-0.1)
                time.sleep(0.8)
                _cmd(ctrl, vx=0)
                tracker.mark_hit(('planned', bx, by))
                _announce_orange(speaker)
                print(f"[Seg2] Ball #{ball_num} hit! Total: {tracker.total_hit}/4")
                state = 'return_route'
                continue
            
            if closest_orange is None:
                if hit_best_dist < HIT_LOST_MARK_DIST_M:
                    _mark_current_orange_hit(tracker, None, perception, ball_detector,
                                             speaker, "lost after close contact")
                else:
                    print("[Seg2] Lost orange while hitting, returning to route")
                state = 'return_route'
                continue

            hit_best_dist = min(hit_best_dist,
                                ball_detector.estimate_ball_distance(closest_orange))
            if _drive_into_orange(ctrl, perception, ball_detector, closest_orange):
                _mark_current_orange_hit(tracker, closest_orange, perception,
                                         ball_detector, speaker, None)
                _walk(ctrl, vx=-0.10, duration_s=0.8)
                state = 'return_route'
                continue

            if time.perf_counter() - hit_start > HIT_TIMEOUT_S:
                if hit_best_dist < HIT_LOST_MARK_DIST_M:
                    _mark_current_orange_hit(tracker, closest_orange, perception,
                                             ball_detector, speaker,
                                             "timeout after close contact")
                else:
                    print("[Seg2] Hit attempt timeout, returning to route")
                _walk(ctrl, vx=-0.08, duration_s=0.5)
                state = 'return_route'
                continue

        elif state == 'return_route':
            if _go_to_waypoint(ctrl, perception, return_wp):
                print("[Seg2] Back on route")
                state = return_state

        elif state == 'exit':
            if closest_orange is not None and _should_hit_orange(
                    perception, ball_detector, closest_orange, blue_balls,
                    approach_dist=ORANGE_EXIT_APPROACH_DIST_M,
                    center_px=HIT_CENTER_PX):
                dist = ball_detector.estimate_ball_distance(closest_orange)
                print(f"[Seg2] Exit path close orange at {dist:.2f}m, bumping before exit")
                return_wp = active_wp
                return_state = 'exit'
                state = 'hit_orange'
                hit_start = time.perf_counter()
                hit_best_dist = dist
                continue

            if not exit_started:
                exit_started = True
                if exit_waypoints:
                    active_wp = exit_waypoints.pop(0)
            if _go_to_waypoint(ctrl, perception, active_wp):
                if exit_waypoints:
                    active_wp = exit_waypoints.pop(0)
                    print(f"[Seg2] Next exit waypoint: {active_wp}")
                else:
                    print("[Seg2] Exit waypoint reached")
                    ctrl.stop_moving()
                    return True

        time.sleep(0.05)


def _execute_geometry_sweep(ctrl, perception, speaker):
    """
    Blind world-frame navigation using actual waypoint feedback.

    The previous implementation held one cardinal heading per phase and used a
    shared timeout, so after phase 1 timed out every later phase immediately
    timed out and still printed "Ball hit".  This version only bumps a planned
    ball after reaching the matching approach pose.
    """
    print("[Seg2] Smart blind navigation with position feedback")
    print(f"[Seg2] Start position={perception.position}, heading={perception.heading:.1f} deg")
    print(f"[Seg2] Approach waypoints: {course_map.S2_WAYPOINTS}")
    print(f"[Seg2] Planned orange balls: {course_map.S2_BALL_HIT_WAYPOINTS}")

    seg_start = time.perf_counter()

    attempted_hits = 0
    hit_by_approach = _build_s2_hit_lookup()
    for phase, target in enumerate(course_map.S2_WAYPOINTS, start=1):
        if time.perf_counter() - seg_start > SEGMENT_TIMEOUT_S:
            print(f"[Seg2] Segment timeout before phase {phase}; going to exit")
            break

        print(f"[Seg2] Phase {phase}: navigating to ({target[0]}, {target[1]})")
        is_hit_approach = target[:2] in hit_by_approach
        pos_tol = 0.12 if is_hit_approach else 0.20
        if not _navigate_to_pose(ctrl, perception, target, phase,
                                 timeout_s=35.0, pos_tol=pos_tol):
            print(f"[Seg2] Phase {phase}: waypoint not reached, continuing route from current pose")
            continue

        if not is_hit_approach:
            continue

        bx, by, heading, ball_num = hit_by_approach[target[:2]]
        if not _turn_to_angle(ctrl, perception, heading, turn_rate=0.5,
                              tol_deg=10.0, timeout_s=8.0):
            print(f"[Seg2] Phase {phase}: skipped ball #{ball_num}; heading not reached")
            continue

        _bump_planned_ball(ctrl, speaker, ball_num)
        attempted_hits += 1
        print(f"[Seg2] Ball #{ball_num} bump attempted. Total attempts: {attempted_hits}/4, "
              f"pos={perception.position}, heading={perception.heading:.1f} deg")

    print(f"[Seg2] Exiting through S2 gate to S3 entry {course_map.S2_EXIT_COORD}")
    for exit_idx, exit_wp in enumerate(course_map.S2_EXIT_WAYPOINTS, start=1):
        if not _navigate_to_s2_exit(ctrl, perception, exit_wp, f"exit-{exit_idx}",
                                    timeout_s=40.0):
            print(f"[Seg2] Exit waypoint {exit_idx} not reached; current pos={perception.position}, "
                  f"heading={perception.heading:.1f} deg")
            break

    ctrl.stop_moving()
    total_elapsed = time.perf_counter() - seg_start
    print(f"[Seg2] Smart blind navigation complete. Ball bump attempts: "
          f"{attempted_hits}/4. Total time: {total_elapsed:.1f}s")
    return True


def _detect_balls(perception, ball_detector):
    rgb = perception.latest_rgb
    if rgb is None:
        return [], []
    try:
        balls = ball_detector.detect_all_balls(rgb)
        orange = [b for b in balls if b['color'] == 'orange']
        blue = [b for b in balls if b['color'] == 'light_blue']
        return orange, blue
    except Exception as e:
        print(f"[Seg2] Ball detection error: {e}")
        return [], []


def _build_s2_hit_lookup():
    lookup = {}
    for bx, by, heading, ball_num in course_map.S2_BALL_HIT_WAYPOINTS:
        best_wp = min(
            course_map.S2_WAYPOINTS,
            key=lambda wp: (wp[0] - bx) ** 2 + (wp[1] - by) ** 2,
        )
        lookup[best_wp[:2]] = (bx, by, heading, ball_num)
    return lookup


def _go_to_waypoint(ctrl, perception, waypoint):
    """
    Drive toward a world-frame waypoint (x, y).

    Proportional heading + lateral correction in world frame.
    """
    px, py = perception.position
    heading_deg = perception.heading
    dx = waypoint[0] - px
    dy = waypoint[1] - py
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < WAYPOINT_TOL_M:
        ctrl.stop_moving()
        return True

    h_rad = math.radians(heading_deg)
    cos_h = math.cos(-h_rad)
    sin_h = math.sin(-h_rad)
    bx = dx * cos_h - dy * sin_h
    by = dx * sin_h + dy * cos_h
    heading_error = math.atan2(by, max(0.15, bx))

    vx = _clamp(bx * 0.35, -0.06, NAV_VX_MAX)
    vy = _clamp(by * 0.35, -NAV_VY_MAX, NAV_VY_MAX)
    wz = _clamp(heading_error * 0.8, -NAV_WZ_MAX, NAV_WZ_MAX)
    if bx < -0.05:
        vx = -0.05
    _cmd(ctrl, vx=vx, vy=vy, wz=wz)
    return False


def _should_hit_orange(perception, ball_detector, orange, blue_balls,
                       approach_dist=ORANGE_APPROACH_DIST_M,
                       center_px=HIT_CENTER_PX):
    dist = ball_detector.estimate_ball_distance(orange)
    centered = abs(orange['u'] - perception.cx) < center_px
    low_enough = orange.get('v', 0.0) > perception.cy * 0.45
    if dist > approach_dist or not centered or not low_enough:
        return False
    return _blocking_blue_for_orange(perception, ball_detector, orange, blue_balls) is None


def _drive_into_orange(ctrl, perception, ball_detector, orange):
    du = orange['u'] - perception.cx
    steer = _clamp(-math.atan2(du, perception.fx) * 2.8, -0.45, 0.45)
    dist = ball_detector.estimate_ball_distance(orange)
    if dist > HIT_DIST_M:
        vx = _clamp(dist * 0.20, 0.10, 0.24)
        _cmd(ctrl, vx=vx, wz=steer)
        return False

    print(f"[Seg2] Final bump into orange at {dist:.2f}m")
    _cmd(ctrl, vx=0.32, wz=steer)
    time.sleep(0.75)
    return True


def _approach_orange_ball(ctrl, perception, closest_orange, ball_det, blue_balls):
    u = closest_orange['u']
    du = u - 320.0
    da = math.atan2(du, perception.fx)
    steer = max(-0.5, min(0.5, -da * 2.5))
    dist = ball_det.estimate_ball_distance(closest_orange)

    blocking_blue = _blocking_blue_for_orange(
        perception, ball_det, closest_orange, blue_balls)
    if blocking_blue is not None:
        bu = blocking_blue['u']
        side = -1.0 if bu > 320.0 else 1.0
        print("[Seg2] Blue ball blocking orange approach, sidestepping")
        _cmd(ctrl, vx=-0.04, vy=side * 0.18, wz=steer * 0.2)
        time.sleep(0.65)
        return False

    if dist > MIN_HIT_DIST_M:
        speed = min(HIT_APPROACH_SPEED, dist * 0.4)
        _cmd(ctrl, vx=speed, wz=steer)
        return False
    _cmd(ctrl, vx=HIT_APPROACH_SPEED * 2.0, wz=steer)
    time.sleep(0.45)
    return True


def _search_step(ctrl, phase, step_count, strafe_dir, boundary_steer, elapsed):
    sweep = SEARCH_SWEEP_WZ if int(elapsed / 2.0) % 2 == 0 else -SEARCH_SWEEP_WZ
    wz = max(-0.3, min(0.3, boundary_steer + sweep))
    if phase == 'forward':
        _cmd(ctrl, vx=SEARCH_SPEED * 0.35, vy=0.0, wz=wz)
        step_count += 1
        if step_count >= 14:
            return 'strafe', 0, -strafe_dir
        return phase, step_count, strafe_dir

    _cmd(
        ctrl,
        vx=SEARCH_SPEED * 0.10,
        vy=strafe_dir * SEARCH_SPEED * 0.75,
        wz=boundary_steer * 0.5 + sweep * 0.4,
    )
    step_count += 1
    if step_count >= 20:
        return 'forward', 0, strafe_dir
    return phase, step_count, strafe_dir


def _drive_exit(ctrl, started_at):
    exit_elapsed = time.perf_counter() - started_at
    if exit_elapsed < 1.4:
        _cmd(ctrl, vx=0.05, vy=0.16, wz=0.15)
        return False
    if exit_elapsed < EXIT_DRIVE_TIME_S:
        _cmd(ctrl, vx=0.18, vy=0.08)
        return False
    print("[Seg2] Exit line crossed by timed odometry fallback")
    ctrl.stop_moving()
    return True


def _exit_segment(ctrl):
    started = time.perf_counter()
    while not _drive_exit(ctrl, started):
        time.sleep(0.05)


def _wait_for_rgb(perception, timeout_s):
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        if perception.latest_rgb is not None:
            return True
        time.sleep(0.05)
    return False


def _avoid_blue_if_needed(ctrl, perception, ball_detector, blue_balls):
    danger = _closest_center_blue(perception, ball_detector, blue_balls)
    if danger is None:
        return False
    side = -1.0 if danger['u'] > 320.0 else 1.0
    dist = ball_detector.estimate_ball_distance(danger)
    print(f"[Seg2] Avoiding blue ball at {dist:.2f}m")
    _cmd(ctrl, vx=-0.04, vy=side * 0.16, wz=0.0)
    time.sleep(0.55)
    return True


def _closest_center_blue(perception, ball_detector, blue_balls):
    centered = [
        b for b in blue_balls
        if abs(b['u'] - perception.cx) < BLUE_CENTER_PX and
        ball_detector.estimate_ball_distance(b) < BLUE_DANGER_DIST_M and
        b.get('v', 0.0) > perception.cy * 0.55
    ]
    if not centered:
        return None
    return min(centered, key=ball_detector.estimate_ball_distance)


def _blocking_blue_for_orange(perception, ball_detector, orange, blue_balls):
    orange_dist = ball_detector.estimate_ball_distance(orange)
    blockers = []
    for blue in blue_balls:
        blue_dist = ball_detector.estimate_ball_distance(blue)
        same_ray = abs(blue['u'] - orange['u']) < BLUE_BLOCK_U_PX
        closer = blue_dist < orange_dist - 0.12
        very_close_center = (
            abs(blue['u'] - perception.cx) < BLUE_CENTER_PX * 0.65 and
            blue_dist < 0.75
        )
        if (same_ray and closer) or very_close_center:
            blockers.append(blue)
    if not blockers:
        return None
    return min(blockers, key=ball_detector.estimate_ball_distance)


def _boundary_steer(perception, boundary_detector):
    rgb = perception.latest_rgb
    if rgb is None:
        return 0.0
    try:
        left_u, right_u = boundary_detector.detect_left_right_lines(rgb)
    except Exception:
        return 0.0
    width = rgb.shape[1]
    steer = 0.0
    # Yellow line close to an image edge means we are near that boundary.
    if left_u is not None and left_u > width * 0.28:
        steer -= 0.18
    if right_u is not None and right_u < width * 0.72:
        steer += 0.18
    return max(-0.25, min(0.25, steer * BOUNDARY_GAIN))


def _ball_key(ball, perception, ball_det):
    try:
        rx, ry, yaw = perception.odom_pose
        wx, wy = ball_det.ball_to_world_estimate(ball, rx, ry, yaw)
        return (round(wx / 0.35), round(wy / 0.35))
    except Exception:
        return (int(ball['u'] // 80), int(ball['v'] // 80))


def _mark_current_orange_hit(tracker, ball, perception, ball_detector,
                             speaker, reason=None):
    if ball is None:
        rx, ry, _ = perception.odom_pose
        key = ('lost', round(rx / 0.35), round(ry / 0.35))
    else:
        key = _ball_key(ball, perception, ball_detector)
    if tracker.is_hit(key):
        return
    tracker.mark_hit(key)
    suffix = f" ({reason})" if reason else ""
    print(f"[Seg2] Orange ball hit! Total: {tracker.total_hit}/4{suffix}")
    _announce_orange(speaker)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _wrap_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _log_detection_state(perception, ball_detector, orange_balls, blue_balls):
    if perception.latest_rgb is None:
        print("[Seg2] Detect: no RGB frame")
        return
    if orange_balls:
        nearest = min(orange_balls, key=ball_detector.estimate_ball_distance)
        dist = ball_detector.estimate_ball_distance(nearest)
        print("[Seg2] Detect: "
              f"orange={len(orange_balls)} nearest={dist:.2f}m "
              f"u={nearest['u']:.0f}, blue={len(blue_balls)}")
    else:
        if blue_balls:
            nearest_blue = min(blue_balls, key=ball_detector.estimate_ball_distance)
            blue_dist = ball_detector.estimate_ball_distance(nearest_blue)
            print("[Seg2] Detect: "
                  f"orange=0, blue={len(blue_balls)} "
                  f"nearest_blue={blue_dist:.2f}m "
                  f"u={nearest_blue['u']:.0f}, v={nearest_blue['v']:.0f}, "
                  f"r={nearest_blue['radius']:.0f}")
        else:
            print("[Seg2] Detect: orange=0, blue=0")


def _navigate_to_pose(ctrl, perception, waypoint, phase, timeout_s=35.0,
                      pos_tol=0.20, final_heading_tol=12.0):
    """Navigate to (x, y, heading_deg) with a bounded timeout."""
    tx, ty, target_heading = waypoint
    start = time.perf_counter()
    debug_count = 0

    while time.perf_counter() - start < timeout_s:
        px, py = perception.position
        dx = tx - px
        dy = ty - py
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < pos_tol:
            ctrl.stop_moving()
            if target_heading is not None:
                return _turn_to_angle(
                    ctrl, perception, target_heading,
                    turn_rate=0.5, tol_deg=final_heading_tol, timeout_s=8.0)
            return True

        _drive_world_error(ctrl, perception, dx, dy)
        time.sleep(0.05)

        debug_count += 1
        if debug_count % 40 == 0:
            elapsed = time.perf_counter() - start
            print(f"[Seg2] Phase {phase}: current=({px:.3f}, {py:.3f}), "
                  f"target=({tx:.2f}, {ty:.2f}), dist={dist:.3f}m, "
                  f"heading={perception.heading:.1f} deg, elapsed={elapsed:.1f}s")

    ctrl.stop_moving()
    return False


def _navigate_to_s2_exit(ctrl, perception, waypoint, phase, timeout_s=40.0):
    """Drive through the S2 exit line instead of accepting a loose radius."""
    tx, ty, _ = waypoint
    start = time.perf_counter()
    debug_count = 0

    while time.perf_counter() - start < timeout_s:
        px, py = perception.position
        dx = tx - px
        dy = ty - py
        x_err = abs(dx)

        if py >= ty - 0.02 and x_err < 0.22:
            ctrl.stop_moving()
            print(f"[Seg2] S2 exit crossed at ({px:.3f}, {py:.3f}), "
                  f"target=({tx:.2f}, {ty:.2f})")
            return True

        _drive_world_error(ctrl, perception, dx, dy)
        time.sleep(0.05)

        debug_count += 1
        if debug_count % 40 == 0:
            elapsed = time.perf_counter() - start
            dist = math.sqrt(dx * dx + dy * dy)
            print(f"[Seg2] Phase {phase}: current=({px:.3f}, {py:.3f}), "
                  f"target=({tx:.2f}, {ty:.2f}), dist={dist:.3f}m, "
                  f"x_err={x_err:.3f}m, heading={perception.heading:.1f} deg, "
                  f"elapsed={elapsed:.1f}s")

    ctrl.stop_moving()
    return False


def _drive_world_error(ctrl, perception, dx, dy):
    """Convert world-frame error into body-frame vx/vy/wz."""
    heading_rad = math.radians(perception.heading)
    cos_h = math.cos(-heading_rad)
    sin_h = math.sin(-heading_rad)
    bx = dx * cos_h - dy * sin_h
    by = dx * sin_h + dy * cos_h
    dist = math.sqrt(dx * dx + dy * dy)

    heading_error = math.atan2(by, max(0.08, bx))
    vx = _clamp(bx * 0.45, -0.06, 0.16)
    if dist > 0.5 and bx > 0.0:
        vx = max(vx, 0.08)
    vy = _clamp(by * 0.45, -0.12, 0.12)
    wz = _clamp(heading_error * 0.9, -0.35, 0.35)
    _cmd(ctrl, vx=vx, vy=vy, wz=wz)


def _bump_planned_ball(ctrl, speaker, ball_num):
    print(f"[Seg2] Bumping planned ball #{ball_num}")
    _cmd(ctrl, vx=0.16)
    time.sleep(0.95)
    _cmd(ctrl, vx=-0.10)
    time.sleep(0.55)
    _cmd(ctrl, vx=0.0)
    _announce_orange(speaker)


def _announce_orange(speaker):
    if speaker:
        try:
            speaker.announce('orange_ball')
        except Exception:
            pass


def _cmd(ctrl, vx=0.0, vy=0.0, wz=0.0):
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=vx, vy=vy, wz=wz,
        step_h_max=0.06, step_h_min=0.04,
        duration_ms=0,
    )


def _walk(ctrl, vx, duration_s):
    _cmd(ctrl, vx=vx)
    time.sleep(duration_s)
