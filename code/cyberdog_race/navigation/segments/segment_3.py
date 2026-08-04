"""Segment 3: Curved Sprint (曲道冲锋)."""
import time
import math
from ...lcm_types import GAIT_TROT_SLOW
from .. import course_map


SEGMENT_TIMEOUT_S = 120.0
ENTRY_TOL_M = 0.20
WAYPOINT_TOL_M = 0.15


def execute(ctrl, perception, navigator=None):
    """
    Execute segment 3 with world-frame waypoint navigation.

    The old implementation planned waypoints but never called the navigator; it
    instead drove from LiDAR side-distance heuristics, so the printed route did
    not control the robot.  This version first converges to the S3 entry if
    segment 2 ended off-target, then drives each configured waypoint directly.
    """
    print("[Seg3] Starting Curved Sprint with world-frame navigation")
    print(f"[Seg3] Start position={perception.position}, heading={perception.heading:.1f} deg")
    print(f"[Seg3] Waypoints: {course_map.S3_WAYPOINTS}")

    seg_start = time.perf_counter()

    entry_x, entry_y, entry_heading = course_map.seg_start(3)
    entry_dist = _dist_to(perception, entry_x, entry_y)
    if entry_dist > ENTRY_TOL_M:
        print(f"[Seg3] Re-aligning to S3 entry ({entry_x}, {entry_y}), "
              f"current dist={entry_dist:.2f}m")
        _navigate_to_pose(
            ctrl, perception, (entry_x, entry_y, entry_heading),
            "entry", seg_start, timeout_s=35.0, tol_m=ENTRY_TOL_M)
    else:
        _turn_to_angle(ctrl, perception, entry_heading, timeout_s=8.0)

    start_index = _nearest_waypoint_index(perception, course_map.S3_WAYPOINTS)
    if start_index > 0:
        print(f"[Seg3] Starting from nearest waypoint index {start_index + 1}")

    for index, waypoint in enumerate(course_map.S3_WAYPOINTS[start_index:], start=start_index + 1):
        elapsed = time.perf_counter() - seg_start
        if elapsed > SEGMENT_TIMEOUT_S:
            print("[Seg3] Timeout")
            ctrl.stop_moving()
            return True

        print(f"[Seg3] Waypoint {index}/{len(course_map.S3_WAYPOINTS)}: {waypoint}")
        reached = _navigate_to_pose(
            ctrl, perception, waypoint, index, seg_start,
            timeout_s=min(30.0, SEGMENT_TIMEOUT_S - elapsed),
            tol_m=WAYPOINT_TOL_M)
        if not reached:
            print(f"[Seg3] Waypoint {index} not reached; current pos={perception.position}, "
                  f"heading={perception.heading:.1f} deg")

    print("[Seg3] Curved sprint complete (world-frame navigation done)")
    ctrl.stop_moving()
    return True


def _navigate_to_pose(ctrl, perception, waypoint, label, seg_start,
                      timeout_s=30.0, tol_m=WAYPOINT_TOL_M):
    tx, ty, target_heading = waypoint
    start = time.perf_counter()
    debug_count = 0

    while time.perf_counter() - start < timeout_s:
        if time.perf_counter() - seg_start > SEGMENT_TIMEOUT_S:
            ctrl.stop_moving()
            return False

        px, py = perception.position
        dx = tx - px
        dy = ty - py
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < tol_m:
            ctrl.stop_moving()
            if target_heading is not None:
                return _turn_to_angle(ctrl, perception, target_heading,
                                      tol_deg=12.0, timeout_s=8.0)
            return True

        _drive_world_error(ctrl, perception, dx, dy)
        time.sleep(0.05)

        debug_count += 1
        if debug_count % 40 == 0:
            elapsed = time.perf_counter() - seg_start
            print(f"[Seg3] {label}: current=({px:.3f}, {py:.3f}), "
                  f"target=({tx:.2f}, {ty:.2f}), dist={dist:.3f}m, "
                  f"heading={perception.heading:.1f} deg, elapsed={elapsed:.1f}s")

    ctrl.stop_moving()
    return False


def _drive_world_error(ctrl, perception, dx, dy):
    heading_rad = math.radians(perception.heading)
    cos_h = math.cos(-heading_rad)
    sin_h = math.sin(-heading_rad)
    bx = dx * cos_h - dy * sin_h
    by = dx * sin_h + dy * cos_h
    dist = math.sqrt(dx * dx + dy * dy)

    heading_error = math.atan2(by, max(0.08, bx))
    vx = _clamp(bx * 0.65, -0.10, 0.32)
    if dist > 0.6 and bx > 0.0:
        vx = max(vx, 0.14)
    vy = _clamp(by * 0.60, -0.16, 0.16)
    wz = _clamp(heading_error * 1.05, -0.50, 0.50)
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=vx, vy=vy, wz=wz,
        step_h_max=0.06, step_h_min=0.04,
        duration_ms=0,
    )


def _turn_to_angle(ctrl, perception, target_deg, tol_deg=10.0, timeout_s=8.0):
    start = time.perf_counter()
    target = target_deg % 360.0
    while time.perf_counter() - start < timeout_s:
        diff = _angle_diff(target, perception.heading)
        if abs(diff) <= tol_deg:
            ctrl.stop_moving()
            return True
        wz = _clamp(diff * math.pi / 180.0 * 1.2, -0.45, 0.45)
        ctrl.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=0.0, vy=0.0, wz=wz,
            step_h_max=0.06, step_h_min=0.04,
            duration_ms=0,
        )
        time.sleep(0.05)
    ctrl.stop_moving()
    return False


def _dist_to(perception, x, y):
    px, py = perception.position
    return math.sqrt((x - px) ** 2 + (y - py) ** 2)


def _nearest_waypoint_index(perception, waypoints):
    distances = [_dist_to(perception, wp[0], wp[1]) for wp in waypoints]
    best = min(range(len(distances)), key=distances.__getitem__)
    return best if distances[best] < 0.35 else 0


def _angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))
