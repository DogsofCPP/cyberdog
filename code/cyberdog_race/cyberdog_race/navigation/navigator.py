"""
Navigator - Global World-Frame Reactive Navigation.

Maintains a world-frame waypoint plan and provides reactive go-to logic:
  - Plan through a list of waypoints
  - Detect when the dog drifts off the planned path
  - Detect when the dog is stuck and needs recovery
  - Compute velocity commands to drive toward the current target
  - Advance to next waypoint when within tolerance

Usage in segments:
    nav = Navigator(perception)
    nav.plan_through(course_map.S1_WAYPOINTS)
    while True:
        state = nav.update(ctrl)
        if state == 'arrived':
            break
        # handle segment-specific logic (ball hitting, bridge crossing, etc.)
        time.sleep(0.05)
"""
import math
import time
from . import course_map


class Navigator:
    """World-frame reactive waypoint navigator."""

    def __init__(self, perception):
        self._perc = perception
        self._waypoints = []          # list of (x, y, yaw_deg) tuples
        self._wp_index = 0
        self._state = 'idle'

        # Tracking
        self._last_pose = None
        self._last_pose_time = None
        self._stuck_start = None

        # Deviation recovery
        self._recovery_target = None

        # Params
        self._wp_tol = course_map.WAYPOINT_TOL_M
        self._heading_tol = course_map.HEADING_TOL_DEG
        self._stuck_speed = course_map.STUCK_SPEED_THRESHOLD
        self._stuck_time = course_map.STUCK_TIME_S
        self._dev_thresh = course_map.DEVIATION_THRESHOLD_M

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan_through(self, waypoints, start_index=0):
        """
        Set the full waypoint plan.

        Args:
            waypoints: list of (x, y) or (x, y, yaw_deg) tuples
            start_index: index to begin from (default 0)
        """
        normalized = []
        for wp in waypoints:
            if len(wp) == 2:
                normalized.append((wp[0], wp[1], None))
            elif len(wp) >= 3:
                normalized.append((wp[0], wp[1], wp[2]))
        self._waypoints = normalized
        self._wp_index = start_index
        self._state = 'navigating'
        self._stuck_start = None
        self._recovery_target = None

    def plan_to(self, x, y, yaw_deg=None):
        """Set a single waypoint target."""
        self._waypoints = [(x, y, yaw_deg)]
        self._wp_index = 0
        self._state = 'navigating'
        self._stuck_start = None
        self._recovery_target = None

    def is_arrived(self):
        """Return True if currently at the active waypoint."""
        if not self._waypoints or self._wp_index >= len(self._waypoints):
            return True
        tx, ty, _ = self._waypoints[self._wp_index]
        px, py = self._perc.position
        dx = tx - px
        dy = ty - py
        return math.sqrt(dx*dx + dy*dy) < self._wp_tol

    def is_done(self):
        """Return True if all waypoints have been processed."""
        if not self._waypoints:
            return True
        return self._wp_index >= len(self._waypoints)

    @property
    def state(self):
        return self._state

    @property
    def current_target(self):
        """Return (x, y, yaw_deg) of current target waypoint."""
        if not self._waypoints or self._wp_index >= len(self._waypoints):
            return None
        return self._waypoints[self._wp_index]

    @property
    def current_index(self):
        return self._wp_index

    @property
    def total_waypoints(self):
        return len(self._waypoints)

    def reset(self):
        """Reset navigator state."""
        self._waypoints = []
        self._wp_index = 0
        self._state = 'idle'
        self._stuck_start = None
        self._recovery_target = None
        self._last_pose = None
        self._last_pose_time = None

    # ------------------------------------------------------------------
    # Main update loop
    # ------------------------------------------------------------------

    def update(self, ctrl, gait_id=2, step_h_max=0.06, step_h_min=0.04):
        """
        Call each tick to drive toward current waypoint.

        Returns navigation state string:
          'arrived'      - current waypoint reached, advancing or done
          'navigating'   - actively driving toward waypoint
          'stuck'        - detected stuck, executing recovery
          'deviation'    - detected off-plan, steering back
          'idle'         - no plan active
        """
        # No active plan
        if not self._waypoints or self._wp_index >= len(self._waypoints):
            self._state = 'idle'
            return self._state

        # Check if current waypoint is reached
        if self._check_advance():
            self._advance_waypoint()
            if self._wp_index >= len(self._waypoints):
                self._state = 'arrived'
                return self._state

        # Check for stuck
        if self._check_stuck():
            self._state = 'stuck'
            self._recover_stuck(ctrl, gait_id, step_h_max, step_h_min)
            return self._state

        # Check for deviation from plan
        if self._check_deviation():
            self._state = 'deviation'
            self._recover_deviation(ctrl, gait_id, step_h_max, step_h_min)
            return self._state

        # Normal navigation
        self._state = 'navigating'
        vx, vy, wz = self._compute_velocity(
            gait_id, step_h_max, step_h_min)
        ctrl.locomotion(
            gait_id=gait_id,
            vx=vx, vy=vy, wz=wz,
            step_h_max=step_h_max, step_h_min=step_h_min,
            duration_ms=0,
        )
        return self._state

    # ------------------------------------------------------------------
    # Velocity computation (proportional control toward waypoint)
    # ------------------------------------------------------------------

    def _compute_velocity(self, gait_id, step_h_max, step_h_min):
        """Compute (vx, vy, wz) to drive toward current target."""
        tx, ty, t_yaw = self._waypoints[self._wp_index]
        px, py = self._perc.position
        heading_deg = self._perc.heading

        # Distance and direction to target
        dx = tx - px
        dy = ty - py
        dist = math.sqrt(dx*dx + dy*dy)

        # Body-frame error
        h_rad = math.radians(heading_deg)
        cos_h = math.cos(-h_rad)
        sin_h = math.sin(-h_rad)
        bx = dx * cos_h - dy * sin_h
        by = dx * sin_h + dy * cos_h

        # Speed scales with distance
        speed = self._speed_for_dist(dist)

        # Proportional heading correction
        heading_error = math.atan2(by, max(0.1, bx))
        max_wz = self._wz_limit(gait_id)
        wz = _clamp(heading_error * 2.0, -max_wz, max_wz)

        # Lateral correction
        max_vy = self._vy_limit(gait_id)
        vy = _clamp(by * 0.8, -max_vy, max_vy)

        # Forward speed
        if bx < -0.1:
            vx = -0.05
        else:
            vx = _clamp(dist * 0.5, 0.05, speed)

        return vx, vy, wz

    def _speed_for_dist(self, dist):
        """Return max forward speed based on distance to waypoint."""
        if dist > 3.0:
            return 0.30
        elif dist > 1.0:
            return 0.20
        elif dist > 0.5:
            return 0.15
        else:
            return 0.10

    def _wz_limit(self, gait_id):
        """Max angular speed for gait."""
        if gait_id == 2:   # GAIT_TROT_FAST
            return 0.5
        elif gait_id == 1:  # GAIT_TROT_SLOW
            return 0.35
        return 0.25

    def _vy_limit(self, gait_id):
        """Max lateral speed for gait."""
        return 0.12

    # ------------------------------------------------------------------
    # Waypoint advance
    # ------------------------------------------------------------------

    def _check_advance(self):
        """Return True if current waypoint is close enough to advance."""
        return self.is_arrived()

    def _advance_waypoint(self):
        """Move to next waypoint. Returns True if done."""
        if self._wp_index < len(self._waypoints):
            tx, ty, t_yaw = self._waypoints[self._wp_index]
            print(f"[Nav] Waypoint {self._wp_index} reached: ({tx:.2f}, {ty:.2f})")
            self._wp_index += 1

        if self._wp_index >= len(self._waypoints):
            print("[Nav] All waypoints reached")
            return True
        return False

    def force_advance(self):
        """Manually advance to next waypoint."""
        self._advance_waypoint()

    def skip_remaining(self):
        """Skip all remaining waypoints."""
        self._wp_index = len(self._waypoints)
        self._state = 'arrived'

    # ------------------------------------------------------------------
    # Stuck detection and recovery
    # ------------------------------------------------------------------

    def _check_stuck(self):
        """Return True if robot appears stuck."""
        now = time.perf_counter()
        rx, ry, _ = self._perc.odom_pose

        if self._last_pose is None:
            self._last_pose = (rx, ry)
            self._last_pose_time = now
            self._stuck_start = None
            return False

        dt = now - self._last_pose_time
        if dt < 0.1:
            return False

        dx = rx - self._last_pose[0]
        dy = ry - self._last_pose[1]
        speed = math.sqrt(dx*dx + dy*dy) / dt

        self._last_pose = (rx, ry)
        self._last_pose_time = now

        if speed < self._stuck_speed:
            if self._stuck_start is None:
                self._stuck_start = now
            return (now - self._stuck_start) > self._stuck_time
        else:
            self._stuck_start = None
            return False

    def _recover_stuck(self, ctrl, gait_id, step_h_max, step_h_min):
        """Execute stuck recovery: back up and rotate."""
        print("[Nav] Stuck recovery: backing up and turning")
        ctrl.locomotion(
            gait_id=gait_id,
            vx=-0.10, vy=0.0, wz=0.0,
            step_h_max=step_h_max, step_h_min=step_h_min,
            duration_ms=0,
        )
        time.sleep(0.5)
        ctrl.locomotion(
            gait_id=gait_id,
            vx=0.0, vy=0.0, wz=0.4,
            step_h_max=step_h_max, step_h_min=step_h_min,
            duration_ms=0,
        )
        time.sleep(0.4)
        self._stuck_start = None

    # ------------------------------------------------------------------
    # Deviation detection and recovery
    # ------------------------------------------------------------------

    def _check_deviation(self):
        """
        Return True if robot is significantly off the planned path.

        Compares current position to the expected position along the
        straight line from the previous waypoint to the current target.
        """
        if self._wp_index == 0 or self._wp_index >= len(self._waypoints):
            return False

        # Expected position: project current odom along the line from
        # previous waypoint to current target
        prev = self._waypoints[self._wp_index - 1]
        curr = self._waypoints[self._wp_index]
        px, py = self._perc.position

        # Cross-track error: perpendicular distance from current position
        # to the line connecting prev and curr waypoints
        pwx, pwy = prev[0], prev[1]
        cwx, cwy = curr[0], curr[1]

        dx_wp = cwx - pwx
        dy_wp = cwy - pwy
        len_wp = math.sqrt(dx_wp*dx_wp + dy_wp*dy_wp)

        if len_wp < 0.01:
            return False

        # Project px,py onto the line segment
        t = max(0.0, min(1.0,
            ((px - pwx) * dx_wp + (py - pwy) * dy_wp) / (len_wp * len_wp)
        ))
        ex = pwx + t * dx_wp
        ey = pwy + t * dy_wp

        err = math.sqrt((px - ex)**2 + (py - ey)**2)
        return err > self._dev_thresh

    def _recover_deviation(self, ctrl, gait_id, step_h_max, step_h_min):
        """
        Steer back toward the expected position on the plan.

        Computes the point on the plan line closest to current position,
        then drives toward it.
        """
        if self._wp_index == 0:
            return

        prev = self._waypoints[self._wp_index - 1]
        curr = self._waypoints[self._wp_index]
        px, py = self._perc.position
        heading_deg = self._perc.heading

        pwx, pwy = prev[0], prev[1]
        cwx, cwy = curr[0], curr[1]

        dx_wp = cwx - pwx
        dy_wp = cwy - pwy
        len_wp_sq = dx_wp*dx_wp + dy_wp*dy_wp

        if len_wp_sq < 0.0001:
            return

        t = max(0.0, min(1.0,
            ((px - pwx) * dx_wp + (py - pwy) * dy_wp) / len_wp_sq
        ))
        tx = pwx + t * dx_wp
        ty = pwy + t * dy_wp

        print(f"[Nav] Deviation recovery: heading to ({tx:.2f}, {ty:.2f})")

        # Drive toward recovery target
        h_rad = math.radians(heading_deg)
        cos_h = math.cos(-h_rad)
        sin_h = math.sin(-h_rad)
        dx = tx - px
        dy = ty - py
        bx = dx * cos_h - dy * sin_h
        by = dx * sin_h + dy * cos_h

        heading_error = math.atan2(by, max(0.1, bx))
        wz = _clamp(heading_error * 2.5, -0.3, 0.3)
        vx = 0.12

        ctrl.locomotion(
            gait_id=gait_id,
            vx=vx, vy=0.0, wz=wz,
            step_h_max=step_h_max, step_h_min=step_h_min,
            duration_ms=0,
        )

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def dist_to_target(self):
        """Return distance from current position to active target."""
        if not self._waypoints or self._wp_index >= len(self._waypoints):
            return 0.0
        tx, ty, _ = self._waypoints[self._wp_index]
        px, py = self._perc.position
        dx = tx - px
        dy = ty - py
        return math.sqrt(dx*dx + dy*dy)

    def angle_to_target(self):
        """Return angle from current heading to target (radians)."""
        if not self._waypoints or self._wp_index >= len(self._waypoints):
            return 0.0
        tx, ty, _ = self._waypoints[self._wp_index]
        px, py = self._perc.position
        dx = tx - px
        dy = ty - py
        h_rad = math.radians(self._perc.heading)
        return math.atan2(dy, dx) - h_rad


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))
