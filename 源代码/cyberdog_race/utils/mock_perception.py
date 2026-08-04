"""
Mock perception for testing without ROS2 / real robot hardware.
Simulates LiDAR, camera, IMU, and odometry data.
"""
import math
import time
import numpy as np
import threading


class MockPerception:
    """Simulated perception for standalone testing without ROS2."""

    def __init__(self):
        self.scan_data = MockScanData()
        self.imu_data = MockImuData()
        self.latest_rgb = None
        self.latest_depth = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._lock = threading.Lock()

    @property
    def odom_pose(self):
        with self._lock:
            return (self._x, self._y, self._yaw)

    @property
    def position(self):
        """Return (x, y) in world frame meters."""
        with self._lock:
            return (self._x, self._y)

    @property
    def heading(self):
        """Return heading in degrees [0, 360). 0 = +X axis."""
        with self._lock:
            deg = math.degrees(self._yaw)
            deg = deg % 360.0
            if deg < 0:
                deg += 360.0
            return deg

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._x = x
            self._y = y
            self._yaw = yaw

    def set_velocity(self, vx, vy):
        with self._lock:
            self._vx = vx
            self._vy = vy

    def step(self, dt=0.05):
        """Advance odometry by dt seconds."""
        with self._lock:
            self._x += self._vx * dt
            self._y += self._vy * dt

    def get_lidar_ranges(self):
        return self.scan_data.ranges

    def get_lidar_angles(self):
        return self.scan_data.angles

    def get_front_distance(self, fov_rad=0.5):
        return self.scan_data.front_distance(fov_rad)

    def get_min_distance(self):
        return self.scan_data.min_distance()

    def get_side_distances(self, side='left'):
        return self.scan_data.side_distance(side)

    def get_cartesian_ranges(self):
        return self.scan_data.cartesian()

    def get_imu_yaw_rate(self):
        return self.imu_data.yaw_rate

    def get_imu_linear_acc(self):
        return self.imu_data.linear_acc

    def is_fallen(self, acc_threshold=3.0):
        return self.imu_data.is_fallen(acc_threshold)

    def body_to_world(self, bx, by, bz=0.0):
        import math
        rx, ry, yaw = self.odom_pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        wx = rx + bx * cos_yaw - by * sin_yaw
        wy = ry + bx * sin_yaw + by * cos_yaw
        return (wx, wy, bz)

    def depth_to_3d(self, u, v, depth):
        return (0.0, 0.0, float(depth))

    def depth_to_world(self, u, v, depth):
        return (0.0, 0.0, float(depth))


class MockScanData:
    """Simulated LaserScan data."""

    def __init__(self, n_points=360):
        self.n_points = n_points
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (self.angle_max - self.angle_min) / n_points
        self.range_min = 0.05
        self.range_max = 12.0
        # Simulate a corridor: walls on both sides
        self._set_corridor()

    def _set_corridor(self, left_dist=0.25, right_dist=0.25,
                      front_dist=2.0, back_dist=0.5):
        n = self.n_points
        self.ranges = np.full(n, float('inf'))
        angles = np.linspace(-math.pi, math.pi, n)

        for i, angle in enumerate(angles):
            abs_angle = abs(angle)
            if abs_angle < 0.3:  # front
                self.ranges[i] = front_dist + np.random.uniform(-0.05, 0.05)
            elif abs_angle > math.pi - 0.3:  # back
                self.ranges[i] = back_dist + np.random.uniform(-0.05, 0.05)
            elif 0.3 <= angle <= math.pi * 0.7:  # left
                self.ranges[i] = left_dist + np.random.uniform(-0.02, 0.02)
            elif -math.pi * 0.7 <= angle <= -0.3:  # right
                self.ranges[i] = right_dist + np.random.uniform(-0.02, 0.02)

        self.ranges += np.random.uniform(-0.01, 0.01, n)

    @property
    def ranges(self):
        return self._ranges

    @ranges.setter
    def ranges(self, value):
        self._ranges = value

    def front_distance(self, fov_rad=0.5):
        n = self.n_points
        angles = np.linspace(-math.pi, math.pi, n)
        front_mask = np.abs(angles) < fov_rad
        vals = self._ranges[front_mask]
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            return float('inf')
        return float(np.min(vals))

    def min_distance(self):
        vals = self._ranges[np.isfinite(self._ranges)]
        if len(vals) == 0:
            return float('inf')
        return float(np.min(vals))

    def side_distance(self, side='left'):
        n = self.n_points
        angles = np.linspace(-math.pi, math.pi, n)
        if side == 'left':
            mask = (angles > math.pi * 0.25) & (angles < math.pi * 0.75)
        else:
            mask = (angles < -math.pi * 0.25) & (angles > -math.pi * 0.75)
        vals = self._ranges[mask]
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            return float('inf')
        return float(np.mean(vals))

    def cartesian(self):
        n = self.n_points
        angles = np.linspace(-math.pi, math.pi, n)
        valid = np.isfinite(self._ranges)
        x = self._ranges[valid] * np.cos(angles[valid])
        y = self._ranges[valid] * np.sin(angles[valid])
        return x, y


class MockImuData:
    """Simulated IMU data."""

    def __init__(self):
        self.yaw_rate = 0.0
        self.linear_acc = (0.0, 0.0, 9.81)

    def set_moving(self, ax=0.0, ay=0.0, az=9.81, yaw_rate=0.0):
        self.linear_acc = (ax, ay, az)
        self.yaw_rate = yaw_rate

    def is_fallen(self, acc_threshold=3.0):
        ax, ay, az = self.linear_acc
        total = math.sqrt(ax**2 + ay**2 + az**2)
        return total < acc_threshold
