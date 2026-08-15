"""
ROS2 Perception Layer for CyberDog Race.
Provides unified access to camera, LiDAR, IMU, and odometry.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import tf2_ros
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import numpy as np
import cv_bridge
import math
import time

try:
    import gazebo_msgs.msg as gazebo_msgs
    GAZEBO_MSGS_AVAILABLE = True
except ImportError:
    GAZEBO_MSGS_AVAILABLE = False

try:
    import gazebo_msgs.srv as gazebo_srvs
    GAZEBO_SRVS_AVAILABLE = True
except ImportError:
    GAZEBO_SRVS_AVAILABLE = False


# Default camera intrinsics (from task brief, verify in simulation)
DEFAULT_FX = 615.0
DEFAULT_FY = 615.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0


class ROS2Perception(Node):
    """Unified perception node for all sensor data.

    Publishes to the following internal callbacks:
    - _on_scan       -> self.scan_data
    - _on_imu        -> self.imu_data
    - _on_rgb        -> self.latest_rgb
    - _on_depth      -> self.latest_depth
    - _on_tf         -> self.odom_pose (x, y, yaw)
    - _on_model_states -> self._gazebo_pose_override (absolute position from Gazebo)

    Priority: Gazebo model_states > /tf or /odom (Gazebo updates on Reset World)
    """

    def __init__(self, node_name='cyberdog_perception'):
        super().__init__(node_name)

        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Force Gazebo model name for /gazebo/model_states truth.
        # This avoids auto-detect picking the wrong model when multiple exist.
        self._model_name = 'robot'
        self._model_name_locked = True

        # ----- Camera intrinsics -----
        self.fx = DEFAULT_FX
        self.fy = DEFAULT_FY
        self.cx = DEFAULT_CX
        self.cy = DEFAULT_CY

        self._bridge = cv_bridge.CvBridge()
        self._all_subs = []
        self._qos_profiles = (sensor_qos, best_effort_qos, reliable_qos)

        # ----- Subscriptions -----
        # Camera topics: use sensor_data QoS (BEST_EFFORT compatible)
        # Gazebo may publish RELIABLE or BEST_EFFORT, sensor_data handles both
        self._subscribe_one(sensor_msgs.LaserScan, '/scan', self._on_scan, sensor_qos)
        self._subscribe_one(sensor_msgs.Imu, '/imu', self._on_imu, sensor_qos)
        self._subscribe_one(sensor_msgs.Image, '/D435/rgb/image_raw', self._on_rgb, sensor_qos)
        self._subscribe_one(sensor_msgs.Image, '/D435/image_raw', self._on_rgb, sensor_qos)
        self._subscribe_one(sensor_msgs.Image, '/D435/depth/image_raw', self._on_depth, sensor_qos)
        self._subscribe_one(sensor_msgs.Image, '/camera/rgb/image_raw', self._on_rgb, sensor_qos)
        self._subscribe_one(sensor_msgs.Image, '/camera/image_raw', self._on_rgb, sensor_qos)
        self._subscribe_one(sensor_msgs.Image, '/camera/depth/image_raw', self._on_depth, sensor_qos)
        self._subscribe_one(sensor_msgs.Image, '/camera/color/image_raw', self._on_rgb, sensor_qos)
        self._subscribe_one(TFMessage, '/tf', self._on_tf, sensor_qos)
        self._subscribe_one(Odometry, '/odom', self._on_odom, sensor_qos)
        self._subscribe_one(geometry_msgs.TransformStamped, '/tf_transform', self._on_transform, sensor_qos)

        # Gazebo model states (absolute position, updates on Reset World)
        # NOTE: default to 'robot' so we read the correct model pose.
        if GAZEBO_MSGS_AVAILABLE:
            self._subscribe_many(gazebo_msgs.ModelStates, '/gazebo/model_states', self._on_model_states)
            self._subscribe_many(gazebo_msgs.ModelStates, '/model_states', self._on_model_states)
        self._gazebo_entity_clients = []
        if GAZEBO_SRVS_AVAILABLE and hasattr(gazebo_srvs, 'GetEntityState'):
            for service_name in (
                '/gazebo/get_entity_state',
                '/get_entity_state',
                '/world/default/get_entity_state',
            ):
                try:
                    client = self.create_client(gazebo_srvs.GetEntityState, service_name)
                    self._gazebo_entity_clients.append((service_name, client))
                except Exception:
                    pass
        self._gazebo_model_clients = []
        if GAZEBO_SRVS_AVAILABLE and hasattr(gazebo_srvs, 'GetModelState'):
            for service_name in (
                '/gazebo/get_model_state',
                '/get_model_state',
            ):
                try:
                    client = self.create_client(gazebo_srvs.GetModelState, service_name)
                    self._gazebo_model_clients.append((service_name, client))
                except Exception:
                    pass
        self._known_topic_names = set()

        # ----- Data holders -----
        self.scan_data = None          # sensor_msgs.LaserScan
        self.imu_data = None           # sensor_msgs.Imu
        self.latest_rgb = None         # np.ndarray (H, W, 3) BGR
        self.latest_depth = None       # np.ndarray (H, W) meters
        self.odom_pose = (0.0, 0.0, 0.0)  # (x, y, yaw) in world frame
        self._last_scan_time = 0.0
        self._last_imu_time = 0.0
        self._last_rgb_time = 0.0
        self._last_depth_time = 0.0
        self._last_odom_time = 0.0
        self._last_tf_time = 0.0
        self._last_gazebo_time = 0.0
        self._gazebo_pose_override = None  # Higher priority than odom
        self._gazebo_pose_full = None     # (x, y, z, roll, pitch, yaw) from Gazebo
        self._gazebo_model_poses = {}     # name -> (x, y, z, roll, pitch, yaw)
        self._odom_full = None           # (x, y, z, roll, pitch, yaw) from odometry
        self._position_offset = (0.0, 0.0, 0.0)  # (x, y, yaw) offset for coordinate calibration

        self.get_logger().info('Perception node started')

    def set_position_offset(self, offset_x, offset_y, offset_yaw=0.0):
        """Set coordinate offset to calibrate Gazebo world to course_map coordinates.
        
        If Gazebo spawns robot at (4.75, 0.16) but course_map expects (0, 0),
        call set_position_offset(-4.75, -0.16, 0.0) to correct.
        """
        self._position_offset = (float(offset_x), float(offset_y), float(offset_yaw))
        self.get_logger().info(f'Position offset set to: {self._position_offset}')

    def get_gazebo_model_pose(self, model_name, apply_offset=True):
        """Return Gazebo model pose as (x, y, z, roll, pitch, yaw), if available."""
        pose = self._gazebo_model_poses.get(model_name)
        if pose is None:
            pose = self._query_gazebo_entity_pose(model_name)
        if pose is None:
            return None
        if not apply_offset:
            return pose
        ox, oy, oyaw = self._position_offset
        x, y, z, roll, pitch, yaw = pose
        return (x + ox, y + oy, z, roll, pitch, yaw + oyaw)

    def _query_gazebo_entity_pose(self, model_name, timeout_s=0.08):
        """Query Gazebo entity state service when model_states topic is unavailable."""
        if not self._gazebo_entity_clients and not self._gazebo_model_clients:
            return None

        pose = self._query_gazebo_get_entity_state(model_name, timeout_s)
        if pose is not None:
            return pose
        return self._query_gazebo_get_model_state(model_name, timeout_s)

    def _query_gazebo_get_entity_state(self, model_name, timeout_s):
        for service_name, client in self._gazebo_entity_clients:
            if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.03):
                continue
            try:
                req = gazebo_srvs.GetEntityState.Request()
                req.name = model_name
                req.reference_frame = 'world'
                future = client.call_async(req)
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline and not future.done():
                    time.sleep(0.005)
                if not future.done():
                    continue
                resp = future.result()
                if resp is None or not getattr(resp, 'success', False):
                    continue
                pose = resp.state.pose
                q = pose.orientation
                roll, pitch, yaw = self._quat_to_rpy(q)
                result = (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    roll,
                    pitch,
                    yaw,
                )
                self._gazebo_model_poses[model_name] = result
                self._last_gazebo_time = time.monotonic()
                return result
            except Exception as exc:
                if self._last_gazebo_time == 0.0:
                    self.get_logger().warn(
                        f'Gazebo entity service query failed on {service_name}: {exc}')
        return None

    def _query_gazebo_get_model_state(self, model_name, timeout_s):
        for service_name, client in self._gazebo_model_clients:
            if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.03):
                continue
            try:
                req = gazebo_srvs.GetModelState.Request()
                req.model_name = model_name
                req.relative_entity_name = 'world'
                future = client.call_async(req)
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline and not future.done():
                    time.sleep(0.005)
                if not future.done():
                    continue
                resp = future.result()
                if resp is None or not getattr(resp, 'success', False):
                    continue
                pose = resp.pose
                q = pose.orientation
                roll, pitch, yaw = self._quat_to_rpy(q)
                result = (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    roll,
                    pitch,
                    yaw,
                )
                self._gazebo_model_poses[model_name] = result
                self._last_gazebo_time = time.monotonic()
                return result
            except Exception as exc:
                if self._last_gazebo_time == 0.0:
                    self.get_logger().warn(
                        f'Gazebo model service query failed on {service_name}: {exc}')
        return None

    @staticmethod
    def _quat_to_rpy(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    # ------------------------------------------------------------------
    # Position & Heading (for hardcoded waypoint navigation)
    # ------------------------------------------------------------------

    @property
    def position(self):
        """Return (x, y) in world frame meters.

        Prefers Gazebo absolute position over odom when available,
        so Reset World is properly reflected. Applies coordinate offset if set.
        """
        ox, oy, _ = self._position_offset
        if self._gazebo_pose_override is not None:
            return (self._gazebo_pose_override[0] + ox, self._gazebo_pose_override[1] + oy)
        return (self.odom_pose[0] + ox, self.odom_pose[1] + oy)

    @property
    def heading(self):
        """Return heading in degrees [0, 360). 0 = +X axis (forward in Gazebo).

        Priority: Gazebo pose > IMU orientation > odometry pose.
        IMU is preferred because it doesn't accumulate drift during turns.
        """
        _, _, oyaw = self._position_offset

        # Priority 1: Gazebo absolute pose (most reliable)
        if self._gazebo_pose_override is not None:
            yaw_rad = self._gazebo_pose_override[2] + oyaw
            return self._rad_to_deg(yaw_rad)

        # Priority 2: IMU orientation (no drift)
        if self.imu_data is not None and self._last_imu_time > 0.0:
            q = self.imu_data.orientation
            yaw_rad = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                q.w**2 + q.x**2 - q.y**2 - q.z**2
            )
            return self._rad_to_deg(yaw_rad + oyaw)

        # Priority 3: odometry (can drift)
        yaw_rad = self.odom_pose[2] + oyaw
        return self._rad_to_deg(yaw_rad)

    @property
    def body_pose(self):
        """Return full body pose (x, y, z, roll, pitch, yaw).

        Tries Gazebo first, falls back to odometry.
        """
        if self._gazebo_pose_full is not None:
            ox, oy, oz, oroll, opitch, oyaw = self._position_offset
            x = self._gazebo_pose_full[0] + ox
            y = self._gazebo_pose_full[1] + oy
            z = self._gazebo_pose_full[2]
            roll = self._gazebo_pose_full[3] + oroll
            pitch = self._gazebo_pose_full[4] + opitch
            yaw = self._gazebo_pose_full[5] + oyaw
            return (x, y, z, roll, pitch, yaw)
        
        if self._odom_full is not None:
            ox, oy, oz, oroll, opitch, oyaw = self._position_offset
            x = self._odom_full[0] + ox
            y = self._odom_full[1] + oy
            z = self._odom_full[2]
            roll = self._odom_full[3] + oroll
            pitch = self._odom_full[4] + opitch
            yaw = self._odom_full[5] + oyaw
            return (x, y, z, roll, pitch, yaw)
        
        return None

    @property
    def body_height(self):
        """Return body height (z) from Gazebo or odometry."""
        if self._gazebo_pose_full is not None:
            return self._gazebo_pose_full[2]
        if self._odom_full is not None:
            return self._odom_full[2]
        return None

    @property
    def body_pitch(self):
        """Return body pitch in degrees from Gazebo, odometry, or IMU."""
        # Try Gazebo first
        if self._gazebo_pose_full is not None:
            return math.degrees(self._gazebo_pose_full[4])
        
        # Try odometry
        if self._odom_full is not None:
            return math.degrees(self._odom_full[4])
        
        # Try IMU as fallback
        if self.imu_data is not None and self._last_imu_time > 0.0:
            q = self.imu_data.orientation
            # Extract pitch from IMU quaternion
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)
            return math.degrees(pitch)
        
        return None

    @staticmethod
    def _rad_to_deg(yaw_rad):
        """Convert radians to degrees in [0, 360) range."""
        deg = math.degrees(yaw_rad) % 360.0
        if deg < 0:
            deg += 360.0
        return deg

    @property
    def pose(self):
        """Return (x, y, yaw) in world frame. Prefers Gazebo over odom. Applies offset."""
        ox, oy, oyaw = self._position_offset
        if self._gazebo_pose_override is not None:
            return (self._gazebo_pose_override[0] + ox,
                    self._gazebo_pose_override[1] + oy,
                    self._gazebo_pose_override[2] + oyaw)
        return (self.odom_pose[0] + ox,
                self.odom_pose[1] + oy,
                self.odom_pose[2] + oyaw)

    def _subscribe_many(self, msg_type, topic_name, callback):
        for qos in self._qos_profiles:
            self._all_subs.append(
                self.create_subscription(msg_type, topic_name, callback, qos))

    def _subscribe_one(self, msg_type, topic_name, callback, qos):
        """Subscribe with a single specific QoS profile."""
        self._all_subs.append(
            self.create_subscription(msg_type, topic_name, callback, qos))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_scan(self, msg: sensor_msgs.LaserScan):
        self.scan_data = msg
        self._last_scan_time = time.monotonic()

    def _on_imu(self, msg: sensor_msgs.Imu):
        self.imu_data = msg
        self._last_imu_time = time.monotonic()

    def _on_rgb(self, msg: sensor_msgs.Image):
        try:
            try:
                self.latest_rgb = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception:
                image = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
                if len(image.shape) == 3 and image.shape[2] >= 3:
                    self.latest_rgb = image[:, :, :3]
                else:
                    raise
            self._last_rgb_time = time.monotonic()
        except Exception as e:
            if self._last_rgb_time == 0.0:
                self.get_logger().warn(f'RGB conversion failed once: {e}')

    def _on_depth(self, msg: sensor_msgs.Image):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
            if hasattr(depth, 'dtype') and depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            self.latest_depth = depth
            self._last_depth_time = time.monotonic()
        except Exception as e:
            if self._last_depth_time == 0.0:
                self.get_logger().warn(f'Depth conversion failed once: {e}')

    def _on_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            child = transform.child_frame_id.lower()
            parent = transform.header.frame_id.lower()
            is_robot_tf = (
                child in ('base_link', 'base', 'body', 'trunk') or
                ('base' in child and parent in ('odom', 'world', 'map'))
            )
            if is_robot_tf:
                self._on_transform(transform)
                return

    def _on_transform(self, msg: geometry_msgs.TransformStamped):
        q = msg.transform.rotation
        # Quaternion to yaw (ZYX Euler convention, standard for ROS/Gazebo)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            q.w**2 + q.x**2 - q.y**2 - q.z**2
        )
        self.odom_pose = (
            msg.transform.translation.x,
            msg.transform.translation.y,
            yaw,
        )
        now = time.monotonic()
        self._last_tf_time = now
        self._last_odom_time = now

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        
        # Extract full orientation (roll, pitch, yaw from quaternion)
        # Roll
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        self.odom_pose = (x, y, yaw)
        
        # Store full odometry pose for height/pitch
        self._odom_full = (x, y, z, roll, pitch, yaw)
        self._last_odom_time = time.monotonic()

    def _on_model_states(self, msg: gazebo_msgs.ModelStates):
        """Handle Gazebo model states - provides absolute position, updates on Reset World."""
        model_poses = {}
        for name, pose_i in zip(msg.name, msg.pose):
            q_i = pose_i.orientation
            sinr_cosp_i = 2.0 * (q_i.w * q_i.x + q_i.y * q_i.z)
            cosr_cosp_i = 1.0 - 2.0 * (q_i.x * q_i.x + q_i.y * q_i.y)
            roll_i = math.atan2(sinr_cosp_i, cosr_cosp_i)
            sinp_i = 2.0 * (q_i.w * q_i.y - q_i.z * q_i.x)
            if abs(sinp_i) >= 1:
                pitch_i = math.copysign(math.pi / 2, sinp_i)
            else:
                pitch_i = math.asin(sinp_i)
            siny_cosp_i = 2.0 * (q_i.w * q_i.z + q_i.x * q_i.y)
            cosy_cosp_i = 1.0 - 2.0 * (q_i.y * q_i.y + q_i.z * q_i.z)
            yaw_i = math.atan2(siny_cosp_i, cosy_cosp_i)
            model_poses[name] = (
                pose_i.position.x,
                pose_i.position.y,
                pose_i.position.z,
                roll_i,
                pitch_i,
                yaw_i,
            )
        self._gazebo_model_poses = model_poses

        # Auto-detect robot model name if not yet detected
        if not getattr(self, '_model_name_locked', False) and self._model_name is None and len(msg.name) > 0:
            robot_names = ('cyberdog', 'robot', 'dog', 'body', 'trunk', 'base_link', 'quadruped')
            
            for name in msg.name:
                if any(r in name.lower() for r in robot_names):
                    self._model_name = name
                    self.get_logger().info(f'Gazebo model detected: {name}')
                    break
            
            # If no match, use the first model that's not a ground plane or world
            if self._model_name is None:
                for name in msg.name:
                    name_lower = name.lower()
                    if not any(x in name_lower for x in ('ground', 'world', 'plane', 'wall', 'obstacle')):
                        if len(name) > 0:
                            self._model_name = name
                            self.get_logger().warn(f'No robot model match, using: {name}')
                            break

        if self._model_name is None:
            return

        try:
            idx = msg.name.index(self._model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        q = pose.orientation
        
        # Extract full orientation (roll, pitch, yaw from quaternion)
        # Roll
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self._gazebo_pose_override = (
            pose.position.x,
            pose.position.y,
            yaw,
        )
        self._gazebo_pose_full = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            roll,
            pitch,
            yaw,
        )
        self._last_gazebo_time = time.monotonic()
        self._last_odom_time = time.monotonic()

    # ------------------------------------------------------------------
    # Sensor readiness helpers
    # ------------------------------------------------------------------

    @property
    def odom_available(self):
        return self._last_odom_time > 0.0

    @property
    def last_sensor_time(self):
        return max(
            self._last_scan_time,
            self._last_imu_time,
            self._last_rgb_time,
            self._last_depth_time,
            self._last_odom_time,
        )

    def sensor_status(self):
        return {
            'scan': self._last_scan_time > 0.0,
            'rgb': self._last_rgb_time > 0.0,
            'depth': self._last_depth_time > 0.0,
            'imu': self._last_imu_time > 0.0,
            'odom': self.odom_available,
            'gazebo': self._last_gazebo_time > 0.0,
            'gazebo_model': self._model_name,
            'gazebo_models': sorted(self._gazebo_model_poses.keys())[:12],
            'gazebo_msgs_available': GAZEBO_MSGS_AVAILABLE,
            'gazebo_srvs_available': GAZEBO_SRVS_AVAILABLE,
            'gazebo_entity_services': [name for name, _ in self._gazebo_entity_clients],
            'gazebo_model_services': [name for name, _ in self._gazebo_model_clients],
            'gazebo_entity_services_ready': [
                name for name, client in self._gazebo_entity_clients
                if client.service_is_ready()
            ],
            'gazebo_model_services_ready': [
                name for name, client in self._gazebo_model_clients
                if client.service_is_ready()
            ],
        }

    def is_ready(self, require_rgb=False, require_depth=False):
        status = self.sensor_status()
        required = status['scan'] and status['odom']
        if require_rgb:
            required = required and status['rgb']
        if require_depth:
            required = required and status['depth']
        return required

    def wait_until_ready(self, timeout_s=10.0, require_rgb=False, require_depth=False):
        deadline = time.monotonic() + timeout_s
        next_log = 0.0
        while time.monotonic() < deadline:
            self.discover_sensor_topics()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.is_ready(require_rgb=require_rgb, require_depth=require_depth):
                return True
            now = time.monotonic()
            if now >= next_log:
                self.get_logger().info(
                    f'Waiting sensors: {self.sensor_status()}')
                next_log = now + 1.0
        return False

    def discover_sensor_topics(self):
        """Subscribe to likely sensor topics exposed by the simulator."""
        try:
            topics = self.get_topic_names_and_types()
        except Exception:
            return

        for name, type_names in topics:
            if name in self._known_topic_names:
                continue
            type_set = set(type_names)
            type_set_lower = {t.lower() for t in type_names}
            lowered = name.lower()

            if 'sensor_msgs/msg/laserscan' in type_set:
                self._subscribe_many(sensor_msgs.LaserScan, name, self._on_scan)
                self._known_topic_names.add(name)
                self.get_logger().info(f'Auto-subscribed LaserScan: {name}')

            elif GAZEBO_MSGS_AVAILABLE and 'gazebo_msgs/msg/modelstates' in type_set_lower:
                self._subscribe_many(gazebo_msgs.ModelStates, name, self._on_model_states)
                self._known_topic_names.add(name)
                self.get_logger().info(f'Auto-subscribed Gazebo model states: {name}')

            elif 'sensor_msgs/msg/image' in type_set:
                is_depth = 'depth' in lowered or 'aligned' in lowered
                is_rgb = (
                    'rgb' in lowered or 'color' in lowered or
                    'image_raw' in lowered or 'camera' in lowered
                )
                if is_depth:
                    self._subscribe_many(sensor_msgs.Image, name, self._on_depth)
                    self._known_topic_names.add(name)
                    self.get_logger().info(f'Auto-subscribed depth image: {name}')
                elif is_rgb:
                    self._subscribe_many(sensor_msgs.Image, name, self._on_rgb)
                    self._known_topic_names.add(name)
                    self.get_logger().info(f'Auto-subscribed RGB image: {name}')

    # ------------------------------------------------------------------
    # Depth-to-3D helpers
    # ------------------------------------------------------------------

    def depth_to_3d(self, u, v, depth):
        """Convert depth pixel to camera-frame 3D point (meters)."""
        z = float(depth)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (x, y, z)

    def depth_to_world(self, u, v, depth):
        """Convert depth pixel to world-frame 3D point using odometry."""
        x_cam, y_cam, z_cam = self.depth_to_3d(u, v, depth)
        rx, ry, yaw = self.odom_pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        x_body = z_cam
        y_body = -x_cam
        z_body = -y_cam

        x_world = rx + x_body * cos_yaw - y_body * sin_yaw
        y_world = ry + x_body * sin_yaw + y_body * cos_yaw
        z_world = z_body

        return (x_world, y_world, z_world)

    # ------------------------------------------------------------------
    # LiDAR helpers
    # ------------------------------------------------------------------

    def get_lidar_ranges(self):
        """Return numpy array of range readings (meters)."""
        if self.scan_data is None:
            return np.array([])
        return np.array(self.scan_data.ranges)

    def get_lidar_angles(self):
        """Return numpy array of angle for each range (radians)."""
        if self.scan_data is None:
            return np.array([])
        n = len(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        return angle_min + np.arange(n) * angle_increment

    def get_cartesian_ranges(self):
        """Return (x, y) arrays of LiDAR points in robot frame."""
        if self.scan_data is None:
            return np.array([]), np.array([])
        r = self.get_lidar_ranges()
        theta = self.get_lidar_angles()
        valid = np.isfinite(r)
        x = r[valid] * np.cos(theta[valid])
        y = r[valid] * np.sin(theta[valid])
        return x, y

    def get_front_distance(self, fov_rad=0.5):
        """Return distance to nearest obstacle directly ahead (meters)."""
        if self.scan_data is None:
            return float('inf')
        r = self.get_lidar_ranges()
        theta = self.get_lidar_angles()
        front_mask = np.abs(theta) < fov_rad
        front_ranges = r[front_mask]
        valid = front_ranges[np.isfinite(front_ranges)]
        if len(valid) == 0:
            return float('inf')
        return float(np.min(valid))

    def get_min_distance(self):
        """Return minimum range reading across all angles."""
        if self.scan_data is None:
            return float('inf')
        r = self.get_lidar_ranges()
        valid = r[np.isfinite(r)]
        if len(valid) == 0:
            return float('inf')
        return float(np.min(valid))

    def get_side_distances(self, side='left'):
        """Return average side distance. side: 'left' or 'right'."""
        if self.scan_data is None:
            return float('inf')
        r = self.get_lidar_ranges()
        theta = self.get_lidar_angles()
        if side == 'left':
            mask = (theta > math.pi * 0.25) & (theta < math.pi * 0.75)
        else:
            mask = (theta < -math.pi * 0.25) & (theta > -math.pi * 0.75)
        side_ranges = r[mask]
        valid = side_ranges[np.isfinite(side_ranges)]
        if len(valid) == 0:
            return float('inf')
        return float(np.mean(valid))

    # ------------------------------------------------------------------
    # IMU helpers
    # ------------------------------------------------------------------

    def get_imu_yaw_rate(self):
        """Return angular velocity around Z axis (rad/s)."""
        if self.imu_data is None:
            return 0.0
        return self.imu_data.angular_velocity.z

    def get_imu_linear_acc(self):
        """Return linear acceleration (ax, ay, az) in m/s^2."""
        if self.imu_data is None:
            return (0.0, 0.0, 0.0)
        a = self.imu_data.linear_acceleration
        return (a.x, a.y, a.z)

    def is_fallen(self, acc_threshold=3.0):
        """Heuristic: robot is likely fallen if total accel is low."""
        ax, ay, az = self.get_imu_linear_acc()
        total = math.sqrt(ax**2 + ay**2 + az**2)
        return total < acc_threshold

    # ------------------------------------------------------------------
    # Coordinate transform (body -> world)
    # ------------------------------------------------------------------

    def body_to_world(self, bx, by, bz=0.0):
        """Transform a point from body frame to world frame."""
        rx, ry, yaw = self.odom_pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        wx = rx + bx * cos_yaw - by * sin_yaw
        wy = ry + bx * sin_yaw + by * cos_yaw
        wz = bz
        return (wx, wy, wz)
