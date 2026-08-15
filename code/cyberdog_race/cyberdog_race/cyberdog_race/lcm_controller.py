"""
CyberDog Race: LCM Controller Wrapper
Thread-safe LCM communication with the motion controller.
Implements the Robot_Ctrl pattern from basic_motion/main.py.
"""
import lcm
import time
import threading
from threading import Lock
from .lcm_types import (
    robot_control_cmd_lcmt,
    robot_control_response_lcmt,
    MODE_PURE_DAMPER, MODE_LOCOMOTION, MODE_RECOVERY_STAND,
    MODE_POSITION_INTERP, MODE_ACTION_TRIGGER, MODE_TWO_LEG_STAND,
    GAIT_TROT_SLOW, GAIT_BOUND, GAIT_TROT_FAST, GAIT_SELF_FREQ,
    GAIT_USER_DEFINED, GAIT_LOW_CRAWL, CONTACT_ALL,
)

# Built-in action table: JUMP3D actions
MODE_JUMP3D = 16
from .lcm_types.file_send_lcmt import file_send_lcmt


LCM_SEND_URL = "udpm://239.255.76.67:7671?ttl=255"
LCM_RECV_URL = "udpm://239.255.76.67:7670?ttl=255"
CHAN_CMD = "robot_control_cmd"
CHAN_RESP = "robot_control_response"
CHAN_USER_GAIT = "user_gait_file"
CHAN_GAIT_EXEC = "user_gait_exec"
PROCESS_BAR_DONE = 95


class LCMController:
    """Thread-safe CyberDog LCM controller.

    Manages two LCM instances (send / receive) with a background receive
    thread and a heartbeat send thread.  All access to the shared command
    message is protected by a lock.
    """

    def __init__(self):
        self._lc_recv = lcm.LCM(LCM_RECV_URL)
        self._lc_send = lcm.LCM(LCM_SEND_URL)
        self._cmd = robot_control_cmd_lcmt()
        self._resp = robot_control_response_lcmt()

        self._send_lock = Lock()
        self._delay_cnt = 0
        self._running = True

        # Shared response state (written by receive thread, read by caller)
        self._resp_mode = 0
        self._resp_gait = 0
        self._resp_bar = 0
        self._resp_lock = Lock()

        self._recv_thread = threading.Thread(target=self._recv_loop,
                                             daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop,
                                             daemon=True)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Start receive and send background threads."""
        self._running = True
        self._lc_recv.subscribe(CHAN_RESP, self._on_response)
        self._recv_thread.start()
        self._send_thread.start()

    def stop(self):
        """Stop background threads."""
        self._running = False
        self._recv_thread.join(timeout=2.0)
        self._send_thread.join(timeout=2.0)

    def shutdown(self):
        """Send PureDamper and stop."""
        self.pure_damper()
        time.sleep(0.5)
        self.stop()

    # ------------------------------------------------------------------
    # LCM callback
    # ------------------------------------------------------------------

    def _on_response(self, _channel, data):
        self._resp = robot_control_response_lcmt().decode(data)
        with self._resp_lock:
            self._resp_mode = self._resp.mode
            self._resp_gait = self._resp.gait_id
            self._resp_bar = self._resp.order_process_bar

    # ------------------------------------------------------------------
    # Background threads
    # ------------------------------------------------------------------

    def _recv_loop(self):
        while self._running:
            self._lc_recv.handle()
            time.sleep(0.002)

    def _send_loop(self):
        while self._running:
            self._send_lock.acquire()
            if self._delay_cnt >= 10:  # Changed from > 20 to >= 10 for faster sending
                self._lc_send.publish(CHAN_CMD, self._cmd.encode())
                self._delay_cnt = 0
            self._delay_cnt += 1
            self._send_lock.release()
            time.sleep(0.005)  # 5ms loop = 200Hz, sends at ~20Hz when delay_cnt >= 10

    # ------------------------------------------------------------------
    # Low-level send
    # ------------------------------------------------------------------

    def _send(self, mode=None, gait_id=None, contact=None,
              vel_des=None, rpy_des=None, pos_des=None,
              acc_des=None, ctrl_point=None, foot_pose=None,
              step_height=None, value=None, duration=None):
        """Pack and send a command (must already set life_count += 1)."""
        self._send_lock.acquire()
        self._delay_cnt = 15  # Reduced for faster sending (~75ms instead of 100ms)
        if mode is not None:
            self._cmd.mode = mode
        if gait_id is not None:
            self._cmd.gait_id = gait_id
        if contact is not None:
            self._cmd.contact = contact
        if vel_des is not None:
            for i in range(3):
                self._cmd.vel_des[i] = vel_des[i]
        if rpy_des is not None:
            for i in range(3):
                self._cmd.rpy_des[i] = rpy_des[i]
        if pos_des is not None:
            for i in range(3):
                self._cmd.pos_des[i] = pos_des[i]
        if acc_des is not None:
            for i in range(6):
                self._cmd.acc_des[i] = acc_des[i]
        if ctrl_point is not None:
            for i in range(3):
                self._cmd.ctrl_point[i] = ctrl_point[i]
        if foot_pose is not None:
            for i in range(6):
                self._cmd.foot_pose[i] = foot_pose[i]
        if step_height is not None:
            for i in range(2):
                v = step_height[i] if step_height[i] is not None else 0.06
                self._cmd.step_height[i] = v
        if value is not None:
            self._cmd.value = value
        if duration is not None:
            self._cmd.duration = duration
        self._send_lock.release()

    def _increment_life(self):
        self._cmd.life_count += 1

    # ------------------------------------------------------------------
    # Response helpers
    # ------------------------------------------------------------------

    def get_response(self):
        """Return (mode, gait_id, order_process_bar) from last received resp."""
        with self._resp_lock:
            return self._resp_mode, self._resp_gait, self._resp_bar

    def wait_finish(self, mode, gait_id, timeout_s=10.0):
        """Poll until order_process_bar >= 95, matching mode/gait_id."""
        count = 0
        max_count = int(timeout_s / 0.005)
        while self._running and count < max_count:
            with self._resp_lock:
                if (self._resp_mode == mode and
                        self._resp_gait == gait_id and
                        self._resp_bar >= PROCESS_BAR_DONE):
                    return True
            count += 1
            time.sleep(0.005)
        return False

    # ------------------------------------------------------------------
    # High-level commands
    # ------------------------------------------------------------------

    def recovery_stand(self):
        """Enter recovery-stand mode (mode=12)."""
        self._increment_life()
        self._send(mode=MODE_RECOVERY_STAND, gait_id=0, duration=0)
        return self.wait_finish(MODE_RECOVERY_STAND, 0, timeout_s=10.0)

    def pure_damper(self):
        """Enter pure-damper / lie-down mode (mode=7)."""
        self._increment_life()
        self._send(mode=MODE_PURE_DAMPER, gait_id=0)
        return self.wait_finish(MODE_PURE_DAMPER, 0, timeout_s=5.0)

    def set_height(self, height_m=0.22, duration_ms=500):
        """Adjust body centroid height via position interpolation."""
        self._increment_life()
        self._send(
            mode=MODE_POSITION_INTERP,
            gait_id=5,
            contact=CONTACT_ALL,
            pos_des=[0.0, 0.0, height_m],
            rpy_des=[0.0, 0.0, 0.0],
            duration=duration_ms,
        )
        time.sleep(duration_ms / 1000.0 + 0.1)

    def set_body_pose(self, height_m=0.22, pitch_rad=0.0, duration_ms=300):
        """Adjust body height and pitch in one position-interp command."""
        self._increment_life()
        self._send(
            mode=MODE_POSITION_INTERP,
            gait_id=5,
            contact=CONTACT_ALL,
            pos_des=[0.0, 0.0, height_m],
            rpy_des=[0.0, pitch_rad, 0.0],
            duration=duration_ms,
        )
        time.sleep(duration_ms / 1000.0 + 0.05)

    def look_up(self, pitch_rad=0.3, duration_ms=500):
        """Pitch body to look up."""
        self._increment_life()
        self._send(
            mode=MODE_POSITION_INTERP,
            gait_id=0,
            rpy_des=[0.0, pitch_rad, 0.0],
            duration=duration_ms,
        )
        time.sleep(duration_ms / 1000.0 + 0.05)

    def look_forward(self, pitch_rad=-0.3, duration_ms=300):
        """Pitch body to look forward."""
        self._increment_life()
        self._send(
            mode=MODE_POSITION_INTERP,
            gait_id=0,
            rpy_des=[0.0, pitch_rad, 0.0],
            duration=duration_ms,
        )
        time.sleep(duration_ms / 1000.0 + 0.05)

    def two_leg_stand(self):
        """Two-leg stand (mode=64)."""
        self._increment_life()
        self._send(mode=MODE_TWO_LEG_STAND, gait_id=0)
        return self.wait_finish(MODE_TWO_LEG_STAND, 0, timeout_s=5.0)

    def locomotion(self, gait_id=GAIT_TROT_SLOW,
                   vx=0.0, vy=0.0, wz=0.0,
                   step_h_max=0.06, step_h_min=0.06,
                   body_height=None, body_pitch=None, body_roll=None,
                   duration_ms=0, value=None):
        """Send a locomotion command (continuous if duration_ms=0).

        Args:
            body_height: body centroid height override in meters (None = default ~0.22)
            body_pitch:  body pitch override in radians (negative = head down)
            body_roll:   body roll override in radians (positive = roll left)
        """
        self._increment_life()
        pos_des = None
        if body_height is not None:
            pos_des = [0.0, 0.0, body_height]

        rpy_des = None
        if body_roll is not None or body_pitch is not None:
            roll = 0.0 if body_roll is None else float(body_roll)
            pitch = 0.0 if body_pitch is None else float(body_pitch)
            rpy_des = [roll, pitch, 0.0]

        self._send(
            mode=MODE_LOCOMOTION,
            gait_id=gait_id,
            contact=CONTACT_ALL,
            vel_des=[vx, vy, wz],
            step_height=[step_h_max, step_h_min],
            pos_des=pos_des,
            rpy_des=rpy_des,
            value=value,
            duration=duration_ms,
        )

    def user_gait_locomotion(self, vx=0.10, vy=0.0, wz=0.0,
                             body_z=-0.035, pitch_rad=0.0, roll_rad=0.0,
                             step_height=(0.03, 0.03),
                             contact=1, value=1, duration_ms=85000):
        """Trigger an uploaded USER_DEFINED gait with full command fields."""
        self._increment_life()
        self._send(
            mode=MODE_LOCOMOTION,
            gait_id=GAIT_USER_DEFINED,
            contact=contact,
            vel_des=[vx, vy, wz],
            rpy_des=[roll_rad, pitch_rad, 0.0],
            pos_des=[0.0, 0.0, body_z],
            step_height=list(step_height),
            value=value,
            duration=duration_ms,
        )

    def stop_moving(self, body_height=0.22):
        """Send zero-velocity locomotion to halt in place.

        Args:
            body_height: body height to maintain while stopping (default 0.22m)
        """
        self.locomotion(
            vx=0.0, vy=0.0, wz=0.0,
            body_height=body_height,
            duration_ms=500)
        time.sleep(0.6)

    def action_trigger(self, gait_id):
        """Trigger a built-in action (mode=62, gait_id selects action)."""
        self._increment_life()
        self._send(mode=MODE_ACTION_TRIGGER, gait_id=gait_id)
        return self.wait_finish(MODE_ACTION_TRIGGER, gait_id, timeout_s=10.0)

    # ------------------------------------------------------------------
    # Convenience motion helpers
    # ------------------------------------------------------------------

    def walk_forward(self, speed=0.25, duration_s=None):
        """Walk forward at given speed (m/s)."""
        self.locomotion(
            gait_id=GAIT_TROT_SLOW,
            vx=speed, vy=0.0, wz=0.0,
            step_h_max=0.06, step_h_min=0.06,
            duration_ms=0,
        )
        if duration_s:
            time.sleep(duration_s)
            self.stop_moving()

    def walk_straight(self, vx, duration_s, gait_id=GAIT_TROT_SLOW):
        """Walk straight at velocity vx for duration_s seconds."""
        self.locomotion(
            gait_id=gait_id, vx=vx, vy=0.0, wz=0.0,
            duration_ms=0,
        )
        time.sleep(duration_s)
        self.stop_moving()

    def turn(self, wz, duration_s, gait_id=GAIT_SELF_FREQ):
        """Rotate in place at angular velocity wz (rad/s) for duration_s."""
        self.locomotion(gait_id=gait_id, vx=0.0, vy=0.0, wz=wz)
        time.sleep(duration_s)
        self.stop_moving()

    def strafe_left(self, speed=0.15, duration_s=None):
        """Strafe left."""
        self.locomotion(
            gait_id=GAIT_TROT_SLOW, vx=0.0, vy=-speed, wz=0.0,
            duration_ms=0,
        )
        if duration_s:
            time.sleep(duration_s)
            self.stop_moving()

    def strafe_right(self, speed=0.15, duration_s=None):
        """Strafe right."""
        self.locomotion(
            gait_id=GAIT_TROT_SLOW, vx=0.0, vy=speed, wz=0.0,
            duration_ms=0,
        )
        if duration_s:
            time.sleep(duration_s)
            self.stop_moving()

    def bound_jump(self, vx=0.20, duration_s=None):
        """Use bound/pronk gait for jumping."""
        self.locomotion(
            gait_id=GAIT_BOUND,
            vx=vx, vy=0.0, wz=0.0,
            step_h_max=0.15, step_h_min=0.05,
            duration_ms=0,
        )
        if duration_s:
            time.sleep(duration_s)
            self.stop_moving()

    def jump3d(self, gait_id=1, duration_ms=900, timeout_s=3.0):
        """Trigger built-in JUMP3D actions.

        gait_id=1: standing long jump
        gait_id=6: standing vertical jump
        """
        duration_ms = max(800, int(duration_ms))

        self._increment_life()
        self._send(
            mode=MODE_JUMP3D,
            gait_id=gait_id,
            contact=CONTACT_ALL,
            vel_des=[0.0, 0.0, 0.0],
            rpy_des=[0.0, 0.0, 0.0],
            pos_des=[0.0, 0.0, 0.20],
            duration=duration_ms,
        )

        finished = self.wait_finish(MODE_JUMP3D, gait_id, timeout_s=timeout_s)
        if not finished:
            time.sleep(duration_ms / 1000.0 + 0.3)
        return finished

    def stand_up_and_wait(self, height=0.22, wait_s=2.0, retries=2):
        """Recovery stand, set height, wait.

        In some sim runs the controller may not enter recovery stand on the
        first try (e.g. mode transition lag). We retry a few times and fail
        fast if standing cannot be achieved.
        """
        for attempt in range(1, max(1, int(retries)) + 1):
            ok = self.recovery_stand()
            mode, gait, bar = self.get_response()
            print(f"[LCM] stand_up attempt {attempt}: recovery_stand ok={ok} resp=(mode={mode},gait={gait},bar={bar})")
            time.sleep(wait_s)
            self.set_height(height, duration_ms=500)
            time.sleep(0.6)
            mode, gait, bar = self.get_response()
            # MODE_POSITION_INTERP is expected after set_height.
            if ok:
                return
            time.sleep(0.4)

        raise RuntimeError("stand_up_and_wait: recovery_stand failed after retries")

    def reset_pose(self):
        """Set upright neutral pose."""
        self.set_height(0.22, duration_ms=400)
        self._increment_life()
        self._send(
            mode=MODE_POSITION_INTERP,
            gait_id=5,
            rpy_des=[0.0, 0.0, 0.0],
            duration=300,
        )
        time.sleep(0.4)

    # ------------------------------------------------------------------
    # User-defined gait support (low-crawl for height-bar passages)
    # ------------------------------------------------------------------

    def send_gait_file(self, toml_string: str):
        """Upload a gait definition/parameter TOML file to the motion controller.

        Must be called before execute_gait_steps() for user-defined gaits.
        The controller will store the gait definition by gait_id.
        """
        msg = file_send_lcmt()
        msg.data = toml_string
        payload = msg.encode()
        print(f"[LCM] Sending user gait file: chars={len(toml_string)}, "
              f"bytes={len(payload)}, hash=0x{msg.get_hash():016x}")
        self._lc_send.publish(CHAN_USER_GAIT, payload)
        time.sleep(0.1)

    def execute_gait_steps(self, steps: list):
        """Execute a sequence of gait steps.

        Each step dict should contain:
            mode (int): control mode (11 = LOCOMOTION)
            gait_id (int): gait identifier
            contact (int): contact bitmask
            vel_des (list[4]): [vx, vy, wz, _]
            rpy_des (list[3]): [roll, pitch, yaw] in rad
            pos_des (list[3]): [x, y, z] body position in m
            ctrl_point (list[3]): foothold relative control point
            foot_pose (list[6]): foot relative positions
            step_height (list[4]): [fr_h, fl_h, rr_h, rl_h] in mm (encoded)
            duration (int): step duration in ms

        Args:
            steps: list of step parameter dicts
        """
        for step in steps:
            self._increment_life()
            self._send_lock.acquire()
            self._cmd.mode = step.get('mode', MODE_LOCOMOTION)
            self._cmd.gait_id = step.get('gait_id', GAIT_USER_DEFINED)
            self._cmd.contact = step.get('contact', CONTACT_ALL)
            self._delay_cnt = 15  # Reduced for faster gait step sending

            vd = step.get('vel_des', [0.0, 0.0, 0.0, 0.0])
            for i in range(3):
                self._cmd.vel_des[i] = vd[i]

            rpy = step.get('rpy_des', [0.0, 0.0, 0.0])
            for i in range(3):
                self._cmd.rpy_des[i] = rpy[i]

            pd = step.get('pos_des', [0.0, 0.0, 0.0])
            for i in range(3):
                self._cmd.pos_des[i] = pd[i]

            ad = step.get('acc_des', [0.0] * 6)
            for i in range(6):
                self._cmd.acc_des[i] = ad[i]

            cp = step.get('ctrl_point', [0.0] * 3)
            for i in range(3):
                self._cmd.ctrl_point[i] = cp[i]

            fp = step.get('foot_pose', [0.0] * 6)
            for i in range(6):
                self._cmd.foot_pose[i] = fp[i]

            sh = step.get('step_height', [0.0, 0.0])
            for i in range(2):
                self._cmd.step_height[i] = sh[i]

            self._cmd.value = step.get('value', 0)
            self._cmd.duration = step.get('duration', 300)
            self._lc_send.publish(CHAN_CMD, self._cmd.encode())
            self._send_lock.release()

            dur_ms = step.get('duration', 300)
            time.sleep(dur_ms / 1000.0 + 0.05)

    def change_gait(self, def_path: str, full_path: str, params_path: str):
        """Load and upload low-height gait TOML files via LCM.

        Args:
            def_path: Path to gait definition TOML file (low_Def.toml)
            full_path: Path to gait params full TOML file
            params_path: Path to gait params TOML file (low_Params.toml)
        """
        import toml
        import os

        steps = toml.load(params_path)
        full_steps = {'step': []}
        k = 0
        for i in steps['step']:
            cmd = {
                'mode': 11, 'gait_id': 110, 'contact': 0, 'life_count': 0,
                'vel_des': [0.0, 0.0, 0.0],
                'rpy_des': [0.0, 0.0, 0.0],
                'pos_des': [0.0, 0.0, 0.0],
                'acc_des': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'ctrl_point': [0.0, 0.0, 0.0],
                'foot_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'step_height': [0.0, 0.0],
                'value': 0, 'duration': 0
            }
            cmd['duration'] = i['duration']
            if i.get('type') == 'usergait':
                cmd['mode'] = 11
                cmd['gait_id'] = 110
                cmd['vel_des'] = i['body_vel_des']
                cmd['rpy_des'] = i['body_pos_des'][0:3]
                cmd['pos_des'] = i['body_pos_des'][3:6]
                cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + \
                                        math.ceil(i['step_height'][1] * 1e3) * 1e3
                cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + \
                                        math.ceil(i['step_height'][3] * 1e3) * 1e3
                cmd['acc_des'] = i['weight']
                cmd['value'] = i.get('use_mpc_traj', 0)
                cmd['contact'] = math.floor(i.get('landing_gain', 0.5) * 1e1)
                cmd['ctrl_point'][2] = i.get('mu', 0.3)
            if k == 0:
                full_steps['step'] = [cmd]
            else:
                full_steps['step'].append(cmd)
            k += 1

        with open(full_path, 'w') as f:
            f.write("# Gait Params\n")
            f.writelines(toml.dumps(full_steps))

        with open(def_path, 'r') as file_obj_gait_def, open(full_path, 'r') as file_obj_gait_params:
            self.send_gait_file(file_obj_gait_def.read())
            time.sleep(0.5)
            self.send_gait_file(file_obj_gait_params.read())
            time.sleep(0.1)
        print(f"[LCM] change_gait: uploaded gait files from {os.path.basename(def_path)}")

    def user_gait_execute(self, duration_ms: int = 840):
        """Execute user-defined gait with mode=62, gait_id=110.

        Args:
            duration_ms: Duration to sleep after sending command (default 840ms)
        """
        self._increment_life()
        self._send_lock.acquire()
        self._cmd.mode = 62
        self._cmd.gait_id = 110
        self._cmd.contact = 0
        self._cmd.life_count = self._cmd.life_count
        self._delay_cnt = 5
        self._lc_send.publish(CHAN_CMD, self._cmd.encode())
        self._send_lock.release()
        time.sleep(duration_ms / 1000.0)

    def user_gait_execute_reverse(self, duration_ms: int = 840):
        """Execute user-defined gait in reverse with mode=62, gait_id=110.

        Used for low_height_re gait (backward walking after height bar).
        """
        self.user_gait_execute(duration_ms)
