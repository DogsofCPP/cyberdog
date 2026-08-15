#!/usr/bin/env python3
"""
test3.py —— 第三关 PID 独立测试工具

用法:
    # 离线调参（不需要 ROS，在任意电脑上运行）
    python3 test3.py              # 默认：trackbar 调参模式
    python3 test3.py --help       # 查看参数说明
    python3 test3.py --step       # 阶跃响应测试
    python3 test3.py --sine       # 正弦波跟踪测试
    python3 test3.py --compare    # bang-bang vs PID 对比

    # 实机控制（需要在 Cyberdog 上运行，需要 ROS + LCM）
    python3 test3.py --live       # 起立后等待按 s，订阅 ground_topic 控制小狗

快捷键（所有模式通用）:
    r       - 重置 PID 状态
    q / ESC - 退出
    空格    - 暂停/继续
    s       - 实机模式开始/停止寻线
"""

import cv2
import numpy as np
import sys
import time
import math
from threading import Lock, Thread


DEFAULT_KP = 0.00150
DEFAULT_KI = 0.00003
DEFAULT_KD = 0.00010
DEFAULT_VY_KP = 0.000113
DEFAULT_OUT_MIN = -0.30
DEFAULT_OUT_MAX = 0.30
DEFAULT_INT_MAX = 0.05
DEFAULT_DT = 0.040
DEFAULT_VX = 0
GAP_DEADBAND = 4.0
VALID_MODES = ("--step", "--sine", "--compare", "--live")


def print_usage():
    print("用法: python3 test3.py [--step|--sine|--compare|--live|--help]")
    print("也可以: ros2 run cyberdog_demo test3 -- [--step|--sine|--compare|--live]")
    print("")
    print("默认模式: 打开 PID Tuner，通过滑块实时调 Kp/Ki/Kd 和模拟 gap。")
    print("--step:    自动注入阶跃 gap，观察 PID 阶跃响应。")
    print("--sine:    自动注入正弦 gap，观察输出是否平滑。")
    print("--compare: 对比 PID 和原 bang-bang 控制输出。")
    print("--live:    先起立，按 s 开始/停止寻线，退出时站立后趴下，会真实移动。")


def bump_life_count(cmd_msg):
    cmd_msg.life_count += 1
    if cmd_msg.life_count >= 50:
        cmd_msg.life_count = 1


def fill_robot_cmd(cmd_msg, mode, gait_id, vel_des=None,
                   step_height=None, duration=0):
    cmd_msg.mode = mode
    cmd_msg.gait_id = gait_id
    cmd_msg.vel_des = vel_des if vel_des is not None else [0.0, 0.0, 0.0]
    cmd_msg.step_height = step_height if step_height is not None else [0.06, 0.06]
    cmd_msg.duration = duration
    bump_life_count(cmd_msg)


def send_robot_cmd(ctrl, cmd_msg, mode, gait_id, vel_des=None,
                   step_height=None, duration=0):
    fill_robot_cmd(cmd_msg, mode, gait_id, vel_des, step_height, duration)
    ctrl.Send_cmd(cmd_msg)


def stand_robot(ctrl, cmd_msg):
    send_robot_cmd(
        ctrl, cmd_msg,
        mode=12, gait_id=0,
        vel_des=[0.0, 0.0, 0.0],
        duration=0
    )
    ctrl.Wait_finish(12, 0)


def rest_robot(ctrl, cmd_msg):
    send_robot_cmd(
        ctrl, cmd_msg,
        mode=7, gait_id=1,
        vel_des=[0.0, 0.0, 0.0],
        duration=0
    )


class LiveRobotCtrl:
    """LCM command sender copied from master.py for live PID testing."""

    def __init__(self):
        import lcm
        from cyberdog_interfaces.robot_control_cmd_lcmt import robot_control_cmd_lcmt
        from cyberdog_interfaces.robot_control_response_lcmt import robot_control_response_lcmt

        self.rec_thread = Thread(target=self.rec_response)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.response_type = robot_control_response_lcmt
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.running = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = self.response_type().decode(data)
        self.gait_ok = self.rec_msg.gait_id
        if self.rec_msg.order_process_bar >= 95:
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_response(self):
        while self.running:
            if hasattr(self.lc_r, "handle_timeout"):
                self.lc_r.handle_timeout(100)
            else:
                self.lc_r.handle()
            time.sleep(0.002)

    def Wait_finish(self, mode, gait_id):
        count = 0
        while self.running and count < 2000:
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            time.sleep(0.005)
            count += 1
        return False

    def send_publish(self):
        while self.running:
            self.send_lock.acquire()
            if self.delay_cnt > 20:
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.running = 0
        self.rec_thread.join()
        self.send_thread.join()


# ============================================================
# PIDController — 从 master.py 复制，保持接口一致
# ============================================================
class PIDController:
    """简易 PID 控制器，带积分限幅抗饱和"""

    def __init__(self, kp, ki, kd, out_min, out_max, int_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.int_max = int_max
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, error, dt):
        # 死区
        if abs(error) <= GAP_DEADBAND:
            error = 0.0

        p_term = self.kp * error
        if self.ki != 0.0:
            self.integral += error * dt
            if self.integral > self.int_max:
                self.integral = self.int_max
            elif self.integral < -self.int_max:
                self.integral = -self.int_max
        i_term = self.ki * self.integral
        if dt > 0.0 and self.kd != 0.0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        self.prev_error = error
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        output = p_term + i_term + d_term
        if output > self.out_max:
            output = self.out_max
        elif output < self.out_min:
            output = self.out_min
        return output


# ============================================================
# BangBangController — 原第三关的 bang-bang 逻辑
# ============================================================
class BangBangController:
    """原第三关的 bang-bang 控制器，用于对比"""

    def __init__(self):
        self.reset()

    def reset(self):
        pass

    def update(self, error, dt):
        if abs(error) <= GAP_DEADBAND:
            return 0.0
        return 0.26 if error >= 0 else -0.26


# ============================================================
# Telemetry — 遥测可视化
# ============================================================
class Telemetry:
    """PID 遥测可视化"""

    def __init__(self, title="PID Telemetry"):
        self.title = title
        self.max_history = 400
        self.gap_history = []
        self.vyaw_history = []
        self.gap_raw_history = []

    def update(self, gap_raw, gap, vyaw, p_term, i_term, d_term,
               bang_vyaw=None, paused=False):
        self.gap_raw_history.append(gap_raw)
        self.gap_history.append(gap)
        self.vyaw_history.append(vyaw)
        if len(self.gap_history) > self.max_history:
            self.gap_history.pop(0)
            self.vyaw_history.pop(0)
            self.gap_raw_history.pop(0)

        canvas = np.zeros((520, 680, 3), dtype=np.uint8)

        # ---------- 数值面板 ----------
        y = 22
        cv2.putText(canvas, f"{'[PAUSED] ' if paused else ''}PID Telemetry",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        y += 28
        g_color = (0, 255, 0) if abs(gap_raw) < 20 else \
                  (0, 255, 255) if abs(gap_raw) < 80 else (0, 0, 255)
        cv2.putText(canvas, f"Gap(raw): {gap_raw:7.1f} px  ->  Gap(deadband): {gap:7.1f} px",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, g_color, 2)

        y += 30
        vyaw_color = (0, 255, 0) if abs(vyaw) < 0.25 else \
                     (0, 255, 255) if abs(vyaw) < 0.29 else (0, 0, 255)
        cv2.putText(canvas, f"vyaw: {vyaw:+.4f} rad/s",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, vyaw_color, 2)

        if bang_vyaw is not None:
            cv2.putText(canvas, f"bang-bang: {bang_vyaw:+.4f} rad/s",
                        (280, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (150, 150, 150), 1)

        y += 26
        cv2.putText(canvas, f"P: {p_term:+.4f}   I: {i_term:+.4f}   D: {d_term:+.4f}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (100, 200, 255), 1)

        # 统计
        y += 22
        if len(self.gap_raw_history) > 0:
            abs_gaps = [abs(g) for g in self.gap_raw_history]
            avg = sum(abs_gaps) / len(abs_gaps)
            mx = max(abs_gaps)
            cv2.putText(canvas, f"|Gap| avg: {avg:6.1f}  max: {mx:6.1f}  samples: {len(self.gap_raw_history)}",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

        # ---------- gap 曲线 ----------
        plot_y0 = 145
        plot_h = 150
        plot_w = 660
        self._draw_curve(canvas, plot_y0, plot_h, plot_w,
                         self.gap_raw_history, "Gap (px)", (0, 255, 255),
                         gap_raw, g_color)

        # ---------- vyaw 曲线 ----------
        plot_y0_2 = plot_y0 + plot_h + 20
        self._draw_curve(canvas, plot_y0_2, plot_h, plot_w,
                         self.vyaw_history, "vyaw (rad/s)", (100, 200, 255),
                         vyaw, vyaw_color)

        # ±0.26 参考线
        vyaw_data = self.vyaw_history
        if vyaw_data:
            v_range = max(abs(min(vyaw_data)), abs(max(vyaw_data)), 0.3)
        else:
            v_range = 0.3
        v_mid = plot_y0_2 + plot_h // 2
        v_scale = (plot_h // 2 - 5) / v_range
        for ref_val, ref_label in [(0.26, "+0.26"), (-0.26, "-0.26")]:
            ry = int(v_mid - ref_val * v_scale)
            if plot_y0_2 < ry < plot_y0_2 + plot_h:
                cv2.line(canvas, (10, ry), (10 + plot_w, ry), (60, 60, 60), 1)
                cv2.putText(canvas, ref_label, (10, ry - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (60, 60, 60), 1)

        # 底部提示
        cv2.putText(canvas, "Keys: [r]eset  [space]pause  [q]uit",
                    (10, 510), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

        cv2.imshow(self.title, canvas)

    def _draw_curve(self, canvas, y0, h, w, data, label, color, cur_val, cur_color):
        cv2.putText(canvas, label, (10, y0 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)
        if len(data) < 2:
            return
        d_min = min(data)
        d_max = max(data)
        d_range = max(abs(d_min), abs(d_max), 1e-6)
        mid = y0 + h // 2
        scale = (h // 2 - 5) / d_range
        cv2.line(canvas, (10, mid), (10 + w, mid), (60, 60, 60), 1)
        for i in range(1, len(data)):
            x1 = 10 + (i - 1) * w // self.max_history
            x2 = 10 + i * w // self.max_history
            y1 = int(mid - data[i - 1] * scale)
            y2 = int(mid - data[i] * scale)
            y1 = max(y0, min(y0 + h, y1))
            y2 = max(y0, min(y0 + h, y2))
            cv2.line(canvas, (x1, y1), (x2, y2), color, 1)
        # 当前值
        cx = 10 + (len(data) - 1) * w // self.max_history
        cy = int(mid - cur_val * scale)
        cy = max(y0, min(y0 + h, cy))
        cv2.circle(canvas, (cx, cy), 4, cur_color, -1)

    def close(self):
        cv2.destroyWindow(self.title)


# ============================================================
# Trackbar 调参窗口
# ============================================================
class TuningWindow:
    """用 cv2 trackbar 实时调节 Kp/Ki/Kd 和 gap 输入"""

    def __init__(self):
        cv2.namedWindow("PID Tuner", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("PID Tuner", 500, 360)
        cv2.moveWindow("PID Tuner", 50, 50)

        # trackbar: name, window, initial, max, callback
        # kp: 0 ~ 0.005, 步长 0.00001 -> 映射为 0~500
        cv2.createTrackbar("Kp(x0.00001)", "PID Tuner", 150, 500, self._nop)
        cv2.createTrackbar("Ki(x0.00001)", "PID Tuner", 3, 100, self._nop)
        cv2.createTrackbar("Kd(x0.00001)", "PID Tuner", 10, 200, self._nop)
        cv2.createTrackbar("Vx(x0.01m/s)", "PID Tuner", int(DEFAULT_VX * 100), 50, self._nop)
        cv2.createTrackbar("Gap (px)", "PID Tuner", 320, 640, self._nop)

        # Gap 模式选择
        cv2.createTrackbar("Gap Mode", "PID Tuner", 0, 3, self._nop)
        # 0 = manual (slider)
        # 1 = step wave
        # 2 = sine wave
        # 3 = random walk

        self.step_timer = 0.0
        self.sine_timer = 0.0
        self.walk_gap = 0.0

    def _nop(self, x):
        pass

    def get_kp(self):
        return cv2.getTrackbarPos("Kp(x0.00001)", "PID Tuner") * 0.00001

    def get_ki(self):
        return cv2.getTrackbarPos("Ki(x0.00001)", "PID Tuner") * 0.00001

    def get_kd(self):
        return cv2.getTrackbarPos("Kd(x0.00001)", "PID Tuner") * 0.00001

    def get_vx(self):
        return cv2.getTrackbarPos("Vx(x0.01m/s)", "PID Tuner") * 0.01

    def get_gap(self, dt):
        """根据模式返回当前的 gap 值"""
        mode = cv2.getTrackbarPos("Gap Mode", "PID Tuner")

        if mode == 0:
            # 手动滑块：范围 -320 ~ 320 px
            return cv2.getTrackbarPos("Gap (px)", "PID Tuner") - 320

        elif mode == 1:
            # 阶跃波：每 2.5 秒切换
            self.step_timer += dt
            period = 2.5
            t = self.step_timer % (period * 4)
            if t < period:
                return 0.0
            elif t < period * 2:
                return 80.0
            elif t < period * 3:
                return 0.0
            else:
                return -80.0

        elif mode == 2:
            # 正弦波：振幅 120px，周期 5 秒
            self.sine_timer += dt
            return 120.0 * math.sin(self.sine_timer * 2 * math.pi / 5.0)

        elif mode == 3:
            # 随机游走
            self.walk_gap += np.random.normal(0, 8)
            self.walk_gap = max(-200, min(200, self.walk_gap))
            return self.walk_gap

        return 0.0

    def get_mode_label(self):
        labels = ["Manual Slider", "Step Wave (+-80px)", "Sine Wave (120px)", "Random Walk"]
        mode = cv2.getTrackbarPos("Gap Mode", "PID Tuner")
        return labels[mode]

    def draw_info(self):
        """在 tuner 窗口显示模式信息"""
        canvas = np.zeros((360, 500, 3), dtype=np.uint8)
        kp, ki, kd = self.get_kp(), self.get_ki(), self.get_kd()
        vx = self.get_vx()
        gap = cv2.getTrackbarPos("Gap (px)", "PID Tuner") - 320

        cv2.putText(canvas, f"Kp = {kp:.5f}", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(canvas, f"Ki = {ki:.5f}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(canvas, f"Kd = {kd:.5f}", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(canvas, f"Vx = {vx:.2f} m/s", (10, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(canvas, f"Manual Gap = {gap} px", (10, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        mode = self.get_mode_label()
        cv2.putText(canvas, f"Mode: {mode}", (10, 245),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.putText(canvas, "Modes: 0=Slider  1=Step  2=Sine  3=Walk", (10, 290),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        cv2.putText(canvas, "[r]eset  [space]pause  [q]uit", (10, 325),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

        cv2.imshow("PID Tuner", canvas)


# ============================================================
# 阶跃响应分析
# ============================================================
class StepResponseAnalyzer:
    """分析阶跃响应：超调量、上升时间、稳态时间"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.prev_gap = 0.0
        self.step_started = False
        self.step_direction = 0
        self.peak_vyaw = 0.0
        self.rise_time = None
        self.settling_time = None
        self.overshoot_pct = 0.0
        self.step_elapsed = 0.0
        self.ready = False

    def update(self, gap_raw, vyaw, dt):
        """检测阶跃并计算指标"""
        STEP_THRESH = 30  # gap 变化超过此值视为阶跃

        if abs(gap_raw - self.prev_gap) > STEP_THRESH and not self.step_started:
            # 检测到阶跃
            self.step_started = True
            self.step_direction = 1 if gap_raw > self.prev_gap else -1
            self.step_elapsed = 0.0
            self.peak_vyaw = 0.0
            self.rise_time = None
            self.settling_time = None
            self.ready = False

        if self.step_started:
            self.step_elapsed += dt

            # 记录峰值
            if abs(vyaw) > abs(self.peak_vyaw):
                self.peak_vyaw = vyaw
                # 上升时间：首次达到峰值 90%
                if self.rise_time is None and abs(vyaw) > abs(self.peak_vyaw) * 0.9:
                    self.rise_time = self.step_elapsed

            # 稳态判断：vyaw 降到峰值 10% 以下且保持
            if self.rise_time is not None and abs(vyaw) < abs(self.peak_vyaw) * 0.1:
                if self.settling_time is None:
                    self.settling_time = self.step_elapsed
                    self.ready = True

            # 超调量
            if self.peak_vyaw != 0 and gap_raw != 0:
                steady = self._estimate_steady_state()
                self.overshoot_pct = (abs(self.peak_vyaw) - abs(steady)) / abs(steady) * 100 if steady != 0 else 0

        self.prev_gap = gap_raw
        return self.ready

    def _estimate_steady_state(self):
        """根据 Kp 估计稳态 vyaw 值（简化）"""
        return 0.0  # 目标总是居中

    def get_summary(self):
        if not self.step_started:
            return "Waiting for step input..."
        lines = [
            f"Step detected: {'+' if self.step_direction > 0 else '-'}",
            f"Peak vyaw: {self.peak_vyaw:+.4f} rad/s",
        ]
        if self.rise_time:
            lines.append(f"Rise time: {self.rise_time:.3f}s")
        if self.settling_time:
            lines.append(f"Settling time: {self.settling_time:.3f}s")
        if self.overshoot_pct:
            lines.append(f"Overshoot: {self.overshoot_pct:.1f}%")
        return "\n".join(lines)


# ============================================================
# 主循环
# ============================================================
def run_trackbar_mode():
    """默认模式：trackbar 调参"""
    tuner = TuningWindow()
    tele = Telemetry("PID Telemetry")
    pid = PIDController(kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
                        out_min=DEFAULT_OUT_MIN, out_max=DEFAULT_OUT_MAX,
                        int_max=DEFAULT_INT_MAX)
    pid.reset()

    paused = False
    dt = DEFAULT_DT
    last_time = time.time()

    print("=" * 60)
    print("test3.py — Trackbar 调参模式")
    print("  PID Tuner 窗口: 调节 Kp/Ki/Kd 和 Gap 输入")
    print("  PID Telemetry 窗口: 查看 PID 响应曲线")
    print("  按键: [r]eset  [space]暂停  [q]uit")
    print("  Gap Mode: 0=滑块  1=阶跃  2=正弦  3=随机游走")
    print("=" * 60)

    while True:
        # 处理暂停
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # q or ESC
            break
        elif key == ord('r'):
            pid.reset()
            tele = Telemetry("PID Telemetry")
            print("[PID reset]")
        elif key == ord(' '):
            paused = not paused
            print(f"[{'PAUSED' if paused else 'RESUMED'}]")

        if paused:
            tuner.draw_info()
            continue

        # 从 trackbar 读取当前参数
        kp = tuner.get_kp()
        ki = tuner.get_ki()
        kd = tuner.get_kd()
        pid.set_gains(kp, ki, kd)

        # 获取当前 gap
        gap_raw = tuner.get_gap(dt)

        # PID 更新
        vyaw = pid.update(gap_raw, dt)
        gap = gap_raw if abs(gap_raw) > GAP_DEADBAND else 0.0

        # 遥测
        tele.update(gap_raw, gap, vyaw,
                    pid.last_p, pid.last_i, pid.last_d,
                    paused=paused)

        # Tuner 信息
        tuner.draw_info()

        # 控制循环频率 ~25Hz
        elapsed = time.time() - last_time
        last_time = time.time()
        # 不强制 sleep，靠 cv2.waitKey(1) 自然控制帧率

    tele.close()
    cv2.destroyAllWindows()


def run_step_mode():
    """阶跃响应测试模式"""
    tuner = TuningWindow()
    tele = Telemetry("PID Telemetry - Step Response")
    pid = PIDController(kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
                        out_min=DEFAULT_OUT_MIN, out_max=DEFAULT_OUT_MAX,
                        int_max=DEFAULT_INT_MAX)
    pid.reset()
    analyzer = StepResponseAnalyzer()

    # 强制阶跃模式
    cv2.setTrackbarPos("Gap Mode", "PID Tuner", 1)

    paused = False
    dt = DEFAULT_DT
    step_count = 0
    results = []

    print("=" * 60)
    print("test3.py — 阶跃响应测试")
    print("  自动注入 ±80px 阶跃信号")
    print("  分析超调量、上升时间、稳态时间")
    print("=" * 60)

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('r'):
            pid.reset()
            analyzer.reset()
            tele = Telemetry("PID Telemetry - Step Response")
            print("[reset]")
        elif key == ord(' '):
            paused = not paused

        if paused:
            tuner.draw_info()
            continue

        kp, ki, kd = tuner.get_kp(), tuner.get_ki(), tuner.get_kd()
        pid.set_gains(kp, ki, kd)

        gap_raw = tuner.get_gap(dt)
        vyaw = pid.update(gap_raw, dt)
        gap = gap_raw if abs(gap_raw) > GAP_DEADBAND else 0.0

        # 阶跃分析
        if analyzer.update(gap_raw, vyaw, dt):
            step_count += 1
            summary = analyzer.get_summary()
            print(f"\n--- Step {step_count} ---")
            print(summary)
            results.append(summary)
            analyzer.reset()
            analyzer.prev_gap = gap_raw  # 重置跟踪

        tele.update(gap_raw, gap, vyaw,
                    pid.last_p, pid.last_i, pid.last_d,
                    paused=paused)
        tuner.draw_info()

    print(f"\nTotal steps analyzed: {step_count}")
    tele.close()
    cv2.destroyAllWindows()


def run_sine_mode():
    """正弦波跟踪测试"""
    tuner = TuningWindow()
    tele = Telemetry("PID Telemetry - Sine Tracking")
    pid = PIDController(kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
                        out_min=DEFAULT_OUT_MIN, out_max=DEFAULT_OUT_MAX,
                        int_max=DEFAULT_INT_MAX)
    pid.reset()

    cv2.setTrackbarPos("Gap Mode", "PID Tuner", 2)

    paused = False
    dt = DEFAULT_DT
    print("=" * 60)
    print("test3.py — 正弦波跟踪测试")
    print("  振幅 120px，周期 5s")
    print("  观察 vyaw 是否平滑跟随 gap 变化")
    print("=" * 60)

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('r'):
            pid.reset()
            tele = Telemetry("PID Telemetry - Sine Tracking")
            print("[reset]")
        elif key == ord(' '):
            paused = not paused

        if paused:
            tuner.draw_info()
            continue

        kp, ki, kd = tuner.get_kp(), tuner.get_ki(), tuner.get_kd()
        pid.set_gains(kp, ki, kd)

        gap_raw = tuner.get_gap(dt)
        vyaw = pid.update(gap_raw, dt)
        gap = gap_raw if abs(gap_raw) > GAP_DEADBAND else 0.0

        tele.update(gap_raw, gap, vyaw,
                    pid.last_p, pid.last_i, pid.last_d,
                    paused=paused)
        tuner.draw_info()

    tele.close()
    cv2.destroyAllWindows()


def run_compare_mode():
    """bang-bang vs PID 对比测试"""
    tuner = TuningWindow()
    tele = Telemetry("PID vs Bang-Bang")

    pid = PIDController(kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
                        out_min=DEFAULT_OUT_MIN, out_max=DEFAULT_OUT_MAX,
                        int_max=DEFAULT_INT_MAX)
    bang = BangBangController()
    pid.reset()

    cv2.setTrackbarPos("Gap Mode", "PID Tuner", 1)

    paused = False
    dt = DEFAULT_DT
    print("=" * 60)
    print("test3.py — bang-bang vs PID 对比")
    print("  同一 gap 输入，同时显示两种控制器的输出")
    print("  PID 曲线: 橙色   bang-bang: 灰色虚线 ±0.26")
    print("=" * 60)

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('r'):
            pid.reset()
            bang.reset()
            tele = Telemetry("PID vs Bang-Bang")
            print("[reset]")
        elif key == ord(' '):
            paused = not paused

        if paused:
            tuner.draw_info()
            continue

        kp, ki, kd = tuner.get_kp(), tuner.get_ki(), tuner.get_kd()
        pid.set_gains(kp, ki, kd)

        gap_raw = tuner.get_gap(dt)
        vyaw = pid.update(gap_raw, dt)
        bang_vyaw = bang.update(gap_raw, dt)
        gap = gap_raw if abs(gap_raw) > GAP_DEADBAND else 0.0

        tele.update(gap_raw, gap, vyaw,
                    pid.last_p, pid.last_i, pid.last_d,
                    bang_vyaw=bang_vyaw, paused=paused)

        # 在 PID Tuner 中显示对比
        canvas = np.zeros((320, 500, 3), dtype=np.uint8)
        kp, ki, kd = tuner.get_kp(), tuner.get_ki(), tuner.get_kd()
        gap_manual = cv2.getTrackbarPos("Gap (px)", "PID Tuner") - 320
        cv2.putText(canvas, f"Kp={kp:.5f} Ki={ki:.5f} Kd={kd:.5f}",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        pid_color = (0, 255, 0) if abs(vyaw) < 0.29 else (0, 0, 255)
        cv2.putText(canvas, f"PID:      {vyaw:+.4f} rad/s",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, pid_color, 2)
        cv2.putText(canvas, f"BangBang: {bang_vyaw:+.4f} rad/s",
                    (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)

        diff = abs(vyaw) - abs(bang_vyaw)
        diff_color = (0, 255, 0) if diff < 0 else (0, 0, 255)
        cv2.putText(canvas, f"PID - Bang: {diff:+.4f}  {'(softer)' if diff < 0 else '(harder)'}",
                    (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.55, diff_color, 1)

        cv2.putText(canvas, f"Mode: {tuner.get_mode_label()}", (10, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(canvas, "[r]eset  [space]pause  [q]uit", (10, 290),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

        cv2.imshow("PID Tuner", canvas)

    tele.close()
    cv2.destroyAllWindows()


# ============================================================
# --live 模式：连接 ROS + LCM，实时控制小狗
# ============================================================
def run_live_mode():
    """实机模式：订阅真实 ground_topic，PID 计算后通过 LCM 控制小狗。

    前提条件：
      - 在 Cyberdog 上运行
      - ROS 2 Galactic 环境已 source
      - ground_node.py 正在运行（发布 ground_topic）
      - 运动控制板 LCM 通道可达
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from cyberdog_interfaces.robot_control_cmd_lcmt import robot_control_cmd_lcmt

    # ---- ROS 2 初始化 ----
    rclpy.init()

    # 全局变量，供 ROS callback 写入、主循环读取
    ros_data = {"gap": 0, "edge_num": 0, "running": True}

    class GroundListener(Node):
        """只订阅 ground_topic，不发布任何东西"""
        def __init__(self):
            super().__init__("test3_live_node")
            self.sub = self.create_subscription(
                String, "ground_topic", self.callback, 10)
            self.get_logger().info("test3_live_node: listening to ground_topic")

        def callback(self, msg):
            try:
                edge_num, gap = map(int, msg.data.split())
                ros_data["edge_num"] = edge_num
                ros_data["gap"] = gap
            except Exception:
                pass

    ros_node = GroundListener()

    # ---- LCM 初始化：和 master.py 一样，通过控制线程持续发送并等待反馈 ----
    ctrl = LiveRobotCtrl()
    ctrl.run()
    cmd_msg = robot_control_cmd_lcmt()

    # ---- PID + 可视化 ----
    tuner = TuningWindow()
    tele = Telemetry("PID Live - Real Robot")
    pid = PIDController(kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
                        out_min=DEFAULT_OUT_MIN, out_max=DEFAULT_OUT_MAX,
                        int_max=DEFAULT_INT_MAX)
    pid.reset()

    paused = False
    seeking = False
    dt = DEFAULT_DT  # 25Hz 控制周期

    print("=" * 60)
    print("test3.py --live  实机 PID 控制模式")
    print("  订阅: ground_topic (来自 ground_node.py)")
    print("  LCM:  udpm://239.255.76.67:7671 -> robot_control_cmd")
    print("  启动: 自动起立，按 [s] 开始/停止寻线")
    print("  按键: [s]寻线开关  [r]eset  [space]暂停/恢复  [q]uit")
    print("  退出: 自动站立后趴下")
    print("  WARNING: 小狗会真实移动！请确保周围安全。")
    print("=" * 60)

    print("Starting: stand up robot...")
    stand_robot(ctrl, cmd_msg)
    print("[READY] 小狗已起立，按 s 开始寻线。")

    last_time = time.time()

    try:
        while ros_data["running"]:
            # ROS spin (非阻塞)
            rclpy.spin_once(ros_node, timeout_sec=0.001)

            # 键盘处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('r'):
                pid.reset()
                tele = Telemetry("PID Live - Real Robot")
                print("[PID reset]")
            elif key == ord('s'):
                seeking = not seeking
                pid.reset()
                if seeking:
                    paused = False
                    stand_robot(ctrl, cmd_msg)
                    print("[SEEK ON] 开始寻线")
                else:
                    stand_robot(ctrl, cmd_msg)
                    print("[SEEK OFF] 停止寻线，小狗保持站立")
            elif key == ord(' '):
                paused = not paused
                if paused:
                    stand_robot(ctrl, cmd_msg)
                    print("[PAUSED] 小狗已站立")
                else:
                    print("[RESUMED] 恢复 PID 控制")

            if paused or not seeking:
                tuner.draw_info()
                time.sleep(0.02)
                continue

            # 从 trackbar 读取当前 Kp/Ki/Kd
            kp = tuner.get_kp()
            ki = tuner.get_ki()
            kd = tuner.get_kd()
            pid.set_gains(kp, ki, kd)

            # 从 ROS 获取真实 gap（ground_topic 的值为当前 gap 平均值）
            gap_raw = float(ros_data["gap"])
            edge_num = ros_data["edge_num"]

            # PID 更新
            vyaw = pid.update(gap_raw, dt)
            gap = gap_raw if abs(gap_raw) > GAP_DEADBAND else 0.0

            # 横移（沿用原 P 控制）
            vy = DEFAULT_VY_KP * gap_raw if abs(gap_raw) > GAP_DEADBAND else 0.0
            vy = max(-0.10, min(0.10, vy))

            # 前进速度
            vx = tuner.get_vx()

            # ---- 通过 LCM 发送寻线运动指令 ----
            send_robot_cmd(
                ctrl, cmd_msg,
                mode=11,
                gait_id=12,
                vel_des=[vx, vy, vyaw],
                duration=0
            )

            # 遥测显示
            tele.update(gap_raw, gap, vyaw,
                        pid.last_p, pid.last_i, pid.last_d,
                        paused=False)

            # Tuner 信息
            canvas = np.zeros((360, 500, 3), dtype=np.uint8)
            cv2.putText(canvas, f"Kp={kp:.5f} Ki={ki:.5f} Kd={kd:.5f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(canvas, f"Gap (from ground_node): {gap_raw:.1f} px",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(canvas, f"Edges detected: {edge_num}",
                        (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
            cv2.putText(canvas, f"Vx slider: {vx:.2f} m/s",
                        (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            vyaw_color = (0, 255, 0) if abs(vyaw) < 0.25 else \
                         (0, 255, 255) if abs(vyaw) < 0.29 else (0, 0, 255)
            cv2.putText(canvas, f"CMD -> vx={vx:.2f}  vy={vy:+.4f}  vyaw={vyaw:+.4f}",
                        (10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.65, vyaw_color, 2)
            cv2.putText(canvas, f"P={pid.last_p:+.4f} I={pid.last_i:+.4f} D={pid.last_d:+.4f}",
                        (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)

            # LCM 发送状态
            cv2.putText(canvas, f"LCM sent OK (life_count={cmd_msg.life_count})", (10, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.putText(canvas, "[s]seek  [r]reset  [space]pause  [q]quit", (10, 325),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

            cv2.imshow("PID Tuner", canvas)

            # 频率控制 ~25Hz
            elapsed = time.time() - last_time
            if elapsed < dt:
                time.sleep(dt - elapsed)
            last_time = time.time()
    finally:
        # ---- 退出：站立 -> 趴下 -> 清理 ----
        print("Stopping: stand up robot...")
        stand_robot(ctrl, cmd_msg)
        print("Stopping: rest robot...")
        rest_robot(ctrl, cmd_msg)
        time.sleep(0.5)
        ctrl.quit()

    tele.close()
    cv2.destroyAllWindows()
    ros_node.destroy_node()
    rclpy.shutdown()
    print("Done.")


def main(args=None):
    """Console-script entry point for `ros2 run cyberdog_demo test3`."""
    argv = list(sys.argv[1:] if args is None else args)
    if argv and argv[0] == "--":
        argv = argv[1:]
    mode = argv[0] if argv else ""

    if mode in ("--help", "-h"):
        print_usage()
    elif mode == "--step":
        run_step_mode()
    elif mode == "--sine":
        run_sine_mode()
    elif mode == "--compare":
        run_compare_mode()
    elif mode == "--live":
        run_live_mode()
    else:
        if mode and mode not in VALID_MODES:
            print(f"未知参数: {mode}")
            print_usage()
            return 2
        return run_trackbar_mode()
    return 0


# ============================================================
# Entry point
# ============================================================
if __name__ == "__main__":
    main()
