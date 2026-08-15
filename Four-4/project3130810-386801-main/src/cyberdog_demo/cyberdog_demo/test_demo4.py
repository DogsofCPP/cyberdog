'''
第四关专项测试脚本

从 master.py 中抽取第四关相关逻辑，保留：
- adjust_topic 订阅
- ground_topic 订阅
- PID 对齐
- 自定义步态切换
- 第四关三段动作流程
'''
import copy
import math
import sys
import time
import toml
import cv2
import numpy as np
import lcm
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Lock, Thread

from cyberdog_interfaces import *
from cyberdog_interfaces.file_send_lcmt import file_send_lcmt
from cyberdog_interfaces.robot_control_cmd_lcmt import robot_control_cmd_lcmt


DEFAULT_KP = 0.00170
DEFAULT_KI = 0.00003
DEFAULT_KD = 0.00010
DEFAULT_VY_KP = 0.000113
DEFAULT_PID_OUT_MIN = -0.30
DEFAULT_PID_OUT_MAX = 0.30
DEFAULT_PID_INT_MAX = 0.05
DEFAULT_VY_OUT_MIN = -0.10
DEFAULT_VY_OUT_MAX = 0.10

class PIDController:
    """简易 PID 控制器，带积分限幅抗饱和

    用法:
        pid = PIDController(kp=0.0015, ki=0.00003, kd=0.0001,
                            out_min=-0.30, out_max=0.30, int_max=0.05)
        pid.reset()                    # 每个修正阶段前调用
        output = pid.update(error, dt) # 每帧调用
    """
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

    def update(self, error, dt):
        # 比例项
        p_term = self.kp * error

        # 积分项（带限幅抗饱和）
        if self.ki != 0.0:
            self.integral += error * dt
            if self.integral > self.int_max:
                self.integral = self.int_max
            elif self.integral < -self.int_max:
                self.integral = -self.int_max
        i_term = self.ki * self.integral

        # 微分项（对误差微分）
        if dt > 0.0 and self.kd != 0.0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        self.prev_error = error

        # 保存三项分解，供遥测窗口读取
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term

        # 合成输出并限幅
        output = p_term + i_term + d_term
        if output > self.out_max:
            output = self.out_max
        elif output < self.out_min:
            output = self.out_min

        return output


class PIDTelemetry:
    """PID 遥测可视化窗口，用于实时调参

    显示内容：
      - 上方：当前 gap 值、vyaw/vy 输出值（大字体）
      - 中间：vyaw 的 P/I/D 三项分别贡献
      - 下方：gap 和 vyaw 的历史曲线（滚动）
    """
    def __init__(self, title="PID Telemetry"):
        self.title = title
        self.max_history = 200
        self.gap_history = []
        self.vyaw_history = []
        self.vy_history = []
        # 创建窗口并设置位置
        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.title, 640, 480)
        cv2.moveWindow(self.title, 680, 50)  # 放在相机窗口右边

    def update(self, gap, vyaw, vy, p_term=0.0, i_term=0.0, d_term=0.0):
        """更新遥测数据并刷新窗口。

        Args:
            gap: 当前像素偏差
            vyaw: vyaw 输出值
            vy: vy 输出值
            p_term, i_term, d_term: vyaw PID 三项分解
        """
        self.gap_history.append(gap)
        self.vyaw_history.append(vyaw)
        self.vy_history.append(vy)
        if len(self.gap_history) > self.max_history:
            self.gap_history.pop(0)
            self.vyaw_history.pop(0)
            self.vy_history.pop(0)

        # 创建黑色画布
        canvas = np.zeros((480, 640, 3), dtype=np.uint8)

        # ---- 数值面板 ----
        # 标题
        cv2.putText(canvas, "PID Telemetry - Level 3", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)

        # gap 大字体（颜色随偏差变化）
        if abs(gap) < 20:
            g_color = (0, 255, 0)
        elif abs(gap) < 80:
            g_color = (0, 255, 255)
        else:
            g_color = (0, 0, 255)
        cv2.putText(canvas, f"Gap: {gap:7.1f} px", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, g_color, 2)

        # vyaw / vy 输出
        vyaw_color = (0, 255, 0) if abs(vyaw) < 0.25 else (0, 255, 255) if abs(vyaw) < 0.29 else (0, 0, 255)
        cv2.putText(canvas, f"vyaw: {vyaw:+.4f} rad/s", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, vyaw_color, 2)
        cv2.putText(canvas, f"vy:   {vy:+.4f} m/s", (10, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # PID 三项分解
        cv2.putText(canvas, "vyaw PID breakdown:", (10, 165),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
        cv2.putText(canvas, f"  P: {p_term:+.4f}", (10, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 200, 255), 1)
        cv2.putText(canvas, f"  I: {i_term:+.4f}", (160, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 1)
        cv2.putText(canvas, f"  D: {d_term:+.4f}", (310, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1)

        # 统计信息
        if len(self.gap_history) > 0:
            avg_gap = sum(self.gap_history) / len(self.gap_history)
            max_gap = max(self.gap_history, key=abs)
            cv2.putText(canvas, f"Avg Gap: {avg_gap:6.1f}  Max |Gap|: {abs(max_gap):6.1f}", (10, 220),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        # ---- 历史曲线 ----
        plot_y0 = 250
        plot_h = 100
        plot_w = 620

        if len(self.gap_history) > 1:
            # gap 曲线（上半）
            cv2.putText(canvas, f"Gap (px)  range: [{min(self.gap_history):.0f}, {max(self.gap_history):.0f}]",
                        (10, plot_y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
            gap_range = max(abs(min(self.gap_history)), abs(max(self.gap_history)), 1)
            gap_mid = plot_y0 + plot_h // 2
            gap_scale = (plot_h // 2 - 5) / gap_range
            # 零线
            cv2.line(canvas, (10, gap_mid), (10 + plot_w, gap_mid), (80, 80, 80), 1)
            for i in range(1, len(self.gap_history)):
                x1 = 10 + (i - 1) * plot_w // self.max_history
                x2 = 10 + i * plot_w // self.max_history
                y1 = int(gap_mid - self.gap_history[i - 1] * gap_scale)
                y2 = int(gap_mid - self.gap_history[i] * gap_scale)
                y1 = max(plot_y0, min(plot_y0 + plot_h, y1))
                y2 = max(plot_y0, min(plot_y0 + plot_h, y2))
                cv2.line(canvas, (x1, y1), (x2, y2), (0, 255, 255), 1)
            # 当前值标记
            cur_x = 10 + (len(self.gap_history) - 1) * plot_w // self.max_history
            cur_y = int(gap_mid - gap * gap_scale)
            cur_y = max(plot_y0, min(plot_y0 + plot_h, cur_y))
            cv2.circle(canvas, (cur_x, cur_y), 4, g_color, -1)

        # vyaw 曲线（下半）
        plot_y0_2 = plot_y0 + plot_h + 15
        if len(self.vyaw_history) > 1:
            cv2.putText(canvas, f"vyaw (rad/s)  range: [{min(self.vyaw_history):+.3f}, {max(self.vyaw_history):+.3f}]",
                        (10, plot_y0_2 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
            vyaw_range = max(abs(min(self.vyaw_history)), abs(max(self.vyaw_history)), 0.01)
            vyaw_mid = plot_y0_2 + plot_h // 2
            vyaw_scale = (plot_h // 2 - 5) / vyaw_range
            # 零线
            cv2.line(canvas, (10, vyaw_mid), (10 + plot_w, vyaw_mid), (80, 80, 80), 1)
            # ±0.26 参考线（原 bang-bang 值）
            ref_y_pos = int(vyaw_mid - 0.26 * vyaw_scale)
            ref_y_neg = int(vyaw_mid + 0.26 * vyaw_scale)
            cv2.line(canvas, (10, ref_y_pos), (10 + plot_w, ref_y_pos), (60, 60, 60), 1)
            cv2.line(canvas, (10, ref_y_neg), (10 + plot_w, ref_y_neg), (60, 60, 60), 1)
            for i in range(1, len(self.vyaw_history)):
                x1 = 10 + (i - 1) * plot_w // self.max_history
                x2 = 10 + i * plot_w // self.max_history
                y1 = int(vyaw_mid - self.vyaw_history[i - 1] * vyaw_scale)
                y2 = int(vyaw_mid - self.vyaw_history[i] * vyaw_scale)
                y1 = max(plot_y0_2, min(plot_y0_2 + plot_h, y1))
                y2 = max(plot_y0_2, min(plot_y0_2 + plot_h, y2))
                cv2.line(canvas, (x1, y1), (x2, y2), (100, 200, 255), 1)
            # 当前值标记
            cur_y2 = int(vyaw_mid - vyaw * vyaw_scale)
            cur_y2 = max(plot_y0_2, min(plot_y0_2 + plot_h, cur_y2))
            cv2.circle(canvas, (cur_x, cur_y2), 4, vyaw_color, -1)

        # 底部提示
        cv2.putText(canvas, "Gap (yellow)  |  vyaw (orange)  |  --- = old bang-bang +/-0.26",
                    (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)

        cv2.imshow(self.title, canvas)
        cv2.waitKey(1)

    def close(self):
        """关闭遥测窗口"""
        cv2.destroyWindow(self.title)


class Robot_Ctrl_Node(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_response)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.running = 1
        self.jump_num = 0

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if self.rec_msg.order_process_bar >= 95:
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_response(self):
        while self.running:
            self.lc_r.handle()
            time.sleep(0.002)

    def Wait_finish(self, mode, gait_id):
        count = 0
        while self.running and count < 2000:  # 10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def send_publish(self):
        while self.running:
            self.send_lock.acquire()
            if (
                self.delay_cnt > 20
            ):  # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
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


class TestDemo4Node(Node):
    def __init__(self):
        super().__init__("test_demo4_node")
        self.lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.usergait_msg = file_send_lcmt()
        self.cmd_msg = robot_control_cmd_lcmt()
        self.Ctrl = Robot_Ctrl_Node()
        self.Ctrl.run()

        #other node msg.data;
        self.adjust_gap = "0 0" # 边界数，空格，gap大小
        self.ground_gap = '0 0'

        #create a Adjust_sub
        self.adjust_subscription = self.create_subscription(String,'adjust_topic',self.adjust_callback,10)
        self.adjust_subscription
        self.get_logger().info(f"Subscribing to topic: adjust_topic")

        #create a ground_sub
        self.ground_subscription = self.create_subscription(String,'ground_topic',self.ground_callback,10)
        self.ground_subscription
        self.get_logger().info(f"Subscribing to topic: ground_topic")
        self.test_thread = None

    def adjust_callback(self,msg):
        self.adjust_gap = msg.data

    def ground_callback(self,msg):
        self.ground_gap = msg.data

    def send_motion(self, gait_id, vel_des, duration_sleep,
                    mode=11, step_height=(0.06, 0.06), duration=0):
        self.cmd_msg.mode = mode
        self.cmd_msg.gait_id = gait_id
        self.cmd_msg.vel_des = list(vel_des)
        self.cmd_msg.step_height = list(step_height)
        self.cmd_msg.duration = duration
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count >= 50:
            self.cmd_msg.life_count = 1
        self.Ctrl.Send_cmd(self.cmd_msg)
        time.sleep(duration_sleep)

    def stand(self, sleep_after=0.0):
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.vel_des = [0, 0, 0]
        self.cmd_msg.duration = 0
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count >= 50:
            self.cmd_msg.life_count = 1
        self.Ctrl.Send_cmd(self.cmd_msg)
        self.Ctrl.Wait_finish(12, 0)
        if sleep_after > 0.0:
            time.sleep(sleep_after)

    def run_pid_align(self, gap_source, loop_count, vx, gait_id=12,
                      dt=0.040, telemetry_title=None):
        pid_vyaw = PIDController(
            kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
            out_min=DEFAULT_PID_OUT_MIN,
            out_max=DEFAULT_PID_OUT_MAX,
            int_max=DEFAULT_PID_INT_MAX
        )
        pid_vy = PIDController(
            kp=DEFAULT_VY_KP, ki=0.0, kd=0.0,
            out_min=DEFAULT_VY_OUT_MIN,
            out_max=DEFAULT_VY_OUT_MAX,
            int_max=0.0
        )
        pid_vyaw.reset()
        pid_vy.reset()

        telemetry = PIDTelemetry(telemetry_title) if telemetry_title else None
        try:
            for i in range(0, loop_count):
                if gap_source == "ground":
                    edge_num, gap = map(int, self.ground_gap.split())
                else:
                    edge_num, gap = map(int, self.adjust_gap.split())

                if abs(gap) > 4:
                    vyaw = pid_vyaw.update(float(gap), dt)
                    vy = pid_vy.update(float(gap), dt)
                else:
                    vyaw = pid_vyaw.update(0.0, dt)
                    vy = pid_vy.update(0.0, dt)

                if telemetry is not None:
                    telemetry.update(float(gap), vyaw, vy,
                                     pid_vyaw.last_p,
                                     pid_vyaw.last_i,
                                     pid_vyaw.last_d)

                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = gait_id
                self.cmd_msg.vel_des = [vx, vy, vyaw]
                self.cmd_msg.step_height = [0.06, 0.06]
                self.cmd_msg.duration = 0
                self.cmd_msg.life_count += 1
                if self.cmd_msg.life_count >= 50:
                    self.cmd_msg.life_count = 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                time.sleep(dt)
        finally:
            if telemetry is not None:
                telemetry.close()

    def run_usergait_cycles(self, cycles, sleep_sec=0.84):
        self.get_logger().info(f"执行自定义步态 {cycles} 次")
        for i in range(0, cycles):
            self.cmd_msg.mode = 62
            self.cmd_msg.gait_id = 110
            self.cmd_msg.life_count += 1
            if self.cmd_msg.life_count >= 50:
                self.cmd_msg.life_count = 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(sleep_sec)

    def change_gait(self, deff, full, params):
        robot_cmd = {
            'mode': 0, 'gait_id': 0, 'contact': 0, 'life_count': 0,
            'vel_des': [0.0, 0.0, 0.0],
            'rpy_des': [0.0, 0.0, 0.0],
            'pos_des': [0.0, 0.0, 0.0],
            'acc_des': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ctrl_point': [0.0, 0.0, 0.0],
            'foot_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'step_height': [0.0, 0.0],
            'value': 0, 'duration': 0
        }
        steps = toml.load(params)
        full_steps = {'step': [robot_cmd]}
        k = 0
        for i in steps['step']:
            cmd = copy.deepcopy(robot_cmd)
            cmd['duration'] = i['duration']
            if i['type'] == 'usergait':
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
                cmd['value'] = i['use_mpc_traj']
                cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                cmd['ctrl_point'][2] = i['mu']
            if k == 0:
                full_steps['step'] = [cmd]
            else:
                full_steps['step'].append(cmd)
            k += 1

        with open(full, 'w') as f:
            f.write("# Gait Params\n")
            f.writelines(toml.dumps(full_steps))

        with open(deff, 'r') as file_obj_gait_def, open(full, 'r') as file_obj_gait_params:
            self.usergait_msg.data = file_obj_gait_def.read()
            self.lcm_usergait.publish("user_gait_file", self.usergait_msg.encode())
            time.sleep(0.5)
            self.usergait_msg.data = file_obj_gait_params.read()
            self.lcm_usergait.publish("user_gait_file", self.usergait_msg.encode())
            time.sleep(0.1)

    def run_level4_test(self):
        try:
            self.get_logger().info("开始第四关专项测试")

            self.stand()

            self.run_pid_align("adjust", 120, vx=0.1)

            self.get_logger().info("第四关入口: 左移")
            self.send_motion(gait_id=10, vel_des=[0.0, 0.25, 0.0], duration_sleep=3.5)
            self.stand()

            self.get_logger().info("第四关入口: 前进")
            self.send_motion(gait_id=10, vel_des=[0.4, 0.0, 0.0], duration_sleep=3.5)

            self.change_gait("/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait/low_Def.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait/low_Params_full.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait/low_Params.toml")
            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            self.get_logger().info("矫正方向")
            self.run_pid_align("ground", 210, vx=0.1)
            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)


            self.get_logger().info("sh开始循环")
            # oni自定义步态6
            for i in range(0,8):
                self.cmd_msg.mode =62
                self.cmd_msg.gait_id = 110
                self.cmd_msg.life_count += 1
                if self.cmd_msg.life_count >= 50:
                    self.cmd_msg.life_count = 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                self.get_logger().info("等待自定义步态完成")
                time.sleep(0.84)

            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)
            self.get_logger().info("sh开始循环")

            self.change_gait("/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait_re/low_Def.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait_re/low_Params_full.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait_re/low_Params.toml")
            time.sleep(1)

            # oni自定义步态6
            for i in range(0,10):
                self.cmd_msg.mode =62
                self.cmd_msg.gait_id = 110
                self.cmd_msg.life_count += 1
                if self.cmd_msg.life_count >= 50:
                    self.cmd_msg.life_count = 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                self.get_logger().info("等待自定义步态完成")
                time.sleep(0.84)


            self.get_logger().info("矫正方向")
            self.run_pid_align("ground", 200, vx=0.1)
            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)



            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 12
            self.cmd_msg.vel_des = [-0.3, 0, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(5.5)


            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

#  第四关第二个通道
            # 向左前行
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 12
            self.cmd_msg.vel_des = [0.3, 0.2, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(5)

            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 12
            self.cmd_msg.vel_des = [0, 0, -0.1]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(1.3)


            self.run_pid_align("ground", 120, vx=0.1)

            # 向前
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.2,0.0, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(5)

            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            # 后退
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [-0.2,0.0, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(8)

            self.run_pid_align("ground", 120, vx=0.2)

            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [-0.4,0.0, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(3)

            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            time.sleep(5)



#  第四关第三个通道
            # 向左后走
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 12
            self.cmd_msg.vel_des = [-0.2, 0.27, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(3.3)


            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            self.change_gait("/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait/low_Def.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait/low_Params_full.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait/low_Params.toml")
            time.sleep(1)

            self.get_logger().info("sh开始循环")
            # oni自定义步态6
            for i in range(0,6):
                self.cmd_msg.mode =62
                self.cmd_msg.gait_id = 110
                self.cmd_msg.life_count += 1
                if self.cmd_msg.life_count >= 50:
                    self.cmd_msg.life_count = 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                self.get_logger().info("等待自定义步态完成")
                time.sleep(0.84)


            self.run_pid_align("ground", 160, vx=0.2)

            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 12
            self.cmd_msg.vel_des = [0.2, 0, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(2)


            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 12
            self.cmd_msg.vel_des = [-0.2, 0, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(4)

            self.run_pid_align("ground", 100, vx=0.2)

            self.get_logger().info("sh开始循环")

            self.change_gait("/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait_re/low_Def.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait_re/low_Params_full.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/low_gait_re/low_Params.toml")
            time.sleep(1)

            # oni自定义步态6
            for i in range(0,22):
                self.cmd_msg.mode =62
                self.cmd_msg.gait_id = 110
                self.cmd_msg.life_count += 1
                if self.cmd_msg.life_count >= 50:
                    self.cmd_msg.life_count = 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                self.get_logger().info("等待自定义步态完成")
                time.sleep(0.84)
            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)



            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 12
            self.cmd_msg.vel_des = [0,-0.3, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(10)

            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)
            time.sleep(3)
            # time.sleep(1000)




# 第五关回环路
            self.get_logger().info("第五关")
            self.get_logger().info("矫正方向")
            self.run_pid_align("ground", 80, vx=0.05)

            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            for i in range(0,1):
                self.cmd_msg.mode = 16          # 原地跳远模式
                self.cmd_msg.gait_id = 1        # 原地跳远子动作
                self.cmd_msg.vel_des = [0, 0, 0]  # 保持静止
                self.cmd_msg.step_height = [0.05, 0.05]  # 降低步高到0.05m（在0~0.06m范围内）
                self.cmd_msg.duration = 1000    # 设置1000ms执行时间（>790ms）
                self.cmd_msg.life_count += 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                time.sleep(1)  # 等待动作完成
                
                # 恢复站立
                self.cmd_msg.mode = 12          # 恢复站立模式
                self.cmd_msg.gait_id = 0        # 站立子动作
                self.cmd_msg.duration = 0       # 持续执行直到稳定
                self.cmd_msg.life_count += 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                time.sleep(2)  # 等待站立稳定

            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)
            # 休息
            self.cmd_msg.mode = 7
            self.cmd_msg.gait_id = 1
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.get_logger().info("2运动结束")

        except KeyboardInterrupt:
            self.cmd_msg.mode = 7 #PureDamper before KeyboardInterrupt:
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.Ctrl.Send_cmd(self.cmd_msg)

            self.Ctrl.quit()
            return
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.cmd_msg.mode = 7 #PureDamper before KeyboardInterrupt:
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.Ctrl.Send_cmd(self.cmd_msg)

            self.Ctrl.quit()
            return
        finally:
            self.Ctrl.quit()
            return

    def shutdown(self):
        self.Ctrl.quit()
        self.destroy_node()


def main():
    rclpy.init()
    node = TestDemo4Node()

    while True:
        terminal_input = input("输入 4 开始第四关测试：").strip()
        if terminal_input == "4":
            node.test_thread = Thread(target=node.run_level4_test)
            node.test_thread.daemon = True
            node.test_thread.start()
            break

    try:
        while rclpy.ok() and node.test_thread.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.shutdown()
        rclpy.shutdown()
        sys.exit()


if __name__ == '__main__':
    main()
