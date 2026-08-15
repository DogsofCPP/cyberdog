'''
This demo show the communication interface of MR813 motion control board based on Lcm
- robot_control_cmd_lcmt.py
- file_send_lcmt.py
- Gait_Def_moonwalk.toml
- Gait_Params_moonwalk.toml
- Usergait_List.toml
'''
import lcm
import sys
import time
import toml
import copy
import math
import cv2
import numpy as np
from cyberdog_interfaces.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from cyberdog_interfaces.file_send_lcmt import file_send_lcmt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from threading import Lock, Thread
from cyberdog_interfaces import *
from cyberdog_demo.test_demo import masterNode_test as masterNode2


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









class masterNode(Node):
    def __init__(self):
        self.lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.usergait_msg = file_send_lcmt()
        self.cmd_msg = robot_control_cmd_lcmt()
        self.Ctrl = Robot_Ctrl_Node()
        self.Ctrl.run()
        super().__init__("master_node")
        self.get_logger().info("Master_node started!")

        #other node msg.data;
        self.adjust_gap = "0 0" # 边界数，空格，gap大小
        self.ground_gap = '0 0'
        self.seek_dist_001 = "0.0 0" # ratio大小，空格，dist大小
        self.Ctrl.mode_ok = 0
        self.fish_rgap = 0.0
        self.hit_ratio = "0.0 0.0" #left_ratio_hit 空格 right_ratio_hit
        self.hit_time = 0
        self.fish_line = '0.0 0.0'  #left_line 空格 line
        self.jump_lock = 600
        self.jump_num = 0

        # #将mode_ok发布到其他节点：
        # self.publisher_mode = self.create_publisher(String, 'master_mode_topic', 10)
        # self.timer = self.create_timer(0.1, self.mode_pub_callback)

        #create a Adjust_sub
        self.adjust_subscription = self.create_subscription(String,'adjust_topic',self.adjust_callback,10)
        self.adjust_subscription
        self.get_logger().info(f"Subscribing to topic: adjust_topic")

        #create a ground_sub
        self.ground_subscription = self.create_subscription(String,'ground_topic',self.ground_callback,10)
        self.ground_subscription
        self.get_logger().info(f"Subscribing to topic: ground_topic")

        #create a Seek_sub_001
        self.seek_subscription = self.create_subscription(String,'seek_001_topic',self.seek_001_callback,10)
        self.seek_subscription
        self.get_logger().info(f"Subscribing to topic: seek_topic")

        #create a fish_rgap_sub
        self.fish_rgap_subscription = self.create_subscription(String,'fish_rgap_topic',self.fish_rgap_callback,10)
        self.fish_rgap_subscription
        self.get_logger().info(f"Subscribing to topic: fish_rgap_topic")

        #create a hit_ratio_sub
        self.hit_subscription = self.create_subscription(String,'hit_ratio_topic',self.hit_ratio_callback,10)
        self.hit_subscription
        self.get_logger().info(f"Subscribing to topic: hit_ratio_topic")

        #create a fish_line_sub
        self.fish_line_subscription = self.create_subscription(String,'fish_line_topic',self.fish_line_callback,10)
        self.fish_line_subscription
        self.get_logger().info(f"Subscribing to topic: fish_line_topic")

        
        # 主脚本线程由 main 中的终端输入触发启动
        self.master_thread = None

        self.switch = 0
        self.get_logger().info("开始进入事件循环（监听别的节点发过来的消息）。")

    def mode_pub_callback(self):
        msg = String()
        msg.data = '%d' %(self.Ctrl.mode_ok) # 传输的数据： mode_ok
        self.publisher_mode.publish(msg)

    def adjust_callback(self,msg):
        self.adjust_gap = msg.data

    def ground_callback(self,msg):
        self.ground_gap = msg.data

    def seek_001_callback(self,msg):
        self.seek_dist_001 = msg.data

    def fish_rgap_callback(self,msg):
        self.fish_rgap = msg.data

    def hit_ratio_callback(self,msg):
        self.hit_ratio = msg.data

    def fish_line_callback(self,msg):
        self.fish_line = msg.data


########## 自定义路线区域 ############

    def sec002_seek(self):               
        left,right=  map(float,self.hit_ratio.split())  # 面积占比可能是小数，如 0.0027
        rgap = float(self.fish_rgap)
        self.get_logger().info("左"+str(left)+"右"+str(right))

        if  (left<=0.02 and right<=0.02) or self.switch > 0:#继续前进;
            if rgap >0.01:#向右调整;
                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = 10
                self.cmd_msg.vel_des = [0.3, -rgap/60, -rgap/50]
                self.cmd_msg.step_height = [0.06, 0.06]
                self.cmd_msg.duration = 0
                self.cmd_msg.life_count += 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                time.sleep(0.040)
                pass 

            elif rgap <-0.01:#向左调整;
                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = 10
                self.cmd_msg.vel_des = [0.24, -rgap/50, -rgap/35]
                self.cmd_msg.step_height = [0.06, 0.06]
                self.cmd_msg.duration = 0
                self.cmd_msg.life_count += 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                time.sleep(0.040)

            else :
                self.cmd_msg.mode = 11   # keep 
                self.cmd_msg.gait_id = 10
                self.cmd_msg.vel_des = [0.24, 0.0, 0.0]
                self.cmd_msg.step_height = [0.06, 0.06]
                self.cmd_msg.duration = 0
                self.cmd_msg.life_count += 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                time.sleep(0.040)
            self.switch = max(0, self.switch - 1)

        elif left >=0.018 and self.switch == 0 :# 识别到小球，开始撞击;
            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            self.cmd_msg.mode = 11   # keep 
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.22, 0.25, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(1.2)
            self.cmd_msg.mode = 11   # keep 
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0, -0.30, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(1.2)
            self.hit_time +=1
            self.switch = 20
            self.get_logger().info("发现小球")
            pass
        elif right >=0.02 and self.switch == 0:# 识别到小球，开始撞击;
            vx = 0.25
            if self.hit_time == 1:
                vx = 0.18
            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            self.cmd_msg.mode = 11   # keep 
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [vx, -0.29, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(2)
            self.cmd_msg.mode = 11   # keep 
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0, 0.35, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(2)
            self.hit_time +=1
            self.switch = 30
            self.get_logger().info("发现小球")
        if self.cmd_msg.life_count >= 50:
            self.cmd_msg.life_count =1

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


########## 自定义路线区域 ############
               
    def master_line(self):
        try:
            
            self.change_gait("/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/sec001_gait/sec001_Def.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/sec001_gait/sec001_Param_full.toml",
                        "/home/cyberdog_race2026_ws/src/cyberdog_demo/cyberdog_demo/Gait_use/sec001_gait/sec001_Param.toml")
            
            self.get_logger().info("开始运动")
            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            
            self.get_logger().info("矫正方向")
            # 矫正方向
            self.run_pid_align("ground", 120, vx=0, telemetry_title="PID Telemetry - Level 1")

##################################正式运动：：####################

            # 恢复站立
            self.get_logger().info("开始运动")
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

# 开始进行第一关:跳跃通过
            self.get_logger().info("第一关")
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.22,0, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(0.3)

            for i in range(0,5):
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
# 第一关结束

            #左拐
            self.get_logger().info("第二关")
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.36,0, 1.03]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(3.1)

            # 右转矫正方向
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.32,0, -0.8]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(2.3)

            # #  向左横行
            # self.cmd_msg.mode = 11
            # self.cmd_msg.gait_id = 10
            # self.cmd_msg.vel_des = [0,0.265, 0.0]
            # self.cmd_msg.step_height = [0.06, 0.06]
            # self.cmd_msg.duration = 0
            # self.cmd_msg.life_count += 1
            # self.Ctrl.Send_cmd(self.cmd_msg)
            # time.sleep(1.7)

            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)
#第一关结束

#开始第二关：
# 一周目：检测直行
            self.get_logger().info("开始寻找小球")
            # 跟随橙色小球
            for i in range(0,4800):  # 0.04s更新一下gap
                self.get_logger().info(f"hit_time{self.hit_time}")
                if self.hit_time ==2:
                    self.get_logger().info("识别了两个球，将要结束")
                    break

                else:
                    self.sec002_seek()

# 一周目：左拐
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.37, 0, 0.98]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(2)
            
#中间段：直行
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.42, 0, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(3)

#二周目：左拐
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.37, 0, 1.03]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(1.7)

# # 二周目：无检测直行
#             self.cmd_msg.mode = 11
#             self.cmd_msg.gait_id = 10
#             self.cmd_msg.vel_des = [0.42, 0, 0]
#             self.cmd_msg.step_height = [0.06, 0.06]
#             self.cmd_msg.duration = 0
#             self.cmd_msg.life_count += 1
#             self.Ctrl.Send_cmd(self.cmd_msg)
#             time.sleep(3)

            # # 恢复站立
            # self.cmd_msg.mode = 12
            # self.cmd_msg.gait_id = 0
            # self.cmd_msg.life_count += 1
            # self.Ctrl.Send_cmd(self.cmd_msg)
            # self.Ctrl.Wait_finish(8, 0)

#二周目：第一段检测直行

            self.cmd_msg.mode = 11  
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.41, 0.05, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(3)

            self.get_logger().info("开始寻找小球，第二条道")
            # 跟随橙色小球
            for i in range(0, 3390):  # 0.04s更新一下gap
                self.sec002_seek()
                if self.hit_time ==3:
                    self.switch =100
                    # 二周目：转身（360）
                    self.cmd_msg.mode = 11
                    self.cmd_msg.gait_id = 10
                    self.cmd_msg.vel_des = [0, 0, -1.1]
                    self.cmd_msg.step_height = [0.06, 0.06]
                    self.cmd_msg.duration = 0
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    time.sleep(3.2)
                    self.switch = 200
                    break

#二周目：第二段检测直行
            self.get_logger().info("开始寻找小球，第二条道")
            # 跟随橙色小球
            for i in range(0,5400):  # 0.04s更新一下gap
                if self.hit_time ==4:
                    break
                self.sec002_seek()
#弯道第三关：
            #  向左横行 向前运动
            self.get_logger().info("第三关")
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.26,0.25, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 4400
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(1.1)


            self.get_logger().info("矫正方向")
            self.run_pid_align("ground", 526, vx=0.2,
                               telemetry_title="PID Telemetry - Level 3")

            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            '''
            # 第三关末尾 旋转一下
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.0,0, 0.25]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 1700
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(1.8)
            '''
            # 第三关末尾 往前走一点，方便矫正
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.1,-0.1, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(0.3)
            self.get_logger().info("矫正方向")

            self.run_pid_align("adjust", 120, vx=0.1)


            # 恢复站立
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            time.sleep(5)

# 第四关开始：
            self.get_logger().info("第四关")
            # 向左移动(第四关)
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.0,0.25,0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(3.5)

            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)

            time.sleep(5)

            # 向前移动(第四关)
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 10
            self.cmd_msg.vel_des = [0.4,0.0, 0.0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(3.5)
            
            # self.get_logger().info("sh开始循环")
            # # oni自定义步态6
            # for i in range(0,80):
            #     self.cmd_msg.mode =62
            #     self.cmd_msg.gait_id = 110
            #     self.cmd_msg.life_count += 1
            #     if self.cmd_msg.life_count >= 50:
            #         self.cmd_msg.life_count = 1
            #     self.Ctrl.Send_cmd(self.cmd_msg)
            #     self.get_logger().info("等待自定义步态完成")
            #     time.sleep(0.81)

#  第四关第一个通道
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
            time.sleep(6)

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
            time.sleep(10.7)

            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            self.Ctrl.Wait_finish(12, 0)
            time.sleep(3)




# 第五关回环路
            self.get_logger().info("第五关")
            self.get_logger().info("矫正方向")
            self.run_pid_align("ground", 200, vx=0)

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
                



            
            '''
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 120
            self.cmd_msg.vel_des = [0.2, 0, 0]
            self.cmd_msg.step_height = [0.06, 0.06]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count += 1
            self.Ctrl.Send_cmd(self.cmd_msg)
            time.sleep(19)


            turn_num = 0
            gaps = [0]
            adjust_timer = 0


            for i in range(0, 95600):  # 0.04s更新一下gap (建议后续改为while循环+ROS定时器，避免长时间阻塞)
                if self.jump_num ==0 :
                    tvalue = 0.076
                elif self.jump_num ==1 :
                    tvalue = 0.069
                elif  self.jump_num==2:
                    tvalue = 0.03
                elif self.jump_num ==3:
                    tvalue = 0.03
        
                if self.jump_num<=3 and self.jump_num>=1:
                    id = 2
                else:
                    id =120
                
                edge_num ,gap = map(int,self.adjust_gap.split())

                left ,right = map(float,self.fish_line.split())
                
                right = 1.9*right
                
                vx = 0.29  # 前进速度保持不变
                vy = 0.0    # 横移速度
                vyaw = 0.0  # 旋转速度
                if i<=500:
                    vx = 0.23
                if abs(gap) > 0:  # 缩小死区到5，或者你可以设为0看效果
                    gaps.append(gap)
                    vy =  gap/1800.0
                    if len(gaps)>10:
                        del gaps[0]
                        vyaw = (sum(gaps)/float(len(gaps)))/600.0
                    else:
                        vyaw = gap/700.0
                    
                    max_yaw = 0.13  # 例如
                    vyaw = max(-max_yaw, min(max_yaw, vyaw)) 

                else:
                    # 在死区内，缓慢衰减到0，而不是瞬间归零
                    vyaw = 0.0
                    vy = 0.0
                if self.jump_num >=1 and self.jump_num<=3:
                    vx = 0.5



                if self.jump_num>=1 and self.jump_num<=3:

                    vy = 0.00235
                    vyaw = -0.105

                    if self.jump_num ==2:
                        vy = 0.0024
                        vyaw = -0.09

                if ( (left>0.015 and left<=tvalue) or (right<=tvalue and right >0.015)) and self.jump_lock==0 : # 识别到某一边为空
                    turn_num +=1
                    if turn_num>=4:
                        ti = 0
                        if self.jump_num ==0:
                            ti = 2.1
                        elif self.jump_num ==1:
                            ti = 2.0
                        elif self.jump_num ==2:
                            ti = 1
                        elif self.jump_num ==3:
                            ti = 1

                        self.cmd_msg.mode = 11
                        self.cmd_msg.gait_id = id
                        self.cmd_msg.vel_des = [vx, vy, vyaw]
                        self.cmd_msg.step_height = [0.06, 0.06]
                        self.cmd_msg.duration = 0
                        self.cmd_msg.life_count += 1
                        if self.cmd_msg.life_count >= 50:
                            self.cmd_msg.life_count = 1
                        self.Ctrl.Send_cmd(self.cmd_msg)
                        time.sleep(ti)
                            # 恢复站立
                        self.cmd_msg.mode = 12
                        self.cmd_msg.vel_des = [0, 0, 0]
                        self.cmd_msg.gait_id = 0
                        self.cmd_msg.life_count += 1
                        self.Ctrl.Send_cmd(self.cmd_msg)
                        self.Ctrl.Wait_finish(12, 0)

                        if self.jump_num ==0:

                            self.get_logger().info(f"检测到应当跳转？？左？{left}")
                            self.cmd_msg.mode = 16
                            self.cmd_msg.gait_id = 0
                            self.cmd_msg.vel_des = [0, 0, 1]
                            self.cmd_msg.step_height = [0.04, 0.04]
                            self.cmd_msg.duration = 2000
                            self.cmd_msg.life_count += 1
                            self.Ctrl.Send_cmd(self.cmd_msg)
                            time.sleep(2.0)
                            self.jump_num +=1
                            self.jump_lock = 900

                        elif self.jump_num>=1:

                            self.get_logger().info(f"检测到应当跳转？？右？{right}")
                            self.cmd_msg.mode = 16
                            self.cmd_msg.gait_id = 3
                            self.cmd_msg.vel_des = [0, 0, -1]
                            self.cmd_msg.step_height = [0.04, 0.04]
                            self.cmd_msg.duration = 2000
                            self.cmd_msg.life_count += 1
                            self.Ctrl.Send_cmd(self.cmd_msg)
                            time.sleep(2.0)

                            # 恢复站立
                            self.cmd_msg.mode = 12
                            self.cmd_msg.vel_des = [0, 0, 0]
                            self.cmd_msg.gait_id = 0
                            self.cmd_msg.life_count += 1
                            self.Ctrl.Send_cmd(self.cmd_msg)
                            self.Ctrl.Wait_finish(12, 0)
                            turn_num = 0

                            if self.jump_num >=1 and self.jump_num<=2:
                                #转身微调：
                                self.cmd_msg.mode = 11
                                self.cmd_msg.gait_id = 120
                                self.cmd_msg.vel_des = [0, 0, 0.6]
                                self.cmd_msg.step_height = [0.06, 0.06]
                                self.cmd_msg.duration = 800
                                self.cmd_msg.life_count += 1
                                self.Ctrl.Send_cmd(self.cmd_msg)
                                time.sleep(5)

                            self.jump_num +=1
                            self.jump_lock = 900
                        # 恢复站立
                        self.cmd_msg.mode = 12
                        self.cmd_msg.vel_des = [0, 0, 0]
                        self.cmd_msg.gait_id = 0
                        self.cmd_msg.life_count += 1
                        self.Ctrl.Send_cmd(self.cmd_msg)
                        self.Ctrl.Wait_finish(12, 0)
                        turn_num = 0
                        adjust_timer = 0
                

                # 3. 统一发送指令，消除冗余代码
                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = id
                self.cmd_msg.vel_des = [vx, vy, vyaw]
                self.cmd_msg.step_height = [0.06, 0.06]
                self.cmd_msg.duration = 0
                self.cmd_msg.life_count += 1
                if self.cmd_msg.life_count >= 50:
                    self.cmd_msg.life_count = 1
                self.Ctrl.Send_cmd(self.cmd_msg)
                time.sleep(0.030)
                self.jump_lock = max(0,self.jump_lock - 1)
                # self.get_logger().info(f"jump_lock:{self.jump_lock}")
                adjust_timer +=1 
                
                if self.jump_num ==4:
                    # 3. 结束
                    self.cmd_msg.mode = 11
                    self.cmd_msg.gait_id = id
                    self.cmd_msg.vel_des = [0.3, vy, vyaw]
                    self.cmd_msg.step_height = [0.06, 0.06]
                    self.cmd_msg.duration = 0
                    self.cmd_msg.life_count += 1
                    if self.cmd_msg.life_count >= 50:
                        self.cmd_msg.life_count = 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    time.sleep(16)

                    # 恢复站立
                    self.cmd_msg.mode = 12
                    self.cmd_msg.vel_des = [0, 0, 0]
                    self.cmd_msg.gait_id = 0
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    self.Ctrl.Wait_finish(12, 0)
                    turn_num = 0

                    self.get_logger().info(f"准备结束")
                    self.cmd_msg.mode = 16
                    self.cmd_msg.gait_id = 3
                    self.cmd_msg.vel_des = [0, 0, -0.1]
                    self.cmd_msg.step_height = [0.1, 0.1]
                    self.cmd_msg.duration = 2000
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    time.sleep(2)

                    # 恢复站立
                    self.cmd_msg.mode = 12
                    self.cmd_msg.vel_des = [0, 0, 0]
                    self.cmd_msg.gait_id = 0
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    self.Ctrl.Wait_finish(12, 0)
                    turn_num = 0
                    
                    self.cmd_msg.mode = 11
                    self.cmd_msg.gait_id = 120
                    self.cmd_msg.vel_des = [0, 0, 0.6]
                    self.cmd_msg.step_height = [0.06, 0.06]
                    self.cmd_msg.duration = 800
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    time.sleep(5)

                    self.get_logger().info(f"前进一步")
                    self.cmd_msg.mode = 11
                    self.cmd_msg.gait_id = 120
                    self.cmd_msg.vel_des = [0.1, 0, 0]
                    self.cmd_msg.step_height = [0.06, 0.06]
                    self.cmd_msg.duration = 200
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    time.sleep(0.2)

                    # 恢复站立
                    self.cmd_msg.mode = 12
                    self.cmd_msg.vel_des = [0, 0, 0]
                    self.cmd_msg.gait_id = 0
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    self.Ctrl.Wait_finish(12, 0)
                    turn_num = 0

                    self.get_logger().info(f",跳下去")
                    self.cmd_msg.mode = 16
                    self.cmd_msg.gait_id = 1
                    self.cmd_msg.vel_des = [0.35, 0,0]
                    self.cmd_msg.step_height = [0.1, 0.1]
                    self.cmd_msg.duration = 3000
                    self.cmd_msg.life_count += 1
                    self.Ctrl.Send_cmd(self.cmd_msg)
                    time.sleep(3)
                    break'''





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
  
        







        
    def change_gait(self,deff,full,params):
            robot_cmd = {
        'mode':0, 'gait_id':0, 'contact':0, 'life_count':0,
        'vel_des':[0.0, 0.0, 0.0],
        'rpy_des':[0.0, 0.0, 0.0],
        'pos_des':[0.0, 0.0, 0.0],
        'acc_des':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'ctrl_point':[0.0, 0.0, 0.0],
        'foot_pose':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'step_height':[0.0, 0.0],
        'value':0,  'duration':0
        }
            steps = toml.load(params)
            full_steps = {'step':[robot_cmd]}########################################创建完整参数字典，并将第一个参数设置为全零0
            k =0
            for i in steps['step']:
                cmd = copy.deepcopy(robot_cmd)
                cmd['duration'] = i['duration']
                if i['type'] == 'usergait':                
                    cmd['mode'] = 11 # LOCOMOTION
                    cmd['gait_id'] = 110 # USERGAIT
                    cmd['vel_des'] = i['body_vel_des']
                    cmd['rpy_des'] = i['body_pos_des'][0:3]
                    cmd['pos_des'] = i['body_pos_des'][3:6]
                    cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                    cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                    cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                    cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                    cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                    cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                    cmd['acc_des'] = i['weight']
                    cmd['value'] = i['use_mpc_traj']
                    cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                    cmd['ctrl_point'][2] =  i['mu']
                if k == 0:
                    full_steps['step'] = [cmd]
                else:
                    full_steps['step'].append(cmd)
                k=k+1
            f = open(full, 'w')
            f.write("# Gait Params\n")
            f.writelines(toml.dumps(full_steps))#########################################################将完整参数写入_full文件
            f.close()

            file_obj_gait_def = open(deff,'r')
            file_obj_gait_params = open(full,'r')
            self.usergait_msg.data = file_obj_gait_def.read()
            self.lcm_usergait.publish("user_gait_file",self.usergait_msg.encode())
            time.sleep(0.5)
            self.usergait_msg.data = file_obj_gait_params.read()
            self.lcm_usergait.publish("user_gait_file",self.usergait_msg.encode())
            time.sleep(0.1)
            file_obj_gait_def.close()
            file_obj_gait_params.close()#############################################将自定义步态的定义和参数发布到机器人控制板（上传自定义步态信息）










def main():


    rclpy.init()
    node = masterNode()

    while True:
        terminal_input = input("输入 1 开始运动：").strip()
        if terminal_input == "1":
            node.master_thread = threading.Thread(target=node.master_line)
            node.master_thread.daemon = True
            node.master_thread.start()
            break

    # 用 spin_once 循环代替 spin，让主线程可以检查 master_line 是否结束
    while rclpy.ok() and node.master_thread.is_alive():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.Ctrl.quit()
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
