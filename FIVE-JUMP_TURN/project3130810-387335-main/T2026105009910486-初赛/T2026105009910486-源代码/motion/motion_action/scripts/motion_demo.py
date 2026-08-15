import sys
import time
import threading
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
import lcm

# 自定义消息
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt



class TimerBasedDisplay:
    """使用ROS2定时器在主线程显示图像 - 线程安全"""
    def __init__(self, node):
        self.node = node
        self.latest_frames = {}
        self.lock = threading.Lock()
        
        # 创建定时器，在主线程执行显示（30fps）
        self.timer = node.create_timer(0.033, self._display_callback)
        self.node.get_logger().info("显示定时器已创建")
    
    def update_frame(self, window_name, image):
        """更新最新帧（线程安全，可在回调中调用）"""
        with self.lock:
            if image is not None:
                self.latest_frames[window_name] = image.copy()
            else:
                self.latest_frames[window_name] = None
    
    def _display_callback(self):
        """定时器回调 - 在主线程执行，安全显示"""
        import cv2
        
        with self.lock:
            frames = self.latest_frames.copy()
        
        for window_name, image in frames.items():
            if image is not None:
                cv2.imshow(window_name, image)
        
        if frames:
            cv2.waitKey(1)


# 视觉识别超时参数
VISION_TIMEOUT = 8.0          # 视觉识别超时时间（秒）
BALL_DETECTION_THRESHOLD = 0.005  # 球检测阈值（只要有球就检测到）
MAX_NO_BALL_FRAMES = 100      # 连续未检测到球的最大帧数

# 固定路线参数（当视觉失败时使用）
FIXED_ROUTE_FORWARD1 = 3.0    # 固定路线第一段前进时间
FIXED_ROUTE_TURN_LEFT = 2.0   # 固定路线左转时间
FIXED_ROUTE_TURN_RIGHT = 2.0  # 固定路线右转时间
FIXED_ROUTE_FORWARD2 = 4.0    # 固定路线第二段前进时间


# 共用行走参数
GO_FORWARD_SHORT = 1.9   # 跳跃后短直走
GO_FORWARD_LINK = 2     # 左跳后衔接直走
GO_MERGE_START = 1.23     # 合并后赛段二起始直走（原GO_START1）

# 赛段二完整参数
GO_TURN1_LEFT = 2          # 左跳后直走
GO_TURN1_RIGHT = 2         # 右跳后直走
STAGE1_PRE_GO1 = 1.4
STAGE1_PRE_GO2=1.5
STAGE1_HIT1_LEFT = 4
STAGE1_BACK_CENTER = 4.3
STAGE1_BACK_CENTER2= 2.1
STAGE1_HIT2_RIGHT = 2.2
STAGE1_FORWARD = 5.1    
GO_STAGE2_2       = 4.5
GO_STAGE2_3       = 12.5
GO_STAGE2_4       = 4
STAGE2_HIT3_RIGHT = 3
STAGE2_BACK_CENTER = 3.3
STAGE2_BACK_CENTER2=0.68
STAGE2_MID_GO = 8.2
STAGE2_HIT4_LEFT = 2
GO_END = 1.2

FORWARD_SPEED = 0.32
HIT_FORWARD_SPEED = 0.22
HIT_SIDE_SPEED = 0.27
HIT_BACK_SPEED = 0.26


class Robot_Ctrl_12:
    def __init__(self):
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.running = 1
        self.rec_thread = Thread(target=self.rec_response)
        self.send_thread = Thread(target=self.send_publish)

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, c, d):
        m = robot_control_response_lcmt().decode(d)
        if m.order_process_bar >= 95:
            self.mode_ok = m.mode

    def rec_response(self):
        while self.running:
            self.lc_r.handle()
            time.sleep(0.002)

    def send_publish(self):
        while self.running:
            self.send_lock.acquire()
            if self.delay_cnt > 20:
                if self.cmd_msg.life_count >= 120:
                    self.cmd_msg.life_count = 1
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

    def Wait_finish(self, m, g):
        cnt = 0
        while self.running and cnt < 2000:
            if self.mode_ok == m:
                return True
            time.sleep(0.005)
            cnt += 1
        return False

    def quit(self):
        self.running = False
        self.rec_thread.join()
        self.send_thread.join()

class FullStageNode(Node):
    def __init__(self):
        super().__init__("full_stage_node")
        self.Ctrl = Robot_Ctrl_12()
        self.Ctrl.run()
        self.cmd_msg = robot_control_cmd_lcmt()
        self.cmd_msg.life_count = 0
        self.finished = False  # 完成标志
        self.get_logger().info("双赛段启动")

        self.t = Thread(target=self.run_all)
        self.t.daemon = True
        self.t.start()

    def send_raw(self, mode, gait, vx, vy, vyaw, step_h=[0.06,0.06], duration=0):
        self.cmd_msg.mode = mode
        self.cmd_msg.gait_id = gait
        self.cmd_msg.vel_des = [vx, vy, vyaw]
        self.cmd_msg.step_height = step_h
        self.cmd_msg.duration = duration
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count >= 120:
            self.cmd_msg.life_count = 1
        self.Ctrl.Send_cmd(self.cmd_msg)
        time.sleep(0.04)

    def stand(self):
        self.send_raw(12, 0, 0,0,0)
        self.Ctrl.Wait_finish(12, 0)

    def walk(self):
        for _ in range(10):
            self.send_raw(11, 10, 0,0,0)

    def go(self, t, speed=FORWARD_SPEED):
        self.walk()
        st = time.time()
        while time.time()-st < t:
            self.send_raw(11, 10, speed, 0, 0)
        self.send_raw(11, 10, 0,0,0)
        self.stand()

    def jump_left(self):
        self.stand()
        self.walk()
        st = time.time()
        while time.time()-st < 1.4:
            self.send_raw(16, 0, 0,0,0)
        self.stand()

    def jump_right(self):
        self.stand()
        self.walk()
        st = time.time()
        while time.time()-st < 1.4:
            self.send_raw(16, 3, 0,0,0)
        self.stand()

    def run_stage1(self):
        self.get_logger().info("===== 开始 赛段一：5次连续跳跃 =====")
        self.stand()
        time.sleep(1)

        # 起跳准备
        self.send_raw(11, 10, 0.22, 0, 0)
        time.sleep(0.3)

        # 5次跳跃
        for _ in range(5):
            self.send_raw(16, 1, 0,0,0, [0.05,0.05], 1000)
            time.sleep(1.5)
            self.send_raw(12, 0, 0,0,0)
            time.sleep(6.5)

        self.get_logger().info("5次跳跃完成，开始衔接路线")
        
        # 1. 往前走一段距离
        self.go(GO_FORWARD_SHORT)
        self.get_logger().info("直走完成")

        # 2. 原地左跳
        self.jump_left()
        self.get_logger().info("左跳完成")

        # 3. 往前走一段距离
        self.go(GO_FORWARD_LINK)
        self.get_logger().info("衔接直走完成 → 即将进入赛段二")
        
        self.stand()
        self.get_logger().info("===== 赛段一 全部完成 =====")
        time.sleep(1)

    # 视觉相关参数
    LOWER_ORANGE = None  # 在__init__中初始化
    UPPER_ORANGE = None
    LOWER_HIT = None
    UPPER_HIT = None
    MIN_BALL_AREA = 2200
    HIT_THRESHOLD = 0.0362
    RGAP_THRESHOLD = 0.01
    SWITCH_COOLDOWN_LEFT = 70
    SWITCH_COOLDOWN_RIGHT = 80

    def init_vision_params(self):
        """初始化视觉参数（延迟导入numpy）"""
        import numpy as np
        self.LOWER_ORANGE = np.array([10, 130, 80])
        self.UPPER_ORANGE = np.array([100, 210, 160])
        self.LOWER_HIT = np.array([10, 130, 80])
        self.UPPER_HIT = np.array([180, 210, 160])

    def init_vision_system(self):
        """初始化视觉系统 - 使用定时器显示方案"""
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        import numpy as np
        import cv2
        
        self.init_vision_params()
        self.bridge = CvBridge()
        self.left_ratio = 0.0
        self.right_ratio = 0.0
        self.left_hit = 0.0
        self.right_hit = 0.0
        self.fish_rgap = 0.0
        self.switch = 0
        self.vis_lock = threading.Lock()
        
        # 创建显示管理器（关键：使用定时器在主线程显示）
        self.display_mgr = TimerBasedDisplay(self)
        
        # 创建窗口
        cv2.namedWindow("Stage2 - Left Camera", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Stage2 - Left Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Stage2 - Right Camera", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Stage2 - Right Mask", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Stage2 - Left Camera", 640, 480)
        cv2.resizeWindow("Stage2 - Left Mask", 640, 480)
        cv2.resizeWindow("Stage2 - Right Camera", 640, 480)
        cv2.resizeWindow("Stage2 - Right Mask", 640, 480)
        
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(Image, "/ai_fisheye_left/image_raw", self.proc_left, qos)
        self.create_subscription(Image, "/ai_fisheye_right/image_raw", self.proc_right, qos)
        self.get_logger().info("双目鱼眼视觉系统初始化完成")

    def proc_left(self, msg):
        """左相机处理 - 添加可视化显示和超时检测"""
        try:
            import cv2
            import numpy as np
            
            # 如果视觉已禁用，直接返回
            if not getattr(self, 'vision_enabled', True):
                return
            
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, c = img.shape
            
            # 保存原始图像用于显示
            display = img.copy()
            
            # 裁剪处理区域
            img_cropped = img[:, int(w*4/9):]
            h_crop, w_crop, c = img_cropped.shape
            
            # 在显示图像上标注裁剪区域
            cv2.rectangle(display, (int(w*4/9), 0), (w, h), (0, 255, 0), 2)
            cv2.putText(display, "ROI", (int(w*4/9)+5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.LOWER_ORANGE, self.UPPER_ORANGE)
            mask_hit = cv2.inRange(hsv, self.LOWER_HIT, self.UPPER_HIT)
            
            # 形态学操作优化掩图
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            cnts_hit = cv2.findContours(mask_hit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

            area = 0
            if cnts:
                largest_cnt = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(largest_cnt)
                
                # 在显示图像上绘制轮廓
                x_offset = int(w*4/9)
                cnts_display = [cnt + np.array([[x_offset, 0]]) for cnt in cnts]
                cv2.drawContours(display, cnts_display, -1, (0, 255, 0), 2)
                
                # 绘制最大轮廓的边界框
                x, y, w_box, h_box = cv2.boundingRect(largest_cnt)
                cv2.rectangle(display, (x + x_offset, y), (x + x_offset + w_box, y + h_box), (0, 0, 255), 2)
                
                # 绘制中心点
                M = cv2.moments(largest_cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"]) + x_offset
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(display, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(display, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            
            area_hit = 0
            if cnts_hit:
                area_hit = cv2.contourArea(max(cnts_hit, key=cv2.contourArea))

            # 更新数值
            with self.vis_lock:
                self.left_ratio = area/(h_crop*w_crop) if area >= self.MIN_BALL_AREA else 0.0
                self.left_hit = area_hit/(h_crop*w_crop) if area_hit >= self.MIN_BALL_AREA else 0.0
                self.fish_rgap = self.left_ratio * 0.9 - self.right_ratio
            
            # 检测是否识别到球
            ball_detected = (self.left_ratio > BALL_DETECTION_THRESHOLD or 
                            self.right_ratio > BALL_DETECTION_THRESHOLD)
            
            if ball_detected:
                self.ball_detected_count = getattr(self, 'ball_detected_count', 0) + 1
                self.no_ball_frame_count = 0
            else:
                self.no_ball_frame_count = getattr(self, 'no_ball_frame_count', 0) + 1

            # 在显示图像上添加文字信息
            status_text = f"Area:{area:>6} Ratio:{self.left_ratio:.4f} Hit:{self.left_hit:.4f}"
            color = (0, 255, 0) if self.left_ratio > 0.005 else (0, 0, 255)
            cv2.putText(display, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(display, f"LEFT CAMERA", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 添加超时信息
            if getattr(self, 'vision_start_time', None):
                elapsed = time.time() - self.vision_start_time
                timeout_text = f"Time:{elapsed:.1f}s NoBall:{self.no_ball_frame_count}"
                cv2.putText(display, timeout_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # 创建全尺寸掩图用于显示
            mask_full = np.zeros((h, w), dtype=np.uint8)
            mask_full[:, int(w*4/9):] = mask

            # 通过显示管理器更新帧
            self.display_mgr.update_frame("Stage2 - Left Camera", display)
            self.display_mgr.update_frame("Stage2 - Left Mask", mask_full)

            self.get_logger().info(f"[左] 面积:{area:>6} | 占比:{self.left_ratio:.4f} | 未检测:{self.no_ball_frame_count}")
        except Exception as e:
            self.get_logger().error(f"左相机错误: {e}")

    def proc_right(self, msg):
        """右相机处理 - 添加可视化显示"""
        try:
            import cv2
            import numpy as np
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, c = img.shape
            
            # 保存原始图像用于显示
            display = img.copy()
            
            # 裁剪处理区域
            crop_width = int(w*5/9)
            img_cropped = img[:, :crop_width]
            h_crop, w_crop, c = img_cropped.shape
            
            # 在显示图像上标注裁剪区域
            cv2.rectangle(display, (0, 0), (crop_width, h), (0, 255, 0), 2)
            cv2.putText(display, "ROI", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.LOWER_ORANGE, self.UPPER_ORANGE)
            mask_hit = cv2.inRange(hsv, self.LOWER_HIT, self.UPPER_HIT)
            
            # 形态学操作优化掩图
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            cnts_hit = cv2.findContours(mask_hit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

            area = 0
            if cnts:
                largest_cnt = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(largest_cnt)
                
                # 在显示图像上绘制轮廓
                cv2.drawContours(display, cnts, -1, (0, 255, 0), 2)
                
                # 绘制最大轮廓的边界框
                x, y, w_box, h_box = cv2.boundingRect(largest_cnt)
                cv2.rectangle(display, (x, y), (x + w_box, y + h_box), (0, 0, 255), 2)
                
                # 绘制中心点
                M = cv2.moments(largest_cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(display, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(display, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            
            area_hit = 0
            if cnts_hit:
                area_hit = cv2.contourArea(max(cnts_hit, key=cv2.contourArea))

            # 更新数值
            with self.vis_lock:
                self.right_ratio = area/(h_crop*w_crop) if area >= self.MIN_BALL_AREA else 0.0
                self.right_hit = area_hit/(h_crop*w_crop) if area_hit >= self.MIN_BALL_AREA else 0.0
                self.fish_rgap = self.left_ratio * 0.9 - self.right_ratio

            # 在显示图像上添加文字信息
            status_text = f"Area:{area:>6} Ratio:{self.right_ratio:.4f} Hit:{self.right_hit:.4f}"
            color = (0, 255, 0) if self.right_ratio > 0.005 else (0, 0, 255)
            cv2.putText(display, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(display, f"RIGHT CAMERA", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 创建全尺寸掩图用于显示
            mask_full = np.zeros((h, w), dtype=np.uint8)
            mask_full[:, :crop_width] = mask

            # 通过显示管理器更新帧
            self.display_mgr.update_frame("Stage2 - Right Camera", display)
            self.display_mgr.update_frame("Stage2 - Right Mask", mask_full)

            self.get_logger().info(f"[右] 面积:{area:>6} | 占比:{self.right_ratio:.4f} | 撞击:{self.right_hit:.4f}")
        except Exception as e:
            self.get_logger().error(f"右相机错误: {e}")

    def vis_get(self):
        with self.vis_lock:
            return {'lr':self.left_ratio,'rr':self.right_ratio,'lh':self.left_hit,'rh':self.right_hit,'rgap':self.fish_rgap}

    def seek(self):
        """撞击逻辑- 带超时检测"""
        # 检查是否超时
        if getattr(self, 'vision_start_time', None) and not getattr(self, 'use_fixed_route', False):
            elapsed = time.time() - self.vision_start_time
            no_ball = getattr(self, 'no_ball_frame_count', 0)
            
            # 超时检查：时间超时 或 长时间未检测到球
            if elapsed > VISION_TIMEOUT or no_ball > MAX_NO_BALL_FRAMES:
                self.get_logger().warn(f"视觉识别超时！时间:{elapsed:.1f}s, 未检测帧:{no_ball}")
                self.get_logger().warn("切换到固定路线模式")
                self.use_fixed_route = True
                self.vision_enabled = False  # 禁用视觉
                return  # 立即返回，不执行视觉控制
        
        # 如果使用固定路线，直接返回
        if getattr(self, 'use_fixed_route', False):
            return
        
        d = self.vis_get()
        lh, rh = d['lh'], d['rh']
        rgap = d['rgap']
        self.get_logger().info(f"中线差:{rgap:.4f} | Lhit:{lh:.4f} | Rhit:{rh:.4f}")

        FORWARD_SPEED = 0.32
        HIT_FORWARD_SPEED = 0.22
        HIT_SIDE_SPEED = 0.27
        HIT_BACK_SPEED = 0.26

        if (lh <= self.HIT_THRESHOLD and rh <= self.HIT_THRESHOLD) or self.switch > 0:
            if rgap > self.RGAP_THRESHOLD:
                self.send_raw(11,10,FORWARD_SPEED, -rgap/40, -rgap/25)
            elif rgap < -self.RGAP_THRESHOLD:
                self.send_raw(11,10,FORWARD_SPEED, -rgap/50, -rgap/25)
            else:
                self.send_raw(11,10,FORWARD_SPEED,0,0)
            self.switch = max(0, self.switch-1)
        elif lh >= self.HIT_THRESHOLD and self.switch == 0:
            self.stand()
            self.send_raw(11,10,HIT_FORWARD_SPEED,HIT_SIDE_SPEED,0)
            time.sleep(4)
            self.send_raw(11,10,0,-HIT_BACK_SPEED,0)
            time.sleep(4)
            self.switch = self.SWITCH_COOLDOWN_LEFT
            self.stand()
        elif rh >= self.HIT_THRESHOLD and self.switch == 0:
            self.stand()
            self.send_raw(11,10,HIT_FORWARD_SPEED,-HIT_SIDE_SPEED,0)
            time.sleep(4)
            self.send_raw(11,10,0,HIT_BACK_SPEED,0)
            time.sleep(4)
            self.switch = self.SWITCH_COOLDOWN_RIGHT
            self.stand()

    def one_ball(self):
        """撞击一个球 - 带超时检测"""
        for i in range(260):
            # 如果切换到固定路线，提前退出
            if getattr(self, 'use_fixed_route', False):
                self.get_logger().warn("已切换到固定路线，停止one_ball")
                break
            self.seek()
            time.sleep(0.01)
        
        if not getattr(self, 'use_fixed_route', False):
            self.back()

    def two_balls(self):
        """撞击两个球（z1视觉版本）- 带超时检测"""
        self.switch=0
        self.one_ball()
        
        # 如果已经切换到固定路线，不再执行第二个球
        if getattr(self, 'use_fixed_route', False):
            return
        
        self.switch=0
        self.one_ball()
        
        if not getattr(self, 'use_fixed_route', False):
            self.go(2)  # AFTER_HIT_GO

    def run_fixed_route(self):
        """执行固定路线（当视觉识别失败时使用）"""
        self.get_logger().info("===== 开始执行固定路线 =====")
        
        # 固定路线：直走 -> 左转 -> 直走 -> 右转 -> 直走
        self.get_logger().info("固定路线：第一段直走")
        self.go(FIXED_ROUTE_FORWARD1)
        
        self.get_logger().info("固定路线：左转")
        self.send_raw(11, 10, 0, 0, 0.3)  # 原地左转
        time.sleep(FIXED_ROUTE_TURN_LEFT)
        self.stand()
        
        self.get_logger().info("固定路线：第二段直走")
        self.go(FIXED_ROUTE_FORWARD2)
        
        self.get_logger().info("固定路线：右转")
        self.send_raw(11, 10, 0, 0, -0.3)  # 原地右转
        time.sleep(FIXED_ROUTE_TURN_RIGHT)
        self.stand()
        
        self.get_logger().info("固定路线：第三段直走")
        self.go(FIXED_ROUTE_FORWARD1)
        
        self.get_logger().info("===== 固定路线执行完毕 =====")

    def close_vision_windows(self):
        """关闭赛道二的4个相机画面窗口"""
        try:
            import cv2
            cv2.destroyWindow("Stage2 - Left Camera")
            cv2.destroyWindow("Stage2 - Left Mask")
            cv2.destroyWindow("Stage2 - Right Camera")
            cv2.destroyWindow("Stage2 - Right Mask")
            self.get_logger().info("赛道二视觉窗口已关闭")
        except Exception as e:
            self.get_logger().warn(f"关闭视觉窗口时出错: {e}")


    def run_stage2(self):
        """赛段二"""
        self.get_logger().info("===== 开始 赛段二：撞球任务（带超时检测）=====")
        self.stand()
        time.sleep(1)

        # 初始化视觉系统
        self.init_vision_system()
        time.sleep(1)
        
        # 记录视觉开始时间
        self.vision_start_time = time.time()
        self.vision_enabled = True
        self.use_fixed_route = False
        self.no_ball_frame_count = 0
        self.ball_detected_count = 0

        # ====================== 合并点：赛段二开头直走 ======================
        self.go(GO_MERGE_START)
        
        # 继续赛段二原有路线
        self.jump_left()
        self.go(GO_TURN1_LEFT)
        self.jump_right()
        self.go(GO_TURN1_RIGHT)

        # 第一次撞球
        self.get_logger().info("===== 第一次撞球=====")
        self.two_balls()
        
        # 如果视觉失败，执行固定路线
        if getattr(self, 'use_fixed_route', False):
            self.get_logger().warn("第一次撞球视觉失败，执行固定路线")
            self.run_fixed_route()
        else:
            # 视觉成功，继续原有路线
            self.jump_left()
            self.go(GO_STAGE2_2)
            self.jump_left()
            self.go(GO_STAGE2_3)
            self.jump_right()
            self.go(GO_STAGE2_4)
            self.jump_right()

            # 第二次撞球
            self.get_logger().info("===== 第二次撞球=====")
            
            # 重置视觉状态
            self.vision_start_time = time.time()
            self.vision_enabled = True
            self.use_fixed_route = False
            self.no_ball_frame_count = 0
            
            self.two_balls()
            
            if getattr(self, 'use_fixed_route', False):
                self.get_logger().warn(" 第二次撞球视觉失败，执行固定路线")
                self.run_fixed_route()
        
        # 撞球完成后，直线走2s，然后衔接双线循迹
        self.get_logger().info("===== 撞球阶段完成，直线走2s后衔接双线循迹 =====")
        self.go(2.6, speed=0.20)
        
        # 关闭赛道二的视觉窗口
        self.close_vision_windows()
        
        self.get_logger().info("===== 赛段二 完成，准备进入双线循迹 =====")

   
    def run_all(self):
        try:
            self.run_stage1()   # 跳跃 + 直走 + 左跳 + 直走
            self.run_stage2()   # 无缝进入赛段二

            self.get_logger().info("全部赛段执行完毕！")
            # 赛段执行完毕，保持站立姿态，不趴下
            self.stand()
            time.sleep(0.5)
            # 不调用 Ctrl.quit()，保持LCM通信继续
            self.get_logger().info("12.py 执行完成，准备执行 345.py")
            self.finished = True  # 设置完成标志
            # 注意：不调用 destroy_node() 和 rclpy.shutdown()，保持ROS2上下文活跃

        except KeyboardInterrupt:
            self.send_raw(7,0,0,0,0)
            self.Ctrl.quit()
            self.finished = True
            self.destroy_node()
            rclpy.shutdown()

def run_stage12_and_345():
    
    rclpy.init()
    node = FullStageNode()
    
    # 使用 spin_once 循环
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
        
        # 12执行完成后，直接启动345的循迹
            if rclpy.ok() and node.finished:
                print("=" * 50)
                print("12执行完成，准备启动345循迹")
                print("=" * 50)
                # 从executor中移除12的节点
                executor.remove_node(node)
                # 确保12的LCM完全停止
                node.Ctrl.quit()
                print("[过渡] 等待2秒确保LCM资源释放...")
                time.sleep(2)
                # 启动345的循迹
                main()
            
    except KeyboardInterrupt:
        pass
    finally:
        if node.finished:
            pass  # 345已执行，不需要再移除节点
        else:
            executor.remove_node(node)
        
    print("12执行完成（spin退出）")



import sys
import cv2
import numpy as np
import lcm
import time
from threading import Lock, Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

sys.path.append("/home/cyberdog_ws/src/bridges/protocol/lcm")
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

class Robot_Ctrl_345(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_response)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.running = 1
        
        self.actual_vel_x = 0.0
        self.actual_vel_yaw = 0.0
        self.actual_roll = 0.0
        self.actual_pitch = 0.0
        self.actual_yaw = 0.0

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg.decode(data)
        if hasattr(self.rec_msg, 'vel_body'):
            self.actual_vel_x = self.rec_msg.vel_body[0]
            self.actual_vel_yaw = self.rec_msg.vel_body[2] if len(self.rec_msg.vel_body)>=3 else 0.0
        elif hasattr(self.rec_msg, 'odom_vel'):
            self.actual_vel_x = self.rec_msg.odom_vel[0]
            self.actual_vel_yaw = self.rec_msg.odom_vel[2] if len(self.rec_msg.odom_vel)>=3 else 0.0
        elif hasattr(self.rec_msg, 'base_vel'):
            self.actual_vel_x = self.rec_msg.base_vel[0]
            self.actual_vel_yaw = self.rec_msg.base_vel[2] if len(self.rec_msg.base_vel)>=3 else 0.0
        
        if hasattr(self.rec_msg, 'rpy'):
            self.actual_roll = self.rec_msg.rpy[0]
            self.actual_pitch = self.rec_msg.rpy[1]
            self.actual_yaw = self.rec_msg.rpy[2]
        elif hasattr(self.rec_msg, 'imu_rpy'):
            self.actual_roll = self.rec_msg.imu_rpy[0]
            self.actual_pitch = self.rec_msg.imu_rpy[1]
            self.actual_yaw = self.rec_msg.imu_rpy[2]

    def rec_response(self):
        while self.running:
            self.lc_r.handle()
            time.sleep(0.002)

    def send_publish(self):
        while self.running:
            self.send_lock.acquire()
            if self.delay_cnt > 20:
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def force_stand(self, duration=0, stable_time=2.0):
        print("[强制站立] 发送Mode12站立指令，开启自动防倾倒")
        for _ in range(5):
            ctrl_msg = robot_control_cmd_lcmt()
            ctrl_msg.mode = 12
            ctrl_msg.gait_id = 0
            ctrl_msg.duration = duration
            life_count = int(time.time()*100) % 100
            ctrl_msg.life_count = life_count
            self.Send_cmd(ctrl_msg)
            time.sleep(0.1)
        time.sleep(stable_time)
        print("[强制站立] 机身稳定完成，自动防倾倒已开启")
        return True

    def pitch_stand(self, target_pitch=0.25, duration=0, stable_time=2.0, max_wait=4.0):
        print(f"[抬头站立] 目标俯仰角:{target_pitch:.2f}rad，重心后移防前倾")
        start_time = time.time()
        while time.time() - start_time < max_wait:
            ctrl_msg = robot_control_cmd_lcmt()
            ctrl_msg.mode = 12
            ctrl_msg.gait_id = 0
            ctrl_msg.duration = duration
            ctrl_msg.rpy_des = [0.0, target_pitch, 0.0]
            life_count = int(time.time()*100) % 100
            ctrl_msg.life_count = life_count
            self.Send_cmd(ctrl_msg)
            time.sleep(0.05)
            if abs(self.actual_pitch - target_pitch) < 0.03:
                print(f"[抬头站立] 姿态达标，当前俯仰角:{self.actual_pitch:.2f}rad")
                break
        time.sleep(stable_time)
        print("[抬头站立] 重心后移完成，防前倾就绪")
        return True

    def anti_tip_protection(self, max_forward_tip=-0.35):
        if self.actual_pitch < max_forward_tip:
            print(f"[防翻倒警告] 前倾过大！当前俯仰角:{self.actual_pitch:.2f}rad，立即触发自动修正")
            self.force_stand(stable_time=1.0)
            return True
        return False

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.running = 0
        self.rec_thread.join()
        self.send_thread.join()

class CameraTrackNode(Node):
    def __init__(self, robot_ctrl):
        super().__init__("camera_track_node")
        self.bridge = CvBridge()
        self.robot_ctrl = robot_ctrl
        self.frame = None
        self.center_error = 0
        self.prev_error = 0
        self.track_center_x = None
        self.left_edge = None
        self.right_edge = None
        
        self.center_history = []
        self.max_history = 10
        self.track_width = 200

        self.trigger_lock = Lock()
        self.single_line_stable_count = 0
        self.SINGLE_LINE_STABLE_FRAME = 3
        self.is_stable_single_line = False
        self.prev_stable_single_line = False

        self.obstacle_align_stage = False
        self.obstacle_trigger_ready = False
        self.align_start_time = 0.0
        self.align_total_time = 0.0
        self.ALIGN_REQUIRE_TIME = 6.5
        self.ALIGN_SPEED = 0.06
        self.ALIGN_THRESHOLD = 4

        self.trigger_cool_down = 0
        self.TRIGGER_COOL_DOWN_FRAMES = 2500

        self.double_line_lost_count = 0
        self.MAX_LOST_FRAME = 20

        # 第四关模式
        self.level4_mode_active = False
        self.level4_phase = 0
        self.level4_start_time = None
        self.LEVEL4_STRAIGHT_SPEED = 0.20
        self.LEVEL4_STRAIGHT_TIME = 17.0  # 阶段1直线行走17秒
        
        # 阶段1黄色边线检测（只检测右边）
        self.yellow_right_pixels = 0
        self.YELLOW_RIGHT_THRESHOLD = 100  # 接近右边线的阈值（像素）
        self.YELLOW_DANGER_THRESHOLD = 200  # 危险阈值，需要偏移
        self.lateral_offset_active = False  # 是否正在偏移
        self.lateral_offset_start_time = 0.0  # 偏移开始时间
        self.LATERAL_OFFSET_DURATION = 1.0  # 偏移持续时间（秒）
        self.LATERAL_OFFSET_SPEED = -0.12  # 向左偏移速度（负值表示向左）
        
        # 阶段3浅蓝色正方体检测
        self.light_blue_cube_detected = False
        
        # 阶段5、7、9、11-18直线行走时间
        self.LEVEL4_PHASE5_TIME = 7.0  # 阶段5直线行走7秒
        self.LEVEL4_PHASE7_TIME = 9.5  # 阶段7直线行走9.5秒
        self.LEVEL4_PHASE9_TIME = 7.0  # 阶段9直线行走7秒
        self.LEVEL4_PHASE11_TIME = 15.0  # 阶段11直线行走15秒
        self.LEVEL4_PHASE12_TIME = 15.0  # 阶段12后退直线行走15秒
        self.LEVEL4_PHASE14_TIME = 7.0  # 阶段14直线行走7秒
        self.LEVEL4_PHASE16_TIME = 18.5  # 阶段16直线行走18.5秒
        self.LEVEL4_PHASE18_TIME = 9.0  # 阶段18直线行走9秒
        
        # 阶段11蓝色小球检测
        self.blue_ball_detected = False
        self.blue_ball_center_x = 0  # 蓝色小球中心位置
        self.blue_ball_center_y = 0  # 蓝色小球中心Y位置
        self.BLUE_BALL_THRESHOLD = 500  # 蓝色小球检测像素阈值
        self.blue_ball_history = []  # 小球位置历史，用于检测晃动
        self.blue_ball_shake_detected = False  # 检测到小球晃动
        self.blue_ball_shake_threshold = 20  # 晃动检测阈值（像素）
        
        # 越障恢复和回路模式参数
        self.obstacle_recover_start_time = None
        self.OBSTACLE_RECOVER_THRESHOLD = 35.0  # 35秒后触发回路模式
        self.loop_mode_active = False
        self.loop_mode_start_time = None
        self.loop_mode_phase = 0
        self.loop_mode_completed = False
        self.LOOP_MODE_SPEED = 0.20

        self.CAMERA_TOPIC = "/rgb_camera/image_raw"
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            Image, self.CAMERA_TOPIC, self.image_callback, self.qos_profile
        )
        self.get_logger().info(f"双黄线循迹 | 右单线循迹 | {self.ALIGN_REQUIRE_TIME}s边走边对准 已启动")

    def image_callback(self, msg: Image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_near_centerline()
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")
            self.frame = None
            self.center_error = 0

    def detect_near_centerline(self):
        if self.frame is None:
            self.center_error = 0
            return

        h, w = self.frame.shape[:2]
        display = self.frame.copy()
        
        roi_x_start, roi_x_end = 0, w
        roi_y_start = int(h * 2/3)
        roi_y_end = h
        roi = self.frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        
        cv2.rectangle(display, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 0), 3)
        img_center_x = w // 2
        cv2.line(display, (img_center_x, roi_y_start), (img_center_x, roi_y_end), (100, 100, 100), 1)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_lines = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"]) + roi_x_start
                    cy = int(M["m01"] / M["m00"]) + roi_y_start
                    valid_lines.append(cx)
        
        current_line_count = len(valid_lines)
        current_is_single_line_valid = False
        current_is_double_line_valid = False

        if current_line_count >= 2:
            best_pair = None
            min_diff = float('inf')
            # 回路模式使用250-400像素范围，其他模式使用200-400
            min_dist = 250 if self.loop_mode_active else 200
            max_dist = 400

            for i in range(len(valid_lines)):
                for j in range(i+1, len(valid_lines)):
                    x1, x2 = valid_lines[i], valid_lines[j]
                    dist = abs(x2 - x1)
                    if min_dist < dist < max_dist:
                        diff = abs(dist - self.track_width)
                        if diff < min_diff:
                            min_diff = diff
                            best_pair = sorted([x1, x2])
            
            if best_pair:
                current_is_double_line_valid = True
                self.left_edge, self.right_edge = best_pair
                self.track_width = abs(self.right_edge - self.left_edge)
                
                raw_center = (self.left_edge + self.right_edge) // 2
                self.center_history.append(raw_center)
                if len(self.center_history) > self.max_history:
                    self.center_history.pop(0)
                self.track_center_x = int(np.mean(self.center_history))
                
                self.prev_error = self.center_error
                self.center_error = img_center_x - self.track_center_x
                
                cv2.line(display, (self.left_edge, roi_y_start), (self.left_edge, roi_y_end), (0, 165, 255), 3)
                cv2.line(display, (self.right_edge, roi_y_start), (self.right_edge, roi_y_end), (0, 165, 255), 3)
                cv2.line(display, (self.track_center_x, roi_y_start), (self.track_center_x, roi_y_end), (0, 255, 255), 4)
                status = f"双线循迹 | 宽:{self.track_width} | 偏差:{self.center_error}"
                color = (0, 255, 0)
            else:
                self.center_error = 0
                status = "双线无效"
                color = (0, 0, 255)

        elif current_line_count == 1:
            # 回路模式不启用单线循迹
            if self.loop_mode_active:
                self.center_error = 0
                self.left_edge = None
                self.right_edge = None
                status = "无赛道线"
                color = (0, 0, 255)
            else:
                detected_x = valid_lines[0]
                current_is_single_line_valid = True
                estimated_left = detected_x - self.track_width
                raw_center = (estimated_left + detected_x) // 2

                self.center_history.append(raw_center)
                if len(self.center_history) > self.max_history:
                    self.center_history.pop(0)
                self.track_center_x = int(np.mean(self.center_history))
                self.left_edge = estimated_left
                self.right_edge = detected_x

                self.prev_error = self.center_error
                self.center_error = img_center_x - self.track_center_x

                cv2.line(display, (self.left_edge, roi_y_start), (self.left_edge, roi_y_end), (255, 165, 0), 2)
                cv2.line(display, (self.right_edge, roi_y_start), (self.right_edge, roi_y_end), (0, 165, 255), 3)
                cv2.line(display, (self.track_center_x, roi_y_start), (self.track_center_x, roi_y_end), (0, 255, 255), 4)

                status = f"单线循迹 | 偏差:{self.center_error} | 稳定帧:{self.single_line_stable_count}"
                color = (0, 165, 255)

        else:
            self.center_error = 0
            self.left_edge = None
            self.right_edge = None
            status = "无赛道线"
            color = (0, 0, 255)

        if current_is_single_line_valid:
            self.single_line_stable_count += 1
            if self.single_line_stable_count >= self.SINGLE_LINE_STABLE_FRAME:
                self.is_stable_single_line = True
        else:
            self.single_line_stable_count = 0
            self.is_stable_single_line = False

        if self.trigger_cool_down > 0:
            self.trigger_cool_down -= 1

        with self.trigger_lock:
            # 从单线识别到双线，立即进入第四关模式
            if (self.prev_stable_single_line 
                and current_is_double_line_valid 
                and self.trigger_cool_down == 0
                and not self.level4_mode_active):
                self.level4_mode_active = True
                self.level4_phase = 0
                self.level4_start_time = time.time()
                self.trigger_cool_down = self.TRIGGER_COOL_DOWN_FRAMES
                print(f"[触发成功] 稳定单线→双线切换，立即进入第四关模式！")

        self.prev_stable_single_line = self.is_stable_single_line

        # 检测黄色边线（阶段1使用）- RGB 255,255,0
        self.detect_yellow_edge(roi, roi_x_start, roi_y_start, display)
        
        # 检测浅蓝色正方体（阶段3使用）
        self.detect_light_blue_cube(roi, roi_x_start, roi_y_start, display)

        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        # 构建黄边显示信息（只显示右边）
        offset_display = "偏移中" if self.lateral_offset_active else ""
        cv2.putText(display, f"对准阶段:{self.obstacle_align_stage} 第四关:{self.level4_mode_active} 右边黄线:{self.yellow_right_pixels} {offset_display}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        cv2.putText(display, f"累计对准:{self.align_total_time:.2f}s 浅蓝方块:{self.light_blue_cube_detected} 俯仰:{self.robot_ctrl.actual_pitch:.2f}",
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        mask_full = np.zeros((h, w), dtype=np.uint8)
        mask_full[roi_y_start:roi_y_end, roi_x_start:roi_x_end] = mask
        cv2.imshow("Track Vision", display)
        cv2.imshow("Mask", mask_full)
        cv2.waitKey(1)

    def detect_yellow_edge(self, roi, roi_x_start, roi_y_start, display):
        """检测右边黄色边线（RGB 255,255,0），用于阶段1判断是否需要向左偏移"""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 黄色HSV范围
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 检测ROI全部区域
        h, w = roi.shape[:2]
        
        self.yellow_right_pixels = cv2.countNonZero(mask)
        
        # 在图像上绘制检测区域（整个ROI）
        cv2.rectangle(display, (roi_x_start, roi_y_start), 
                     (roi_x_start + w, roi_y_start + h), (0, 255, 255), 2)
        
        # 显示检测结果和像素计数
        status_text = f"黄线像素:{self.yellow_right_pixels}"
        if self.yellow_right_pixels > self.YELLOW_DANGER_THRESHOLD:
            cv2.putText(display, status_text + " 危险!", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        elif self.yellow_right_pixels > self.YELLOW_RIGHT_THRESHOLD:
            cv2.putText(display, status_text + " 接近", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        else:
            cv2.putText(display, status_text + " 安全", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    def detect_gray_pole(self, roi, roi_x_start, roi_y_start, display):
        """检测灰色限高杆，用于阶段3低姿态通过"""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 灰色HSV范围：低饱和度，中等明度
        # 色相可以是任意值（0-180），但饱和度要低，明度中等
        lower_gray = np.array([0, 0, 80])
        upper_gray = np.array([180, 50, 200])
        
        mask = cv2.inRange(hsv, lower_gray, upper_gray)
        
        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 检测ROI上半部分（限高杆通常在上方）
        h, w = roi.shape[:2]
        upper_roi = mask[:h//2, :]
        upper_gray_pixels = cv2.countNonZero(upper_roi)
        
        GRAY_DETECTION_THRESHOLD = 150
        self.gray_pole_detected = upper_gray_pixels > GRAY_DETECTION_THRESHOLD
        
        if self.gray_pole_detected:
            cv2.putText(display, f"灰杆:检测到({upper_gray_pixels})", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 2)
    
    def detect_light_blue_cube(self, roi, roi_x_start, roi_y_start, display):
        """检测浅蓝色正方体障碍物，用于阶段3触发阶段4"""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 浅蓝色HSV范围
        lower_light_blue = np.array([80, 50, 100])
        upper_light_blue = np.array([110, 255, 255])
        
        mask = cv2.inRange(hsv, lower_light_blue, upper_light_blue)
        
        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 检测ROI全部区域
        light_blue_pixels = cv2.countNonZero(mask)
        
        # 检测轮廓判断是否为正方体
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.light_blue_cube_detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # 面积阈值
                # 近似多边形
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                
                # 如果是四边形（正方体/长方体投影）
                if len(approx) >= 4:
                    self.light_blue_cube_detected = True
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(display, (roi_x_start + x, roi_y_start + y), 
                                 (roi_x_start + x + w, roi_y_start + y + h), (255, 255, 0), 2)
                    break
        
        if self.light_blue_cube_detected:
            cv2.putText(display, f"浅蓝方块:检测到({light_blue_pixels})", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 2)
    
    def detect_blue_ball(self):
        """检测蓝色小球，用于阶段11撞击"""
        if self.frame is None:
            self.blue_ball_detected = False
            self.blue_ball_center_x = 0
            self.blue_ball_center_y = 0
            return
        
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        
        # 蓝色HSV范围
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 检测轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        prev_x = self.blue_ball_center_x
        prev_y = self.blue_ball_center_y
        
        self.blue_ball_detected = False
        self.blue_ball_center_x = 0
        self.blue_ball_center_y = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.BLUE_BALL_THRESHOLD:
                # 计算中心点
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.blue_ball_detected = True
                    self.blue_ball_center_x = cx
                    self.blue_ball_center_y = cy
                    
                    # 检测小球晃动
                    if prev_x != 0 and prev_y != 0:
                        dx = abs(cx - prev_x)
                        dy = abs(cy - prev_y)
                        if dx > self.blue_ball_shake_threshold or dy > self.blue_ball_shake_threshold:
                            self.blue_ball_shake_detected = True
                            print(f"[蓝色小球] 检测到晃动! dx={dx}, dy={dy}")
                    
                    # 记录位置历史
                    self.blue_ball_history.append((cx, cy))
                    if len(self.blue_ball_history) > 10:
                        self.blue_ball_history.pop(0)
                    break

    def get_and_reset_trigger(self):
        with self.trigger_lock:
            if self.obstacle_trigger_ready:
                self.obstacle_trigger_ready = False
                self.align_total_time = 0.0
                return True
            return False

def main():
   
    Ctrl = Robot_Ctrl_345()
    Ctrl.run()
    ctrl_msg = robot_control_cmd_lcmt()
    life_count = 0

    print("[初始化] 机器狗已启动，强制站立稳定")
    Ctrl.force_stand(stable_time=1.0)

    track_node = CameraTrackNode(Ctrl)
    
    FORWARD_SPEED = 0.20
    TURN_KP = 0.004
    TURN_KD = 0.002
    MAX_TURN = 0.35
    NORMAL_STEP_HEIGHT = 0.06
    OBSTACLE_STEP_HEIGHT = 0.06
    OBSTACLE_SPEED = 0.20

    # 越障状态机
    OBSTACLE_STATE_NORMAL = 0
    OBSTACLE_STATE_PREPARE = 1
    OBSTACLE_STATE_JUMP = 2
    OBSTACLE_STATE_LANDING = 3
    OBSTACLE_STATE_RECOVER = 4

    obstacle_state = OBSTACLE_STATE_NORMAL
    obstacle_start_time = 0.0
    
    # 第四关后对准阶段标志
    post_level4_align_stage = False
    post_level4_align_start_time = 0.0
    post_level4_align_total_time = 0.0
    POST_LEVEL4_ALIGN_TIME = 9.0  # 对准9秒

    print("[启动] 赛道循迹开始 (Ctrl+C停止)")
    lost_count = 0
    MAX_LOST = 20
    
    try:
        while rclpy.ok():
            rclpy.spin_once(track_node, timeout_sec=0.002)

            Ctrl.anti_tip_protection()

            # ========== 第四关模式 ==========
            if track_node.level4_mode_active:
                # 阶段0: 旋转左跳
                if track_node.level4_phase == 0:
                    phase0_elapsed = time.time() - track_node.level4_start_time
                    if phase0_elapsed < 0.1:
                        print(f"[第四关-阶段0] 执行旋转左跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 0
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase0_elapsed < 1.5:
                        if int(phase0_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段0] 等待旋转左跳完成... {phase0_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段0完成，进入阶段1：直线行走325cm")
                        Ctrl.force_stand(stable_time=1.0)
                        # 切换到行走模式
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [track_node.LEVEL4_STRAIGHT_SPEED, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 1
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始直线行走")
                    continue
                
                # 阶段1: 0.2速度直线行走24秒（纯直线，不循迹）
                elif track_node.level4_phase == 1:
                    phase1_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase1_elapsed - int(phase1_elapsed)) < 0.05:
                        print(f"[第四关-阶段1] 直线行走中... {phase1_elapsed:.1f}s/{track_node.LEVEL4_STRAIGHT_TIME}s")
                    
                    if phase1_elapsed >= track_node.LEVEL4_STRAIGHT_TIME:
                        print(f"[第四关] 阶段1完成，进入阶段2：旋转右跳")
                        track_node.level4_phase = 2
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        # 纯直线行走，不循迹，不检测黄线
                        ctrl_msg.vel_des = [track_node.LEVEL4_STRAIGHT_SPEED, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段2: 旋转右跳
                elif track_node.level4_phase == 2:
                    phase2_elapsed = time.time() - track_node.level4_start_time
                    if phase2_elapsed < 0.1:
                        print(f"[第四关-阶段2] 执行旋转右跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 3
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase2_elapsed < 1.5:
                        if int(phase2_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段2] 等待旋转右跳完成... {phase2_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段2完成，进入阶段3：双线循迹")
                        Ctrl.force_stand(stable_time=1.0)
                        # 切换到行走模式
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 3
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始双线循迹")
                    continue
                
                # 阶段3: 直线行走，检测浅蓝色正方体后进入阶段4
                elif track_node.level4_phase == 3:
                    phase3_elapsed = time.time() - track_node.level4_start_time
                    
                    # 检测到浅蓝色正方体，进入阶段4
                    if track_node.light_blue_cube_detected:
                        print(f"[第四关] 检测到浅蓝色正方体，进入阶段4：旋转右跳")
                        track_node.level4_phase = 4
                        track_node.level4_start_time = time.time()
                        continue
                    
                    if abs(phase3_elapsed - int(phase3_elapsed)) < 0.05:
                        print(f"[第四关-阶段3] 直线行走中... 检测浅蓝色正方体")
                    
                    ctrl_msg.mode = 11
                    ctrl_msg.gait_id = 27
                    life_count = (life_count + 1) % 100
                    ctrl_msg.life_count = life_count
                    ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                    # 纯直线行走，不循迹
                    ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                    Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段4: 旋转右跳
                elif track_node.level4_phase == 4:
                    phase4_elapsed = time.time() - track_node.level4_start_time
                    if phase4_elapsed < 0.1:
                        print(f"[第四关-阶段4] 执行旋转右跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 3
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase4_elapsed < 1.5:
                        if int(phase4_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段4] 等待旋转右跳完成... {phase4_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段4完成，进入阶段5：直线行走7s")
                        Ctrl.force_stand(stable_time=1.0)
                        # 切换到行走模式
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 5
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始阶段5直线行走")
                    continue
                
                # 阶段5: 直线行走7s
                elif track_node.level4_phase == 5:
                    phase5_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase5_elapsed - int(phase5_elapsed)) < 0.05:
                        print(f"[第四关-阶段5] 直线行走中... {phase5_elapsed:.1f}s/{track_node.LEVEL4_PHASE5_TIME}s")
                    
                    if phase5_elapsed >= track_node.LEVEL4_PHASE5_TIME:
                        print(f"[第四关] 阶段5完成，进入阶段6：旋转左跳")
                        track_node.level4_phase = 6
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段6: 旋转左跳
                elif track_node.level4_phase == 6:
                    phase6_elapsed = time.time() - track_node.level4_start_time
                    if phase6_elapsed < 0.1:
                        print(f"[第四关-阶段6] 执行旋转左跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 0
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase6_elapsed < 1.5:
                        if int(phase6_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段6] 等待旋转左跳完成... {phase6_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段6完成，进入阶段7：直线行走4s")
                        Ctrl.force_stand(stable_time=1.0)
                        # 切换到行走模式
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 7
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始阶段7直线行走")
                    continue
                
                # 阶段7: 直线行走7s
                elif track_node.level4_phase == 7:
                    phase7_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase7_elapsed - int(phase7_elapsed)) < 0.05:
                        print(f"[第四关-阶段7] 直线行走中... {phase7_elapsed:.1f}s/{track_node.LEVEL4_PHASE7_TIME}s")
                    
                    if phase7_elapsed >= track_node.LEVEL4_PHASE7_TIME:
                        print(f"[第四关] 阶段7完成，进入阶段8：旋转左跳")
                        track_node.level4_phase = 8
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段8: 旋转左跳
                elif track_node.level4_phase == 8:
                    phase8_elapsed = time.time() - track_node.level4_start_time
                    if phase8_elapsed < 0.1:
                        print(f"[第四关-阶段8] 执行旋转左跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 0
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase8_elapsed < 1.5:
                        if int(phase8_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段8] 等待旋转左跳完成... {phase8_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段8完成，进入阶段9：直线行走4s")
                        Ctrl.force_stand(stable_time=1.0)
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 9
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始阶段9直线行走")
                    continue
                
                # 阶段9: 直线行走4s
                elif track_node.level4_phase == 9:
                    phase9_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase9_elapsed - int(phase9_elapsed)) < 0.05:
                        print(f"[第四关-阶段9] 直线行走中... {phase9_elapsed:.1f}s/{track_node.LEVEL4_PHASE9_TIME}s")
                    
                    if phase9_elapsed >= track_node.LEVEL4_PHASE9_TIME:
                        print(f"[第四关] 阶段9完成，进入阶段10：旋转右跳")
                        track_node.level4_phase = 10
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段10: 旋转右跳
                elif track_node.level4_phase == 10:
                    phase10_elapsed = time.time() - track_node.level4_start_time
                    if phase10_elapsed < 0.1:
                        print(f"[第四关-阶段10] 执行旋转右跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 3
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase10_elapsed < 1.5:
                        if int(phase10_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段10] 等待旋转右跳完成... {phase10_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段10完成，进入阶段11：寻找蓝色小球")
                        Ctrl.force_stand(stable_time=1.0)
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 11
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始阶段11寻找蓝色小球")
                    continue
                
                # 阶段11: 直线行走15s
                elif track_node.level4_phase == 11:
                    phase11_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase11_elapsed - int(phase11_elapsed)) < 0.05:
                        print(f"[第四关-阶段11] 直线行走中... {phase11_elapsed:.1f}s/{track_node.LEVEL4_PHASE11_TIME}s")
                    
                    if phase11_elapsed >= track_node.LEVEL4_PHASE11_TIME:
                        print(f"[第四关] 阶段11完成，进入阶段12：后退直线行走")
                        track_node.level4_phase = 12
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段12: 后退直线行走15s
                elif track_node.level4_phase == 12:
                    phase12_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase12_elapsed - int(phase12_elapsed)) < 0.05:
                        print(f"[第四关-阶段12] 后退直线行走中... {phase12_elapsed:.1f}s/{track_node.LEVEL4_PHASE12_TIME}s")
                    
                    if phase12_elapsed >= track_node.LEVEL4_PHASE12_TIME:
                        print(f"[第四关] 阶段12完成，进入阶段13：旋转右跳")
                        track_node.level4_phase = 13
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [-0.15, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段13: 旋转右跳
                elif track_node.level4_phase == 13:
                    phase13_elapsed = time.time() - track_node.level4_start_time
                    if phase13_elapsed < 0.1:
                        print(f"[第四关-阶段13] 执行旋转右跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 3
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase13_elapsed < 1.5:
                        if int(phase13_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段13] 等待旋转右跳完成... {phase13_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段13完成，进入阶段14：直线行走7s")
                        Ctrl.force_stand(stable_time=1.0)
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 14
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始阶段14直线行走")
                    continue
                
                # 阶段14: 直线行走7s
                elif track_node.level4_phase == 14:
                    phase14_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase14_elapsed - int(phase14_elapsed)) < 0.05:
                        print(f"[第四关-阶段14] 直线行走中... {phase14_elapsed:.1f}s/{track_node.LEVEL4_PHASE14_TIME}s")
                    
                    if phase14_elapsed >= track_node.LEVEL4_PHASE14_TIME:
                        print(f"[第四关] 阶段14完成，进入阶段15：旋转右跳")
                        track_node.level4_phase = 15
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段15: 旋转右跳
                elif track_node.level4_phase == 15:
                    phase15_elapsed = time.time() - track_node.level4_start_time
                    if phase15_elapsed < 0.1:
                        print(f"[第四关-阶段15] 执行旋转右跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 3
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase15_elapsed < 1.5:
                        if int(phase15_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段15] 等待旋转右跳完成... {phase15_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段15完成，进入阶段16：直线行走15.5s")
                        Ctrl.force_stand(stable_time=1.0)
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 16
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始阶段16直线行走")
                    continue
                
                # 阶段16: 直线行走15.5s
                elif track_node.level4_phase == 16:
                    phase16_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase16_elapsed - int(phase16_elapsed)) < 0.05:
                        print(f"[第四关-阶段16] 直线行走中... {phase16_elapsed:.1f}s/{track_node.LEVEL4_PHASE16_TIME}s")
                    
                    if phase16_elapsed >= track_node.LEVEL4_PHASE16_TIME:
                        print(f"[第四关] 阶段16完成，进入阶段17：旋转左跳")
                        track_node.level4_phase = 17
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段17: 旋转左跳
                elif track_node.level4_phase == 17:
                    phase17_elapsed = time.time() - track_node.level4_start_time
                    if phase17_elapsed < 0.1:
                        print(f"[第四关-阶段17] 执行旋转左跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 0
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase17_elapsed < 1.5:
                        if int(phase17_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段17] 等待旋转左跳完成... {phase17_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段17完成，进入阶段18：直线行走8s")
                        Ctrl.force_stand(stable_time=1.0)
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(0.5)
                        track_node.level4_phase = 18
                        track_node.level4_start_time = time.time()
                        print(f"[第四关] 已切换到行走模式，开始阶段18直线行走")
                    continue
                
                # 阶段18: 直线行走8s
                elif track_node.level4_phase == 18:
                    phase18_elapsed = time.time() - track_node.level4_start_time
                    
                    if abs(phase18_elapsed - int(phase18_elapsed)) < 0.05:
                        print(f"[第四关-阶段18] 直线行走中... {phase18_elapsed:.1f}s/{track_node.LEVEL4_PHASE18_TIME}s")
                    
                    if phase18_elapsed >= track_node.LEVEL4_PHASE18_TIME:
                        print(f"[第四关] 阶段18完成，进入阶段19：旋转左跳")
                        track_node.level4_phase = 19
                        track_node.level4_start_time = time.time()
                    else:
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                        ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                        Ctrl.Send_cmd(ctrl_msg)
                    continue
                
                # 阶段19: 旋转左跳
                elif track_node.level4_phase == 19:
                    phase19_elapsed = time.time() - track_node.level4_start_time
                    if phase19_elapsed < 0.1:
                        print(f"[第四关-阶段19] 执行旋转左跳")
                        ctrl_msg.mode = 16
                        ctrl_msg.gait_id = 0
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                    elif phase19_elapsed < 1.5:
                        if int(phase19_elapsed * 10) % 5 == 0:
                            print(f"[第四关-阶段19] 等待旋转左跳完成... {phase19_elapsed:.1f}s/1.5s")
                    else:
                        print(f"[第四关] 阶段19完成，退出第四关模式，进入对准阶段")
                        Ctrl.force_stand(stable_time=1.0)
                        track_node.level4_mode_active = False
                        track_node.level4_phase = 0
                        # 启动第四关后的对准阶段
                        post_level4_align_stage = True
                        post_level4_align_start_time = time.time()
                        post_level4_align_total_time = 0.0
                        print(f"[第四关后] 启动对准阶段，目标{POST_LEVEL4_ALIGN_TIME}s")
                    continue

            # ========== 第四关后对准阶段 ==========
            if post_level4_align_stage:
                post_level4_align_total_time = time.time() - post_level4_align_start_time
                
                if abs(post_level4_align_total_time - int(post_level4_align_total_time)) < 0.05:
                    print(f"[第四关后-对准] 低速前进对准 累计:{post_level4_align_total_time:.2f}s/{POST_LEVEL4_ALIGN_TIME}s 偏差:{track_node.center_error}")
                
                if post_level4_align_total_time >= POST_LEVEL4_ALIGN_TIME:
                    print(f"[第四关后-对准完成] 满{POST_LEVEL4_ALIGN_TIME}s，触发越障！")
                    post_level4_align_stage = False
                    obstacle_state = OBSTACLE_STATE_PREPARE
                else:
                    ctrl_msg.mode = 11
                    ctrl_msg.gait_id = 27
                    life_count = (life_count + 1) % 100
                    ctrl_msg.life_count = life_count
                    ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                    
                    # 有偏差就边走边转向对准
                    if track_node.center_error != 0 and track_node.left_edge is not None:
                        p_term = track_node.center_error * TURN_KP
                        d_term = (track_node.center_error - track_node.prev_error) * TURN_KD
                        turn_speed = np.clip(p_term + d_term, -MAX_TURN, MAX_TURN)
                        ctrl_msg.vel_des = [0.06, 0.0, turn_speed]
                    else:
                        ctrl_msg.vel_des = [0.06, 0.0, 0.0]
                    
                    Ctrl.Send_cmd(ctrl_msg)
                continue

            # ========== 越障状态机 ==========
            if obstacle_state != OBSTACLE_STATE_NORMAL:
                if obstacle_state == OBSTACLE_STATE_PREPARE:
                    print("[越障][1/4] 准备阶段：抬头稳定机身")
                    Ctrl.pitch_stand(target_pitch=0.25, stable_time=1.0)
                    obstacle_start_time = time.time()
                    obstacle_state = OBSTACLE_STATE_JUMP
                    continue

                elif obstacle_state == OBSTACLE_STATE_JUMP:
                    if time.time() - obstacle_start_time < 15.0:
                        print(f"[越障][2/4] 高抬腿循迹中... 剩余: {15.0 - (time.time() - obstacle_start_time):.1f}s")
                        ctrl_msg.mode = 11
                        ctrl_msg.gait_id = 27
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        ctrl_msg.step_height = [OBSTACLE_STEP_HEIGHT, OBSTACLE_STEP_HEIGHT]
                        ctrl_msg.rpy_des = [0.0, 0.25, 0.0]

                        if track_node.center_error != 0 and track_node.left_edge is not None:
                            p_term = track_node.center_error * TURN_KP
                            d_term = (track_node.center_error - track_node.prev_error) * TURN_KD
                            turn_speed = np.clip(p_term + d_term, -MAX_TURN, MAX_TURN)
                            ctrl_msg.vel_des = [OBSTACLE_SPEED, 0.0, turn_speed]
                        elif track_node.center_error != 0:
                            turn_speed = np.clip(track_node.center_error * TURN_KP * 1.2, -MAX_TURN, MAX_TURN)
                            ctrl_msg.vel_des = [OBSTACLE_SPEED, 0.0, turn_speed]
                        else:
                            ctrl_msg.vel_des = [OBSTACLE_SPEED, 0.0, 0.0]

                        Ctrl.Send_cmd(ctrl_msg)
                    else:
                        obstacle_state = OBSTACLE_STATE_LANDING
                    continue

                elif obstacle_state == OBSTACLE_STATE_LANDING:
                    print("[越障][3/4] 越障完成，过渡稳定")
                    time.sleep(1.0)
                    obstacle_state = OBSTACLE_STATE_RECOVER
                    continue

                elif obstacle_state == OBSTACLE_STATE_RECOVER:
                    print("[越障][4/4] 恢复正常站立")
                    Ctrl.force_stand(stable_time=1.0)
                    obstacle_state = OBSTACLE_STATE_NORMAL
                    print("[越障] 全流程完成，进入35秒恢复倒计时")
                    # 启动回路模式计时
                    track_node.obstacle_recover_start_time = time.time()
                    continue

            # ========== 越障恢复倒计时 ==========
            if track_node.obstacle_recover_start_time is not None and not track_node.loop_mode_completed:
                recover_duration = time.time() - track_node.obstacle_recover_start_time
                
                # 实时显示恢复倒计时
                if abs(recover_duration - int(recover_duration)) < 0.05:
                    remaining = track_node.OBSTACLE_RECOVER_THRESHOLD - recover_duration
                    print(f"[越障恢复计时] {recover_duration:.1f}s/{track_node.OBSTACLE_RECOVER_THRESHOLD}s, 还剩{remaining:.1f}s触发回路")
                
                # 检查是否达到35秒阈值，触发回路模式
                if recover_duration >= track_node.OBSTACLE_RECOVER_THRESHOLD and not track_node.loop_mode_active:
                    print(f"[回路模式] 越障恢复超过{track_node.OBSTACLE_RECOVER_THRESHOLD}秒，进入回路模式！")
                    track_node.loop_mode_active = True
                    track_node.loop_mode_start_time = time.time()
                    track_node.loop_mode_phase = 0
                
                # 回路模式运行中
                if track_node.loop_mode_active:
                    loop_elapsed = time.time() - track_node.loop_mode_start_time
                    
                    # 阶段0: 直走7.5秒（不循迹）
                    if track_node.loop_mode_phase == 0:
                        if abs(loop_elapsed - int(loop_elapsed)) < 0.05:
                            expected_distance = track_node.LOOP_MODE_SPEED * loop_elapsed
                            print(f"[回路模式-阶段0] 已行进:{expected_distance*100:.1f}cm ({loop_elapsed:.1f}s/7.5s)")

                        if loop_elapsed >= 7.5:
                            print(f"[回路模式] 阶段0完成，进入阶段1：旋转左跳")
                            track_node.loop_mode_phase = 1
                            track_node.loop_mode_start_time = time.time()
                        else:
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                            ctrl_msg.vel_des = [0.11, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段1: 旋转左跳
                    elif track_node.loop_mode_phase == 1:
                        phase1_elapsed = time.time() - track_node.loop_mode_start_time
                        if phase1_elapsed < 0.1:
                            print(f"[回路模式-阶段1] 执行旋转左跳")
                            ctrl_msg.mode = 16
                            ctrl_msg.gait_id = 0
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            Ctrl.Send_cmd(ctrl_msg)
                        elif phase1_elapsed < 1.5:
                            if int(phase1_elapsed * 10) % 5 == 0:
                                print(f"[回路模式-阶段1] 等待旋转左跳完成... {phase1_elapsed:.1f}s/1.5s")
                        else:
                            print(f"[回路模式] 阶段1完成，进入阶段2：双线循迹")
                            Ctrl.force_stand(stable_time=0.5)
                            track_node.loop_mode_phase = 2
                            track_node.loop_mode_start_time = time.time()
                        continue
                    
                    # 阶段2: 双线循迹35秒
                    elif track_node.loop_mode_phase == 2:
                        track_elapsed = time.time() - track_node.loop_mode_start_time

                        if track_elapsed >= 30.0:
                            print(f"[回路模式] 阶段2完成，进入阶段3：直线行走")
                            track_node.loop_mode_phase = 3
                            track_node.loop_mode_start_time = time.time()
                        else:
                            if abs(track_elapsed - int(track_elapsed)) < 0.05:
                                print(f"[回路模式-阶段2] 双线循迹中... {track_elapsed:.1f}s/30s")
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]

                            if track_node.center_error != 0 and track_node.left_edge is not None:
                                p_term = track_node.center_error * TURN_KP
                                d_term = (track_node.center_error - track_node.prev_error) * TURN_KD
                                turn_speed = np.clip(p_term + d_term, -MAX_TURN, MAX_TURN)
                                ctrl_msg.vel_des = [track_node.LOOP_MODE_SPEED, 0.0, turn_speed]
                            else:
                                # 回路模式不启用单线循迹，没识别到双线就原地踏步等待
                                ctrl_msg.vel_des = [0.0, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段3: 0.2速度直线行走5秒
                    elif track_node.loop_mode_phase == 3:
                        phase3_elapsed = time.time() - track_node.loop_mode_start_time

                        if phase3_elapsed >= 5.0:
                            print(f"[回路模式] 阶段3完成，进入阶段4：旋转右跳")
                            track_node.loop_mode_phase = 4
                            track_node.loop_mode_start_time = time.time()
                        else:
                            if abs(phase3_elapsed - int(phase3_elapsed)) < 0.05:
                                print(f"[回路模式-阶段3] 直线行走中... {phase3_elapsed:.1f}s/5s")
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                            ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段4: 旋转右跳
                    elif track_node.loop_mode_phase == 4:
                        phase4_elapsed = time.time() - track_node.loop_mode_start_time
                        if phase4_elapsed < 0.1:
                            print(f"[回路模式-阶段4] 执行旋转右跳")
                            ctrl_msg.mode = 16
                            ctrl_msg.gait_id = 3
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            Ctrl.Send_cmd(ctrl_msg)
                        elif phase4_elapsed < 1.5:
                            if int(phase4_elapsed * 10) % 5 == 0:
                                print(f"[回路模式-阶段4] 等待旋转右跳完成... {phase4_elapsed:.1f}s/1.5s")
                        else:
                            print(f"[回路模式] 阶段4完成，进入阶段5：双线循迹")
                            Ctrl.force_stand(stable_time=0.5)
                            track_node.loop_mode_phase = 5
                            track_node.loop_mode_start_time = time.time()
                        continue
                    
                    # 阶段5: 双线循迹25秒
                    elif track_node.loop_mode_phase == 5:
                        track_elapsed = time.time() - track_node.loop_mode_start_time

                        if track_elapsed >= 25.0:
                            print(f"[回路模式] 阶段5完成，进入阶段6：直线行走")
                            track_node.loop_mode_phase = 6
                            track_node.loop_mode_start_time = time.time()
                        else:
                            if abs(track_elapsed - int(track_elapsed)) < 0.05:
                                print(f"[回路模式-阶段5] 双线循迹中... {track_elapsed:.1f}s/25s")
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]

                            if track_node.center_error != 0 and track_node.left_edge is not None:
                                p_term = track_node.center_error * TURN_KP
                                d_term = (track_node.center_error - track_node.prev_error) * TURN_KD
                                turn_speed = np.clip(p_term + d_term, -MAX_TURN, MAX_TURN)
                                ctrl_msg.vel_des = [track_node.LOOP_MODE_SPEED, 0.0, turn_speed]
                            else:
                                # 回路模式不启用单线循迹，没识别到双线就原地踏步等待
                                ctrl_msg.vel_des = [0.0, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段6: 0.2速度直线行走5秒
                    elif track_node.loop_mode_phase == 6:
                        phase6_elapsed = time.time() - track_node.loop_mode_start_time

                        if phase6_elapsed >= 5.0:
                            print(f"[回路模式] 阶段6完成，进入阶段7：旋转右跳")
                            track_node.loop_mode_phase = 7
                            track_node.loop_mode_start_time = time.time()
                        else:
                            if abs(phase6_elapsed - int(phase6_elapsed)) < 0.05:
                                print(f"[回路模式-阶段6] 直线行走中... {phase6_elapsed:.1f}s/5s")
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                            ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段7: 旋转右跳
                    elif track_node.loop_mode_phase == 7:
                        phase7_elapsed = time.time() - track_node.loop_mode_start_time
                        if phase7_elapsed < 0.1:
                            print(f"[回路模式-阶段7] 执行旋转右跳")
                            ctrl_msg.mode = 16
                            ctrl_msg.gait_id = 3
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            Ctrl.Send_cmd(ctrl_msg)
                        elif phase7_elapsed < 1.5:
                            if int(phase7_elapsed * 10) % 5 == 0:
                                print(f"[回路模式-阶段7] 等待旋转右跳完成... {phase7_elapsed:.1f}s/1.5s")
                        else:
                            print(f"[回路模式] 阶段7完成，进入阶段8：双线循迹")
                            Ctrl.force_stand(stable_time=0.5)
                            track_node.loop_mode_phase = 8
                            track_node.loop_mode_start_time = time.time()
                        continue
                    
                    # 阶段8: 双线循迹25秒
                    elif track_node.loop_mode_phase == 8:
                        track_elapsed = time.time() - track_node.loop_mode_start_time

                        if track_elapsed >= 25.0:
                            print(f"[回路模式] 阶段8完成，进入阶段9：直线行走")
                            track_node.loop_mode_phase = 9
                            track_node.loop_mode_start_time = time.time()
                        else:
                            if abs(track_elapsed - int(track_elapsed)) < 0.05:
                                print(f"[回路模式-阶段8] 双线循迹中... {track_elapsed:.1f}s/25s")
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]

                            if track_node.center_error != 0 and track_node.left_edge is not None:
                                p_term = track_node.center_error * TURN_KP
                                d_term = (track_node.center_error - track_node.prev_error) * TURN_KD
                                turn_speed = np.clip(p_term + d_term, -MAX_TURN, MAX_TURN)
                                ctrl_msg.vel_des = [track_node.LOOP_MODE_SPEED, 0.0, turn_speed]
                            else:
                                # 回路模式不启用单线循迹，没识别到双线就原地踏步等待
                                ctrl_msg.vel_des = [0.0, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段9: 0.2速度直线行走5秒
                    elif track_node.loop_mode_phase == 9:
                        phase9_elapsed = time.time() - track_node.loop_mode_start_time

                        if phase9_elapsed >= 5.0:
                            print(f"[回路模式] 阶段9完成，进入阶段10：旋转右跳")
                            track_node.loop_mode_phase = 10
                            track_node.loop_mode_start_time = time.time()
                        else:
                            if abs(phase9_elapsed - int(phase9_elapsed)) < 0.05:
                                print(f"[回路模式-阶段9] 直线行走中... {phase9_elapsed:.1f}s/5s")
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
                            ctrl_msg.vel_des = [0.20, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段10: 旋转右跳
                    elif track_node.loop_mode_phase == 10:
                        phase10_elapsed = time.time() - track_node.loop_mode_start_time
                        if phase10_elapsed < 0.1:
                            print(f"[回路模式-阶段10] 执行旋转右跳")
                            ctrl_msg.mode = 16
                            ctrl_msg.gait_id = 3
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            Ctrl.Send_cmd(ctrl_msg)
                        elif phase10_elapsed < 1.5:
                            if int(phase10_elapsed * 10) % 5 == 0:
                                print(f"[回路模式-阶段10] 等待旋转右跳完成... {phase10_elapsed:.1f}s/1.5s")
                        else:
                            print(f"[回路模式] 阶段10完成，进入阶段11：双线循迹")
                            Ctrl.force_stand(stable_time=0.5)
                            track_node.loop_mode_phase = 11
                            track_node.loop_mode_start_time = time.time()
                        continue
                    
                    # 阶段11: 双线循迹25秒
                    elif track_node.loop_mode_phase == 11:
                        track_elapsed = time.time() - track_node.loop_mode_start_time

                        if track_elapsed >= 25.0:
                            print(f"[回路模式] 阶段11完成，进入阶段12：原地趴下")
                            track_node.loop_mode_phase = 12
                        else:
                            if abs(track_elapsed - int(track_elapsed)) < 0.05:
                                print(f"[回路模式-阶段11] 双线循迹中... {track_elapsed:.1f}s/25s")
                            ctrl_msg.mode = 11
                            ctrl_msg.gait_id = 120
                            life_count = (life_count + 1) % 100
                            ctrl_msg.life_count = life_count
                            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]

                            if track_node.center_error != 0 and track_node.left_edge is not None:
                                p_term = track_node.center_error * TURN_KP
                                d_term = (track_node.center_error - track_node.prev_error) * TURN_KD
                                turn_speed = np.clip(p_term + d_term, -MAX_TURN, MAX_TURN)
                                ctrl_msg.vel_des = [track_node.LOOP_MODE_SPEED, 0.0, turn_speed]
                            else:
                                # 回路模式不启用单线循迹，没识别到双线就原地踏步等待
                                ctrl_msg.vel_des = [0.0, 0.0, 0.0]
                            Ctrl.Send_cmd(ctrl_msg)
                        continue

                    # 阶段12: 原地趴下并退出
                    elif track_node.loop_mode_phase == 12:
                        print("[回路模式] 阶段12：原地趴下，整个程序结束")
                        ctrl_msg = robot_control_cmd_lcmt()
                        ctrl_msg.mode = 7
                        ctrl_msg.gait_id = 1
                        life_count = (life_count + 1) % 100
                        ctrl_msg.life_count = life_count
                        Ctrl.Send_cmd(ctrl_msg)
                        time.sleep(2)
                        print("[程序结束] 机器狗已趴下，退出运行")
                        break

            # 正常循迹PD控制（等待单线→双线触发第四关）
            ctrl_msg.mode = 11
            ctrl_msg.gait_id = 27
            life_count = (life_count + 1) % 100
            ctrl_msg.life_count = life_count

            ctrl_msg.step_height = [NORMAL_STEP_HEIGHT, NORMAL_STEP_HEIGHT]
            if track_node.center_error != 0 and track_node.left_edge is not None:
                p_term = track_node.center_error * TURN_KP
                d_term = (track_node.center_error - track_node.prev_error) * TURN_KD
                turn_speed = np.clip(p_term + d_term, -MAX_TURN, MAX_TURN)
                
                # 优化：减小速度变化幅度，使行走更连续
                abs_err = abs(track_node.center_error)
                if abs_err > 30:
                    current_speed = 0.12
                elif abs_err > 15:
                    current_speed = 0.15
                elif abs_err > 6:
                    current_speed = 0.18
                else:
                    current_speed = FORWARD_SPEED
                
                ctrl_msg.vel_des = [current_speed, 0.0, turn_speed]
                lost_count = 0
                
            elif track_node.center_error != 0:
                # 单线循迹时也保持一定速度
                turn_speed = np.clip(track_node.center_error * TURN_KP * 1.2, -MAX_TURN, MAX_TURN)
                ctrl_msg.vel_des = [0.12, 0.0, turn_speed]
                lost_count = 0
                
            else:
                lost_count += 1
                if lost_count < MAX_LOST:
                    scan_turn = 0.2 if (lost_count//5)%2 ==0 else -0.2
                    ctrl_msg.vel_des = [0.0, 0.0, scan_turn]
                else:
                    ctrl_msg.vel_des = [0.0, 0.0, 0.0]

            Ctrl.Send_cmd(ctrl_msg)

    except KeyboardInterrupt:
        print("\n[停止] 机器狗趴下")
        ctrl_msg = robot_control_cmd_lcmt()
        ctrl_msg.mode = 7
        ctrl_msg.gait_id = 1
        life_count = (life_count + 1) % 100
        ctrl_msg.life_count = life_count
        Ctrl.Send_cmd(ctrl_msg)
        time.sleep(2)

    finally:
        Ctrl.quit()
        track_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def run_stage345():
    main()


if __name__ == '__main__':
    
    print("=" * 50)
    print("=" * 50)
    run_stage12_and_345()
