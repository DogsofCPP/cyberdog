import time
from threading import Lock, Thread
import copy
import lcm
from std_msgs.msg import String
import rclpy
from cyberdog_interfaces.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from cyberdog_interfaces.robot_control_response_lcmt import robot_control_response_lcmt
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
##from pyzbar.pyzbar import decode
#from geometry_msgs.msg import Twist


def apply_fisheye_distortion_fast(img, strength=1.0):
    """
    高效版：将正常图片添加鱼眼畸变效果
    strength: 畸变强度，越大鱼眼效果越明显，推荐 0.3~2.0
    """
    h, w = img.shape[:2]
    
    # 中心点
    cx, cy = w / 2.0, h / 2.0
    
    # 归一化半径
    r_max = min(cx, cy)
    
    # 生成输出图像的所有像素坐标网格
    u_out, v_out = np.meshgrid(np.arange(w), np.arange(h))
    
    # 归一化到 [-1, 1] 范围
    x = (u_out - cx) / r_max
    y = (v_out - cy) / r_max
    
    r = np.sqrt(x**2 + y**2)
    
    # ===== 鱼眼畸变公式 =====
    # 径向畸变: r_d = r * (1 + k1*r^2 + k2*r^4)
    # 这里简化为 r_d = r * (1 + strength * r^2)
    r_dst = r * (1.0 + strength * r * r)
    
    # 计算缩放因子，避免除零
    scale = np.where(r > 1e-6, r_dst / r, 1.0)
    
    # 映射回原图坐标
    map_x = (x * scale * r_max + cx).astype(np.float32)
    map_y = (y * scale * r_max + cy).astype(np.float32)
    
    # 使用 remap 快速重映射
    dst = cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    
    return dst

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__("camera_opencv_node")


        #ne订阅相机的话题
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 )
        
        self.subscription = self.create_subscription(
            Image,
            "/rgb_camera/image_ground",
            # '/image_rgb',
            self.image_callback,      
            qos_profile )
        self.subscription 

        #将gap发布到h主文件：
        self.publisher_ = self.create_publisher(String, 'ground_topic', 10)
        self.timer = self.create_timer(0.04, self.adjust_pub_callback)
        self.Gap = 0
        self.edge_num = 0
        self.gap_history = []

        #create a mode_ok_sub
        self.seek_subscription = self.create_subscription(String,'master_mode_topic',self.mode_ok_callback,10)
        self.seek_subscription
        self.get_logger().info(f"Subscribing to topic: seek_topic")
        self.mode_ok = 0


        self.get_logger().info("Camera OpenCV Node started!")

    def mode_ok_callback(self,msg):
        self.mode_ok = int(msg.data)
        if self.mode_ok ==7:
            self.destroy_node()
            self.get_logger().warning("Adjust_node is about to be destoryed!!!")
            self.destory_node()
            sys.exit()


    def adjust_pub_callback(self):
        msg = String()
        msg.data = '%d %d' %(self.edge_num, self.Gap) # 传输的数据： 边界数，空格，gap大小
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Adjust_node::: edge_num:{self.edge_num},gap:{self.Gap}")
        #self.get_logger().info('Publishing: "%s"' % msg.data)


    def denoise_and_sharpen(self,img):
        # 轻微高斯去噪
        blurred = cv2.GaussianBlur(img, (3,3), 0)
        # 非锐化掩模
        sharp = cv2.addWeighted(img, 1.5, blurred, -0.5, 0)
        return sharp



    

#####识别小球##########################################################################################################3


    def image_callback(self,msg: Image):
        try:
            imge = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if imge is not None:
                #imge = self.denoise_and_sharpen(imge)
                #imge = cv2.GaussianBlur(imge, (5, 5), 0)
                #分析图片，获得n边界掩模
                h = imge.shape[0]         
                w = imge.shape[1]

                # 截取下半部分（从中间行开始到末尾）
                # img = imge[(h*2)//3:h, :]  # 取下半1/3
                dp = copy.deepcopy(imge)
                #####开始识别图像####黄色跑道在hsv状态下为绿色
                hsv_image = cv2.cvtColor(imge,cv2.COLOR_BGR2HSV)
                lower_yellow = np.array([30, 30, 30])
                upper_yellow = np.array([90, 255, 255])
                mask_image = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

                contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= 2000]
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                


                #self.get_logger().info(f"real:{len(contours)},self: {self.edge_num}")
                gap =0 # hih初始化间隙
                image_center_x = mask_image.shape[1]//2
                
                if contours is not None:
                    centers_x = []
                    for cnt in contours:
                        # print(f"边界大小{cv2.contourArea(cnt)}") 
                        M = cv2.moments(cnt)
                        if M['m00'] != 0:
                            cx = int(M['m10'] / M['m00'])
                            centers_x.append(cx)#计算两轮廓的中心坐标

                    #遍历所有，根据道路宽度找到最合适的两对边界：
                    min_diff = 1000
                    best_pair = None
                    for i in range(len(centers_x)):
                        for j in range(i+1, len(centers_x)):
                            x1, x2 = centers_x[i], centers_x[j]
                            dist = abs(x2 - x1)
                            # 过滤：双线宽度需在 80~400 像素之间
                            if 500 < dist < 1100:
                                print(f"当前dist为：{dist}")
                                diff = abs(dist - 300)
                                if diff < min_diff:
                                    min_diff = diff
                                    best_pair = sorted([x1, x2])

                    if best_pair is not None:
                        lane_center_x = (best_pair[0] + best_pair[1])//2
                        #计算偏差
                        gap = image_center_x - lane_center_x 
                        self.gap_history.append(gap)
                        if len(self.gap_history) > 7:
                            del self.gap_history[0]
                        if len(self.gap_history) > 0:
                            self.Gap = sum(self.gap_history) / len(self.gap_history)
                        else:
                            self.Gap = 0.0 # 或者其他默认值

                         # 3. 绘制识别到的左右边界（橙色）


                    else:
                        if len(contours) == 1:
                            gap = centers_x[0] - image_center_x
                            self.gap_history.append(gap)
                            if len(self.gap_history) > 5:
                                del self.gap_history[0]
                            if len(self.gap_history) > 0:
                                self.Gap = sum(self.gap_history) / len(self.gap_history)
                            else:
                                self.Gap = 0.0 # 或者其他默认值
                        
                        else:
                            if len(self.gap_history) > 0:
                                self.Gap = sum(self.gap_history) / len(self.gap_history)
                            else:
                                self.Gap = 0.0 # 或者其他默认值

                else:
                    if len(self.gap_history) > 0:
                        self.Gap = sum(self.gap_history) / len(self.gap_history)
                    else:
                        self.Gap = 0.0 # 或者其他默认值
                
                self.edge_num = len(contours)


                # ===== 可视化（PID 调参用，调试完毕后可注释掉） =====
                vis = dp.copy()
                # 轮廓重心竖线（蓝）
                for idx, cx in enumerate(centers_x):
                    cv2.line(vis, (cx, 0), (cx, vis.shape[0]-1), (255, 0, 0), 2)
                    cx_gap = image_center_x - cx
                    label = f"gap:{cx_gap:+d}"
                    label_x = max(5, min(cx + 4, vis.shape[1] - 105))
                    label_y = 88 - (idx % 3) * 18
                    label_y = max(18, min(label_y, vis.shape[0] - 8))
                    cv2.rectangle(vis, (label_x - 2, label_y - 13),
                                  (label_x + 95, label_y + 4),
                                  (255, 255, 255), -1)
                    cv2.putText(vis, label, (label_x, label_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
                # 图片中心线（白）
                cv2.line(vis, (image_center_x, 0), (image_center_x, vis.shape[0]-1), (255, 255, 255), 2)
                # 道路中心线（红）
                if best_pair is not None:
                    cv2.line(vis, (lane_center_x, 0), (lane_center_x, vis.shape[0]-1), (0, 0, 255), 2)
                    # 车道边界线（绿虚线）
                    cv2.line(vis, (best_pair[0], 0), (best_pair[0], vis.shape[0]-1), (0, 255, 0), 2)
                    cv2.line(vis, (best_pair[1], 0), (best_pair[1], vis.shape[0]-1), (0, 255, 0), 2)
                # 绘制检测到的轮廓（黄色）
                cv2.drawContours(vis, contours, -1, (0, 255, 255), 2)
                # gap 数值叠加
                gap_color = (0, 255, 0) if abs(self.Gap) < 20 else (0, 255, 255) if abs(self.Gap) < 80 else (0, 0, 255)
                cv2.putText(vis, f"Gap: {self.Gap:.1f} px", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, gap_color, 2)
                cv2.putText(vis, f"Edges: {self.edge_num}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                # gap 水平方向指示条
                bar_y = vis.shape[0] - 20
                bar_center = vis.shape[1] // 2
                bar_len = int(abs(self.Gap) * 3)  # 缩放
                bar_len = min(bar_len, vis.shape[1] // 2 - 10)
                if self.Gap > 0:
                    cv2.line(vis, (bar_center, bar_y), (bar_center + bar_len, bar_y), gap_color, 4)
                    cv2.circle(vis, (bar_center + bar_len, bar_y), 5, gap_color, -1)
                elif self.Gap < 0:
                    cv2.line(vis, (bar_center, bar_y), (bar_center - bar_len, bar_y), gap_color, 4)
                    cv2.circle(vis, (bar_center - bar_len, bar_y), 5, gap_color, -1)
                cv2.circle(vis, (bar_center, bar_y), 4, (255, 255, 255), -1)
                cv2.putText(vis, "0", (bar_center - 10, bar_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                # 显示
                cv2.imshow('Ground Camera - Lane Detection', vis)
                cv2.waitKey(1)
                # ===== 可视化结束 =====


            else:
                if len(self.gap_history) > 0:
                    self.Gap = sum(self.gap_history) / len(self.gap_history)
                else:
                    self.Gap = 0.0 # 或者其他默认值
                self.edge_num = 0

        except Exception as e:
            self.get_logger().error(f"image_callback function failded!!!: {str(e)}")
        


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()

    rclpy.spin(node)

    node.Ctrl.quit()
    node.destroy_node()
    rclpy.shutdown()


# Main function
if __name__ == "__main__":
    main()
