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

K = np.array([[280.0, 0.0, 320.0],
              [0.0, 280.0, 240.0],
              [0.0, 0.0, 1.0]], dtype=np.float32)
D = np.array([[-0.03, 0.01, -0.002, 0.001]], dtype=np.float32)

def undistort_fisheye(image, K, D, balance=1.0, output_size=None):
    """
    对鱼眼图像进行去畸变，返回校正后的透视图像。

    参数：
        image : np.ndarray
            输入的鱼眼图像 (BGR 或灰度)。
        K : np.ndarray (3x3)
            相机内参矩阵。
        D : np.ndarray (1x4 或 4,)
            鱼眼畸变系数 [k1, k2, k3, k4]。
        balance : float, 0.0~1.0
            调节输出图像的视野与有效像素比例。
            0.0 保留全视野（可能包含黑边）；1.0 完全裁剪黑边（损失部分视野）。
            默认 1.0 给出无黑边的最大矩形图像。
        output_size : tuple (width, height), 可选
            输出图像尺寸。如果未指定，则使用输入图像尺寸。

    返回：
        undistorted : np.ndarray
            去畸变后的图像，尺寸为 output_size（或输入尺寸）。
    """
    if output_size is None:
        h, w = image.shape[:2]
    else:
        w, h = output_size

    # 计算新的内参矩阵，balance 控制有效区域
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), np.eye(3), balance=balance
    )

    # 生成映射表（只需计算一次，若参数固定可缓存）
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2
    )

    # 重映射得到去畸变图像
    undistorted = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__("camera_opencv_node")


        #ne订阅相机的话题left_fisheye
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 )
        self.subscription = self.create_subscription(
            Image,
            "/image_left",
            self.left_callback,      
            qos_profile )
        self.subscription 

        #ne订阅相机的话题right_fisheye
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 )
        self.subscription = self.create_subscription(
            Image,
            "/image_right",
            self.right_callback,      
            qos_profile )
        self.subscription 


        #将rratio and lratio发布到h主文件：
        self.publisher_hit = self.create_publisher(String, 'hit_ratio_topic', 10)
        self.timer = self.create_timer(0.04, self.whether_hit_callback)
        self.left_ratio_hit =0
        self.right_ratio_hit =0

        #将fish_line发布到h主文件：
        self.publisher_line = self.create_publisher(String, 'fish_line_topic', 10)
        self.timer = self.create_timer(0.04, self.fish_line_callback)
        self.left_line = 0
        self.right_line = 0

        #将fish_rgap发布到h主文件：
        self.publisher_adjust = self.create_publisher(String, 'fish_rgap_topic', 10)
        self.timer = self.create_timer(0.04, self.fish_rgap_callback)
        self.fish_rgap = 0
        self.left_ratio = 0
        self.right_ratio = 0

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

    def whether_hit_callback(self):
        msg = String()
        msg.data = '%.4f %.4f' %( self.left_ratio_hit,self.right_ratio_hit) # 传输的数据： right_ratio_hit 空格 left_ratio_hit
        self.publisher_hit.publish(msg)
       
    def fish_rgap_callback(self):
        msg = String()
        msg.data = '%.4f' %( self.fish_rgap*100) # 传输的数据： fish_rgap
        self.publisher_adjust.publish(msg)

    def fish_line_callback(self):
        msg = String()
        msg.data = '%.4f %.4f' %( self.left_line,self.right_line) # 传输的数据： right_ratio_hit 空格 left_ratio_hit
        self.publisher_line.publish(msg)
        #self.get_logger().info(f"(l and r): {self.left_line}、{self.right_line}")
    

#####自动调整##########################################################################################################3
    
    def left_callback(self,msg: Image):
        try:
            imge = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if imge is not None:

                le = imge.shape[1]
                imge = imge[:, le*4//9 : le]
                h = imge.shape[0]
                l = imge.shape[1]
                imge_l = copy.deepcopy(imge[(h*3)//4:h, ])

                #####开始识别图像####橙色小球在hsv状态下为橙色
                hsv_image = cv2.cvtColor(imge,cv2.COLOR_BGR2HSV)
                hsv_line = cv2.cvtColor(imge_l,cv2.COLOR_BGR2HSV)

                lower_ball = np.array([10,130, 80])
                upper_ball = np.array([180, 210, 160])

                lower_orange = np.array([10,130, 80])
                upper_orange = np.array([100, 210, 160])

                lower_yellow = np.array([30, 30, 30])
                upper_yellow = np.array([90, 255, 255])

                mask_image = cv2.inRange(hsv_image, lower_ball, upper_ball)
                mask_hit = cv2.inRange(hsv_image, lower_orange, upper_orange)
                mask_line = cv2.inRange(hsv_line, lower_yellow, upper_yellow)

                contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_hit, _ = cv2.findContours(mask_hit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_line, _ = cv2.findContours(mask_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # cv2.imshow('fisheye_left',mask_line)
                # cv2.waitKey(1)
                #self.get_logger().info(f"real:{len(contours)},self: {self.edge_num}")
                di =0 # hit初始化间隙

                if contours:
                    cont_max = max(contours, key=cv2.contourArea)  # 直接取最大
                    orange_center = cv2.moments(cont_max)
                    if orange_center["m00"] == 0:
                        return
                    area = cv2.contourArea(cont_max) # 求面积
                    h, w = mask_image.shape[:2]
                    if area>=2200:
                        ra = area / (h * w) # 求掩膜占图片的比例
                        self.left_ratio = ra
                else:
                    self.left_ratio = 0

                if contours_hit:
                    cont_max = max(contours_hit, key=cv2.contourArea)  # 直接取最大
                    orange_center = cv2.moments(cont_max)
                    if orange_center["m00"] == 0:
                        pass
                    area = cv2.contourArea(cont_max) # 求面积
                    h, w = mask_hit.shape[:2]
                    ra_hit = area / (h * w) # 求掩膜占图片的比例
                    self.left_ratio_hit = ra_hit
                else:
                    self.left_ratio_hit = 0

                if contours_line:
                    cont_max = max(contours_line, key=cv2.contourArea)  # 直接取最大
                    line_center = cv2.moments(cont_max)
                    if line_center["m00"] == 0:
                        return
                    area = cv2.contourArea(cont_max) # 求面积
                    h, w = mask_line.shape[:2]
                    line_ra = area / (h * w) # 求掩膜占图片的比例
                    self.left_line = line_ra

                else:
                    self.left_line = 0

            else:
                self.left_line = 0
            self.fish_rgap = self.left_ratio*0.9 - self.right_ratio

        except Exception as e:
            self.get_logger().error(f"image_callback function failded!!!: {str(e)}")
        

    
    def right_callback(self,msg: Image):
        try:
            imge = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if imge is not None:

                le = imge.shape[1]
                imge = imge[:, 0 : le*5//9]
                h = imge.shape[0]
                l = imge.shape[1]
                imge_l = copy.deepcopy(imge[(h*3)//4:h,  ])

                #####开始识别图像####橙色小球在hsv状态下为橙色
                hsv_image = cv2.cvtColor(imge,cv2.COLOR_BGR2HSV)
                hsv_line = cv2.cvtColor(imge_l,cv2.COLOR_BGR2HSV)

                lower_ball = np.array([10,130, 80])
                upper_ball = np.array([180, 210, 160])

                lower_orange = np.array([10,130, 80])
                upper_orange = np.array([100, 210, 160])

                lower_yellow = np.array([30, 30, 30])
                upper_yellow = np.array([90, 255, 255])

                mask_image = cv2.inRange(hsv_image, lower_ball, upper_ball)
                mask_hit = cv2.inRange(hsv_image, lower_orange, upper_orange)
                mask_line = cv2.inRange(hsv_line, lower_yellow, upper_yellow)


                # cv2.imshow('fisheye_right',mask_line)
                # cv2.waitKey(1)
                contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_hit, _ = cv2.findContours(mask_hit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_line, _ = cv2.findContours(mask_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # cv2.imshow('fisheye_left',mask_image)
                # cv2.waitKey(1)
                #self.get_logger().info(f"real:{len(contours)},self: {self.edge_num}")
                di =0 # hih初始化间隙

                if contours:
                    cont_max = max(contours, key=cv2.contourArea)  # 直接取最大
                    orange_center = cv2.moments(cont_max)
                    if orange_center["m00"] == 0:
                        return
                    area = cv2.contourArea(cont_max) # 求面积
                    h, w = mask_image.shape[:2]
                    if area >=2200:
                        ra = area / (h * w) # 求掩膜占图片的比例
                        self.right_ratio = ra
                else:
                    self.right_ratio = 0

                if contours_hit:
                    cont_max = max(contours_hit, key=cv2.contourArea)  # 直接取最大
                    orange_center = cv2.moments(cont_max)
                    if orange_center["m00"] == 0:
                        pass
                    area = cv2.contourArea(cont_max) # 求面积
                    h, w = mask_hit.shape[:2]
                    ra_hit = area / (h * w) # 求掩膜占图片的比例
                    self.right_ratio_hit = ra_hit
                else:
                    self.right_ratio_hit = 0

                if contours_line:
                    cont_max = max(contours_line, key=cv2.contourArea)  # 直接取最大
                    line_center = cv2.moments(cont_max)
                    if line_center["m00"] == 0:
                        return
                    area = cv2.contourArea(cont_max) # 求面积
                    h, w = mask_line.shape[:2]
                    line_ra = area / (h * w) # 求掩膜占图片的比例
                    self.right_line = line_ra

                else:
                    self.right_line = 0

            else:
                self.right_line = 0
                   
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