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
            "/rgb_camera/image_raw",
            self.image_callback,      
            qos_profile )
        self.subscription 

        #将gap发布到h主文件：
        self.publisher_ = self.create_publisher(String, 'seek_001_topic', 10)
        self.timer = self.create_timer(0.04, self.orange_pub_callback)
        self.dist = 0
        self.ratio = 0

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


    def orange_pub_callback(self):
        msg = String()
        msg.data = '%.4f %d' %( self.ratio,self.dist) # 传输的数据： ratio，空格，distance大小
        self.publisher_.publish(msg)
        self.get_logger().info(f"Distance and ratio: {self.dist},{self.ratio}")
        #self.get_logger().info('Publishing: "%s"' % msg.data)


    def denoise_and_sharpen(self,img):
        # 轻微高斯去噪
        blurred = cv2.GaussianBlur(img, (3,3), 0)
        # 非锐化掩模
        sharp = cv2.addWeighted(img, 1.5, blurred, -0.5, 0)
        return sharp

    

#####自动调整##########################################################################################################3
    
    def image_callback(self,msg: Image):
        try:
            imge = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if imge is not None:
                #imge = self.denoise_and_sharpen(imge)

                #imge = cv2.GaussianBlur(imge, (5, 5), 0)
                #分析图片，获得n边界掩模
                # h = imge.shape[0]         
                # w = imge.shape[1]
                # imge = imge[:, w//2 : w]
                # 截取下半部分（从中间行开始到末尾）
                #####开始识别图像####橙色小球在hsv状态下为橙色
                hsv_image = cv2.cvtColor(imge,cv2.COLOR_BGR2HSV)
                lower_orange = np.array([10,130, 80])
                upper_orange = np.array([100, 210, 160])
                # cv2.imshow('Seek_orange',hsv_image)
                # cv2.waitKey(1)
                mask_image = cv2.inRange(hsv_image, lower_orange, upper_orange)

                contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # cv2.imshow('Seek_orange',mask_image)
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
                    ra = area / (h * w) # 求掩膜占图片的比例

                    orange_x = int(orange_center["m10"] / orange_center["m00"])
                    image_center_x = mask_image.shape[1]//2  
                    di = image_center_x - orange_x

                else:
                    di = 0
                    ra = 0

            else:
                di = 0
                ra = 0

            self.dist = di
            self.ratio = ra

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