

import os
import lcm
import sys
lcm_dir = os.path.join(os.path.dirname(__file__), 'lcm')
if lcm_dir not in sys.path:
    sys.path.insert(0, lcm_dir) 
import time
import toml
import copy
import math
import threading
import numpy as np 
from simulator_lcmt import *
from robot_control_response_lcmt import *
from robot_control_cmd_lcmt import *

# 基础路径
base_path = "./lcm"

# 添加模块所在路径到sys.path
sys.path.append(base_path)

# 文件路径
robot_control_cmd_lcmt_path = os.path.join(base_path, "robot_control_cmd_lcmt")
file_send_lcmt_path = os.path.join(base_path, "file_send_lcmt")
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from file_send_lcmt import file_send_lcmt

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
    
class Pos_msg(object):
    def __init__(self, data_lock):
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.rec_msg = simulator_lcmt()
        self.lc_r.subscribe("simulator_state", self.msg_handler)
        self.data_lock = data_lock
        self.position = [0,0,0]
        self.rpy=[0.0,0.0,0.0]
        self.quat = [0.0, 0.0, 0.0, 0.0]

    def run(self):
        while True:
            self.lc_r.handle()  # 持续处理消息

    def msg_handler(self, channel, data):
        self.rec_msg = simulator_lcmt().decode(data)
        with self.data_lock:
            self.position[:] = self.rec_msg.p[:]
            self.rec_msg.rpy=list(self.rec_msg.rpy)
            for i in range(len(self.rec_msg.rpy)):
                self.rec_msg.rpy[i]=self.rec_msg.rpy[i]*180/math.pi
            flag=True if abs((abs(self.rec_msg.rpy[0])-0))<abs((abs(self.rec_msg.rpy[0])-180)) else False
            self.rpy=self.rec_msg.rpy[:]
            if (flag==False):
                self.rpy[2]=self.rpy[2]+180
            self.quat = self.rec_msg.quat[:]

class Gait_msg(object):
    def __init__(self, data_lock):
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.rec_msg = robot_control_response_lcmt()
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.data_lock = data_lock
        self.gait_mode=[0,0]

    def run(self):
        while True:
            self.lc_r.handle()  # 持续处理消息

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        # print('order_process_bar:', self.rec_msg.mode, self.rec_msg.order_process_bar)
        with self.data_lock:
            self.gait_mode = [self.rec_msg.gait_id, self.rec_msg.mode]


def generate_waypoints():
    
    waypoints = []
    
    # print("A")
    x, y = 0.0,0.0  


    for i in range(10):
        x = 0.0 + i * (3.0 / 9)
        y = 0.0
        yaw = -3.1
        waypoints.append((x, y, yaw))#############################################################0-9

    for i in range(3):
        x = 3.0
        y = 0.3 + i * 0.325             # 0.3, 0.625, 0.95
        yaw = 1.57
        waypoints.append((x, y, yaw))################################################################10-12
    
    for i in range(5):
        x = 3.0 - i * 0.3625
        y = 0.95
        yaw = 3.1
        waypoints.append((x, y, yaw))#############################################################13-17 @3-@4

    for i in range(3):
        x = 1.55
        y = 0.95 + i * 0.19
        yaw = 1.57
        waypoints.append((x, y, yaw))#############################################################19-21 @4-@5

    for i in range(3):
        x = 1.55 - i * 0.325
        y = 1.33
        yaw = 1.57
        waypoints.append((x, y, yaw))#############################################################22-24 @5-@6

    for i in range(3):
        x = 0.9 + i * 0.325
        y = 1.33
        yaw = 1.57
        waypoints.append((x, y, yaw))#############################################################25-27 @6-@7

    for i in range(3):
        x = 1.55 - i * 0.025
        y = 1.33 + i * 0.435
        yaw = 1.57
        waypoints.append((x, y, yaw))#############################################################28-30 @7-@8

    for i in range(3):
        x = 1.48 + i * 0.175
        y = 2.2
        yaw = 1.57
        waypoints.append((x, y, yaw))############################################################# 31-33@8-@9    
    
    for i in range(3):
        x = 1.83 - i * 0.175
        y = 2.2
        yaw = 1.57
        waypoints.append((x, y, yaw))############################################################# 34-36 @9-@10

    for i in range(2):
        x = 1.48
        y = 2.2 + i * 0.4
        yaw = 1.57
        waypoints.append((x, y, yaw))############################################################# 37-38 @10-@11

    for i in range(5):
        x = 1.48 + i * 0.2975
        y = 2.6
        yaw = 0.0
        waypoints.append((x, y, yaw))############################################################# 39-43 @11-@12

    for i in range(3):
        x = 2.67
        y = 2.6 + i * 0.21  
        yaw = 1.57
        waypoints.append((x, y, yaw))############################################################# 44-46 @12-@13

    for i in range(3):
        x = 2.67 + i * 0.19 
        y = 3.02
        yaw = 1.57
        waypoints.append((x, y, yaw))############################################################# 47-49 @13-@14

    for i in range(3):
        x = 3.05 - i * 0.19  
        y = 3.02
        yaw = 1.57
        waypoints.append((x, y, yaw))############################################################# 50-52 @14-@15

    for i in range(5):
        x = 2.67
        y = 3.02 + i * 0.295 
        yaw = 1.57
        waypoints.append((x, y, yaw))############################################################# 53-57 @15-@16

    for i in range(15):
        x = 2.67 - i * 0.212142857
        y = 4.2 + i * 0.003571429
        yaw = 3.1
        waypoints.append((x, y, yaw))############################################################# 58-72 @16-@17

    for i in range(3):
        x = -0.3
        y = 4.25 - i * 0.175  
        yaw = 3.1
        waypoints.append((x, y, yaw))############################################################# 73-75 @17-@18

    for i in range(3):
        x = -0.3
        y = 3.9 + i * 0.175 
        yaw = 3.1
        waypoints.append((x, y, yaw))############################################################# 76-78 @18-@19


    waypoints.append((-0.3, 4.25, 1.56))   # 79
    waypoints.append((-0.3, 4.45, 1.56))   # 
    waypoints.append((-0.324, 4.55, 1.445))
    waypoints.append((-0.274, 4.781, 1.445))
    waypoints.append((-0.274, 4.781, 0.956))
    waypoints.append((-0.041, 5.112, 0.956))
    waypoints.append((-0.041, 5.113, 0.685))
    waypoints.append((0.277, 5.333, 0.685))
    waypoints.append((0.541, 5.467, 0.479))
    waypoints.append((0.540, 5.467, 0.205))
    waypoints.append((0.795, 5.549, 0.205))
    waypoints.append((0.995, 5.549, 0.205))
    waypoints.append((1.311, 5.604, 0.205))
    waypoints.append((1.311, 5.604, 0.067))
    waypoints.append((1.760, 5.658, 0.067))
    waypoints.append((2.043, 5.688, 0.263))
    waypoints.append((2.184, 5.726, 0.263))
    waypoints.append((2.176, 5.741, 0.460))
    waypoints.append((2.432, 5.843, 0.459))
    waypoints.append((2.673, 5.962, 0.459))
    waypoints.append((2.674, 5.962, 0.794))
    waypoints.append((2.875, 6.2224, 0.84))
    waypoints.append((3.00, 6.364, 0.841))
    waypoints.append((3.00, 6.364, 1.306))
    waypoints.append((3.058, 6.608, 1.473))
    waypoints.append((3.079, 6.786, 1.57))

 #103

    for i in range(5):
        x = 3.103 - i * ((3.074 - 2.001) / 4)   
        y = 7.095 + i * ((7.131 - 7.072) / 4)  
        yaw = 1.57
        waypoints.append((x, y, yaw))#104-108，过完弯道 向左平移 

    for i in range(18):
        t = i / 17.0  # 从 0 到 1，共 18 个点
        x = 2.001 + t * (2.065 - 2.001)
        y = 7.131 + t * (10.122 - 7.131)
        yaw = 1.57
        waypoints.append((x, y, yaw))#109-126 前进 来限高杆


    # 生成5个点，沿 y 轴正方向移动
    for i in range(4):
        x = 2.134
        y = 10.521 + i * 0.190625
        yaw = 1.57
        waypoints.append((x, y, yaw))#127-130 #过限高杆

    for i in range(10):
        x = 2.134
        y = 11.0335 - i * 0.190625  # 每次递减 0.190625
        yaw = -1.57
        waypoints.append((x, y, yaw))#131-140 #返回限高杆

    
    for i in range(5):
        x = 2.134
        y = 9.317875 - i * 0.11446875  # 
        yaw = -1.57
        waypoints.append((x, y, yaw))#141-145  #过完限高杆 返程 接右跳

    for i in range(8):
        x = 2.134 - i * 0.1678571429
        y = 8.86
        yaw = 3.1
        waypoints.append((x, y, yaw))#146-153 #进第4的第二
    
    for i in range(3):
        x = 0.959
        y = 8.86 + i * 0.2
        yaw = 3.1
        waypoints.append((x, y, yaw))#154-156 右移 接右跳 

    for i in range(18):
        t = i / 17.0
        x = 0.959 + t * (0.978 - 0.959)
        y = 9.26 + t * (11 - 9.06)
        yaw = 1.57
        waypoints.append((x, y, yaw))#157-174，前往蓝球

    for i in range(18):
        t = i / 17.0
        x = 0.978 + t * (0.959 - 0.978)
        y = 10.9 + t * (9.06 - 10.9)
        yaw = -1.57
        waypoints.append((x, y, yaw)) #175-192 从蓝球返回

    for i in range(10):
        t = i / 9.0
        x = 0.961 + t * (0.0 - 0.961)
        y = 9.213 + t * (8.183 - 9.213)
        yaw = -2.347 + t * (-2.348 - (-2.347))
        waypoints.append((x, y, yaw))#193-202 前往可乐瓶赛道 接右跳

    for i in range(5):
        t = i / 4.0
        x = 0.0
        y = 8.183
        yaw = 0.848 + t * (1.57 - 0.848)
        waypoints.append((x, y, yaw))#203-207 #前进


    for i in range(8):
        t = i / 7.0
        x = 0.0
        y = 8.183 + t * (9.175 - 8.183)
        yaw = 1.57
        waypoints.append((x, y, yaw))#208-215 #限高杆前

    for i in range(8):
        t = i / 7.0
        x = 0.0
        y = 9.175 + t * (9.875 - 9.175)
        yaw = 1.57
        waypoints.append((x, y, yaw))#216-223 限高杆
    
    for i in range(15):
        t = i / 14.0
        x = 0.0
        y = 9.875 + t * (11.113 - 9.875)
        yaw = 1.57
        waypoints.append((x, y, yaw))#224-238 撞可乐瓶 #接2跳
    
    # 15个点：原路返回 (0.0, 11.013) -> (0.0, 9.875)
    for i in range(15):
        t = i / 14.0
        x = 0.0
        y = 11.013 + t * (9.875 - 11.113)
        yaw = -1.57
        waypoints.append((x, y, yaw))#239-253

    #返回限高杆
    for i in range(13):
        t = i / 12.0
        x = 0.0
        y = 9.875 + t * (9.175 - 9.875)
        yaw = -1.57 
        waypoints.append((x, y, yaw))#254-266

    #限高杆前
    for i in range(12):
        t = i /11.0
        x = 0.0
        y = 9.175 + t * (7.083 - 9.175)
        yaw = -1.57
        waypoints.append((x, y, yaw))#267-278

    for i in range(4):
        t = i / 3.0  # 4个点用 3 段间隔
        x = 0.0  # 固定不变
        y = 7.0830 + t * (7.090 - 7.0830)  # y 平滑上升
        yaw = -1.57 + t * (0.0 - (-1.57))   # 偏航角平滑转正
        waypoints.append((x, y, yaw)) #278- 281

    #前往准备进5阶段
    for i in range(15):
        x = 0.0 + i * ((3.04 - 0.0) / 14)   # x 从 0.0 均匀到 3.04
        y = 7.090                           # y 固定不变
        yaw = 0.0                           # 偏航角固定
        waypoints.append((x, y, yaw))#准备进5阶段，接左跳       282-296

    for i in range(4):
        t = i / 3.0
        x = 3.0400  # 固定不变
        y = 7.090 + t * (7.0340 - 7.090)   # y 从 7.090 → 7.0340
        yaw = 0.0 + t * (-1.57 - 0.0)      # yaw 从 0 → -1.57
        waypoints.append((x, y, yaw))    #297-300

    #纠正
    for i in range(3):
        x = 3.04 + i * ((3.139 - 3.04) / 2)   # x 均匀插值
        y = 7.034 + i * ((7.382 - 7.034) / 2) # y 均匀插值
        yaw = -1.57                           # 偏航角固定
        waypoints.append((x, y, yaw))#准备起跳，接前跳    300-302
#########################################################################################第五阶段############################################
    #第一条道
    for i in range(28):
        x = 3.139  # x 固定不变
        y = 8.032 + i * ((12.35239 - 8.032) / 27)  # 28个点分27段间隔
        yaw = -1.57  # 偏航角固定
        waypoints.append((x, y, yaw))#第一条道走完了，接右跳  303-330 332



    #第二条道
    for i in range(20):
        x = 3.139 + i * ((-0.449 - 3.139) / 19)
        y = 12.35389 + i * ((12.4073 - 12.389) / 19)
        yaw = -1.57
        waypoints.append((x, y, yaw))#第二条道走完了，接右跳  333- 352

    for i in range(5):
        x = -0.297 + i * (-0.32 - (-0.277)) / 4
        y = 12.4073
        yaw = -1.57 + i * (-3.1 - (-1.57)) / 4
        waypoints.append((x, y, yaw))       #353-   357

    #第三条道
    for i in range(30):
        x = -0.38 + i * ((-0.38 - (-0.32)) / 29)
        y = 12.473 + i * ((15.163 - 12.473) / 29)
        yaw = 3.1
        waypoints.append((x, y, yaw))  #第三条道走完了，接右跳  358- 387

    for i in range(5):
        x = -0.36 + i * (0.06 / 4)
        y = 15.163+ i*0.05
        yaw = 3.1 + i * (4.67 / 4)
        waypoints.append((x, y, yaw))  #388-   392

    #第四条道
    for i in range(19):
        x = -0.284 + i * ((3.188 - (-0.284)) / 19)
        y = 15.4 
        yaw = 1.57
        waypoints.append((x, y, yaw))#第四条道走完了，接右跳  393   - 412

    for i in range(6):
        x = 3.1433684210526318 + i * ((3.025 - 3.0033684210526318) / 5)
        y = 15.462947368421053 + i * ((14.281 - 15.462947368421053) / 5)
        yaw = 1.57
        waypoints.append((x, y, yaw))       #413-   418

    for i in range(5):
        x = 3.1425
        y = 14.281
        yaw = 1.57 + i * (3.1 - 1.57) / 4
        waypoints.append((x, y, yaw))     #  419-423

    for i in range(5):
        x = 3.00185
        y = 14.281 - i * (14.281 - 13.613) / 4
        yaw = 3.1
        waypoints.append((x, y, yaw))##第五条道走完了，接右跳和前跳 424-428

    

    #################################################################################第六赛段
    for i in range(20):
        x = 2.3900 + i * (-0.1047316)
        y = 13.8130
        yaw = 3.1
        waypoints.append((x, y, yaw))  # 429-448

    for i in range(4):
        x = 0.4001 + i * (-0.0000333)
        y = 13.8130 + i * 0.2956667
        yaw = 3.1 + i * (-0.51)
        waypoints.append((x, y, yaw))   # 449-452

    for i in range(3):
        x = 0.4
        y = 14.7 + i * (0.1)
        yaw = 1.57
        waypoints.append((x, y, yaw))   # 453-455

    for i in range(3):
        x = 0.4
        y = 14.9 + i * (-0.1)
        yaw = 1.57
        waypoints.append((x, y, yaw))   #  456-458

    for i in range(3):
        x = 0.4 + i * (-0.15)
        y = 14.8
        yaw = 1.57
        waypoints.append((x, y, yaw))   #  459-461

    for i in range(13):
        x = 0.1 + i * (0.2042857)
        y = 14.8
        yaw = 1.57
        waypoints.append((x, y, yaw))   #  462-474

    for i in range(20):
        x = 2.52514284
        y = 14.68
        yaw = 1.57 - i * (0.1744/2)
        waypoints.append((x, y, yaw))  #   475-494     

    for i in range(5):
        x = 2.5514284 - i * 0.0523571
        y = 14.7 - i * 0.03575
        yaw = 0.0004 + i * 0.0999
        waypoints.append((x, y, yaw))#· 495-499

    for i in range(5):
        x = 2.342 + i * 0.00825
        y = 14.557 + i * 0.1075
        yaw = 0.4 - i * 0.1
        waypoints.append((x, y, yaw))#  500-504

    for i in range(3):
        x = 2.375 + i * 0.0625
        y = 14.920
        yaw = 0.0
        waypoints.append((x, y, yaw))   #505-   507

    for i in range(17):
        x = 2.5
        y = 14.920 - i * (1.739 / 16)
        yaw = 0.0
        waypoints.append((x, y, yaw))    #508-524

     
    for i in range(5):
        x = 2.5
        y = 12.981 + i * (13.3 - 12.981) / 4
        yaw = 0.0
        waypoints.append((x, y, yaw))   #525-   529

    # 2. (2.5,13.3,0.0) → (2.3,13.3,0.0)  3个点
    for i in range(3):
        x = 2.5 + i * (2.3 - 2.5) / 2
        y = 13.3
        yaw = 0.0
        waypoints.append((x, y, yaw))   #530-532

    # 3. (2.3,13.3,0.0) → (2.3,13.0,0.0)  3个点
    for i in range(3):
        x = 2.3
        y = 13.3 + i * (13.0 - 13.3) / 2
        yaw = 0.0
        waypoints.append((x, y, yaw))   #533-535

    # 4. (2.3,13.0,0.0) → (4.0,13.0,0.0)  10个点
    for i in range(10):
        x = 2.3 + i * (4.0 - 2.3) / 9
        y = 13.0
        yaw = 0.0
        waypoints.append((x, y, yaw))   #536    -545

    # 5. (4.0,13.0,0.0) → (3.05,12.9,0.0)  8个点
    for i in range(8):
        x = 4.0 + i * (3.05 - 4.0) / 7
        y = 13.1 + i * (12.9 - 13.0) / 7
        yaw = 0.0
        waypoints.append((x, y, yaw))   #546-553




    

    return waypoints

def eval_velocity(position, target_position, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve):
    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    cx, cy, cyaw = position 
    tx, ty, tyaw = target_position
    
    kp = 1.55
    ki = 0.0
    kd = 0.0

    kp_yaw = 1.00
    ki_yaw = 0.01
    kd_yaw = 0.001
        
    error_x = tx - cx
    error_y = ty - cy
    # raw_error_yaw = tyaw - cyaw
    error_yaw = normalize_angle(tyaw - cyaw)

    
    slip_angle = normalize_angle(math.atan2(error_y, error_x))  
    dist=math.sqrt(error_x**2 + error_y**2)

    vx= kp*dist*math.cos(slip_angle-cyaw)
    vy= kp*dist*math.sin(slip_angle-cyaw)

    integral_yaw += error_yaw
    derivative_yaw = error_yaw - prev_error_yaw

    speed_x= vx
    speed_y= vy
    np.clip(speed_y, -1, 1)
    np.clip(speed_x, -1, 1)
    
    speed_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw

    prev_error_yaw = error_yaw
    if math.sqrt(error_x**2 + error_y**2) < 0.10 and abs(error_yaw) < 0.17:

        print(f"Moving to waypoint: x :{tx}, its y: {ty}, its yaw: {tyaw}")
        achieve = True
 
    return speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw

def get_yaw_from_quaternion(orientation):
    # print("朝向:", orientation)
    w, x, y, z  = orientation
    norm = math.sqrt(x**2 + y**2 + z**2 + w**2)
    if norm == 0:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
    return yaw

class Robot_Ctrl(object):
    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        # self.steps = toml.load("/home/cyberdog_sim/src/T202410486992998-3472/code/toml/usergait.toml")
        self.num = 0  # 初始状态为站立
        self.running = True

        
        self.msg.mode        = 12
        self.msg.gait_id     = 0
        self.msg.contact     = 0
        self.msg.value       = 0
        self.msg.duration    = 5000
        self.msg.vel_des     = [ 0.0, 0.0, 0.0,]
        self.msg.rpy_des     = [ 0.0, 0.0, 0.0,]
        self.msg.pos_des     = [ 0.0, 0.0, 0.05,]
        self.msg.acc_des     = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
        self.msg.foot_pose   = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
        self.msg.ctrl_point  = [ 0.0, 0.0, 0.0,]
        self.msg.step_height = [ 0.0, 0.0,]
        self.msg.life_count =(self.msg.life_count + 1) % 127
    
        
        self.msg.step_height = [ 0.04, 0.04]
        self.msg.life_count =(self.msg.life_count + 1) % 127
        
    def run(self):
        while self.running:
            self.update_and_publish()
            time.sleep(0.2)

    def update_and_publish(self):
        # print('duration:', self.msg.duration)
        self.lc.publish("robot_control_cmd", self.msg.encode())


def main():
    my_ctrl     = Robot_Ctrl()
    lcm_cmd     = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")    # 建立lcm  
    cmd_msg     = robot_control_cmd_lcmt()                        # 机器人控制消息格式
    data_lock   = threading.Lock()                                # 数据线程锁
    pos_msg     = Pos_msg(data_lock)
    gait_msg    = Gait_msg(data_lock)
    ctrl_thread = threading.Thread(target=my_ctrl.run, daemon=True)
    rec_thread  = threading.Thread(target=pos_msg.run, daemon=True)
    gait_thread = threading.Thread(target=gait_msg.run, daemon=True)
    
    ctrl_thread.start()
    time.sleep(4)
    rec_thread.start()
    gait_thread.start()

    with data_lock: 
        print('pid start show')
        print(f"当前位置: {pos_msg.position} 机身朝向{pos_msg.quat}")

    # print('try start')
    # time.sleep(5.0)
    # # with data_lock: 
    # #     print(f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy}")
    waypoints = generate_waypoints()#[:1]
    print('生成了', len(waypoints), '个路点')

    
 
    
    my_ctrl.msg.mode        = 11
    my_ctrl.msg.gait_id     = 10
    # my_ctrl.msg.contact     = 15
    my_ctrl.msg.value       = 0
    my_ctrl.msg.duration    = 0
    my_ctrl.msg.vel_des     = [ 0.0, 0.0, 0.0,]
    my_ctrl.msg.rpy_des     = [ 0.0, 0.0, 0.0,]
    my_ctrl.msg.pos_des     = [ 0, 0, 0.28]
    my_ctrl.msg.acc_des     = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
    my_ctrl.msg.foot_pose   = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
    my_ctrl.msg.ctrl_point  = [ 0.0, 0.0, 0.0,]
    my_ctrl.msg.step_height = [ 0.045, 0.045,]
    my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127 
        
    
    dist = np.inf
    dist_error_yaw = np.inf 

    prev_error_yaw = 0.0
    integral_yaw = 0.00
    prev_error_x = 0.0
    integral_x = 0.0
    prev_error_y = 0.0
    integral_y = 0.0
    
    speed_x      = 0.00
    speed_y      = 0.00
    speed_yaw    = 0.00
    
   
    try:
        # ########### 行走过程 ###########
        time.sleep(5)
        id = 0
        m=0
        n=0

        with data_lock: 
            print('跳1')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立0')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('跳1')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('走1')
            my_ctrl.msg.mode        = 11
            my_ctrl.msg.gait_id     = 27
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[:10]:
            id += 1
            print(id, 'waypoint:', waypoint)
            achieve = False
            tx, ty, tyaw = waypoint
            dist = np.inf
            dist_error_yaw = np.inf
            with data_lock:
                my_ctrl.msg.mode = 11
                my_ctrl.msg.gait_id = 10
                my_ctrl.msg.duration = 0
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                       
            while dist > 0.2 and dist_error_yaw > 0.2:
                print('pid start')
                time.sleep(1)
                with data_lock: 
                    cx, cy, _ = pos_msg.position
                    cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                
                    
                ############################
                my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                ############################
                
                if achieve:
                    break 
            print('路标刷新')
            #break 
        
        with data_lock: 
            print('跳11')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立3')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)   
            

        for waypoint in waypoints[10:13]:
            id += 1
            print(id, 'waypoint:', waypoint)
            achieve = False
            tx, ty, tyaw = waypoint
            dist = np.inf
            dist_error_yaw = np.inf
            with data_lock:
                my_ctrl.msg.mode = 11
                my_ctrl.msg.gait_id = 10
                my_ctrl.msg.duration = 0
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

            while dist > 0.2 and dist_error_yaw > 0.2:
                    print('pid start')
                    time.sleep(1)
                    with data_lock: 
                        cx, cy, _ = pos_msg.position
                        cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                    speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                    
                        
                    ############################
                    my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                    my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                    ############################
                    
                    if achieve:
                        break 
                    print('路标刷新')
                #break 

        with data_lock: 
            print('立5')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0) 

        with data_lock: 
            print('左跳,跳4')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立6')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0) 

        for waypoint in waypoints[14:18]:
            id += 1
            print(id, 'waypoint:', waypoint)
            achieve = False
            tx, ty, tyaw = waypoint
            dist = np.inf
            dist_error_yaw = np.inf
            with data_lock:
                my_ctrl.msg.mode = 11
                my_ctrl.msg.gait_id = 10
                my_ctrl.msg.duration = 0
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 
            while dist > 0.2 and dist_error_yaw > 0.2:
                    print('pid start')
                    time.sleep(1)
                    with data_lock: 
                        cx, cy, _ = pos_msg.position
                        cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                    speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                    
                        
                    ############################
                    my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                    my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                    ############################
                    
                    if achieve:
                        break 
                    print('路标刷新')
                #break 

        with data_lock: 
            print('立7')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)  

        with data_lock: 
            print('右跳,跳5')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[19:36]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                            
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break 

        with data_lock: 
            print('立8')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)     

        for waypoint in waypoints[37:38]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                            
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break 
        
        with data_lock: 
            print('右跳,跳6')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立9')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0) #######################################################@11

        for waypoint in waypoints[39:43]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                            
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break 
        with data_lock: 
            print('左跳,跳7')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立10')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0) #######################################################@12

        for waypoint in waypoints[44:57]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                            
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break 

        with data_lock: 
            print('左跳,跳8')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立11')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0) #######################################################@16

        for waypoint in waypoints[58:78]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                            
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        with data_lock: 
            print('右跳,跳9')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立12')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0) #######################################################@19

        for waypoint in waypoints[79:103]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                            
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        with data_lock: 
            print('立13')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        for waypoint in waypoints[104:126]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                            
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        with data_lock: 
            print('立13_937')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[127:130]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                print('948')
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 80
                    my_ctrl.msg.value = 1

                    # 运动位姿参数
                    my_ctrl.msg.pos_des = [0.0, 0.0, -0.1]
                    my_ctrl.msg.step_height = [0.005, 0.005]
                    print('957')

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        print('966')
                        print(0.4*speed_x)    
                        ############################
                        my_ctrl.msg.vel_des = [0.4*speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        



        with data_lock: 
            print('立14')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        

        for waypoint in waypoints[130:131]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                print('948')
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.value = 1

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        print('1012')
                        print(0.05*speed_x)    
                        ############################
                        my_ctrl.msg.vel_des = [0.2*speed_x,0.0*speed_y, 0.0*speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break
            
        with data_lock: 
            print('跳1')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立0')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('右跳1')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)
        
        for waypoint in waypoints[131:135]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                print('948')
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 80
                    my_ctrl.msg.value = 1

                    # 运动位姿参数
                    my_ctrl.msg.pos_des = [0.0, 0.0, -0.1]
                    my_ctrl.msg.step_height = [0.005, 0.005]
                    print(speed_x)
                    print('957')

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        print('返程限高杆')
                        print(0.25*speed_x)
                        ############################
                        my_ctrl.msg.vel_des = [0.2*speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        



        with data_lock: 
            print('立15')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

       

####################前往蓝球
        for waypoint in waypoints[136:145]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('立13_1101')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        

        with data_lock: 
            print('立13_1179')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 0
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)
      

        with data_lock: 
            print('右跳1 4 2')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        with data_lock: 
            print('立1 4 2')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[146:156]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('记得右移')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        for waypoint in waypoints[154:156]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('记得右移')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,1.5*speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        with data_lock: 
            print('右跳1 4 2 2')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        with data_lock: 
            print('前往篮球')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[157:174]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('立13_1101')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break


        with data_lock: 
            print('立1 4 2 3')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('右跳1 4 2 3')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 2000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        with data_lock: 
            print('立1 4 2 3')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('右跳1 4 2 4')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 2000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        with data_lock: 
            print('立1 4 2 4')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立1 4 2 4')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立1 4 2 4')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 2500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

#前往4阶段的3道

        for waypoint in waypoints[175:192]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('立13_1101')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        for waypoint in waypoints[193:202]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('立13_1101')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break


        with data_lock: 
            print('立1 4 3 1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('右跳1 4 3 1')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        with data_lock: 
            print('立1 4 3 1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[203:215]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('立13_1101')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break


        with data_lock: 
            print('立1 4 3 2')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

#限高杆
        for waypoint in waypoints[216:223]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                print('948')
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 80
                    my_ctrl.msg.value = 1

                    # 运动位姿参数
                    my_ctrl.msg.pos_des = [0.0, 0.0, -0.1]
                    my_ctrl.msg.step_height = [0.005, 0.005]
                    print(speed_x)
                    print('957')

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        print('1018')
                        ############################
                        my_ctrl.msg.vel_des = [0.5*speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

        with data_lock: 
            print('立1 4 3 3')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[223:224]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('1576')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break



        with data_lock: 
            print('立1590')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[224:238]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('1544')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break


        with data_lock: 
            print('立1 4 3 4')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('右跳1 4 3 5')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        with data_lock: 
            print('立1 4 3 5')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('右跳1 4 3 5')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 3
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 1000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        with data_lock: 
            print('立1 4 3 5')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        for waypoint in waypoints[239:253]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('1544')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break


        with data_lock: 
            print('立1 4 3 6')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立1 4 4 1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[254:261]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 80
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('1731')   
                        ############################
                        my_ctrl.msg.vel_des = [0.4*speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break

###########################################################################################################
        for waypoint in waypoints[266:330]:###262
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                print('948')
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 
                    
                    # 运动位姿参数
                    print(speed_x)
                    print('957')

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        print('1018')
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break




        for waypoint in waypoints[330:428]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0

                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('立13_1101')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,0.7*speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break




        with data_lock: 
            print('立1 4 3 2')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('右跳1 4 3 5')
            my_ctrl.msg.mode        = 16
            my_ctrl.msg.gait_id     = 1
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 3000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)



        with data_lock: 
            print('立1 5 1 1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立1 5 1 1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('立1 5 1 1')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 3000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        for waypoint in waypoints[428:569]:
                id += 1
                print(id, 'waypoint:', waypoint)
                achieve = False
                tx, ty, tyaw = waypoint
                dist = np.inf
                dist_error_yaw = np.inf
                with data_lock:
                    my_ctrl.msg.mode = 11
                    my_ctrl.msg.gait_id = 10
                    my_ctrl.msg.duration = 0
                    my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127 

                while dist > 0.2 and dist_error_yaw > 0.2:
                        print('pid start')
                        time.sleep(1)
                        with data_lock: 
                            cx, cy, _ = pos_msg.position
                            cyaw      = get_yaw_from_quaternion(pos_msg.quat)
                        speed_x, speed_y, speed_yaw, prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, achieve, dist, error_yaw = eval_velocity([cx, cy, cyaw], [tx, ty, tyaw], prev_error_x, integral_x, prev_error_y, integral_y, prev_error_yaw, integral_yaw, speed_x, speed_y, speed_yaw, achieve)
                        
                        print('立13_1101')   
                        ############################
                        my_ctrl.msg.vel_des = [speed_x,speed_y, speed_yaw]
                        my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127
                        ############################
                        
                        if achieve:
                            break 
                        print('路标刷新')
                    #break
        
        with data_lock: 
            print('立1 5 1 2')
            my_ctrl.msg.mode        = 12
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 500
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)

        with data_lock: 
            print('')
            my_ctrl.msg.mode        = 7
            my_ctrl.msg.gait_id     = 0
            # my_ctrl.msg.contact     = 15
            my_ctrl.msg.duration    = 2000
            my_ctrl.msg.life_count  =(my_ctrl.msg.life_count + 1) % 127    
            time.sleep(3.0)


        


        with data_lock:
            my_ctrl.msg.mode        = 7
            my_ctrl.msg.gait_id     = 0
            my_ctrl.duration        = 0
            my_ctrl.msg.vel_des     = [0.0, 0.0, 0.0]
            my_ctrl.msg.step_height = [0.03, 0.03]
            my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127 

###############################################################################################################################

    except KeyboardInterrupt:
        cmd_msg.mode = 7  # PureDamper before KeyboardInterrupt
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
    
if __name__ == '__main__':
    main()

