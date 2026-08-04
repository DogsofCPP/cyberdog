# CyberDog Race Controller - Xiaomi Cup 2026

Simulation-first autonomous race controller for the 2026 全国大学生计算机系统能力大赛——智能系统创新设计赛（小米杯）荒野寻宝赛道.

## 目录结构

```
cyberdog_race/
├── race_main.py              # 主入口点
├── lcm_controller.py         # LCM 通信控制器
├── lcm_types/                # LCM 消息类型定义
│   └── __init__.py
├── perception/                # 感知层
│   ├── ros2_perception.py    # ROS2 感知节点
│   └── detectors/            # 检测器
│       ├── ball_detector.py   # 球体检测（橙色/浅蓝色）
│       ├── object_detector.py # 物体检测（可乐/足球/限高杆）
│       └── boundary_detector.py # 黄色边界线检测
├── navigation/               # 导航层
│   ├── state_machine.py     # 六赛段状态机
│   └── segments/            # 各赛段策略
│       ├── segment_1.py     # 石径探路
│       ├── segment_2.py     # 荒野寻珠
│       ├── segment_3.py     # 曲道冲锋
│       ├── segment_4.py     # 深隧寻珍
│       ├── segment_5.py     # 孤梁稳渡
│       └── segment_6.py     # 撷金建功
├── motion/                   # 运动程序库
│   └── programs.py          # 预制运动序列
├── utils/                   # 工具
│   ├── speech.py             # espeak 语音播报
│   ├── mock_perception.py   # 测试用模拟感知
│   └── __init__.py
└── config.toml              # 赛道参数配置
```

## 快速开始

### 第一步：安装依赖

```bash
pip3 install lcm toml opencv-python-headless numpy transforms3d
sudo apt install -y espeak libasound2-dev python3-pip
```

### 第二步：启动仿真

容器内三个终端：

```bash
# 终端 1 - 仿真器
cd /home/cyberdog_sim
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py

# 终端 2 - 运动管理服务
cd /home/cyberdog_ws
source /opt/ros/galactic/setup.bash
source /home/cyberdog_ws/install/setup.bash
ros2 run motion_manager motion_manager

# 终端 3 - 比赛程序
cd ~/loco_hl_example
source /opt/ros/galactic/setup.bash
python3 -m cyberdog_race.race_main --mode=sim
```

### 测试单个赛段

```bash
python3 -m cyberdog_race.race_main --test-segment=1
python3 -m cyberdog_race.race_main --test-segment=2
# ...
```

### 纯感知模式

```bash
python3 -m cyberdog_race.race_main --perception-only
```

## 架构说明

### 三层架构

```
感知层 (ROS2)
    ├── /scan          - 激光雷达
    ├── /D435/rgb/image_raw - D435 RGB 相机
    ├── /D435/depth/image_raw - D435 深度相机
    ├── /imu           - IMU
    └── /tf            - 里程计坐标变换
            ↓
决策层 (Python)
    ├── LCMController - LCM 通信
    ├── BallDetector  - 球体检测 (HSV)
    ├── ObjectDetector - 物体检测 (颜色+形状)
    ├── BoundaryDetector - 边界线检测
    └── RaceStateMachine - 六赛段状态机
            ↓
控制层 (LCM)
    └── CyberDog 运动控制器
```

### LCM 通信

- 发送: `udpm://239.255.76.67:7671?ttl=255` → 频道 `robot_control_cmd`
- 接收: `udpm://239.255.76.67:7670?ttl=255` ← 频道 `robot_control_response`
- **关键**: `life_count` 每次发送必须 +1，否则命令不生效

### 控制模式速查

| mode | 含义 |
|------|------|
| 7    | PureDamper / 趴下停止 |
| 11   | Locomotion / 移动控制 |
| 12   | Recovery Stand / 恢复站立 |
| 21   | Position Interpolation / 位置插值 |
| 62   | Action Trigger / 触发内置动作 |
| 64   | Two-leg Stand / 双足站立 |

### 步态速查

| gait_id | 步态 | 用途 |
|---------|------|------|
| 3       | TROT_MEDIUM | 中速 |
| 7       | BOUND/PRONK | 跳跃 |
| 10      | TROT_FAST | 快速 |
| 26      | 自变频率 | 原地转向 |
| 27      | TROT_SLOW | 慢速稳定 |
| 110     | 用户自定义 | 自定义步态 |

## 赛段详解

### 赛段 1: 石径探路
- **地形**: 石板路（宽30cm，高5cm，间隔20cm）+ 弯道
- **策略**: TROT_SLOW (vx=0.15m/s)，LiDAR 双边走廊跟随
- **结束**: 后腿足底离开弯道虚线
- **注意**: 黄线（实线）禁止踩越

### 赛段 2: 荒野寻珠
- **目标**: 4个橙色小球（4x4 小球阵列，每行每列 1 个）
- **策略**: 覆盖式网格搜索，摄像头检测橙色球后对准撞击，撞满 4 个后前往左上出口
- **注意**: 不可触碰浅蓝色小球
- **播报**: "识别到橙色小球"

### 赛段 3: 曲道冲锋
- **地形**: 曲线-直线-曲线赛道
- **策略**: 速度提升到 0.30-0.35m/s，LiDAR 自适应循线
- **结束**: 后腿足底离开弯道虚线

### 赛段 4: 深隧寻珍
- **目标物体**:
  - 可乐瓶 → "识别到可乐瓶" → 撞倒
  - 橙色小球 → "识别到橙色小球" → 撞击晃动
  - 足球 → "识别到足球" → 踢入球门
- **障碍物**:
  - 限高杆 → "识别到限高杆" → 压低身体从下方穿过
  - 方块障碍 → "识别到无法跨越障碍" → 绕行（可穿虚线借道）
- **结束**: 前腿足底碰到独木桥起始端

### 赛段 5: 孤梁稳渡
- **地形**: 连续独木桥（窄）
- **策略**: TROT_SLOW 慢速通过 → 检测到桥末端 → BOUND 跳跃离开
- **注意**: 必须四足都越过虚线后才能跳下

### 赛段 6: 撷金建功
- **目标**: 将足球从出口位置踢出，来到终点
- **策略**: 识别足球位置 → 定位后方 → 踢球
- **结束**: 四条腿足底在终点圈内，趴下

## 参数调优

所有可调参数集中在 `config.toml`:

- 各赛段速度、步态选择
- 球体检测 HSV 阈值
- 语音播报文本
- LiDAR 走廊目标距离
- P 控制增益

## 开发路线图

### 第一阶段：环境验证
- [x] 仿真器启动确认
- [x] LCM 命令链路验证 (basic_motion/main.py)
- [x] 站立、行走、趴下基本动作测试

### 第二阶段：感知开发
- [x] 激光雷达障碍检测
- [x] 黄色边界线检测
- [x] 球体颜色检测 (HSV)
- [x] 物体识别 (可乐/足球/限高杆/方块)

### 第三阶段：导航开发
- [x] 赛段1：直线走廊跟随
- [x] 赛段2：橙色球搜索 + 撞击
- [x] 赛段3：循线高速行进
- [x] 赛段4：物体识别+交互+障碍规避
- [x] 赛段5：独木桥行走 + 跳跃
- [x] 赛段6：踢球 + 终点趴下

### 第四阶段：集成与调优
- [ ] 全流程联调
- [ ] 各赛段参数微调
- [ ] 容错处理（跌倒检测与恢复）
- [ ] 真实赛场实测
