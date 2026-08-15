# Cyberdog Demo 技术文档

## 1. 文档目的

本文档基于 `src/cyberdog_demo` 与 `src/cyberdog_interfaces` 的当前实现，说明本工程在 Cyberdog 比赛场景中的软件结构、节点职责、通信协议、视觉处理方法、运动控制流程和部署运行方式。

## 2. 工程概览

该项目是 ROS 2 工作空间，主要包含以下包：

| 包名 | 作用 |
| --- | --- |
| `cyberdog_demo` | 比赛 Demo 逻辑，包含主控节点、视觉识别节点、启动入口和步态参数文件。 |
| `cyberdog_interfaces` | Cyberdog 运动控制相关 LCM 消息结构和动作参数辅助封装。 |
| `cyberdog_test` | 简单测试/占位包。 |

系统整体采用“视觉节点识别环境状态，主控节点融合状态并通过 LCM 下发运动指令”的结构。

```text
Camera Image Topics
        |
        v
+--------------------------+
| cyberdog_demo vision     |
| adjust / ground / fish   |
+--------------------------+
        |
        | std_msgs/String
        v
+--------------------------+        LCM udp multicast
| cyberdog_demo master     |  --->  robot_control_cmd
+--------------------------+  <---  robot_control_response
        |
        v
Cyberdog motion control board
```

### 2.1 分关策略概览

比赛流程按关卡拆分为五类控制策略。主控节点负责串联各阶段动作，视觉节点负责将相机图像压缩为可用于控制的偏差量、掩膜面积占比和触发信号。

| 关卡 | 核心策略 | 主要感知来源 | 控制方式 |
| --- | --- | --- | --- |
| 第一关 | 使用跳跃动作通过障碍。 | 赛前定点校正和前向边线检测。 | 先校正姿态，再执行 `mode=16` 跳跃动作。 |
| 第二关 | 使用鱼眼相机识别球体掩膜，根据左右掩膜大小纠偏；橙色小球掩膜超过阈值后触发撞击动作。 | 左右鱼眼 `/image_left`、`/image_right`。 | 根据 `fish_rgap_topic` 修正方向，根据 `hit_ratio_topic` 触发撞击。 |
| 第三关 | 使用扫地相机识别道路边界，通过 PID 控制器进行直道矫正。 | `/rgb_camera/image_ground`。 | 根据 `ground_topic` 输出的边界偏差，经 PID 计算后调整横移和偏航。 |
| 第四关 | 使用自定义步态降低重心通过低矮区域。 | 自定义步态 TOML 参数。 | 通过 `user_gait_file` 上传低重心步态，执行 `mode=62, gait_id=110`。 |
| 第五关 | 方向校正后执行跳跃动作。 | `ground_topic` 方向校正。 | PID 校正后执行 `mode=16` 跳跃。 |
| 第六关 | 尚未完成 |

除上述分关逻辑外，定点矫正和直道矫正均采用滑动窗口思想：视觉节点持续缓存最近若干帧的偏差，使用历史均值降低单帧误检、光照扰动和运动模糊带来的抖动。

## 3. 运行环境与依赖

### 3.1 ROS 环境

启动脚本使用 ROS 2 Galactic：

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

### 3.2 Python/ROS 依赖

`cyberdog_demo/package.xml` 声明了以下主要依赖：

| 依赖 | 用途 |
| --- | --- |
| `rclpy` | ROS 2 Python 节点框架。 |
| `std_msgs` | 节点间发布 `String` 状态消息。 |
| `sensor_msgs` | 订阅相机 `Image` 图像消息。 |
| `cv_bridge` | ROS 图像和 OpenCV 图像互转。 |
| `python3-opencv` | 图像处理、HSV 阈值分割、轮廓检测。 |

`cyberdog_interfaces/package.xml` 额外声明：

| 依赖 | 用途 |
| --- | --- |
| `lcm` | 与 Cyberdog 运动控制板通信。 |
| `geometry_msgs` | 当前实现中保留依赖，主链路未直接使用。 |

### 3.3 构建命令

在工作空间根目录执行：

```bash
colcon build
source install/setup.bash
```

## 4. 启动方式

### 4.1 一键启动

主启动脚本：

```bash
python3 src/launch/launch.py
```

该脚本会依次打开多个 `gnome-terminal` 运行：

| 顺序 | 脚本 | 节点 |
| --- | --- | --- |
| 1 | `src/launch/master.sh` | `ros2 run cyberdog_demo master` |
| 2 | `src/launch/adjust_node.sh` | `ros2 run cyberdog_demo adjust_node` |
| 3 | `src/launch/fisheyes_adjust_node.sh` | `ros2 run cyberdog_demo fisheyes_adjust_node` |
| 4 | `src/launch/ground_node.sh` | `ros2 run cyberdog_demo ground_node` |

`telaunch.py` 是测试启动入口，会启动 `test_demo`、`adjust_node`、`fisheyes_adjust_node`，默认不启动 `ground_node`。

`test3launch.py` 是第三关 PID 实机调参启动入口，会启动 `ground_node` 与 `test3 --live`。

### 4.2 单节点启动

```bash
ros2 run cyberdog_demo master
ros2 run cyberdog_demo adjust_node
ros2 run cyberdog_demo fisheyes_adjust_node
ros2 run cyberdog_demo ground_node
ros2 run cyberdog_demo seek_001_node
ros2 run cyberdog_demo test_demo
ros2 run cyberdog_demo test3
```

## 5. 节点与话题设计

### 5.1 节点清单

`cyberdog_demo/setup.py` 注册了以下 console scripts：

| 入口 | 源文件 | 作用 |
| --- | --- | --- |
| `master` | `master.py` | 比赛主控逻辑，订阅视觉状态并发送运动控制 LCM 指令。内置 `PIDController`、`PIDTelemetry` 和 `run_pid_align()` 通用 PID 对齐函数。`main()` 等待终端输入 `1` 后启动运动线程。 |
| `adjust_node` | `adjust_node.py` | 前向 RGB 相机赛道边线检测，输出偏航/横向校正量。 |
| `fisheyes_adjust_node` | `fisheyes_adjust_node.py` | 左右鱼眼图像识别橙色球和边线，输出球占比、左右差值和线占比。 |
| `ground_node` | `ground_node.py` | 地面相机赛道边线检测，输出地面视角校正量。 |
| `seek_001_node` | `seek_001_node.py` | 前向 RGB 相机橙色球检测，输出球面积占比和水平偏差。 |
| `test_demo` | `test_demo.py` | 测试/调试版主控，逻辑与 `master.py` 类似但包含更多试验代码。 |
| `test3` | `test3.py` | 第三关 PID 独立测试工具，支持离线调参、阶跃/正弦/对比测试；实机模式会先起立，按 `s` 开始/停止寻线，退出时站立后趴下。 |

### 5.2 ROS 话题

| 话题 | 类型 | 发布者 | 订阅者 | 数据格式 |
| --- | --- | --- | --- | --- |
| `/rgb_camera/image_raw` | `sensor_msgs/Image` | 相机驱动 | `adjust_node`, `seek_001_node` | BGR 图像输入。 |
| `/rgb_camera/image_ground` | `sensor_msgs/Image` | 地面相机/图像节点 | `ground_node` | BGR 图像输入。 |
| `/image_left` | `sensor_msgs/Image` | 左鱼眼相机 | `fisheyes_adjust_node` | BGR 图像输入。 |
| `/image_right` | `sensor_msgs/Image` | 右鱼眼相机 | `fisheyes_adjust_node` | BGR 图像输入。 |
| `adjust_topic` | `std_msgs/String` | `adjust_node` | `master` | `"<edge_num> <gap>"`。 |
| `ground_topic` | `std_msgs/String` | `ground_node` | `master` | `"<edge_num> <gap>"`。 |
| `seek_001_topic` | `std_msgs/String` | `seek_001_node` | `master` | `"<ratio> <dist>"`。 |
| `fish_rgap_topic` | `std_msgs/String` | `fisheyes_adjust_node` | `master` | `"<fish_rgap>"`。 |
| `hit_ratio_topic` | `std_msgs/String` | `fisheyes_adjust_node` | `master` | `"<left_ratio_hit> <right_ratio_hit>"`。 |
| `fish_line_topic` | `std_msgs/String` | `fisheyes_adjust_node` | `master` | `"<left_line> <right_line>"`。 |
| `master_mode_topic` | `std_msgs/String` | `master/test_demo` 设计上发布 | 视觉节点 | `"<mode_ok>"`，当前 `master.py` 中发布逻辑被注释。 |

注意：当前 `master.py` 中 `fish_line_subscription` 已正确订阅 `fish_line_topic`。

## 6. LCM 运动控制接口

### 6.1 通信通道

主控节点通过 LCM UDP 组播与 Cyberdog 运动控制板通信：

| 方向 | LCM URL | Channel | 消息类型 |
| --- | --- | --- | --- |
| 发送控制命令 | `udpm://239.255.76.67:7671?ttl=255` | `robot_control_cmd` | `robot_control_cmd_lcmt` |
| 接收控制反馈 | `udpm://239.255.76.67:7670?ttl=255` | `robot_control_response` | `robot_control_response_lcmt` |
| 上传自定义步态 | `udpm://239.255.76.67:7671?ttl=255` | `user_gait_file` | `file_send_lcmt` |

### 6.2 控制命令结构

`robot_control_cmd_lcmt` 主要字段：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mode` | int8 | 控制模式，例如站立、运动、跳跃、自定义动作。 |
| `gait_id` | int8 | 模式下的具体步态/动作编号。 |
| `contact` | int8 | 触地/步态相关参数。 |
| `life_count` | int8 | 指令生命计数，主控递增并在达到 50 后重置。 |
| `vel_des` | float[3] | 期望速度 `[vx, vy, vyaw]`。 |
| `rpy_des` | float[3] | 期望姿态 `[roll, pitch, yaw]`。 |
| `pos_des` | float[3] | 期望位置/高度。 |
| `acc_des` | float[6] | 自定义步态权重/加速度相关字段。 |
| `ctrl_point` | float[3] | 自定义步态控制点。 |
| `foot_pose` | float[6] | 自定义步态足端落点。 |
| `step_height` | float[2] | 左右侧步高或编码后的步高参数。 |
| `value` | int32 | 额外控制值，自定义步态中用于 `use_mpc_traj`。 |
| `duration` | int32 | 指令持续时间，单位 ms。 |

### 6.3 控制反馈结构

`robot_control_response_lcmt` 主要字段：

| 字段 | 含义 |
| --- | --- |
| `mode` | 当前反馈模式。 |
| `gait_id` | 当前反馈步态。 |
| `contact` | 触地状态。 |
| `order_process_bar` | 指令执行进度，`master` 中达到 95 以上认为该 `mode` 已完成。 |
| `switch_status` | 切换状态。 |
| `ori_error` | 姿态错误标志。 |
| `footpos_error` | 足端位置错误标志。 |
| `motor_error` | 12 个电机错误状态。 |

### 6.4 发送机制

`Robot_Ctrl_Node` 内部启动两个线程：

| 线程 | 方法 | 功能 |
| --- | --- | --- |
| 接收线程 | `rec_response()` | 循环调用 `lc_r.handle()` 接收 `robot_control_response`。 |
| 发送线程 | `send_publish()` | 每 5 ms 检查一次，在 `delay_cnt > 20` 时重发当前命令维持心跳，约 10 Hz。 |

外部逻辑调用 `Send_cmd(msg)` 时会更新当前命令并把 `delay_cnt` 置为 50，使发送线程尽快发布新指令。

## 7. 视觉处理实现

### 7.1 `adjust_node`: 前向边线校正

输入：`/rgb_camera/image_raw`

处理流程：

1. 将 ROS 图像通过 `cv_bridge` 转成 BGR。
2. 截取图像下方 1/3 区域，降低无关背景影响。
3. 转换到 HSV 空间。
4. 使用黄色阈值 `[30, 30, 30]` 到 `[90, 255, 255]` 分割赛道边线。
5. 查找轮廓并过滤面积小于 2000 的噪声。
6. 计算每个轮廓的中心 x 坐标。
7. 在候选中心中寻找距离在 100 到 600 像素之间且接近 300 像素的一对边线。
8. 以图像中心和赛道中心的差值作为 `gap`。
9. 使用最近 7 帧历史均值平滑 `gap`。
10. 以 25 Hz 发布 `"<edge_num> <gap>"` 到 `adjust_topic`。

输出含义：

| 字段 | 含义 |
| --- | --- |
| `edge_num` | 识别到的有效边线轮廓数量。 |
| `gap` | 图像中心与赛道中心的水平偏差，正负号由代码计算方式决定。 |

该节点承担赛前定点矫正和部分直道矫正任务。`gap_history` 相当于一个短滑动窗口，主控读取的是平滑后的偏差量，因此不会直接响应单帧噪声。

### 7.2 `ground_node`: 地面视角边线校正

输入：`/rgb_camera/image_ground`

处理方法与 `adjust_node` 基本一致，但使用完整图像而不是只截取下 1/3。输出到 `ground_topic`，用于第三关扫地相机视角下的道路边界修正。

扫地相机更接近地面，适合在直道中观察道路边界。节点同样维护 `gap_history` 滑动窗口，对最近多帧边界中心偏差取均值后输出，供主控计算横移速度和偏航速度。

### 7.3 `seek_001_node`: 前向橙色球识别

输入：`/rgb_camera/image_raw`

处理流程：

1. BGR 转 HSV。
2. 使用橙色阈值 `[10, 130, 80]` 到 `[100, 210, 160]`。
3. 查找橙色区域轮廓。
4. 选择最大轮廓，计算面积占图像比例 `ratio`。
5. 计算橙色目标中心相对图像中心的水平偏差 `dist`。
6. 以 25 Hz 发布 `"<ratio> <dist>"` 到 `seek_001_topic`。

当前 `master.py` 订阅了该话题，但主比赛流程主要使用 `fisheyes_adjust_node` 的橙球检测结果进行第二关避障/撞球逻辑。

### 7.4 `fisheyes_adjust_node`: 双鱼眼球和线识别

输入：`/image_left`、`/image_right`

该节点同时处理左右鱼眼图像，是第二关和第五关的主要感知来源。它在每帧图像上构造多类 HSV 掩膜：

| 掩膜/输出 | 计算方式 | 话题 | 用途 |
| --- | --- | --- | --- |
| 全部球体掩膜 | 使用较宽橙色阈值提取左右鱼眼画面中所有疑似球体区域，统计 `left_ratio/right_ratio`。 | `fish_rgap_topic` | 第二关方向纠偏，左右掩膜面积不均衡时向面积较小/较大一侧修正。 |
| 橙色小球掩膜 | 使用较窄橙色阈值提取橙色小球区域，统计 `left_ratio_hit/right_ratio_hit`。 | `hit_ratio_topic` | 当掩膜面积占比超过阈值时，主控触发撞击动作。 |
| 道路边界掩膜 | 在鱼眼图像底部区域提取黄色道路边界，统计 `left_line/right_line`。 | `fish_line_topic` | 第五关跳跃判定，依据左右边界掩膜面积变化判断是否到达触发位置。 |
| 左右球体差值 | `left_ratio * 0.9 - right_ratio`，再乘 100 输出。 | `fish_rgap_topic` | 第二关巡航时的连续纠偏量。 |

图像裁剪策略：

| 相机 | 裁剪范围 |
| --- | --- |
| 左鱼眼 | 取图像右侧 `4/9` 到末尾区域。 |
| 右鱼眼 | 取图像左侧到 `5/9` 区域。 |
| 线检测 | 取裁剪后图像下方 `1/4` 区域。 |

颜色阈值：

| 目标 | HSV 下限 | HSV 上限 |
| --- | --- | --- |
| 橙色球/撞击判断 | `[10, 130, 80]` | `[100, 210, 160]` |
| 较宽橙色球检测 | `[10, 130, 80]` | `[180, 210, 160]` |
| 黄色边线 | `[30, 30, 30]` | `[90, 255, 255]` |

文件顶部定义了 `K`、`D` 和 `undistort_fisheye()`，可用于鱼眼去畸变，但当前回调中尚未实际调用该去畸变函数。

### 7.5 滑动窗口矫正策略

定点矫正和直道矫正都采用滑动窗口降低视觉测量噪声。具体做法是：视觉节点在每次识别到道路边界后，将当前帧的中心偏差加入历史列表，并限制历史列表长度；当主控读取偏差时，使用最近若干帧的平均值作为输出。

该策略的作用包括：

1. 抑制单帧误检导致的瞬时大角速度。
2. 平滑相机曝光变化和运动模糊造成的边界跳变。
3. 让定点矫正、直道矫正的速度控制更连续。

当前实现中，`adjust_node` 与 `ground_node` 均维护 `gap_history`。当识别到双边界时通常保留最近 7 帧；只有单边界或短时丢失时，会使用较短历史或保持最近均值。

## 8. 主控流程

主控入口是 `master.py` 中的 `masterNode`。构造时会：

1. 初始化 LCM 命令、反馈和自定义步态上传接口。
2. 启动 `Robot_Ctrl_Node` 的接收/发送线程。
3. 订阅 `adjust_topic`、`ground_topic`、`seek_001_topic`、`fish_rgap_topic`、`hit_ratio_topic`、`fish_line_topic`。
4. `main()` 函数等待终端输入 `1` 后，启动独立线程执行 `master_line()`，ROS `spin_once` 循环负责处理订阅回调。

> **注意**：`master.py` 中还定义了 `masterNode_test` 类（继承自 `test_demo.py` 的 `masterNode_test`），用于测试场景。当前 `main()` 使用的是 `masterNode`。

### 8.1 初始化和方向校正

`master_line()` 开始时调用：

```python
change_gait(sec001_Def.toml, sec001_Param_full.toml, sec001_Param.toml)
```

用于上传第一段自定义步态。随后进入站立恢复：

| 动作 | 参数 |
| --- | --- |
| 站立恢复 | `mode=12`, `gait_id=0` |

接着调用 `run_pid_align("ground", 120, vx=0)` 进行基于 PID 控制器的方向校正。该函数使用 `PIDController`（默认 `Kp=0.00170, Ki=0.00003, Kd=0.00010`）从 `ground_topic` 获取 gap 值，计算 `vyaw`（PID 输出）和 `vy`（纯 P 控制），死区为 `abs(gap) <= 4` 时输出归零。同时开启 `PIDTelemetry` 可视化窗口实时显示 gap、vyaw、vy 及 P/I/D 三项分解和历史曲线。

### 8.2 第一关：前进与跳跃

第一关的核心动作是跳跃。主控先使用前向边线检测结果完成定点姿态矫正，使机身朝向与障碍方向尽量一致；随后使用 `mode=11, gait_id=10` 低速接近障碍，并多次执行 `mode=16` 跳跃动作通过障碍：

| 动作 | 参数 |
| --- | --- |
| 前进 | `mode=11`, `gait_id=10`, `vel_des=[0.22, 0, 0]` |
| 原地跳远 | `mode=16`, `gait_id=1`, `duration=1000`, `step_height=[0.05, 0.05]` |
| 恢复站立 | `mode=12`, `gait_id=0` |

随后执行左转、右转校正和横移，完成第一关后的姿态整理。

### 8.3 第二关：橙色球检测与绕行/撞击

第二关使用左右鱼眼相机作为主要感知输入。`fisheyes_adjust_node` 同时计算两类球体掩膜：较宽阈值的全部球体掩膜用于方向纠偏，较窄阈值的橙色小球掩膜用于撞击触发。核心函数是 `sec002_seek()`，输入来自：

| 数据 | 来源 |
| --- | --- |
| `left/right` | `hit_ratio_topic`，左右鱼眼橙色小球掩膜面积占比。 |
| `rgap` | `fish_rgap_topic`，左右全部球体掩膜面积差。 |

控制逻辑：

| 条件 | 动作 |
| --- | --- |
| 左右占比都小于 `0.0362`，或 `switch > 0` | 继续前进并根据 `rgap` 微调。 |
| `rgap > 0.01` | `vel_des=[0.32, -rgap/40, -rgap/25]`。 |
| `rgap < -0.01` | `vel_des=[0.32, -rgap/50, -rgap/25]`。 |
| 左侧占比大于等于 `0.0362` | 站立后向左/右执行一组避让或撞击动作，`hit_time += 1`。 |
| 右侧占比大于等于 `0.0362` | 站立后向相反方向执行一组避让或撞击动作，`hit_time += 1`。 |

第二关中多段循环调用 `sec002_seek()`。当全部球体掩膜左右面积不均衡时，主控连续调整 `vy` 和 `vyaw` 保持接近目标区域；当橙色小球掩膜面积超过阈值时，说明目标足够接近，主控触发撞击动作，并通过 `hit_time` 记录撞击次数以切换后续搜索路线。

### 8.4 第三关：地面视角方向修正（PID 控制）

第三关使用扫地相机进行道路边界识别。`ground_node` 输出 `ground_topic`，主控调用 `run_pid_align("ground", 526, vx=0.2)` 进行 PID 直道矫正。

控制策略：

| 参数 | 值 |
| --- | --- |
| 基础前进速度 `vx` | `0.2` |
| 修正话题 | `ground_topic` |
| 修正步态 | `mode=11`, `gait_id=12` |
| PID 控制器 | `Kp=0.00170, Ki=0.00003, Kd=0.00010` |
| vy 控制 | 纯 P，`Kp=0.000113`，限幅 `±0.10` |
| vyaw 控制 | PID，限幅 `±0.30` |
| 死区 | `abs(gap) <= 4` 时输出归零 |
| PID 积分限幅 | `±0.05` |
| 遥测窗口 | `PIDTelemetry`，显示 gap/vyaw/vy 曲线和 P/I/D 分解 |

每轮发送后休眠 40 ms（25 Hz），共 526 轮。循环结束后恢复站立。

第三关末尾还会调用 `run_pid_align("adjust", 120, vx=0.1)` 使用前向相机进行补充校正。

### 8.5 第四关：低矮通道与自定义步态

第四关通过低矮区域，核心是使用自定义步态降低重心。主控前半段先进行左/右转和姿态调整，随后切换低姿态自定义步态：

```python
change_gait(low_Def.toml, low_Params_full.toml, low_Params.toml)
```

然后基于 `adjust_topic` 进行低速方向修正，并执行：

| 动作 | 参数 |
| --- | --- |
| 自定义低姿态步态 | `mode=62`, `gait_id=110` |
| 循环次数 | 约 63 次 |
| 单次等待 | 约 0.84 s |

之后再上传反向/恢复低姿态步态：

```python
change_gait(low_gait_re/low_Def.toml, low_gait_re/low_Params_full.toml, low_gait_re/low_Params.toml)
```

并再次执行 `mode=62, gait_id=110`，最后横移、站立、进入阻尼/休息状态。

### 8.6 第五关：跳跃与结束

当前 `master.py` 中第五关的鱼眼掩膜跳跃判定逻辑（基于 `fish_line_topic` 的左右边界占比判断跳跃时机）已被注释，暂未启用。当前第五关简化为：

1. 调用 `run_pid_align("ground", 80, vx=0.05)` 进行方向校正。
2. 执行一次 `mode=16, gait_id=1` 跳跃动作。
3. 恢复站立后进入休息状态（`mode=7, gait_id=1`）。


### 8.7 结束与异常处理

正常结束：

| 动作 | 参数 |
| --- | --- |
| 站立恢复 | `mode=12`, `gait_id=0` |
| 休息/阻尼 | `mode=7`, `gait_id=1` |

捕获 `KeyboardInterrupt` 时会发送：

| 动作 | 参数 |
| --- | --- |
| PureDamper | `mode=7`, `gait_id=0`, `duration=0` |

随后关闭 LCM 线程并销毁节点。

## 9. 自定义步态上传机制

`change_gait(deff, full, params)` 用于把 TOML 步态定义和参数转换后发给运动控制板。

处理流程：

1. 构造一个全零的 `robot_cmd` 模板。
2. 读取 `params` TOML 中的 `step` 列表。
3. 对每个 `type == "usergait"` 的 step，转换为 LCM 控制字段：
   - `mode = 11`
   - `gait_id = 110`
   - `vel_des = body_vel_des`
   - `rpy_des = body_pos_des[0:3]`
   - `pos_des = body_pos_des[3:6]`
   - `foot_pose` 来自 `landing_pos_des`
   - `step_height` 以毫米为单位编码成复合数值
   - `acc_des = weight`
   - `value = use_mpc_traj`
   - `contact = floor(landing_gain * 10)`
   - `ctrl_point[2] = mu`
4. 写出完整参数文件 `*_full.toml`。
5. 先通过 `user_gait_file` 发送 `*_Def.toml` 内容。
6. 再通过 `user_gait_file` 发送 `*_full.toml` 内容。

当前使用的步态目录：

| 目录 | 用途 |
| --- | --- |
| `Gait_use/sec001_gait` | 第一关/初始自定义步态。 |
| `Gait_use/low_gait` | 低姿态通道前进步态。 |
| `Gait_use/low_gait_re` | 低姿态通道后续/恢复步态。 |
| `Gait_use/sec005_gait` | 保留参数文件，当前 `master.py` 未直接调用。 |

## 10. 关键参数汇总

### 10.1 常用运动模式

| mode | gait_id | 语义 |
| --- | --- | --- |
| 7 | 0/1 | 阻尼或休息相关动作。 |
| 11 | 10 | 常规移动/小跑控制，使用 `vel_des`。 |
| 11 | 12 | 另一组移动步态，第三/四关修正常用。 |
| 12 | 0 | 恢复站立。 |
| 16 | 1 | 原地跳远。 |
| 62 | 110 | 自定义步态执行。 |

### 10.2 视觉阈值

| 场景 | 阈值 |
| --- | --- |
| 黄色赛道线 HSV | `[30,30,30]` 到 `[90,255,255]` |
| 橙色球 HSV | `[10,130,80]` 到 `[100,210,160]` |
| 边线最小轮廓面积 | `2000` |
| 边线候选间距 | `100 < dist < 600` |
| 理想边线间距 | `300` |
| 鱼眼撞击判断占比 | `0.0362` |
| 鱼眼修正死区 | `abs(rgap) <= 0.01` |
| 第五关道路边界掩膜占比 | 需根据鱼眼安装角度和赛道边界宽度现场标定（当前第五关鱼眼掩膜逻辑已注释）。 |
| PID 死区 | `abs(gap) <= 4` 时 vyaw/vy 输出归零。 |
| PID vyaw 输出限幅 | `±0.30` rad/s。 |
| PID vy 输出限幅 | `±0.10` m/s。 |

## 11. 联调注意事项与已知问题

1. `src/launch/*.py` 和 `master.py` 的步态路径写死为 `/home/cyberdog_race2026_ws/...`。如果工作空间路径不同，需要同步修改路径或改为基于包路径动态查找。
2. ~~`master.py` 中 `fish_line_subscription` 当前订阅 `hit_ratio_topic`~~：已修复，当前正确订阅 `fish_line_topic`。
3. `master.py` 中 `publisher_mode` 被注释，因此视觉节点虽然订阅 `master_mode_topic`，实际不会收到主控模式广播。
4. 多个视觉节点的 `main()` 结束时调用 `node.Ctrl.quit()`，但这些节点没有 `Ctrl` 属性；如果 ROS spin 正常退出，可能触发异常。
5. 视觉节点中存在 `destory_node()` 拼写错误，应为 `destroy_node()`。
6. `Robot_Ctrl_Node.Wait_finish()` 检查 `mode_ok` 和 `gait_ok`，但当前 `msg_handler()` 只更新 `mode_ok`，没有更新 `gait_ok`，因此按设计等待具体 gait 完成时可能不准确。
7. `fisheyes_adjust_node.py` 中定义了鱼眼去畸变函数，但主流程没有调用；若实际相机畸变较强，需要评估是否启用。
8. 当前图像阈值、动作持续时间、速度参数都是场地强相关参数，换光照、相机曝光、赛道尺寸或电量状态后需要重新标定。
9. README 已整理为项目说明入口，后续若新增运行方式或参数配置，应同步更新 README 与本文档。
