# CyberDog2 真机迁移运行指导

本文档用于指导将当前仿真版 `cyberdog_race` 项目迁移到 CyberDog2 真机运行。

当前项目主要面向 Gazebo 仿真。迁移到真机时，不建议覆盖 CyberDog2 系统工程，而应作为独立用户项目部署，并逐步验证感知、控制、定位和各赛段策略。

## 1. 总体原则

不要覆盖 CyberDog2 原有系统代码。

推荐方式是在机器狗用户目录下新建项目目录，例如：

```bash
mkdir -p ~/dograce
```

然后把本项目的 `cyberdog_race` 目录复制进去。

推荐目标结构：

```text
~/dograce/
└── cyberdog_race/
    ├── race_main.py
    ├── lcm_controller.py
    ├── config.toml
    ├── navigation/
    ├── perception/
    ├── lcm_types/
    ├── motion/
    ├── utils/
    └── setup.py
```

不要直接覆盖以下目录：

```text
/opt/ros/
/home/cyberdog_ws/
/usr/lib/
/usr/local/
```

这些目录属于系统、ROS 或官方工作区。比赛代码应作为上层用户控制程序运行。

## 2. 复制代码到 CyberDog2

在宿主机执行：

```bash
rsync -av \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude 'cyberdog_race.egg-info' \
  /home/yyy/Desktop/dograce/project3130810-385179/cyberdog_race \
  cyberdog@机器狗IP:~/dograce/
```

如果不采用 `rsync`，也可以使用 `scp`：

```bash
scp -r /home/yyy/Desktop/dograce/project3130810-385179/cyberdog_race \
  cyberdog@机器狗IP:~/dograce/
```

**重要**：上述命令会复制整个 `cyberdog_race/` 目录，其中包含 `config/` 子目录（低姿态 gait 参数 TOML 文件）。Segment 4 的低姿态钻杆依赖 `config/gait_def_low_crawl.toml` 和 `config/gait_params_low_crawl.toml`，**确认这两个文件已经随项目一起复制到真机**：

```bash
ssh cyberdog@机器狗IP ls ~/dograce/cyberdog_race/config/
# 应该看到:
#   gait_def_low_crawl.toml
#   gait_params_low_crawl.toml
#   gait_def_bridge.toml
#   gait_params_bridge.toml
```

如果文件缺失，Segment 4 钻杆段会失败并打印 `Warning: Low-crawl gait upload failed`。

复制完成后，登录 CyberDog2：

```bash
ssh cyberdog@机器狗IP
cd ~/dograce
```

## 3. 设置运行环境

先加载 ROS2 环境：

```bash
source /opt/ros/galactic/setup.bash
```

推荐先使用 `PYTHONPATH` 方式运行，不修改系统 Python 包：

```bash
cd ~/dograce
export PYTHONPATH=~/dograce:$PYTHONPATH
```

如果后续确认依赖稳定，也可以使用 editable 安装：

```bash
cd ~/dograce/cyberdog_race
python3 -m pip install -e .
```

注意：不要随意创建虚拟环境或升级系统 Python 包。CyberDog2 的 `rclpy`、`cv_bridge` 等 ROS2 Python 模块通常与系统 Python 绑定，乱改容易破坏 ROS 环境。

## 4. 必须修改 real 模式

当前 `race_main.py` 的 `real` 模式仍然使用 `MockPerception`，不能直接用于真机。

需要把真机模式改为真实 ROS2 感知。

原逻辑类似：

```python
else:
    print("[Main] Real mode: using mock perception")
    from cyberdog_race.utils.mock_perception import MockPerception
    perception = MockPerception()
```

应改为：

```python
else:
    import threading
    import rclpy
    from cyberdog_race.perception.ros2_perception import ROS2Perception

    rclpy.init()
    perception = ROS2Perception()

    print("[Main] Real mode: waiting for robot sensors...")
    ready = perception.wait_until_ready(timeout_s=25.0)
    status = perception.sensor_status()

    if not ready and status.get('odom', False):
        print(f"[Main] Sensor readiness partial: {status}")
        print("[Main] Continuing with odom; scan/rgb-dependent behavior may degrade")
        ready = True

    if not ready:
        print(f"[Main] Sensor readiness timeout: {status}")
        ctrl.shutdown()
        rclpy.shutdown()
        return

    # 真机没有 Gazebo model_states，不调用 _calibrate_position()
    # 要求机器人启动时放在赛道起点，且 odom 原点与赛道原点一致。
    perception.set_position_offset(0.0, 0.0, 0.0)

    spin_thread = threading.Thread(
        target=_spin_node, args=(perception,), daemon=True)
    spin_thread.start()
    print("[Main] ROS2 perception started on real robot")
```

同时，`finally` 中关闭 ROS2 时不能只判断 `mode == 'sim'`。如果 real 模式也调用了 `rclpy.init()`，结束时也要 `rclpy.shutdown()`。

建议引入一个变量：

```python
rclpy_started = False
```

在 sim 或 real 初始化 ROS2 后设置：

```python
rclpy_started = True
```

结束时：

```python
if rclpy_started:
    rclpy.shutdown()
```

## 5. 确认真机 ROS2 topic

在 CyberDog2 上执行：

```bash
source /opt/ros/galactic/setup.bash
ros2 topic list -t
```

重点确认以下数据是否存在：

```text
里程计：/odom 或其他 odom topic
IMU：/imu 或其他 imu topic
雷达：/scan 或其他 scan topic
相机：/camera/image_raw、/camera/color/image_raw、/D435/image_raw 等
TF：/tf
```

当前代码默认订阅的 topic 包括：

```text
/scan
/imu
/odom
/tf
/D435/rgb/image_raw
/D435/image_raw
/D435/depth/image_raw
/camera/rgb/image_raw
/camera/image_raw
/camera/depth/image_raw
/camera/color/image_raw
```

如果 CyberDog2 上 topic 名称不同，需要修改：

```text
cyberdog_race/perception/ros2_perception.py
```

例如真机相机 topic 是 `/camera/color/image_raw`，但代码没有订阅，则在 `ROS2Perception.__init__()` 中添加：

```python
self._subscribe_one(sensor_msgs.Image, '/camera/color/image_raw', self._on_rgb, sensor_qos)
```

## 6. 检查传感器频率

只看 topic 存在不够，还要确认有数据且频率正常。

```bash
ros2 topic hz /odom
ros2 topic hz /imu
ros2 topic hz /scan
```

相机可检查：

```bash
ros2 topic hz /camera/image_raw
```

如果 topic 名称不同，替换成实际名称。

最低建议：

```text
/odom 可用
/imu 可用
/scan 可用，或代码中不依赖 scan
LCM 控制链路可用
```

如果 `/odom` 不稳定，基于 waypoint 的导航会直接失效。

## 7. 处理真机坐标校准

仿真中代码依赖 Gazebo 绝对位置做校准。真机没有 Gazebo，因此必须处理起点坐标。

首版建议采用最简单方案：

1. 机器人放在赛道起点。
2. 机头朝向代码约定的起点方向。
3. 启动程序时，认为当前 `/odom` 位置就是赛道原点。
4. 不调用 Gazebo 校准。

代码中使用：

```python
perception.set_position_offset(0.0, 0.0, 0.0)
```

如果实际 `/odom` 启动时不是 `(0, 0, 0)`，需要增加真机 offset。思路如下：

```python
raw_x, raw_y, raw_yaw = perception.odom_pose
perception.set_position_offset(-raw_x, -raw_y, 0.0)
```

但这要求启动时狗已经严格放在赛道原点。

## 8. 检查 LCM 控制链路

当前控制器使用：

```text
发送：udpm://239.255.76.67:7671
接收：udpm://239.255.76.67:7670
命令频道：robot_control_cmd
响应频道：robot_control_response
```

对应文件：

```text
cyberdog_race/lcm_controller.py
```

必须确认 CyberDog2 运动控制端监听相同频道。否则程序会正常运行，但机器狗不会动作。

先不要跑完整比赛。先测试基础动作。

## 9. 建议增加最小动作测试脚本

建议新增一个真机动作测试脚本，例如：

```text
cyberdog_race/test_real_basic.py
```

内容：

```python
import time
from cyberdog_race.lcm_controller import LCMController

ctrl = LCMController()
ctrl.start()

try:
    print("stand up")
    ctrl.stand_up_and_wait(height=0.24, wait_s=2.0)

    print("slow forward")
    ctrl.locomotion(vx=0.03, vy=0.0, wz=0.0, duration_ms=0)
    time.sleep(1.0)

    print("stop")
    ctrl.stop_moving()
    time.sleep(0.5)

    print("pure damper")
    ctrl.pure_damper()
finally:
    ctrl.shutdown()
```

运行：

```bash
cd ~/dograce
source /opt/ros/galactic/setup.bash
export PYTHONPATH=~/dograce:$PYTHONPATH
python3 cyberdog_race/test_real_basic.py
```

如果这一步不稳定，不要继续测试赛段。

## 10. 逐步测试顺序

不要直接运行完整比赛。

建议顺序：

### 10.1 感知测试

```bash
cd ~/dograce
source /opt/ros/galactic/setup.bash
export PYTHONPATH=~/dograce:$PYTHONPATH
python3 -m cyberdog_race.race_main --mode=real --perception-only
```

确认感知节点能启动，且没有 topic 长时间缺失。

### 10.2 基础动作测试

运行 `test_real_basic.py`，确认：

```text
站立正常
低速前进正常
停止正常
趴下或阻尼正常
```

### 10.3 单赛段测试

先测第一赛段：

```bash
python3 -m cyberdog_race.race_main --mode=real --test-segment=1 --timeout=120
```

之后按顺序测试：

```bash
python3 -m cyberdog_race.race_main --mode=real --test-segment=2 --timeout=180
python3 -m cyberdog_race.race_main --mode=real --test-segment=3 --timeout=180
python3 -m cyberdog_race.race_main --mode=real --test-segment=4 --timeout=240
python3 -m cyberdog_race.race_main --mode=real --test-segment=5 --timeout=300
python3 -m cyberdog_race.race_main --mode=real --test-segment=6 --timeout=180
```

### 10.4 完整流程测试

只有单赛段都能安全通过后，才运行完整流程：

```bash
python3 -m cyberdog_race.race_main --mode=real --timeout=900
```

## 11. 真机参数需要保守化

仿真参数不能默认等同于真机参数。

**首轮真机测试统一目标**：

```text
常规前进速度：0.05 ~ 0.12 m/s
转向角速度：0.15 ~ 0.30 rad/s
低姿态和桥面动作：单独测试，不直接全流程跑
```

### 11.1 哪些常量在真机上需要改

目前 `cyberdog_race/` 内的速度常量分散在各个 segment 文件中（无 `config.toml` 加载层）。在真机上跑之前先把这些常量手动降一档：

| 文件 | 常量 | 仿真值 | 真机建议 |
|---|---|---|---|
| `nav_helper.go_to_waypoint` | `default_speed` | 0.20 ~ 0.30 | 0.10 ~ 0.15 |
| `segments/segment_1.py` | `WALK_SPEED` | 0.20 | 0.10 |
| `segments/segment_1.py` | `TURN_WZ` | 0.34 | 0.20 |
| `segments/segment_2.py` | `SEARCH_SPEED` | 0.16 | 0.10 |
| `segments/segment_2.py` | `NAV_VX_MAX` | 0.16 | 0.10 |
| `segments/segment_3.py` | `_drive_world_error` vx 上限 | 0.32 | 0.16 |
| `segments/segment_5.py` | `SPEED_UPHILL` | 0.18 | 0.10 |
| `segments/segment_5.py` | `SPEED_TILT` | 0.14 | 0.08 |
| `segments/segment_5.py` | `SPEED_FLAT` | 0.22 | 0.12 |
| `segments/segment_5.py` | `TURN_RATE` | 0.50 | 0.30 |
| `segments/segment_5.py` | `TILT_VY_COMP_MPS` | 0.070 | 0.040 |
| `segments/segment_5.py` | `CURVED_TURN_VX_DEFAULT` | 0.06 | 0.04 |
| `segments/segment_6.py` | walkthrough speed | 0.22 | 0.12 |
| `segments/segment_6.py` | carry-mid speed | 0.26 | 0.14 |

> 说明：上表中的"真机建议"是首轮保守起步值。**确认无震动、保护触发之后再逐步提高**。当前项目没有 `config.toml` 加载层，所以"调速"必须直接改源码常量或重新 `scp` 上真机。

### 11.2 重点关注

```text
segment_1.py：石板步高、前进速度
segment_4.py：低姿态钻杆动作
segment_5.py：侧倾桥面、跳下动作
segment_6.py：宽步态、窄步态、带球参数
```

## 12. 高风险赛段说明

### 第四赛段

低姿态钻杆动作涉及身体高度、俯仰角和自定义步态。真机必须单独测试，不要第一次就放进全流程。

### 第五赛段

独木桥和跳下是真机高风险动作。建议：

```text
先在平地测试第五赛段的转向和姿态命令
再测试上桥
最后测试桥末跳下
```

### 第六赛段

宽步态和窄步态带球依赖运动控制参数。必须确认真机支持运行时 gait 参数修改。如果不支持，需要改成固定步态或预先配置步态参数。

## 13. 推荐真机启动脚本

项目已经自带启动脚本 `code/cyberdog_race/launch.sh`，它会自动：

- 检测并 source ROS2 (`/opt/ros/galactic/setup.bash` 或 `humble`)
- 在 real 模式下设置 `PYTHONPATH=~/dograce`
- 传递 `--mode=real`、`--timeout=900` 和任意额外参数
- 解析第二参数为赛段号（例如 `segment 1` → `--test-segment=1`）

脚本内注释对应本指南 §13。

项目根同时提供了一个薄 wrapper `run_real.sh`，转调 `launch.sh real "$@"`，方便从仓库根目录直接运行：

```text
.
├── run_real.sh                 # wrapper: bash code/cyberdog_race/launch.sh real "$@"
└── code/
    └── cyberdog_race/
        └── launch.sh           # 真正的启动器
```

**CyberDog2 端首次使用需赋权**：

```bash
chmod +x run_real.sh
chmod +x code/cyberdog_race/launch.sh
```

运行单赛段：

```bash
./run_real.sh segment 1
# 等价于
cd ~/dograce && bash code/cyberdog_race/launch.sh real segment 1
```

运行完整比赛：

```bash
./run_real.sh
# 等价于
cd ~/dograce && bash code/cyberdog_race/launch.sh real
```

如果不想用 wrapper，直接调用 `launch.sh` 也可：

```bash
bash code/cyberdog_race/launch.sh real --perception-only
bash code/cyberdog_race/launch.sh real --test-segment=2 --timeout=180
```

> **scp/rsync 复制后 `chmod +x` 会丢失**，每次拷贝到真机后都要重新赋权一次。

## 14. 真机运行前检查清单

运行完整比赛前，逐项确认：

### 14.1 文件与依赖

```text
[ ] 代码已复制到 ~/dograce/cyberdog_race
[ ] config/ 目录已一并复制（关键：gait_def_low_crawl.toml）
[ ] 未覆盖 CyberDog2 系统代码
[ ] PYTHONPATH=~/dograce 已 export
[ ] /opt/ros/galactic/setup.bash 已 source
[ ] espeak / espeak-ng 已安装（语音播报用）
[ ] `pip install -e .` 之后 `python -c "import cyberdog_race.config, os; print(os.listdir(os.path.dirname(cyberdog_race.config.__file__)))"` 能看到 4 个 `.toml`（setup.py 已配置 `package_data`）
```

### 14.2 模式与感知

```text
[ ] real 模式不再使用 MockPerception
[ ] ROS2 topic 名称已确认（见 §5 的 `ros2 topic list -t`）
[ ] /odom 数据正常（ros2 topic hz /odom 输出稳定频率）
[ ] /imu 数据正常（ros2 topic hz /imu 输出稳定频率）
[ ] /scan 数据正常，或相关逻辑已降级
[ ] （可选）RGB 相机 topic 已确认并加入 ros2_perception.py 订阅
```

### 14.3 控制链路

```text
[ ] LCM 控制链路正常（udpm://239.255.76.67:7671 收发）
[ ] stand_up 可用
[ ] stop_moving 可用
[ ] pure_damper 可用
[ ] 低速 locomotion 可用（跑过 test_real_basic.py）
[ ] 起点坐标已校准（odom 启动时狗放在 (0,0,0)，且朝向 +X）
[ ] run_real.sh 与 code/cyberdog_race/launch.sh 都已 chmod +x
```

### 14.4 速度与赛段

```text
[ ] 按 §11 表已调整相关常量
[ ] segment_1 已单独测试
[ ] segment_2 已单独测试（视觉检测若用真机 RGB 需先验证）
[ ] segment_3 已单独测试
[ ] segment_4 低姿态动作已单独测试（test_real_basic.py + test_low_crawl.py）
[ ] segment_5 桥面动作已单独测试（平地转向上桥跳下）
[ ] segment_6 宽/窄步态已单独测试（注意 _calibrate_fixed_start 与真机起点）
```

## 15. 结论

迁移代码的正确方式是：

```text
新建用户目录
复制 cyberdog_race 项目
修改 real 模式使用真实 ROS2 感知
适配真机 topic
校准起点坐标
验证 LCM 控制链路
从基础动作到单赛段逐步测试
最后运行完整比赛
```

不能只把代码移入 CyberDog2 后直接运行完整比赛。当前项目的仿真逻辑可以复用，但真机运行必须先完成真实感知、坐标校准、控制链路和安全动作验证。

## 16. 后续待办（不在本轮范围）

下面这些项目是有意识地推迟的，建议真机基础打通后再做：

- **测试脚本收尾**（已在本轮完成）：
  - `test_tilt_segment.py` / `test_seg4_from_entry.py` / `test_vy.py`
    原本 real 模式分支硬编码 `MockPerception()`，会让真机拿到假里程计。
    本轮已改为 `if mode in ('sim','real')` 都走 `ROS2Perception()`，
    real 模式额外 `set_position_offset(0,0,0)`，finally 用 `rclpy_started`
    跟踪优雅关闭。

- **起点坐标自动校准**：目前依赖手动把狗放在赛道原点开机。
  后续可加一段"开机后从 LiDAR 扫描赛道边界，反推 offset"的逻辑。

- **LiDAR 赛道边界闭环校准**：里程计在 4-5 米外开始漂移。
  后续可加周期性 LiDAR→赛道几何对齐，在 Seg1、Seg3、Seg5 等地方
  把累积误差归零。架构占位见 `code/cyberdog_race/navigation/navigator.py`
  现有的 `_check_deviation` / `_recover_deviation`。

- **统一速度 `config.toml` 加载层**：目前 §11 的"调速"必须直接改源码常量。
  后续可以把 §11 表里的所有数值搬到 `config/runtime_params.toml`，
  通过 `nav_helper` 加载，真机和仿真用同一套加载机制。

- **Segment 6 起点校准**：`_calibrate_fixed_start` 在真机起点不严格
  对齐时会出现抓球点偏移，需要配合 LiDAR 闭环一并解决。

- **MD 排版**：本文档部分小节（如 §2、§4）的反引号代码块与编号列表
  拼接处有视觉错位，不影响信息传达，留待统一排版重排。
