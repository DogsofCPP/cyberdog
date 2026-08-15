# Cyberdog Race 2026 Workspace

湖北工业大学 2026 年小米杯 Cyberdog 比赛代码工作区。

本仓库是一个 ROS 2 工作空间，主要用于 Cyberdog 比赛中的视觉感知、赛道/目标识别、运动主控、LCM 控制接口封装与自定义步态联调。当前主线已包含 `cyberdog_demo` 的比赛 Demo 实现，可作为后续联调、复盘和二次开发的基础。

## 项目内容

| 路径 | 说明 |
| --- | --- |
| `src/cyberdog_demo` | 比赛 Demo 主实现，包含主控节点、视觉识别节点、启动脚本入口和步态参数。 |
| `src/cyberdog_interfaces` | Cyberdog 运动控制相关 LCM 消息结构和辅助接口。 |
| `src/cyberdog_test` | 测试/占位 package。 |
| `src/launch` | 当前比赛 Demo 使用的启动脚本。 |
| `docs/` | 项目PPT、视频、技术文档、接口说明。 |

更完整的节点架构、ROS 话题、LCM 协议、视觉算法、运动流程和已知问题见：

[Cyberdog Demo 技术文档](docs/cyberdog_demo_technical_document.md)

[Cyberdog Race 2026 比赛完赛视频](docs/race.mp4)

## 环境要求

当前脚本按以下运行环境编写：

- Ubuntu/Linux 环境
- ROS 2 Galactic
- Python 3
- OpenCV / `python3-opencv`
- `cv_bridge`
- LCM
- Cyberdog 相关相机话题和运动控制板通信环境

启动脚本默认执行：

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

## 构建

在工作空间根目录执行：

```bash
colcon build
source install/setup.bash
```

提交或发起 PR 前，建议至少确认 `colcon build` 通过。

## 运行

### 一键启动 Demo

```bash
python3 src/launch/launch.py
```

该脚本会通过 `gnome-terminal` 依次启动：

- `master`
- `adjust_node`
- `fisheyes_adjust_node`
- `ground_node`

### 单节点启动

```bash
ros2 run cyberdog_demo master
ros2 run cyberdog_demo adjust_node
ros2 run cyberdog_demo fisheyes_adjust_node
ros2 run cyberdog_demo ground_node
ros2 run cyberdog_demo seek_001_node
ros2 run cyberdog_demo test_demo
```

### 测试启动入口

```bash
python3 src/launch/telaunch.py
```

该入口主要用于调试 `test_demo`、`adjust_node` 和 `fisheyes_adjust_node`。

## 注意事项

- 当前部分启动脚本和步态路径写死为 `/home/cyberdog_race2026_ws/...`，换工作空间路径后需要同步调整。
- 视觉阈值、运动速度、持续时间和循环次数都与比赛场地、光照、相机曝光、电量状态强相关，联调时需要重新标定。
- `build/`、`install/`、`log/` 不应提交到仓库。
- 技术实现细节和当前已知问题请优先查看技术文档。

## 目录与提交规范

允许提交的根目录内容：

- `src/`：正式 ROS 2 功能包代码目录
- `README.md`：项目说明与团队规范
- `docs/`：统一维护的设计文档、接口文档、联调说明
- `.gitignore`：忽略规则
- CI、格式化或静态检查配置

禁止提交的内容：

- `build/`
- `install/`
- `log/`
- 临时测试脚本
- 个人笔记
- 临时导出的数据
- 无法复现用途的调试文件

所有正式代码应放在 `src/<package_name>/` 下，并按 ROS 2 package 组织。

## 分支与 PR 规范

- `main`：稳定主线，只接收已经联调通过、可用于比赛或阶段演示的内容。
- `teamX/develop`：各小队长期集成分支。
- `feature/<team>-<topic>`：功能分支，例如 `feature/team2-lidar-detect`。

推荐流程：

1. 从本队 `teamX/develop` 或指定基线分支切出功能分支。
2. 在功能分支中完成开发和本地自测。
3. 按提交规范完成 commit。
4. 通过 PR 合入目标分支。
5. 合并前说明功能变化、测试情况和影响范围。

## Commit 规范

提交信息推荐使用以下前缀：

- `feat:` 新功能
- `fix:` 问题修复
- `refactor:` 重构
- `docs:` 文档更新
- `chore:` 杂项维护
- `test:` 测试相关

示例：

```text
feat: add team2 lidar obstacle detect node
fix: correct cmd_vel topic remap
docs: update cyberdog demo technical document
```

