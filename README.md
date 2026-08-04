# 初赛提交材料说明

## 学校信息

学校名称：武汉大学

## 队伍信息

队伍 ID：T2026104869910181

队伍名称：能做到

## 提交材料

### 设计文档

文件路径：/T2026104869910181-设计文档.pdf

### 作品说明

文件路径：/T2026104869910181-作品说明.pptx

### 作品演示视频

视频说明文件路径：/参赛视频链接.txt

百度网盘链接：https://pan.baidu.com/s/1hR128sGNFKhf3yuD0_hHiQ

提取码：w2b5

### 源代码

文件路径：/源代码/cyberdog_race/

主程序入口：/源代码/cyberdog_race/race_main.py

主要赛段逻辑：/源代码/cyberdog_race/navigation/segments/

## 初赛项目文件结构

```text
 /
├── README.md
├── T2026104869910181-作品说明.pptx
├── T2026104869910181-设计文档.pdf
├── 参赛视频链接.txt
└── 源代码/
    └── cyberdog_race/
        ├── README.md
        ├── __init__.py
        ├── config.toml
        ├── launch.sh
        ├── lcm_controller.py
        ├── race_main.py
        ├── requirements.txt
        ├── setup.py
        ├── config/
        │   ├── gait_def_bridge.toml
        │   ├── gait_def_low_crawl.toml
        │   ├── gait_params_bridge.toml
        │   └── gait_params_low_crawl.toml
        ├── lcm_types/
        │   ├── __init__.py
        │   ├── file_send_lcmt.py
        │   ├── robot_control_cmd_lcmt.py
        │   └── robot_control_response_lcmt.py
        ├── motion/
        │   ├── __init__.py
        │   └── programs.py
        ├── navigation/
        │   ├── __init__.py
        │   ├── course_map.py
        │   ├── navigator.py
        │   ├── state_machine.py
        │   └── segments/
        │       ├── __init__.py
        │       ├── segment_1.py
        │       ├── segment_2.py
        │       ├── segment_3.py
        │       ├── segment_4.py
        │       ├── segment_5.py
        │       └── segment_6.py
        ├── perception/
        │   ├── __init__.py
        │   ├── ros2_perception.py
        │   └── detectors/
        │       ├── __init__.py
        │       ├── ball_detector.py
        │       ├── boundary_detector.py
        │       ├── bridge_detector.py
        │       └── object_detector.py
        └── utils/
            ├── __init__.py
            ├── mock_perception.py
            ├── nav_helper.py
            └── speech.py
```

## 项目简介

本项目面向 2026 年智能系统创新设计赛“小米杯”赛题“荒野寻宝”，实现了一套基于 CyberDog 四足机器人的自主竞赛系统。系统通过 ROS2 感知节点获取相机、雷达、IMU 和里程计等信息，通过 LCM 控制接口发送运动指令，并使用任务状态机串联六个赛段的过关逻辑。

六个赛段分别由 `navigation/segments/segment_1.py` 至 `segment_6.py` 实现，主要包括石板路通过、小球搜索与撞击、曲道行进、深隧寻物、独木桥稳定通过和足球带出终点等任务。
