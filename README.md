# Rm_opencv_armor_detection

| 编号 | 含义             | 序号 |
|------|------------------|------|
| B1   | 蓝方1号 装甲板   | 0    |
| B2   | 蓝方2号 装甲板   | 1    |
| B3   | 蓝方3号 装甲板   | 2    |
| B4   | 蓝方4号 装甲板   | 3    |
| B5   | 蓝方5号 装甲板   | 4    |
| B7   | 蓝方哨兵 装甲板   | 5    |
| R1   | 红方1号 装甲板   | 6    |
| R2   | 红方2号 装甲板   | 7    |
| R3   | 红方3号 装甲板   | 8    |
| R4   | 红方4号 装甲板   | 9    |
| R5   | 红方5号 装甲板   | 10   |
| R7   | 红方哨兵 装甲板   | 11   |

















# rm_vision

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

## Overview

rm_vision 项目旨在为 RoboMaster 队伍提供一个规范、易用、鲁棒、高性能的视觉框架方案，为 RM 开源生态的建设添砖加瓦

如果本开源项目对于贵战队的视觉技术发展起到了实质性的帮助作用，请在机器人上贴上以下标签以助力该项目的推广，十分感激！

[<img src="docs/rm_vision_inside.svg" alt="rm_vision_inside" width="100" height="100">](docs/rm_vision_inside.svg)

QQ 交流群：797203945

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

[![Build Status](https://github.com/chenjunnn/rm_vision/actions/workflows/ci.yml/badge.svg)](https://github.com/chenjunnn/rm_vision/actions/workflows/ci.yml)

## 包含项目

装甲板自动瞄准算法模块 https://github.com/chenjunnn/rm_auto_aim

MindVision 相机模块 https://github.com/chenjunnn/ros2_mindvision_camera

HikVision 相机模块 https://github.com/chenjunnn/ros2_hik_camera

机器人云台描述文件 https://github.com/chenjunnn/rm_gimbal_description

串口通讯模块 https://github.com/chenjunnn/rm_serial_driver

视觉算法仿真器 https://github.com/chenjunnn/rm_vision_simulator

## 通过 Docker 部署

拉取镜像

```
docker pull chenjunnn/rm_vision:lastest
```

构建开发容器

```
docker run -it --name rv_devel \
--privileged --network host \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ws:/ros_ws \
chenjunnn/rm_vision:lastest \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

构建运行容器

```
docker run -it --name rv_runtime \
--privileged --network host --restart always \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ws:/ros_ws \
chenjunnn/rm_vision:lastest \
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

TBD

## 源码编译

TBD
