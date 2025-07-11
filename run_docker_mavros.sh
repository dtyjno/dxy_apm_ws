#!/bin/bash
# 给出mavrosdocker启动命令
# # 基础启动命令
# docker run -it --rm \
#     --name mavros \
#     --network host \
#     mavros:humble

# # 直接启动MAVROS节点（连接到ArduPilot）
# docker run -it --rm \
#     --name mavros \
#     --network host \
#     mavros \
#     ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555

# # 使用串口连接
# docker run -it --rm \
#     --name mavros \
#     --network host \
#     --device /dev/ttyUSB0:/dev/ttyUSB0 \
#     mavros:humble \
#     ros2 launch mavros apm.launch fcu_url:="serial:///dev/ttyUSB0:57600"
# 

# UDP 连接方式（仿真模式）
docker run -it --rm \
    --name mavros \
    --env ROS_LOCALHOST_ONLY=0 \
    --privileged \
    --network host \
    --ipc host \
    --pid host \
    mavros \
    ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555

# 串口连接方式（真实硬件）
# docker run -it --rm \
#     --name mavros \
#     --env ROS_LOCALHOST_ONLY=0 \
#     --privileged \
#     --network host \
#     --ipc host \
#     --pid host \
#     --device /dev/ttyUSB0:/dev/ttyUSB0 \
#     mavros \
#     ros2 launch mavros apm.launch fcu_url:="serial:///dev/ttyUSB0:57600"
