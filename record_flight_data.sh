#!/bin/bash

# 飞行数据记录脚本
# 使用方法: ./record_flight_data.sh [output_name]

# 设置默认参数
OUTPUT_NAME=${1:-"flight_data_$(date +%Y%m%d_%H%M%S)"}

echo "开始记录飞行数据..."
echo "输出文件: ${OUTPUT_NAME}"

# 创建数据目录
mkdir -p ./logs

# 记录核心飞行数据
ros2 bag record \
    --output ./logs/${OUTPUT_NAME} \
    /mavros/setpoint_position/local \
    /mavros/setpoint_velocity/cmd_vel \
    /mavros/local_position/odom \
    /mavros/imu/data \
    /mavros/state \
    /mavros/battery \
    /mavros/global_position/global \
    /mavros/global_position/compass_hdg \
    /mavros/altitude \
    /mavros/home_position/home \
    /mavros/setpoint_raw/local \
    /mavros/setpoint_raw/attitude \
    /mavros/rangefinder/rangefinder \
    /detected_boxes \
    /statistics \
    /current_state \
    /tf \
    /tf_static

echo "数据记录完成，文件保存在: ./logs/${OUTPUT_NAME}"
echo "使用以下命令查看记录信息:"
echo "ros2 bag info ./logs/${OUTPUT_NAME}"
