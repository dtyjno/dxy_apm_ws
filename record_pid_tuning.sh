#!/bin/bash

# PID调参数据记录脚本
# 使用方法: ./record_pid_tuning.sh [test_name]

TEST_NAME=${1:-"pid_test_$(date +%Y%m%d_%H%M%S)"}

echo "开始记录PID调参数据..."
echo "测试名称: ${TEST_NAME}"

# 创建调参数据目录
mkdir -p ./logs

# 记录PID相关数据
ros2 bag record \
    --output ./logs/${TEST_NAME} \
    /statistics

echo "PID调参数据记录完成，文件保存在: ./logs/${TEST_NAME}"
echo ""
echo "分析命令:"
echo "ros2 bag info ./logs/${TEST_NAME}"
echo "ros2 bag play ./logs/${TEST_NAME}"
echo ""
echo "在PlotJuggler中分析:"
echo "ros2 run plotjuggler plotjuggler"
echo "然后选择 File -> Load Rosbag -> ./logs/${TEST_NAME}"
