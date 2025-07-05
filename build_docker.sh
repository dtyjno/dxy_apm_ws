#!/bin/bash

# Docker镜像构建脚本

echo "开始构建DXY APM Docker镜像..."

# 切换到项目根目录
cd /home/linhao/code/ros2/dxy_apm_ws

# 构建Docker镜像
docker build -t dxy_apm_ws:latest -f Dockerfile/Dockerfile .

if [ $? -eq 0 ]; then
    echo "✅ Docker镜像构建成功!"
    echo "使用以下命令运行:"
    echo "  ./run_docker.sh gazebo    # 启动Gazebo仿真"
    echo "  ./run_docker.sh bash      # 启动交互式shell"
    echo "  ./run_docker.sh full      # 启动完整环境"
else
    echo "❌ Docker镜像构建失败!"
    exit 1
fi
