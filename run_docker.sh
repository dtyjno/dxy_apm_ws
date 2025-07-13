#!/bin/bash

# Docker运行脚本 - 多功能启动
# 支持Gazebo仿真、ArduPilot SITL和MAVROS
# 自动检测并使用GPU设备

# 使用说明
usage() {
    echo "使用方法: $0 [选项]"
    echo "选项:"
    echo "  gazebo    - 启动Gazebo仿真"
    echo "  sitl      - 启动ArduPilot SITL"
    echo "  mavros    - 启动MAVROS"
    echo "  offboard  - 启动Offboard控制"
    echo "  bash      - 启动交互式bash shell"
    echo "  full      - 启动完整仿真环境（Gazebo + SITL + MAVROS）"
    echo ""
    echo "示例:"
    echo "  $0 gazebo"
    echo "  $0 full"
    echo "  $0 bash"
}

# 检查参数
if [ $# -eq 0 ]; then
    echo "错误: 需要指定启动模式"
    usage
    exit 1
fi

MODE=$1

# 允许本地连接到X服务器
xhost +local:docker

# 获取当前用户ID
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# 检测GPU设备可用性
detect_gpu() {
    local gpu_available=false
    local gpu_runtime=""
    local gpu_args=""
    
    # 检查NVIDIA GPU
    if command -v nvidia-smi &> /dev/null; then
        if nvidia-smi &> /dev/null; then
            echo "检测到NVIDIA GPU" >&2
            gpu_available=true
            gpu_runtime="nvidia"
            gpu_args="--gpus all --runtime=nvidia"
            # 检查nvidia-container-toolkit是否安装
            if ! docker info 2>/dev/null | grep -q nvidia; then
                echo "警告: nvidia-container-toolkit未正确安装，将使用CPU模式" >&2
                gpu_available=false
                gpu_args=""
            fi
        else
            echo "NVIDIA GPU驱动未正确安装" >&2
        fi
    fi
    
    # 检查AMD GPU (ROCm)
    if [ "$gpu_available" = false ] && command -v rocm-smi &> /dev/null; then
        if rocm-smi &> /dev/null; then
            echo "检测到AMD GPU (ROCm)" >&2
            gpu_available=true
            gpu_runtime="rocm"
            gpu_args="--device=/dev/kfd --device=/dev/dri --security-opt seccomp=unconfined --group-add video"
        fi
    fi
    
    # 检查Intel GPU
    if [ "$gpu_available" = false ] && [ -d "/dev/dri" ]; then
        echo "检测到Intel GPU或集成显卡" >&2
        gpu_available=true
        gpu_runtime="intel"
        gpu_args="--device /dev/dri:/dev/dri"
    fi
    
    if [ "$gpu_available" = false ]; then
        echo "未检测到可用的GPU设备，使用CPU模式" >&2
        gpu_args=""
    fi
    
    echo "$gpu_args"
}

# 获取GPU参数
GPU_ARGS=$(detect_gpu)

# 基础Docker运行命令
DOCKER_CMD="docker run -it --rm --gpus all \
	--device /dev/dri:/dev/dri \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
    --name dxy_apm_${MODE} \
    --user $USER_ID:$GROUP_ID \
    --env DISPLAY=$DISPLAY \
    --env ROS_LOCALHOST_ONLY=0 \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --privileged \
    --network host \
    --ipc host \
    --pid host \
    dxy_apm_ws:latest"

case $MODE in
    "gazebo")
        echo "启动Gazebo仿真..."
        $DOCKER_CMD bash -c "ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py"
        ;;
    "sitl")
        echo "启动ArduPilot SITL..."
        $DOCKER_CMD bash -c "sim_vehicle.py -D -v ArduCopter -L HDU -f gazebo-iris --model JSON --map --console --out 127.0.0.1:14550 --out 127.0.0.1:14551"
        ;;
    "mavros")
        echo "启动MAVROS..."
        $DOCKER_CMD bash -c "ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555"
        ;;
    "bash")
        echo "启动交互式bash shell..."
        $DOCKER_CMD bash
        ;;
    "full")
        echo "启动完整仿真环境..."
        echo "请在容器内手动启动各个组件:"
        echo "1. Gazebo: ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py"
        echo "2. SITL: sim_vehicle.py -D -v ArduCopter -L HDU -f gazebo-iris --model JSON --map --console --out 127.0.0.1:14550 --out 127.0.0.1:14551"
        echo "3. MAVROS: ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555"
        $DOCKER_CMD bash
        ;;
    "offboard")
        echo "启动Offboard控制..."
        $DOCKER_CMD bash -c "ros2 run px4_ros_com offboard_control"
        ;;
    *)
        echo "错误: 未知的启动模式 '$MODE'"
        usage
        exit 1
        ;;
esac

# 恢复X服务器访问权限
xhost -local:docker