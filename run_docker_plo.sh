#!/bin/bash

# 允许Docker容器访问X11显示
xhost +local:docker

docker run -it --rm  \
    --device /dev/dri:/dev/dri \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --env XDG_RUNTIME_DIR=/tmp \
    --env HOME=/tmp \
    --env ROS_LOCALHOST_ONLY=0 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume $(pwd):/workspace:rw \
    --volume $HOME/Downloads:/workspace/Downloads:rw \
    --workdir /workspace \
    --network host \
    --privileged \
    mavros:latest bash -c "
        source /opt/ros/humble/setup.bash && \
        if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
        mkdir -p /tmp/.config/PlotJuggler && \
        chmod 777 /tmp/.config/PlotJuggler && \
        ros2 run plotjuggler plotjuggler --buffer_size 42000
    "

# 恢复X11访问权限
xhost -local:docker

# --name dxy_apm_${MODE} \