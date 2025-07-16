#!/bin/bash
docker run -it --rm all \
	--device /dev/dri:/dev/dri \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
    --user $USER_ID:$GROUP_ID \
    --env DISPLAY=$DISPLAY \
    --env ROS_LOCALHOST_ONLY=0 \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume $HOME/Downloads:$HOME/Downloads:rw \
    --volume $(pwd):/workspace:rw \
    --volume
    --privileged \
    --network host \
    --ipc host \
    --pid host \
    dxy_apm_ws:latest"

# --name dxy_apm_${MODE} \