#!/bin/bash

# pip install MAVproxy wxpython

# pip install future lxml pymavlink pyserial MAVProxy geocoder empy==3.3.4 ptyprocess dronecan flake8 junitparser wsproto tabulate  pygame intelhex numpy pyparsing psutil matplotlib scipy opencv-python pyyaml  wxpython opencv-python  -i https://pypi.tuna.tsinghua.edu.cn/simple
# mavproxy:
# vim /home/linhao/.local/lib/python3.13/site-packages/MAVProxy/modules/mavproxy_map/mp_tile.py
# :615
# img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
# :wq



# 启动MAVProxy作为主要中继
gnome-terminal -t "mavproxy" -x bash -c "
# MAVProxy连接SITL并转发到多个端口
mavproxy.py \
    --master=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14552 \
    --console --moddebug 3;
exec bash;
"
# 等待MAVProxy启动
sleep 3


# 启动MAVROS连接到MAVProxy转发的端口
gnome-terminal -t "mavros" -x bash -c "
# cd /home/linhao/code/ros2/dxy_apm_ws;
source /opt/ros/jazzy/setup.bash;

ros2 launch mavros apm.launch \
    fcu_url:=udp://127.0.0.1:14551@14555 \
    config_yaml:=./apm_config.yaml;
exec bash;
"