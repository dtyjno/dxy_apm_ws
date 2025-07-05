#!/bin/bash

gnome-terminal -t "mavros" -x bash -c "#cd ~/ws_sensor_combined/;
#cd ~/ardupilot_ws;
source /opt/ros/humble/setup.bash;
#source install/local_setup.bash;
#ros2 launch ard-mavros apm.launch.xml;
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555;

# sudo chmod 777 /dev/ttyACM0 
# 或
# sudo usermod -aG dialout username

ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600;

exec bash;"

# Bus 001 Device 002: ID 1209:5741 Generic fmuv2
#colcon build --allow-overriding mavros

# usbipd list
# sudo usbipd bind --busid 5-1
# usbipd attach --wsl --busid 5-1
# usbipd detach --busid 5-1

# /dev/bus/usb/001/003

# 查看设备是否已被正确识别为 CDC ACM 设备：
# dmesg | grep tty
