#!/bin/bash

# gnome-terminal --title "mavros" -- bash -c "
source install/setup.bash;
ros2 run px4_ros_com offboard_control;
exec bash;
# "


