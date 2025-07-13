#!/bin/bash
gnome-terminal --title "control_print" -- bash -c "
source install/setup.bash;
ros2 run px4_ros_com offboard_control --ros-args -p print_info:=true;
"

