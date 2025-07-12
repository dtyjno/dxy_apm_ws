#!/bin/bash

gnome-terminal --title "img_sub" -- bash -c "
source /opt/ros/humble/setup.bash;
source install/local_setup.bash;
ros2 run topic image_sub_gazebo;
#exec bash;"

