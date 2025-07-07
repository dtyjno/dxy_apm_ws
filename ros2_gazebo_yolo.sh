#!/bin/bash

gnome-terminal -t "ros2" -x bash -c "
source /opt/ros/humble/setup.bash;
source install/local_setup.bash;
ros2 run topic image_sub_gazebo;
#exec bash;"

