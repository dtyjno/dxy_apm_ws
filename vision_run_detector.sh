#!/bin/bash
gnome-terminal --title "detector" -- bash -c "
source install/setup.bash;
ros2 run ros_yolo detector;
exec bash;
"