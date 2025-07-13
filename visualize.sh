#!/bin/bash
gnome-terminal --title "visualize" -- bash -c "
source install/setup.bash;
ros2 run plotjuggler plotjuggler
"