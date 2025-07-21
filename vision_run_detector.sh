#!/bin/bash
gnome-terminal --title "detector" -- bash -c "
conda deactivate 2>/dev/null;
source /opt/ros/jazzy/setup.bash;
source ~/miniconda3/etc/profile.d/conda.sh;
conda activate ros_yolo;

# Verify we're using system Python
echo 'Using Python: $(which python3)';
echo 'Python version: $(python3 --version)';
echo 'PYTHONPATH: $PYTHONPATH';

source install/setup.bash;
ros2 run ros_yolo detector;
exec bash;
"