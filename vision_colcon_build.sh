#!/bin/bash
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash;
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ros_yolo

# Verify we're using system Python
echo "Using Python: $(which python3)"
echo "Python version: $(python3 --version)"
echo "PYTHONPATH: $PYTHONPATH"

# Clean build and rebuild with system environment
rm -rf build/ros_yolo install/ros_yolo
python -m colcon build --packages-select ros_yolo --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release;


# gnome-terminal -t "ros2" -x bash -c "#cd ~/ws_sensor_combined/;
#cd ~/ros2_ws;
# cd ~/ardupilot_ws;
# source /opt/ros/humble/setup.bash;
#colcon build;
# colcon build --packages-select px4_ros_com; 
#cd ~/ros2_ws;
#colcon build --packages-up-to ardupilot_gz_bringup;
# exec bash;"

# cd ~/ardupilot_ws;
# source /opt/ros/humble/setup.bash;
# MAKEFLAGS="-j1 " colcon build --executor sequential --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON;
