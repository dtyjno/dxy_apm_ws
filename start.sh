#!/bin/bash
# Check if the GeographicLib geoid model dataset is installed
if [ ! -f /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]; then
    echo "GeographicLib geoid model dataset not found! Installing..."
    sudo geographiclib-get-geoids egm96-5
fi

# Launch Gazebo
gnome-terminal --title "gazebo" -- bash -c "

source install/setup.bash;

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/code/ros2/dxy_apm_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo:$GZ_SIM_SYSTEM_PLUGIN_PATH;
export GZ_SIM_RESOURCE_PATH=$HOME/code/ros2/dxy_apm_ws/src/ros_gz_sim_ardupilot/models:$HOME/code/ros2/ardupilot_ws/src/ros_gz_sim_ardupilot/worlds:${GZ_SIM_SYSTEM_PLUGIN_PATH};
sleep 1;
ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py;
"

sleep 3s
gnome-terminal --title "SITL" -- bash -c "
sim_vehicle.py -D -v ArduCopter -L HDU -f gazebo-iris --model JSON --map --console \
  --out=udp:127.0.0.1:14550 \
  --out=tcp:127.0.0.1:5760 \
  --add-param-file=src/ros_gz_sim_ardupilot/config/gazebo-iris-gimbal.parm;
"
sleep 13s
gnome-terminal --title "mavros" -- bash -c "
source /opt/ros/humble/setup.bash;
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555;
"
#gnome-terminal -t "mavproxy" -x bash -c "mavproxy.py --console --map --aircraft test --master=:14550"
