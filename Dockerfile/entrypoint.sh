#!/bin/bash

# Source ROS2 setup
. /opt/ros/"${ROS_DISTRO}"/setup.bash

# Change to workspace directory
cd /home/dxy_apm_ws

# Source workspace setup if it exists
if [ -f "install/setup.bash" ]; then
    . install/setup.bash
fi

# Set up environment variables for Gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/dxy_apm_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/home/dxy_apm_ws/src/ros_gz_sim_ardupilot/models:/home/dxy_apm_ws/src/ros_gz_sim_ardupilot/worlds:$GZ_SIM_RESOURCE_PATH

# Start dbus for GUI applications
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    export $(dbus-launch)
fi

# Execute the command
exec "$@"
