#!/bin/bash

# Source ROS2 setup
. /opt/ros/"${ROS_DISTRO}"/setup.bash

# Start dbus for GUI applications
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    export $(dbus-launch)
fi

# Execute the command
exec "$@"
