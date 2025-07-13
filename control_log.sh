#!/bin/bash
gnome-terminal --title "control_log" -- bash -c "
source install/setup.bash;
./record_flight_data.sh;
exec bash;
"