#!/usr/bin/env bash

# Start ssh server
echo f1tenth | sudo -S -k service ssh start

# Source ROS2 packages
source /opt/ros/$ROS_DISTRO/setup.bash

#cd /workspace && colcon build --symlink-install
#source /workspace/install/setup.bash

# Start user prompt and keep it open
if [ -z "${DISPLAY}" ] 
then
#    ros2 launch demo_system_1115 demo_system.launch &
    tail -f /dev/null
    #/bin/sh
else
    GTK_THEME=Adwaita terminator
fi
	
