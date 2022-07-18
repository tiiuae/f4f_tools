#!/bin/bash
source /opt/ros/galactic/setup.bash

echo "alias land=\"ros2 service call /"$DRONE_DEVICE_ID"/control_interface/land std_srvs/srv/Trigger\"" >> ~/.bashrc
echo "alias takeoff=\"ros2 service call /"$DRONE_DEVICE_ID"/control_interface/takeoff std_srvs/srv/Trigger\"" >> ~/.bashrc


bash
