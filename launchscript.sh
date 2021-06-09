#!/bin/bash

cd ~
gnome-terminal --tab\
 -e "bash -c 'roslaunch gazebo_ros iris_world.launch';bash"

sleep 5

gnome-terminal --tab\
 -e "bash -c 'cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -f gazebo-iris';bash"

sleep 10

gnome-terminal --tab\
 -e "bash -c 'rosrun control_pkg server_node.py';bash"

gnome-terminal --tab\
 -e "bash -c 'rosrun control_pkg handler_node.py';bash"

