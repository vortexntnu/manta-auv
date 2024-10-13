#!/bin/bash

# The purpose of this file is to automate launching of
# all nodes found in this repository FOR ROBOSUB 2022
source /opt/ros/noetic/setup.bash
source /home/ubuntu/vortex_ws/devel/setup.bash

echo "##Activate thruster##"
/home/ubuntu/vortex_ws/src/Vortex-AUV/activate_thrusters.sh &
sleep 5s
echo "##Starting FSM##"
roslaunch finite_state_machine robosub_fsm.launch
