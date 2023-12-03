#!/bin/bash
cd $HOME/workspace2/team6_ws
colcon build
source install/setup.bash
ros2 run send_script img_sub &
ros2 run send_script send_script