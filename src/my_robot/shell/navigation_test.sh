#!/bin/sh

xterm  -e  "source ~/Home_Service_Robot_ws/devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm  -e  "source ~/Home_Service_Robot_ws/devel/setup.bash; roslaunch my_robot amcl.launch" 
