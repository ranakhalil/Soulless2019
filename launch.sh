export ROS_MASTER_URI=http://localhost:11311/
source devel/setup.bash
roscore &
# rosrun roscco roscco_node _can_channel=0
roslaunch src/roscco/example/example.launch
