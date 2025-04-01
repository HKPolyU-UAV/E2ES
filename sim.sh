launch_type=$1
launchfile=""

cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source ~/e2es_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/e2es_ws/src/E2ES/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/e2es_ws/src/E2ES/gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/e2es_ws/devel/lib
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_PLUGIN_PATH
# roslaunch e2es rviz.launch &
# sleep 2

if [ "$launch_type" == "map" ]; then
    launchfile="iris_d435_indoor"
elif [ "$launch_type" == "single" ]; then
    launchfile="vanilla"
elif [ "$launch_type" == "swarm" ]; then
    launchfile="swarm"
else
    echo "PLEASE CHECK YOUR INPUT!"
    exit 1
fi

roslaunch e2es $launchfile.launch pause:=false
