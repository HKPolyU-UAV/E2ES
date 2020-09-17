# mav_sim_gazebo
MAV simulation on Gazebo

### Demo Video

### Introduction
This Kit provides an end to end simulation solution for MAV visual SLAM and path planning researches.
In this work, several features are added to the [PX4](https://github.com/PX4/Firmware) Gazebo Environment: <br />
-A realsense D435 model based on ([realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)) <br />
-Modified IRIS MAV model <br />
-Several structured/unstructured simulation world <br />
-Keyborad control support <br />
The work has been verified on Ubuntu 18.04 + ROS melodic environment <br />
We strongly recommend to you to run this simulation with CUDA supported graphic card!

### Usage
Follow the px4's [ROS with Gazebo Simulation](https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html) tutorial and setup basic gazebo environment accordingly<br />
Clone this repository to catkin src folder say: ~/catkin_ws/src
````
cd ~/catkin_ws/src
git clone https://github.com/Ttoto/mav_sim_gazebo.git
````
Compile
````
cd ~/catkin_ws/
catkin_make
````
Run the simulator
````
roscd mav_sim_gazebo
./sim.sh
````
If your work space in not "~/catkin_ws" or your PX4 Firmware is not located at "~/src/Firmware" you should edit the following line in the sim.sh script :
````
cd ~/src/Firmware
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/mav_sim_gazebo/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/mav_sim_gazebo/gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/devel/lib
````
to
````
cd PX4_Firmware_PATH
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/YOUR_WORK_SPACE/src/mav_sim_gazebo/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/YOUR_WORK_SPACE/src/mav_sim_gazebo/gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/YOUR_WORK_SPACE/devel/lib
````
Using keyboard to control the MAV in simulator
````
roslaunch mav_sim_gazebo keyboard_ctr.launch
````

### Acknowledgement
This work are based on [PX4 Projcet](https://github.com/PX4/Firmware) and [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)


### Maintainer
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />
