# mav_sim_gazebo
### Introduction
<img src="others/mavsim.png" width="400">

This Kit provides an end to end simulation solution for MAV visual SLAM and path planning researches. <br />
In this work, several features are added to the default [PX4](https://github.com/PX4/Firmware) Gazebo Simulator: <br />
-A realsense D435 model (based on [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)) <br />
-Modified IRIS MAV model <br />
-Several structured/unstructured simulation world <br />
-Keyborad control support <br />
The work has been verified on Ubuntu 18.04 + ROS melodic environment <br />
We strongly recommend to you to run this simulation with CUDA supported graphic card!
### Demo Video

<a href="https://www.youtube.com/watch?v=sKkA5f62P6g" target="_blank"><img src="https://img.youtube.com/vi/sKkA5f62P6g/0.jpg" 
alt="cla" width="400" border="0" /></a>

| manual control    | Click and fly navigation   |
| ---------------------- | ---------------------- |
| <img src="others/manual_kb_ctl.gif" width="300">  | <img src="others/click_and_fly.gif" width="300">  |

### Usage
Follow the px4's [ROS with Gazebo Simulation](https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html) tutorial and setup basic gazebo environment accordingly<br />
Install pre-required packages
````
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-mavros-msgs libncurses5-dev 
````
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
If your work space in not "catkin_ws" or your PX4 Firmware is not located at "src/Firmware" you should edit the following line in the sim.sh script :
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
<img src="others/kbctr.png" width="300">

### Running End-To-End simulation
You can use the FLVIS-glmapping-FUXI(localization-mapping-planning kits) navigation system. <br />
The first step is to install [FLVIS](https://github.com/HKPolyU-UAV/FLVIS), [glmapping](https://github.com/HKPolyU-UAV/glmapping) and [FUXI](https://github.com/chenhanpolyu/fuxi-planner) accordinly. <br />
Then start the simulator localization mapping and planning kit in sequence. <br />
````
roscd mav_sim_gazebo
./sim.sh
roslaunch mav_sim_gazebo flvis.launch
roslaunch mav_sim_gazebo glmapping.launch
roslaunch mav_sim_gazebo fuxi.launch
````
You can use the 2D-Nav-Goal in RVIZ to publish your destination. <br />

### Acknowledgement
This work are based on [PX4 Projcet](https://github.com/PX4/Firmware) and [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)


### Maintainer
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />
Han Chen(Dept.AAE,PolyU):stark.chen@connect.polyu.
