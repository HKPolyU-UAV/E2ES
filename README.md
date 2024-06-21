# E2ES
## End-to-End UAV Simulation for Visual SLAM and Navigation
### Introduction
<img src="others/mavsim.png" width="400">

This Kit provides an end to end simulation solution for MAV visual SLAM (or path planning) researches. <br />
In this work, several features are added to the default [PX4](https://github.com/PX4/Firmware) Gazebo Simulator: <br />
-A realsense D435 model (based on [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)) <br />
-Modified IRIS MAV model <br />
-Several structured/unstructured simulation world <br />
<!-- -Keyborad control support <br /> -->
The work has been verified on Ubuntu 18.04/20.04 + ROS melodic/noetic environment <br />
Also, if you want to save time,
we strongly suggest you to use docker image! 
The link is right [here](https://github.com/HKPolyU-UAV/airo_docker_lib).

### Demo Video

<a href="https://www.youtube.com/watch?v=sKkA5f62P6g" target="_blank"><img src="https://img.youtube.com/vi/sKkA5f62P6g/0.jpg" 
alt="cla" width="400" border="0" /></a>

| manual control    | Click and fly navigation   |
| ---------------------- | ---------------------- |
| <img src="others/manual_kb_ctl.gif" width="300">  | <img src="others/click_and_fly.gif" width="300">  |

### Main Functions
We include 3 main functionalities to users:
- Mapping environments for path planning and SLAM works.
- A single drone environment for controller design. Disturbances can be added.
- A team of 3 drones for swarm applications.

### Usage
If you still insist not to use docker, follow [this](/install.md) to do the installation.

Run the simulator
````
roscd e2es
./sim.sh map # for mapping environment and camera
./sim.sh single # for control testing, only 1 drone will be spawned
./sim.sh swarm # for control testing, 3 drones will be spawned
````
Using keyboard to control the MAV in simulator
````
roslaunch e2es keyboard_ctr.launch
````
<img src="others/kbctr.png" width="300">

You can also add disturbances to the environemnt via
```
roslaunch e2es disturb.launch
```

### More on End-To-End simulation
#### 1. SLAM
##### A. 
You can use the FLVIS + MLMapping kit combination to run SLAM. Please refer to the respective repo, [FLVIS](https://github.com/HKPolyU-UAV/FLVIS) & [MLMap](https://github.com/HKPolyU-UAV/MLMapping) to build them. Also make sure that you safisfy their prerequisites. 

The launch file for E2ES are [here_FLVIS](https://github.com/HKPolyU-UAV/FLVIS/blob/master/launch/e2es/e2es.launch) & [here_MAP](https://github.com/HKPolyU-UAV/MLMapping/blob/master/launch/mlmapping_e2es.launch). Hence, do
```
roslaunch flvis e2es.launch # for VIO
roslaunch mlmapping mlmapping_e2es.launch # for Mapping
```

##### B. 
The topics there you might be interested are as follows:

| Topic | Topic Type | Topic Name | Hz |
|----------|----------|----------|----------|
| Pose (GT) | geometry_msgs/PoseStamped | /gt_iris_base_link_imu | 50 |
| Pose (VIO) | geometry_msgs/PoseStamped | /mavros/vision_pose/pose | 200 |
| Odom (VIO) | nav_msgs/Odometry | /imu_odom | 200 |
| IMU (PX4) | sensor_msgs/IMU | /mavros/imu/data | 50 |
| IMU (Gazebo) | sensor_msgs/IMU | /iris/imu/data | 200 |
| RAW PCL | sensor_msgs/Image | /camera/depth_aligned_to_color_and_infra1/image_raw | 30 |
| MAP | sensor_msgs/PointCloud2 | /global_map | 20 |

You can modify the launch file, and 


#### 2. Autonomous Navigation (deprecated)
You can use the FLVIS-glmapping-FUXI(localization-mapping-planning kits) navigation system. <br />
The first step is to install [FLVIS](https://github.com/HKPolyU-UAV/FLVIS), [glmapping](https://github.com/HKPolyU-UAV/glmapping) and [FUXI](https://github.com/chenhanpolyu/fuxi-planner) accordinly. <br />
Then start the simulator localization mapping and planning kit in sequence. <br />
````
roscd e2es
./sim.sh

# flvis
roslaunch flvis e2es.launch # under flvis package

# fuxi
## please refer to fuxi repo
````

### Acknowledgement
This work are based on [PX4 Projcet](https://github.com/PX4/Firmware) and [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)


### Maintainer
[Patrick Lo](https://github.com/pattylo), AIRo-Lab, RCUAS, PolyU <br/> 
[Shengyang Chen](https://github.com/Ttoto), Dept.ME, PolyU <br />

