cmake_minimum_required(VERSION 2.8.3)
project(e2es)

add_compile_options(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
    std_msgs
    mavros_msgs
    geometry_msgs
    )

find_package (gazebo REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (yaml-cpp REQUIRED)
find_package (Sophus REQUIRED )
find_package (Curses REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(Sophus_LIBRARIES libSophus.so)

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${CURSES_INCLUDE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/"
    )
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(realsense_gazebo_plugin
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
    )

target_link_libraries(realsense_gazebo_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})
add_dependencies(realsense_gazebo_plugin
    ${catkin_EXPORTED_TARGETS})

add_executable(keyboard_ctl
    src/keyboard_ctl.cpp
    src/movement/generalmove.cpp
    )
target_link_libraries(keyboard_ctl
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${CURSES_LIBRARIES}
    )

add_executable(offb
    src/offb.cpp
    )
target_link_libraries(offb
    ${catkin_LIBRARIES}
    )
