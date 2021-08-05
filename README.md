# realsense_mavros_bridge
This ROS package converts messages from /camera/odom/sample to /mavros/vision_pose/pose, which bridges between Realsense T265 and PX4 Mavros. If you can already use mocap and mavros, this package will make it very convenient to replace mocap with realsense VIO. If not, you can still use this package to fuse VIO and PX4 with the instructions. 

## Interface
Node [/bridge_ros_node]  
Publications: 
 * /mavros/vision_pose/pose [geometry_msgs/PoseStamped]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /camera/odom/sample [nav_msgs/Odometry]

## Dependencies
* ROS Melodic (tested)
* librealsense: https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md
* realsense2_camera: https://github.com/IntelRealSense/realsense-ros

## Installation
```buildoutcfg
cd ~/catkin_ws/src/
git clone https://github.com/ZhiangChen/realsense_mavros_bridge.git
cd ..
catkin build realsense_mavros_bridge
```

## Test
```buildoutcfg
roslaunch realsense2_camera rs_t265.launch
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 camera_odom_frame map 100
roslaunch realsense_mavros_bridge realsense_mavros_bridge.launch
rosrun rviz rviz
```
The static_tf_publisher is only used to visualize odom in map coordinates, which is the same coordinates of /mavros/vision_ppose/pose. The camera mount parameters are defined in the launch file `realsense_mavros_bridge.launch`:
```buildoutcfg
<arg name="rs_x" value="0" />
<arg name="rs_y" value="0" />
<arg name="rs_z" value="-0.1" />
<arg name="rs_roll" value="0" />
<arg name="rs_pitch" value="1.0471975512" />
<arg name="rs_yaw" value="0" />
```
These are the position and euler angles of T265 w.r.t. FCU mavros coordinates (forward, x; left, y; up, z). In this case, the camera is mounted 0.1 meters below the pixhawk, and has 60 degress pitch angle. Note, the position `xyz` should be jointly considered with PX4 parameters, `EKF2_EV_POS_X, EKF2_EV_POS_Y, EKF2_EV_POS_Z`. 

## How to fuse realsense and PX4
Follow the instructions in the document: Using Vision or Motion Capture Systems for Position Estimation (https://docs.px4.io/master/en/ros/external_position_estimation.html). If you already can use mocap, then you just need to replace `rosrun relay` with `roslaunch realsense_mavros_bridge realsense_mavros_bridge.launch`
