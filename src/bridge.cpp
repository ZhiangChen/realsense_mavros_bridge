#include "realsense_mavros_bridge/bridge.h"

Bridge::Bridge(ros::NodeHandle& nh): nh_(nh){
    if(nh.getParam("rs_x", x_)){
        ROS_INFO("Got param rs_x: %f", x_);
    }
    else{
        ROS_ERROR("Failed to get param rs_x, using the default value 0");
    }

    if(nh.getParam("rs_y", y_)){
        ROS_INFO("Got param rs_y: %f", y_);
    }
    else{
        ROS_ERROR("Failed to get param rs_y, using the default value 0");
    }

    if(nh.getParam("rs_z", z_)){
        ROS_INFO("Got param rs_z: %f", z_);
    }
    else{
        ROS_ERROR("Failed to get param rs_z, using the default value 0");
    }

    if(nh.getParam("rs_roll", roll_)){
        ROS_INFO("Got param rs_roll: %f", roll_);
    }
    else{
        ROS_ERROR("Failed to get param rs_roll, using the default value 0");
    }

    if(nh.getParam("rs_pitch", pitch_)){
        ROS_INFO("Got param rs_pitch: %f", pitch_);
    }
    else{
        ROS_ERROR("Failed to get param rs_pitch, using the default value 0");
    }

    if(nh.getParam("rs_yaw", yaw_)){
        ROS_INFO("Got param rs_yaw: %f", yaw_);
    }
    else{
        ROS_ERROR("Failed to get param rs_yaw, using the default value 0");
    }

    tf2::Quaternion rs_quaternion;
    rs_quaternion.setRPY( roll_, pitch_, yaw_);
    rs_quaternion.normalize();

    rs_mount_transform_ = tf2::Transform(rs_quaternion, tf2::Vector3(x_, y_, z_));
    
    tf2::Quaternion rs_quaternion_init;
    rs_quaternion_init.setRPY( roll_, pitch_, 0);
    rs_quaternion_init.normalize();
    rs_init_transform_ = tf2::Transform(rs_quaternion, tf2::Vector3(0, 0, 0));

    rs_quaternion = rs_mount_transform_.getRotation();
    tf2::Vector3 xyz = rs_mount_transform_.getOrigin();
    ROS_INFO_STREAM("Realsense position: \n"<<tf2::toMsg(xyz));

    geometry_msgs::Quaternion rs_quaternion_msg = tf2::toMsg(rs_quaternion);
    ROS_INFO_STREAM("Realsense quaternion: \n" << rs_quaternion_msg);
    
    rs_mount_transform_inverse_ = rs_mount_transform_.inverse();
    rs_init_transform_inverse_ = rs_init_transform_.inverse();

    pub_vision_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 0);
    sub_vision_odom_ = nh_.subscribe("/camera/odom/sample", 1, &Bridge::odomCallback_, this);
}

void Bridge::odomCallback_(const nav_msgs::Odometry& odom){
    geometry_msgs::PoseStamped uav_pose;
    uav_pose.header = odom.header;
    uav_pose.header.frame_id = "map";

    geometry_msgs::Pose rs_pose = odom.pose.pose;
    tf2::Transform rs_transform;
    tf2::fromMsg(rs_pose, rs_transform);

    tf2::Transform uav_tranform = rs_mount_transform_*rs_init_transform_inverse_*rs_transform*rs_mount_transform_inverse_;
    tf2::toMsg(uav_tranform, uav_pose.pose);

    pub_vision_pose_.publish(uav_pose);
}
