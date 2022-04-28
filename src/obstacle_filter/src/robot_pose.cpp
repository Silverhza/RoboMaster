//
// Created by lima on 2020/1/1.
//
#include "../include/obstacle_filter/robot_pose.h"

robot_pose::robot_pose(tf::TransformListener& tf,std::string jackal_ns):tf_(tf),jackal_ns_(jackal_ns){
    global_frame_="/map";
    if (jackal_ns_.find("CAR1") != std::string::npos)
        robot_base_frame_= "CAR2/base_link";
        // robot_base_frame_= !system("rosnode list|grep /CAR2_obstacle_filter_node") ? "CAR2/base_link" :"CAR1/base_link";
    if (jackal_ns_.find("CAR2") != std::string::npos)
        robot_base_frame_= "CAR1/base_link" ;
        // robot_base_frame_= !system("rosnode list|grep /CAR2_obstacle_filter_node") ? "CAR2/base_link" :"CAR1/base_link";
    if (jackal_ns_.find("CAR3") != std::string::npos)
        robot_base_frame_= "CAR4/base_link";
        // robot_base_frame_= !system("rosnode list|grep /CAR2_obstacle_filter_node") ? "CAR2/base_link" :"CAR1/base_link";
    if (jackal_ns_.find("CAR4") != std::string::npos)
        robot_base_frame_= "CAR3/base_link";
        // robot_base_frame_= !system("rosnode list|grep /CAR2_obstacle_filter_node") ? "CAR2/base_link" :"CAR1/base_link";

    std::string tf_error;
    ros::Time last_error = ros::Time::now();
    int error_counter = 0;

    while (ros::ok() && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), \
         ros::Duration(0.01), &tf_error)) {
        // ROS_INFO("[FILTER] WAITTING FOR TRANSFORM FROM ");
        if (error_counter % 1000 == 0)
        ROS_INFO("[FILTER] %s", tf_error.c_str());
        ros::spinOnce();
        if (last_error + ros::Duration(5.0) < ros::Time::now()) {
            last_error = ros::Time::now();
        }
        tf_error.clear();
        error_counter +=1;
    }
}
robot_pose::~robot_pose(){

}
bool robot_pose::GetRobotPose(tf::Stamped<tf::Pose> &global_pose) const{
    global_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now();
    try {
        tf_.transformPose(global_frame_, robot_pose, global_pose);
    }
    catch (tf::LookupException &ex) {
        ROS_ERROR("No Transform Error looking up robot pose: %s", ex.what());
        return false;
    }
    catch (tf::ConnectivityException &ex) {
        ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException &ex) {
        ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
        return false;
    }
    return true;
}

bool robot_pose::GetRobotPose(geometry_msgs::PoseStamped &global_pose) const{
    tf::Stamped<tf::Pose> tf_global_pose;
    if (GetRobotPose(tf_global_pose)) {
        tf::poseStampedTFToMsg(tf_global_pose, global_pose);
        return true;
    } else {
        return false;
    }
}
