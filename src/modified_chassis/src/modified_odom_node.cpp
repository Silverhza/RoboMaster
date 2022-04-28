#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher  odom_pub;

geometry_msgs::PoseStamped amclPose;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

nav_msgs::Odometry modified_odom = *msg;

modified_odom.child_frame_id = "gimbal_fixed";
modified_odom.twist.twist.angular.z = 0;
/*
if(modified_odom.twist.twist.linear.x < 0.01)
{
modified_odom.twist.twist.linear.x = 0;
}

if(modified_odom.twist.twist.linear.y < 0.01)
{
modified_odom.twist.twist.linear.y = 0;
}
*/

odom_pub.publish(modified_odom); 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "modified_odom_node");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/dji_odom", 10, &odomCallback);
//  ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 10, &poseCallback);
odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);

    ros::Rate loop_rate(100);



  while(ros::ok()){


    ros::spinOnce();    
    loop_rate.sleep();
  } 

  return 0;
}
