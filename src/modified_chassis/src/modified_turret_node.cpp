#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include "roborts_msgs/GimbalAngle.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <cmath>
#include <iostream>

roborts_msgs::GimbalAngle gimbalCmd;
std_msgs::Float64 distMsg;
tf::Quaternion goal_quat;
tf::Quaternion current_quat;
tf::Point current_position;
tf::Point goal_position;
tf::Quaternion first_quat;
double dist;
bool flag = false;

ros::Publisher turret_pub;
ros::Publisher dist_pub;

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  flag = true;
  tf::Point pt(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
  goal_position = pt;
  tf::Quaternion quat(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
  goal_quat = quat;

  double yaw_angle = atan2(goal_position.y() - current_position.y(), goal_position.x() - current_position.x());
  first_quat = tf::createQuaternionFromYaw(yaw_angle);

  tf::Matrix3x3 goal_m(first_quat);
  tf::Matrix3x3 current_m(current_quat);
  tf::Matrix3x3 tmp_m(first_quat);

  tmp_m = goal_m * current_m.inverse();

  double roll, pitch, yaw;
  tmp_m.getRPY(roll, pitch, yaw);

  if (fabs(yaw_angle) > 0.1)
  {
    gimbalCmd.yaw_mode = true;
    //gimbalCmd.yaw_angle =yaw*0.2546;
    gimbalCmd.yaw_angle = yaw_angle;
    ROS_INFO("yaw_angle: %f" , yaw_angle);
    turret_pub.publish(gimbalCmd);
    ROS_INFO("in");
  }
}

int i = 0;
void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  tf::Quaternion quat(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
  current_quat = quat;

  tf::Point pt(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
  current_position = pt;

  dist = tf::tfDistance(current_position, goal_position);

  distMsg.data = dist;
  dist_pub.publish(distMsg);

  // ROS_INFO("distance: %f" , dist);

  if (dist <= 0.4 && !goal_position.isZero())
  {
    tf::Matrix3x3 goal_m(goal_quat);
    tf::Matrix3x3 current_m(current_quat);
    tf::Matrix3x3 tmp_m(goal_quat);

    tmp_m = goal_m * current_m.inverse();

    double roll, pitch, yaw;
    tmp_m.getRPY(roll, pitch, yaw);

    // ROS_INFO("yaw:%f", yaw);
    // ROS_INFO("quat.x:%f", goal_quat.x());
    // ROS_INFO("quat.y:%f", goal_quat.y());
    // ROS_INFO("quat.z:%f", goal_quat.z());
    // ROS_INFO("quat.w:%f", goal_quat.w());

    if (fabs(yaw) > 0.1 && flag)
    {
      gimbalCmd.yaw_mode = true;
      //gimbalCmd.yaw_angle =yaw*0.2546;
      gimbalCmd.yaw_angle = yaw;
      turret_pub.publish(gimbalCmd);
      ROS_INFO("in");
    }
    flag = false;
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "modified_turret_node");
  ros::NodeHandle nh;

  ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 10, &goalPoseCallback);
  ros::Subscriber current_pose_sub = nh.subscribe("amcl_pose", 10, &currentPoseCallback);
  turret_pub = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1000);
  dist_pub = nh.advertise<std_msgs::Float64>("distance_to_goal", 1000);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
