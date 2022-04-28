#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "obstacle_detector/Obstacles.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "roborts_msgs/TwistAccel.h"
#include "roborts_msgs/GameStatus.h"
#include <cmath>
#include <type_traits>
#include <tf/transform_listener.h>
#include <algorithm>


ros::Publisher  goal_pub;

geometry_msgs::PoseStamped current_pose;
ros::Subscriber brake_sub;
ros::Subscriber state_sub;
ros::Subscriber pose_sub ;

int state_meet_ctr = 0;

bool pose_normal = false;

void softBrake(){
  goal_pub.publish(current_pose);
}

void updatePose(const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  pose_normal = true;
}

void brakeCallback(const geometry_msgs::PoseStamped& msg)
{
  ROS_WARN("[BRAKE] THE BRAKE IS ON BY CALLING.");
  // goal_pub.publish(msg);
  softBrake();
}

void stateCallback(const roborts_msgs::GameStatus& msg)
{
  if (msg.game_status == 5 && state_meet_ctr < 300){

    if (state_meet_ctr == 0)
      ROS_WARN("[BRAKE] STATE: %d HENCE THE BRAKE IS ON.", msg.game_status);
    softBrake();
  }
  state_meet_ctr ++;

  if (msg.game_status == 0 || msg.game_status == 4)
    state_meet_ctr =0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "brake_node");
  ros::NodeHandle nh("~");


  // nh.param<float>("inflation", inflation_dist, 0.05);

  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
  {
    car_id = env_p;
  }
  else{
    std::cout << "not find ENV CAR_ID" << std::endl;
  }

  // std::shared_ptr <tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  
  // sub_for_brake = nh.subscribe<actionlib_msgs::GoalID>("/brake", 1000, &brakeCallback);
  brake_sub = nh.subscribe("/" + car_id + "/brake", 100, &brakeCallback);
  state_sub = nh.subscribe("/" + car_id + "/game_status", 100, &stateCallback);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + car_id + "/move_base_simple/goal", 5);
  pose_sub = nh.subscribe("/" + car_id + "/amcl_pose", 10, &updatePose);
  //pose_sub_ = local_planner_nh_.subscribe("/" + car_id + "/amcl_pose", 1, &LocalPlannerNode::UpdatePose, this);


  // if (car_id[3] == '2'){
  //   sub_for_merger = nh.subscribe<obstacle_detector::Obstacles>("/obstacle_merged", 1000, &mergerCallback);
  //   obstacle_preprocessed_pub = nh.advertise<obstacle_detector::Obstacles>("/obstacle_preprocessed", 1000);
  // }
  ROS_INFO("[BRAKE] BRAKER IS RUNNING");

  ros::spin();
  return 0;
}

