#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "obstacle_detector/Obstacles.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "roborts_msgs/TwistAccel.h"
#include <cmath>
#include <type_traits>
#include <tf/transform_listener.h>
#include <algorithm>



ros::Subscriber sub_for_inspection;
geometry_msgs::PoseStamped initial_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped dummy_goal;
ros::Subscriber pose_sub ;
ros::Publisher goal_pub;
ros::Subscriber goal_sub;

bool pose_normal = false;
bool navi_normal = false;
bool base_normal = false;

void updatePose(const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  pose_normal = true;
  if (initial_pose.pose.position.x != 0 && (abs(initial_pose.pose.position.x - current_pose.pose.position.x) > 0.1 || abs(initial_pose.pose.position.y - current_pose.pose.position.y) > 0.1)){
    base_normal = true;
    // ROS_INFO("[INSPECTION] NAVIGATION IS NORMAL.");
    // ROS_INFO("[INSPECTION] %f VS %f", initial_pose.pose.position.x, current_pose.pose.position.x);
  }
}

void updateGoal(const geometry_msgs::PoseStamped &msg) {
  navi_normal = true;
}

void test_navi(){

  

  dummy_goal.header.frame_id = "map";
  dummy_goal.header.stamp = ros::Time::now();
  dummy_goal.pose.position.x = 3.60;
  dummy_goal.pose.position.y = 2.16;
  dummy_goal.pose.position.z = 0;
  dummy_goal.pose.orientation.x = 0;
  dummy_goal.pose.orientation.y = 0;
  dummy_goal.pose.orientation.z = 0.75;
  dummy_goal.pose.orientation.w = 0.65;

  // ROS_INFO("[INSPECTION] TEST GOAL (%d, %d) IS PUBLISHED ", dummy_goal.pose.position.x, dummy_goal.pose.position.y);

  goal_pub.publish(dummy_goal);
  ROS_INFO("[INSPECTION] TESTING GOAL (%d, %d) IS PUBLISHED ", dummy_goal.pose.position.x, dummy_goal.pose.position.y);

  for (int i=0; i<3; i++){
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  goal_pub.publish(initial_pose);
  ROS_INFO("[INSPECTION] INITIAL GOAL (%d, %d) IS PUBLISHED ", initial_pose.pose.position.x, initial_pose.pose.position.y);
  
  if (pose_normal && navi_normal && base_normal)
    ROS_WARN("[INSPECTION] SELF INSPECTION PASSED !!!!!!");
  else {
    ROS_ERROR("[INSPECTION] SELF INSPECTION FAILED !!!!!!");
    test_navi();

    ros::Duration(1).sleep();
  
    test_navi();
  }


}

void inspectionCallback(const geometry_msgs::PoseStamped& msg)
{
  ROS_WARN("[INSPECTION] SUCESSFULLY CALLED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_filter_node");
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

  pose_sub = nh.subscribe("/" + car_id + "/amcl_pose", 10, &updatePose);
  goal_sub = nh.subscribe("/" + car_id + "/move_base_simple/goal", 10, &updateGoal);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + car_id + "/move_base_simple/goal", 5);

  // std::shared_ptr <tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  

  // sub_for_inspection = nh.subscribe("CAR2/inspection", 100, &inspectionCallback);
  //pose_sub_ = local_planner_nh_.subscribe("/" + car_id + "/amcl_pose", 1, &LocalPlannerNode::UpdatePose, this);


  // if (car_id[3] == '2'){
  //   sub_for_merger = nh.subscribe<obstacle_detector::Obstacles>("/obstacle_merged", 1000, &mergerCallback);
  //   obstacle_preprocessed_pub = nh.advertise<obstacle_detector::Obstacles>("/obstacle_preprocessed", 1000);
  // }
  int wait_counter = 0;
  ros::Rate rate(10);
  while (!pose_normal){
    if (wait_counter>1000 && wait_counter%1000==0)
      ROS_INFO("[INSPECTION] WAITTING FOR POSE MESSAGE FROM AMCL.");
    rate.sleep();
    ros::spinOnce();
    wait_counter++;
  }
  // ROS_WARN("[INSPECTION] SUCESSFULLY START!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  ros::Duration(1.5).sleep();
  initial_pose = current_pose;
  test_navi();

  ros::spin();
  return 0;
}

