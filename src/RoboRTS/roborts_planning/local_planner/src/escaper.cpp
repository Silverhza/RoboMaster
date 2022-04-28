#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "roborts_msgs/TwistAccel.h"
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

ros::Publisher cmd_vel_acc_pub_;
ros::Publisher cmd_vel_pub_;
ros::Publisher goal_pub;
ros::Subscriber sos_sub, sos_sub_local, pose_sub;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped ref_pose;

geometry_msgs::PoseStamped dummy_goal;

ros::Time sos_time, last_sos_time, sos_start_time;

bool stuck = false;
float vel = 0.3;
bool sent_dummy_goal = false;

bool pose_normal = false;
int quadrant = 1;
int sos_counter = 0;
int sos_counter2 = 0;
int pose_counter = 0; 

// void softBrake(){
//   goal_pub.publish(current_pose);
// }
float pose_dist(const geometry_msgs::PoseStamped& A, const geometry_msgs::PoseStamped& B){
  return std::sqrt(pow(A.pose.position.x - B.pose.position.x, 2) + pow(A.pose.position.y - B.pose.position.y, 2));
}


void poseCallback(const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  pose_counter ++;
  if (pose_counter%10 == 0 || pose_normal == false){
    ref_pose = current_pose;
    // ROS_INFO("[ESCAPER] REFERENCE POSE IS UPDATED");
  }
  pose_normal = true;
  // if (current_pose.pose.position.x > 4.2 && current_pose.pose.position.y > 2.4)
  //   quadrant = 1;
  // if (current_pose.pose.position.x < 4.2 && current_pose.pose.position.y > 2.4)
  //   quadrant = 2;
  // if (current_pose.pose.position.x < 4.2 && current_pose.pose.position.y < 2.4)
  //   quadrant = 3;
  // if (current_pose.pose.position.x > 4.2 && current_pose.pose.position.y < 2.4)
  //   quadrant = 4;
}



void sosCallback(const std_msgs::Empty& msg)
{

  sos_time = ros::Time::now();
  if (!stuck){
    stuck = true;
    sos_start_time = ros::Time::now();
  }
  


  // ROS_INFO("[ESCAPER] SOS SIGNAL GOT!");
  // sos_counter ++;
  // ros::Duration diff = ros::Time::now() - last_sos_time;
  // if (diff.toSec() > 10){
  //   sos_time = ros::Time::now();
  //   last_sos_time = sos_time;
  //   v = 0.3;
  // } // this is a new sos
  // ros::Duration curr_sos_dur = ros::Time::now() - sos_time;


  // if (pose_dist(current_pose, ref_pose) < 0.2)
  //   ROS_INFO("[ESCAPER] THE CAR IS NOT MOVING (%f) !", pose_dist(current_pose, ref_pose) );
  // ROS_WARN("[ESCAPER] DURATION IS %f !", curr_sos_dur.toSec() );

  // if (curr_sos_dur.toSec() > 2 && pose_dist(current_pose, ref_pose) < 0.2 && v == 0.3){
  //   ROS_WARN("[ESCAPER] TRYING TO MOVING FORWARD FOR 2 SEC BUT IT DIDN'T WORK ...");
  //   v = -0.3;
  // }
    

  roborts_msgs::TwistAccel cmd_vel;

  cmd_vel.twist.linear.x = vel;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = 0;

  cmd_vel.accel.linear.x = 0;
  cmd_vel.accel.linear.y = 0;
  cmd_vel.accel.angular.z = 0;

  cmd_vel_acc_pub_.publish(cmd_vel);


}

// void stateCallback(const roborts_msgs::GameStatus& msg)
// {
//   if (msg.game_status == 5 && state_meet_ctr < 300){

//     if (state_meet_ctr == 0)
//       ROS_WARN("[BRAKE] STATE: %d HENCE THE BRAKE IS ON.", msg.game_status);
//     softBrake();
//   }
//   state_meet_ctr ++;

//   if (msg.game_status == 0 || msg.game_status == 4)
//     state_meet_ctr =0;
// }
void send_dummy_goal(){
  dummy_goal.header.frame_id = "map";
  dummy_goal.header.stamp = ros::Time::now();
  dummy_goal.pose.position.x = 3.60;
  dummy_goal.pose.position.y = 2.16;
  dummy_goal.pose.position.z = 0;
  dummy_goal.pose.orientation.x = 0;
  dummy_goal.pose.orientation.y = 0;
  dummy_goal.pose.orientation.z = 0.75;
  dummy_goal.pose.orientation.w = 0.65;

  goal_pub.publish(dummy_goal);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "escaper_node");
  ros::NodeHandle nh("~");


  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
    car_id = env_p;
  else
    std::cout << "ENV CAR_ID NOT FOUND" << std::endl;


  last_sos_time = ros::Time::now();

  

  cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("/" + car_id +"/cmd_vel_acc", 100);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/" + car_id +"/cmd_vel", 100);
  // odom_sub_ = nh.subscribe("/" + car_id +"/odom", 100, &sosCallback);
  sos_sub = nh.subscribe("/" + car_id + "/global_planner_sos", 100, &sosCallback);
  sos_sub_local = nh.subscribe("/" + car_id + "/local_planner_sos", 100, &sosCallback);
  pose_sub = nh.subscribe("/" + car_id + "/amcl_pose", 100, &poseCallback);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + car_id + "/move_base_simple/goal", 50);



  // std_msg::Empty myMsg;
  // Publisher takeOff=n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);


  ROS_INFO("[ESCAPER] ESCAPER IS SAVING YOUR LIFE");
  sos_time = ros::Time::now();

  sos_counter2 = sos_counter;
  ros::Duration safe_time;

  ros::Rate loop_rate(10);
  while(ros::ok())
  {

    safe_time = ros::Time::now() - sos_time;
    if (safe_time.toSec() > 4){
      stuck = false;
      vel = 0.3;
    }
    if (stuck){
      ros::Duration curr_sos_dur = ros::Time::now() - sos_start_time;
      if ((int)curr_sos_dur.toSec()%16 > 8){
        vel = -0.3;
        ROS_WARN("[ESCAPER] TRYING TO MOVING FORWARD FOR 8 SEC BUT IT DIDN'T WORK ...");
      } else {
        vel = 0.3;
        ROS_WARN("[ESCAPER] TRYING TO MOVING FORWARD ...");
      }
      sent_dummy_goal = true;
      send_dummy_goal();
      ROS_INFO("[ESCAPER] DUMMY GOAL SENT!");
    }

    ros::spinOnce();
    loop_rate.sleep();
    }
  return 0;
}

