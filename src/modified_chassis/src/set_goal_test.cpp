#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "roborts_msgs/GlobalPlannerAction.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "chassis_client_node");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction> ac("global_planner_node_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  roborts_msgs::GlobalPlannerGoal recharge_point;
           recharge_point.goal.header.frame_id="map";
           recharge_point.goal.pose.position.x = 1.23488044739;
            recharge_point.goal.pose.position.y = 2.93630886078;
            recharge_point.goal.pose.position.z = 0;
            recharge_point.goal.pose.orientation.x = 0;
            recharge_point.goal.pose.orientation.y = 0;
            recharge_point.goal.pose.orientation.z = 0;
            recharge_point.goal.pose.orientation.w = 1;
  ac.sendGoal(recharge_point);

 ROS_INFO("sleep");
/*
sleep(3);
 ROS_INFO("new goal");
  goal.mode = false;
  ac.sendGoal(goal);
sleep(3);
 ROS_INFO("another new goal");
  goal.mode = true;
  ac.sendGoal(goal);
*/
/*
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
//actionlib::ResultConstPtr res_ptr = ac.getResult();

 //   ROS_INFO("Action result: %d", *res_ptr);
    ROS_INFO("Action result: , *res_ptr");

*/
  //exit
  return 0;
}
