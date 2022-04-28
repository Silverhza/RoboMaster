#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "modified_chassis/RechargeAction.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "robot_recharge_client_node");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<modified_chassis::RechargeAction> ac("findCamera", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  modified_chassis::RechargeGoal goal;
  goal.robot_id = 20;
  ac.sendGoal(goal);

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

  //exit
  return 0;
}
