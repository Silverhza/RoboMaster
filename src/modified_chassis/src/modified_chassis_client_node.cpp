#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "modified_chassis/MoveModeAction.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "chassis_client_node");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<modified_chassis::MoveModeAction> ac("moveAction", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  modified_chassis::MoveModeGoal goal;
  goal.mode = true;
  ac.sendGoal(goal);

 ROS_INFO("sleep");
sleep(3);
 ROS_INFO("new goal");
  goal.mode = false;
  ac.sendGoal(goal);
sleep(3);
 ROS_INFO("another new goal");
  goal.mode = true;
  ac.sendGoal(goal);

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
