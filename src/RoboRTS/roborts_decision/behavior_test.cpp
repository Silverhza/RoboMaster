#include <ros/ros.h>
#include<bits/stdc++.h>
// #include "executor/chassis_executor.h"

#include "example_behavior/goal_behavior.h"

ros::Subscriber debug_sub_2;

void DebugCallback2(const geometry_msgs::PoseStamped::ConstPtr& goal){
  ROS_WARN("[DECISION] DEBUG SIGNAL GOT (MAIN) !");

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    // load namespace
  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
	{
    car_id = env_p;
	}
  else{
    std::cout << "not find ENV CAR_ID" << std::endl;
  }

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path, car_id);

  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);

  ros::NodeHandle debug_nh2;
  debug_sub_2 = debug_nh2.subscribe<geometry_msgs::PoseStamped>("/debug_goal", 1000, &DebugCallback2);

  // auto command_thread= std::thread(Command);
  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    goal_behavior.Run();
    rate.sleep();
  }


  return 0;
}

