#include <ros/ros.h>
// #include <bits/stdc++.h>
// #include "executor/chassis_executor.h"

// #include <actionlib/client/simple_action_client.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

//#include "roborts_msgs/ArmorDetectionAction.h"

// #include "io/io.h"
// #include "../proto/decision.pb.h"
//#include "costmap/costmap_interface.h"

// #include "example_behavior/goal_behavior.h"

ros::Subscriber debug_sub_;

void DebugCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
  ROS_WARN("[DEBUG GOAL] DEBUG SIGNAL GOT !");

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "debug_goal");
  // std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    // load namespace
  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
	{
    car_id = env_p;
	}
  else{
    std::cout << "not find ENV CAR_ID" << std::endl;
  }

  // auto chassis_executor = new roborts_decision::ChassisExecutor;
  // auto blackboard = new roborts_decision::Blackboard(full_path, car_id);

  // roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);

  // auto command_thread= std::thread(Command);

  ROS_WARN("[DEBUG GOAL] BEGING");

  ros::NodeHandle debug_nh;

  debug_sub_ = debug_nh.subscribe<geometry_msgs::PoseStamped>("/debug_goal", 1000, &DebugCallback);


  // ros::Rate rate(10);
  // while(ros::ok()){
  //   ros::spinOnce();
  //   // goal_behavior.Run();
  //   rate.sleep();
  // }

  ros::spin();


  return 0;
}

