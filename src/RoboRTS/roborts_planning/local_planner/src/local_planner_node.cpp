/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <csignal>
#include <math.h>

#include "local_planner/local_planner_node.h"

namespace roborts_local_planner {

using roborts_common::NodeState;
LocalPlannerNode::LocalPlannerNode() :
    local_planner_nh_("~"),
    as_(local_planner_nh_, "/local_planner_node_action", boost::bind(&LocalPlannerNode::ExcuteCB, this, _1), false),
    initialized_(false), node_state_(roborts_common::NodeState::IDLE),
    node_error_info_(roborts_common::ErrorCode::OK), max_error_(5),
    local_cost_(nullptr), tf_(nullptr) {
  if (Init().IsOK()) {
    ROS_INFO("local planner initialize completed.");
  } else {
    ROS_WARN("local planner initialize failed.");
    SetNodeState(NodeState::FAILURE);
  }
  as_.start();
  ROS_INFO("[LOCAL PLANNER] ACTION SEVER STARTS SUCCESSFULLY!!");
}

LocalPlannerNode::~LocalPlannerNode() {
  StopPlanning();
}

roborts_common::ErrorInfo LocalPlannerNode::Init() {
  ROS_INFO("local planner start");
  LocalAlgorithms local_algorithms;
  std::string full_path = ros::package::getPath("roborts_planning") + "/local_planner/config/local_planner.prototxt";
  roborts_common::ReadProtoFromTextFile(full_path.c_str(), &local_algorithms);
  if (&local_algorithms == nullptr) {
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
                                   "Cannot load local planner protobuf configuration file.");
  }
  selected_algorithm_ = local_algorithms.selected_algorithm();
  frequency_ = local_algorithms.frequency();
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  // load namespace
  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
	{
    car_id = env_p;
	}
  else{
    std::cout << "not find ENV CAR_ID" << std::endl;
  }

  std::string map_path= ros::package::getPath("roborts_costmap") + \
        "/config/costmap_parameter_config_for_local_plan_" + car_id + ".prototxt";

  local_cost_ = std::make_shared<roborts_costmap::CostmapInterface>("local_costmap",
                                                                          *tf_,
                                                                          map_path.c_str());
  local_planner_ = roborts_common::AlgorithmFactory<LocalPlannerBase>::CreateAlgorithm(selected_algorithm_);
  if (local_planner_== nullptr) {
    ROS_ERROR("global planner algorithm instance can't be loaded");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
                     "local planner algorithm instance can't be loaded");
  }

  std::string name;
  visual_frame_ = local_cost_->GetGlobalFrameID();
  visual_ = LocalVisualizationPtr(new LocalVisualization(local_planner_nh_, visual_frame_));
  vel_pub_ = local_planner_nh_.advertise<roborts_msgs::TwistAccel>("/cmd_vel_acc", 5);
  brake_pub_ = local_planner_nh_.advertise<geometry_msgs::PoseStamped>("/" + car_id + "/brake", 5);
  // fake_goal_pub_ = local_planner_nh_.advertise<geometry_msgs::PoseStamped>("/" + car_id + "/move_base_simple/goal", 5);
  // cancel_pub_ =  local_planner_nh_.advertise<actionlib_msgs::GoalID>("/" + car_id + "/global_planner_node_action/cancel",100);
  // cancel_pub_local_ =  local_planner_nh_.advertise<actionlib_msgs::GoalID>("/" + car_id + "/local_planner_node_action/cancel",100);
  pose_sub_ = local_planner_nh_.subscribe("/" + car_id + "/amcl_pose", 100, &LocalPlannerNode::UpdatePose, this);
  // goal_sub_ = local_planner_nh_.subscribe("/" + car_id + "/move_base_simple/goal", 1, &LocalPlannerNode::UpdatePlanStartTime, this);
  local_planner_nh_.param<float>("timeout", timeout_, 5.0); 
  ROS_WARN("[LOCAL PLANNER] SUCCESSFULLY INITIALISE ! TIMEOUT IS SET TO %f", timeout_);

  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

// auto planning_start_time = std::chrono::steady_clock::now();
bool brake = true;
bool aborted = false;

void LocalPlannerNode::UpdatePlanStartTime(const geometry_msgs::PoseStamped &msg) {
  planning_start_time = msg.header.stamp;
  print_counter_ = 0;
  ROS_INFO("[LOCAL PLANNER] STARTING TIME IS UPDATED!");
}

void LocalPlannerNode::UpdatePose(const geometry_msgs::PoseStamped &msg) {
  current_pose_ = msg;
}

void LocalPlannerNode::ExcuteCB(const roborts_msgs::LocalPlannerGoal::ConstPtr &command) {
  ROS_INFO("[LOCAL PLANNER] EXCUTION FUNCTION IS CALLED !");
  roborts_common::ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

  if (node_state == NodeState::FAILURE) {
    roborts_msgs::LocalPlannerFeedback feedback;
    roborts_msgs::LocalPlannerResult result;
    feedback.error_code = error_info.error_code();
    feedback.error_msg  = error_info.error_msg();
    result.error_code   = feedback.error_code;
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_ERROR("Initialization Failed, Failed to execute action!");
    return;
  }
  if (plan_mtx_.try_lock()) {
    local_planner_->SetPlan(command->route, local_goal_);
    plan_mtx_.unlock();
    plan_condition_.notify_one();
  }

  ROS_INFO("Send Plan!");
  if (node_state == NodeState::IDLE) {
    // ROS_WARN("[LOCAL PLANNER] WOW! I CAN START!");
    // planning_start_time = std::chrono::steady_clock::now();
    planning_start_time = ros::Time::now();
    StartPlanning();
  } // else {
    // ROS_INFO("[LOCAL PLANNER] I DON'T WANT TO START");
  // }

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    // ROS_INFO("[LOCAL PLANNER] I AM IN A LOOP");

    if (as_.isPreemptRequested()) {
      // ROS_INFO("Action Preempted");
      if (as_.isNewGoalAvailable()) {
        as_.setPreempted();
        break;
      } else {
        as_.setPreempted();
        StopPlanning();
        break;
      }
    }
    node_state = GetNodeState();
    if (node_state == NodeState::RUNNING && brake){

      ros::Duration diff = ros::Time::now() - planning_start_time;

      // if (print_counter_ % 400 == 0)
      //   ROS_INFO("[LOCAL PLANNER] %f S HAS PASSED !!!", diff.toSec() ) ;
      print_counter_++;

      if (diff.toSec() > timeout_ && diff.toSec() < 100){
        // geometry_msgs::PoseStamped debug_goal;
        // debug_goal.header.frame_id = "map";
        // debug_goal.header.stamp = ros::Time::now();
        // debug_goal.pose.position.x = 2.34865617752;
        // debug_goal.pose.position.y = 3.35998225212;
        // debug_goal.pose.position.z = 0;
        // debug_goal.pose.orientation.x = 0;
        // debug_goal.pose.orientation.y = 0;
        // debug_goal.pose.orientation.z = 0.73262944899;
        // debug_goal.pose.orientation.w = 0.680627717973;

        ROS_WARN("[LOCAL PLANNER] TIMEOUT! CURRENT POSE (%d, %d) IS SET AS THE GOAL.", current_pose_.pose.position.x, current_pose_.pose.position.y);

        // fake_goal_pub_.publish(current_pose_);
        brake_pub_.publish(current_pose_);
        StopPlanning();
        // error_info = GetErrorInfo();
        // roborts_msgs::LocalPlannerFeedback feedback;
        // roborts_msgs::LocalPlannerResult result;
        // feedback.error_code = error_info.error_code();
        // feedback.error_msg  = error_info.error_msg();
        // result.error_code   = feedback.error_code;
        // // as_.publishFeedback(feedback);
        // as_.setAborted(result, feedback.error_msg);
        // ROS_ERROR("Initialization Failed, Failed to execute action!");
      }
    }

    error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING|| node_state == NodeState::SUCCESS
        || node_state == NodeState::FAILURE) {
      roborts_msgs::LocalPlannerFeedback feedback;
      roborts_msgs::LocalPlannerResult result;
      if (!error_info.IsOK()) {
        feedback.error_code = error_info.error_code();
        feedback.error_msg = error_info.error_msg();
        SetErrorInfo(roborts_common::ErrorInfo::OK());

        as_.publishFeedback(feedback);
      }
      if(node_state == NodeState::SUCCESS) {
        ROS_WARN("[LOCAL PLANNER] SUCCESS.");
        result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
        StopPlanning();
        break;
      } else if(node_state == NodeState::FAILURE) {
        result.error_code = error_info.error_code();
        as_.setAborted(result,error_info.error_msg());
        StopPlanning();
        break;
      }
    }
  }

}

void LocalPlannerNode::Loop() {

  roborts_common::ErrorInfo error_info = local_planner_->Initialize(local_cost_, tf_, visual_);
  if (error_info.IsOK()) {
    ROS_WARN("local planner algorithm initialize completed.");
  } else {
    ROS_WARN("local planner algorithm initialize failed.");
    SetNodeState(NodeState::FAILURE);
    SetErrorInfo(error_info);
  }
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  int error_count = 0;

  while (GetNodeState() == NodeState::RUNNING) {
    // ROS_INFO("[LOCAL PLANNER] I AM IN ANOTHER LOOP");
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin = std::chrono::steady_clock::now();
    roborts_common::ErrorInfo error_info = local_planner_->ComputeVelocityCommands(cmd_vel_);
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);
    int need_time = 1000 /frequency_;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;

    if (sleep_time <= std::chrono::milliseconds(0)) {
      //LOG_WARNING << "The time planning once is " << cost_time.count() << " beyond the expected time "
        //        << std::chrono::milliseconds(50).count();
      sleep_time = std::chrono::milliseconds(0);
      //SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }

    if (error_info.IsOK()) {
      error_count = 0;
      ROS_INFO("[LOCAL PLANNER] CMD MSG (%f, %f, %f, %f, %f, %f) IS PUBLISHED", cmd_vel_.twist.linear.x, cmd_vel_.twist.linear.y, cmd_vel_.twist.angular.z, cmd_vel_.accel.linear.x, cmd_vel_.accel.linear.y, cmd_vel_.accel.angular.z);
      ROS_INFO("===============================================");
      vel_pub_.publish(cmd_vel_);
      if (local_planner_->IsGoalReached()) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (error_count > max_error_ && max_error_ >0) {
      ROS_WARN("Can not finish plan with max retries( %d  )", max_error_ );
      error_info =  roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
      SetNodeState(NodeState::FAILURE);
    } else {
      error_count++;
      ROS_ERROR("Can not get cmd_vel for once. %s error count:  %d", error_info.error_msg().c_str(), error_count);
    }

    SetErrorInfo(error_info);
  }

  cmd_vel_.twist.linear.x = 0;
  cmd_vel_.twist.linear.y = 0;
  cmd_vel_.twist.angular.z = 0;

  cmd_vel_.accel.linear.x = 0;
  cmd_vel_.accel.linear.y = 0;
  cmd_vel_.accel.angular.z = 0;
//  for (int i = 0; i < 10; ++i) {
    vel_pub_.publish(cmd_vel_);
//    usleep(5000);
//  }
}

void LocalPlannerNode::SetErrorInfo(const roborts_common::ErrorInfo error_info) {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  node_error_info_ = error_info;
}

void LocalPlannerNode::SetNodeState(const roborts_common::NodeState& node_state) {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}

roborts_common::NodeState LocalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}

roborts_common::ErrorInfo LocalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  return node_error_info_;
}

void LocalPlannerNode::StartPlanning() {
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }

  SetNodeState(roborts_common::NodeState::RUNNING);
  local_planner_thread_= std::thread(std::bind(&LocalPlannerNode::Loop,this));
}

void LocalPlannerNode::StopPlanning() {
  SetNodeState(roborts_common::IDLE);
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }
}

void LocalPlannerNode::AlgorithmCB(const roborts_common::ErrorInfo &algorithm_error_info) {
  SetErrorInfo(algorithm_error_info);
}

} // namespace roborts_local_planner

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv) {

  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "local_planner_node", ros::init_options::NoSigintHandler);

  roborts_local_planner::LocalPlannerNode local_planner;

  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  local_planner.StopPlanning();

  return 0;
}
