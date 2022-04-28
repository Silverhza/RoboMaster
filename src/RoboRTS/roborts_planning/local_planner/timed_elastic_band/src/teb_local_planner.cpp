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

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
    LOG_INFO << "Footprint model 'polygon' loaded";
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include "timed_elastic_band/teb_local_planner.h"


namespace roborts_local_planner {

TebLocalPlanner::TebLocalPlanner () {

}

TebLocalPlanner::~TebLocalPlanner () {

}

roborts_common::ErrorInfo TebLocalPlanner::Initialize (std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
                                  std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) {

  ROS_INFO("[Follower] Initialising ............................");
  
  tf_ = tf;
  local_cost_ = local_cost;
  costmap_ = local_cost_.lock()->GetLayeredCostmap()->GetCostMap();

  in_path_vel_ = 0.4;
  to_path_k_ = 0.75;
  angle_k_ = 0.5;
  goal_threshold_ = 0.5;
  // max_lin_vel_ = 0.75;
  max_lin_vel_ = 1.75;
  max_ang_vel_ = 1.5;
  min_lin_vel_ = 0.01;
  min_ang_vel_ = 0.05;
  max_path_offset_ = 3.5;
  parking_scale_ = 0.99; //0.5;

  rotate_to_path_ = true;
  rotate_at_start_ = false;
  rotating_ = false;
  path_index_offset_ = 5;

  //initialize empty global plan
  std::vector<geometry_msgs::PoseStamped> empty_plan;
  empty_plan.push_back(geometry_msgs::PoseStamped());
  global_plan_ = empty_plan;

  goal_reached_ = false;
  is_initialized_ = true;

  ros::NodeHandle nh;
  waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("omni_path_follower/current_waypoint", 1);

  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}



roborts_common::ErrorInfo TebLocalPlanner::ComputeVelocityCommands(roborts_msgs::TwistAccel &cmd_vel) {

  std::lock_guard<std::mutex> guard(plan_mutex_);

  path_length_ = global_plan_.size();

  roborts_msgs::TwistAccel zero_vel;
    if(path_length_ == 0)
    {
      ROS_INFO("[Follower] omni path follower: path is empty");
      cmd_vel = zero_vel;
      return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
    }

    if(goal_.header.frame_id.compare("map")>1)
    {
      ROS_ERROR("[Follower] omni path follower can only process paths in map frame");
      ROS_ERROR_STREAM("goal frame: " << goal_.header.frame_id << std::endl);
      return roborts_common::ErrorInfo(roborts_common::ErrorCode::Error);
    }

    //make sure planner had been initialized
    if(!is_initialized_)
    {
      ROS_ERROR("[Follower] omni_path_follower: planner has not been initialized");
      return roborts_common::ErrorInfo(roborts_common::ErrorCode::Error);
    }

    //get the current robot pose in the costmap
    tf::Stamped<tf::Pose> robot_pose;
    if(!local_cost_.lock()->GetRobotPose(robot_pose))
    {
      cmd_vel = zero_vel;
      ROS_ERROR("[Follower] path_executer: cannot get robot pose");
      return roborts_common::ErrorInfo(roborts_common::ErrorCode::Error);
    }

    //transform robot pose in path frame id
    try
    {
      tf_.lock()->waitForTransform(goal_.header.frame_id, robot_pose.frame_id_,
                             robot_pose.stamp_, ros::Duration(0.2));
      tf_.lock()->transformPose(goal_.header.frame_id, robot_pose, robot_pose);
    }

    catch(tf::TransformException ex)
    {
      ROS_ERROR("path_follower: could not transform robot pose in goal frame, "
                "tf anwered: %s", ex.what());
      cmd_vel = zero_vel;
      return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
    }

    /** calculate velocity commands **/

    int follow_idx = std::min(path_index_+path_index_offset_+1, path_length_-1);
    ROS_INFO("[Follower] follow_idx: %d", follow_idx);
    geometry_msgs::PoseStamped follow_waypoint = global_plan_.at(follow_idx);
    ROS_INFO("[Follower] follow_waypoint calculated !!!");

    //publish follow waypoint for debugging
    waypoint_pub_.publish(follow_waypoint);

    Eigen::Vector2d vec_lastfollow(follow_waypoint.pose.position.x - last_waypoint_.position.x,
                                   follow_waypoint.pose.position.y - last_waypoint_.position.y);
    Eigen::Vector2d vec_lastrob(robot_pose.getOrigin().getX() - last_waypoint_.position.x,
                                robot_pose.getOrigin().getY() - last_waypoint_.position.y);

    double robot_angle = tf::getYaw(robot_pose.getRotation());
    double path_angle = atan2(vec_lastfollow[1],vec_lastfollow[0]);
    double delta_angle = angles::shortest_angular_distance(path_angle,robot_angle);

    double len_lastfollow = vec_lastfollow.norm();

    //shortest distance from robot to path
    double cross = vec_lastfollow[0]*vec_lastrob[1] - vec_lastfollow[1]*vec_lastrob[0];
    double to_path_dist = cross/len_lastfollow;   //TODO norm = 0?!

     if(fabs(to_path_dist) > max_path_offset_)
     {
       ROS_INFO("omni path follower: distance to path too big!");
       cmd_vel = zero_vel;
       return roborts_common::ErrorInfo(roborts_common::ErrorCode::Error);
     }

    //velocity controller
    double to_path_vel = - to_path_k_ * to_path_dist;
    double in_path_vel = in_path_vel_;
    double rotate_vel = -angle_k_ * delta_angle;

    //if we are close to the goal, slow down
    Eigen::Vector2d vec_goalrob(robot_pose.getOrigin().getX() - goal_.pose.position.x,
                                robot_pose.getOrigin().getY() - goal_.pose.position.y);
    double goal_dist = vec_goalrob.norm();

    //parking in to goal
    if(goal_dist < goal_threshold_ || path_index_ >= (path_length_ - 2))
    {
      ROS_DEBUG("parking move");
      //make sure we dont do out of parking move
      path_index_ = path_length_ - 1;

      //get goal in robot coordinate frame
      tf::Transform trafo_robot_in_world(robot_pose.getRotation(), robot_pose.getOrigin());
      tf::Transform trafo_world_in_robot = trafo_robot_in_world.inverse();

      tf::Point goal_in_world;
      tf::pointMsgToTF(goal_.pose.position, goal_in_world);
      tf::Point goal_in_robot = trafo_world_in_robot * goal_in_world;

      double goal_angle = tf::getYaw(goal_.pose.orientation);
      delta_angle = angles::shortest_angular_distance(goal_angle, robot_angle);

      if(goal_dist > goal_threshold_)
      {
        //if we went into parking because of the path index
        ROS_DEBUG("above threshold, scaling!");
        goal_in_robot.setX(goal_in_robot.x() * goal_threshold_ / goal_dist);
        goal_in_robot.setY(goal_in_robot.y() * goal_threshold_ / goal_dist);
      }

      cmd_vel.twist.linear.x = parking_scale_ * in_path_vel * goal_in_robot.x() / goal_threshold_;
      cmd_vel.twist.linear.y = parking_scale_ * in_path_vel * goal_in_robot.y() / goal_threshold_;
      cmd_vel.twist.angular.z = parking_scale_ * -angle_k_ * delta_angle / goal_threshold_;

      //once velocities are under threshold, report goal reached
      if(fabs(cmd_vel.twist.linear.x) < min_lin_vel_ &&
         fabs(cmd_vel.twist.linear.y) < min_lin_vel_ &&
         fabs(cmd_vel.twist.angular.z) < min_ang_vel_)
      {
        cmd_vel = zero_vel;
        goal_reached_ = true;
      }

      return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
    }

    //rotate velocity into robot frame
    cmd_vel.twist.linear.x = cos(delta_angle)*in_path_vel + sin(delta_angle)*to_path_vel;
    cmd_vel.twist.linear.y = -sin(delta_angle)*in_path_vel + cos(delta_angle)*to_path_vel;

    if(rotate_to_path_)
      cmd_vel.twist.angular.z = rotate_vel;
    else
      cmd_vel.twist.angular.z = rotate_vel_;

    //limit velocities
    double abs_lin_vel = hypot(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);
    if(abs_lin_vel > max_lin_vel_)
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.x * max_lin_vel_ / abs_lin_vel;
      cmd_vel.twist.linear.y = cmd_vel.twist.linear.y * max_lin_vel_ / abs_lin_vel;
    }

    if(fabs(cmd_vel.twist.angular.z) > max_ang_vel_)
    {
      cmd_vel.twist.angular.z = cmd_vel.twist.angular.z / fabs(cmd_vel.twist.angular.z) * max_ang_vel_;
    }

    /** update path index **/

    Eigen::Vector2d vec_nextlast(last_waypoint_.position.x - next_waypoint_.position.x,
                                 last_waypoint_.position.y - next_waypoint_.position.y);
    Eigen::Vector2d vec_nextrob(robot_pose.getOrigin().getX() - next_waypoint_.position.x,
                                robot_pose.getOrigin().getY() - next_waypoint_.position.y);

    double dot = vec_nextlast.dot(vec_nextrob);
    while(dot < 0.0 && path_index_ < (path_length_ - 2))
    {
      ROS_DEBUG("[Follower]path_follower: next waypoint");
      path_index_ += 1;
      ROS_INFO("[Follower] path_index_: %d", path_index_);

      last_waypoint_ = global_plan_.at(path_index_).pose;
      next_waypoint_ = global_plan_.at(path_index_+1).pose;
      ROS_INFO("[Follower] next_waypoint_ calcualted");

      vec_nextlast << last_waypoint_.position.x - next_waypoint_.position.x,
          last_waypoint_.position.y - next_waypoint_.position.y;
      vec_nextrob << robot_pose.getOrigin().getX() - next_waypoint_.position.x,
          robot_pose.getOrigin().getY() - next_waypoint_.position.y;

      dot = vec_nextlast.dot(vec_nextrob);
    }

    //rotate only at the start
    if(rotating_)
    {
      if(fabs(delta_angle) > 0.5)
      {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.linear.y = 0;
      }
      else
        rotating_ = false;
    }

    return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);

}

bool TebLocalPlanner::IsGoalReached () {

  return goal_reached_;

}

bool TebLocalPlanner::SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) {

  std::lock_guard<std::mutex> guard(plan_mutex_);

  if(!is_initialized_)
    {
      ROS_ERROR("[Follower] path follower: planner has not been initialized");
      return false;
    }

    ROS_DEBUG("[Follower] path follower: got plan");
    path_index_ = 0;
    global_plan_  = plan.poses;
    path_length_ = global_plan_.size();
    ROS_INFO("[Follower] PATH LENGTH: %d", path_length_);
    //only reset waypoints and path index if the start changed
    std::cout << " --- " << std::endl;
    std::cout << "[Follower] DEBUG: SetPlan() called by process " << ::getpid() << " (parent: " << ::getppid() << ")" << std::endl;


    if(path_length_ > 1)
    {
      ROS_INFO("[Follower] reset path index");
      ROS_INFO("[Follower] path_index_: %d", path_index_);
      last_waypoint_ = global_plan_.at(path_index_).pose;
      next_waypoint_ = global_plan_.at(path_index_ + 1).pose;
      ROS_DEBUG("[Follower] path follower: got plan");

      goal_reached_ = false;
      goal_ = global_plan_.back();
    }
    else if (path_length_ == 1)
    {
      ROS_INFO("[Follower] reset path index B");
      ROS_INFO("[Follower] path_index_: %d", path_index_);
      //robot should directly go into parking move
      last_waypoint_ = global_plan_.at(path_index_).pose;
      next_waypoint_ = global_plan_.at(path_index_).pose;
      ROS_INFO("[Follower] reset path index successfully !!!");
      goal_reached_ = false;
      goal_ = global_plan_.back();
    }
    else
    {
      goal_reached_ = true;
      ROS_INFO("[Follower] goal_reached_!!!!!");
    }

    if(rotate_at_start_)
      rotating_ = true;

    return true;
  }

void TebLocalPlanner::RegisterErrorCallBack(ErrorInfoCallback error_callback) {
  error_callback_ = error_callback;
}

 
} // namespace roborts_local_planner





// bool TebLocalPlanner::CutAndTransformGlobalPlan(int *current_goal_idx) {
//   if (!transformed_plan_.empty()) {
//     transformed_plan_.clear();
//   }

// }

// bool TebLocalPlanner::SetPlanOrientation() {
//   if (global_plan_.poses.size() < 2) {
//     ROS_WARN("can not compute the orientation because the global plan size is: %d", (int)global_plan_.poses.size());
//     return false;
//   } else {
//     //auto goal = DataConverter::LocalConvertGData(global_plan_.poses.back().pose);
//     //auto line_vector = (robot_pose_.GetPosition() - goal.first);
//     //auto  orientation = GetOrientation(line_vector);
//     for (int i = 0; i < global_plan_.poses.size() - 1; ++i) {
//       auto pose = DataConverter::LocalConvertGData(global_plan_.poses[i].pose);
//       auto next_pose = DataConverter::LocalConvertGData(global_plan_.poses[i+1].pose);
//       double x = global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x;
//       double y = global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y;
//       double angle = atan2(y, x);
//       auto quaternion = EulerToQuaternion(0, 0, angle);
//       global_plan_.poses[i].pose.orientation.w = quaternion[0];
//       global_plan_.poses[i].pose.orientation.x = quaternion[1];
//       global_plan_.poses[i].pose.orientation.y = quaternion[2];
//       global_plan_.poses[i].pose.orientation.z = quaternion[3];
//     }
//   }
// }


