/****************************************************************************
 *  Copyright (C) 2021 RoboMaster.
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

#include "chassis.h"

namespace roborts_base{
Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle):
    Module(handle){
  SDK_Init();
  ROS_Init();
}
Chassis::~Chassis(){
  if(heartbeat_thread_.joinable()){
    heartbeat_thread_.join();
  }
}
void Chassis::SDK_Init(){

  verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id,roborts_sdk::cmd_version_id>
      (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
       MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id=0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  verison_client_->AsyncSendRequest(version,
                                    [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                           roborts_sdk::cmd_version_id>::SharedFuture future) {
                                      ROS_INFO("Chassis Firmware Version: %d.%d.%d.%d",
                                               int(future.get()->version_id>>24&0xFF),
                                               int(future.get()->version_id>>16&0xFF),
                                               int(future.get()->version_id>>8&0xFF),
                                               int(future.get()->version_id&0xFF));
                                    });

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_INFO,
                                                           CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                           std::bind(&Chassis::ChassisInfoCallback, this, std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_uwb_info>(COMPATIBLE_CMD_SET, CMD_PUSH_UWB_INFO,
                                                       CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                       std::bind(&Chassis::UWBInfoCallback, this, std::placeholders::_1));

  chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                                                                                MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPD_ACC,
                                                                                    MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

  heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                        MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  heartbeat_thread_ = std::thread([this]{
                                    roborts_sdk::cmd_heartbeat heartbeat;
                                    heartbeat.heartbeat=0;
                                    while(ros::ok()){
                                      heartbeat_pub_->Publish(heartbeat);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                    }
                                  }
  );
}
void Chassis::ROS_Init(){
  // load namespace
  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
  {
    car_id = env_p;
  }
  else{
    std::cout << "not find ENV CAR_ID" << std::endl;
  }

  //ros publisher
  ros_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("odom", 30);
  ros_uwb_pub_ = ros_nh_.advertise<geometry_msgs::PoseStamped>("uwb", 30);
  //ros subscriber
  ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("cmd_vel", 1, &Chassis::ChassisSpeedCtrlCallback, this);
  ros_sub_cmd_chassis_vel_acc_ = ros_nh_.subscribe("cmd_vel_acc", 1, &Chassis::ChassisSpeedAccCtrlCallback, this);
  ros_sub_imu_data = ros_nh_.subscribe("/"+car_id+"/imu/data", 1, &Chassis::ImuInfoCallback, this);
  xsens.orientation = tf::createQuaternionMsgFromYaw(0);

  //ros_message_init
  odom_.header.frame_id = car_id + "/odom";
  odom_.child_frame_id = car_id + "/base_link";

  odom_tf_.header.frame_id = car_id + "/odom";
  odom_tf_.child_frame_id = car_id + "/base_link";

  uwb_data_.header.frame_id = car_id + "/uwb";
}
void Chassis::ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info)
{

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;
  odom_.header.stamp = current_time;
  if(ifFirst)
  {
     glb_x = 0;
     glb_y = 0;
     inital_angle = chassis_info->angle_deg / 1800.0 * M_PI;
     ifFirst = false; 
  }
  odom_.pose.pose.position.z = 0.0;
  double yaw_angle = tf::getYaw(xsens.orientation);
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw_angle);

  odom_.pose.pose.orientation = q;
  odom_.twist.twist.linear.x = chassis_info->v_x_mm / 1000.0;
  odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0;
  double dx = -1 * sin(yaw_angle) * (chassis_info->v_y_mm / 1000.0 * dt) + cos(yaw_angle) * (chassis_info->v_x_mm / 1000.0 * dt);
  double dy = sin(yaw_angle) * (chassis_info->v_x_mm / 1000.0 * dt) + cos(yaw_angle) * (chassis_info->v_y_mm / 1000.0 * dt);
  glb_x += dx ;
  glb_y += dy ;

  odom_.pose.pose.position.x = glb_x;
  odom_.pose.pose.position.y = glb_y;

  odom_.twist.twist.angular.z = yaw_angle;
  ros_odom_pub_.publish(odom_);

  odom_tf_.header.stamp = current_time;
  odom_tf_.transform.translation.x = glb_x;
  odom_tf_.transform.translation.y = glb_y;
  odom_tf_.transform.translation.z = 0.0;
  odom_tf_.transform.rotation = q;
  tf_broadcaster_.sendTransform(odom_tf_);
}
void Chassis::UWBInfoCallback(const std::shared_ptr<roborts_sdk::cmd_uwb_info> uwb_info){

  uwb_data_.header.stamp = ros::Time::now();
  uwb_data_.pose.position.x = ((double)uwb_info->x) / 100.0;
  uwb_data_.pose.position.y = ((double)uwb_info->y) / 100.0;
  uwb_data_.pose.position.z = 0;
  uwb_data_.pose.orientation = tf::createQuaternionMsgFromYaw(uwb_info->yaw/ 180.0 * M_PI);
  ros_uwb_pub_.publish(uwb_data_);

}
void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel){
  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.vx = vel->linear.x*1000;
  chassis_speed.vy = vel->linear.y*1000;
  chassis_speed.vw = vel->angular.z * 1800.0 / M_PI;
  chassis_speed.rotate_x_offset = 0;
  chassis_speed.rotate_y_offset = 0;
  chassis_speed_pub_->Publish(chassis_speed);
}
void Chassis::ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc){
  roborts_sdk::cmd_chassis_spd_acc chassis_spd_acc;
  chassis_spd_acc.vx = vel_acc->twist.linear.x*1000;
  chassis_spd_acc.vy = vel_acc->twist.linear.y*1000;
  chassis_spd_acc.vw = vel_acc->twist.angular.z * 1800.0 / M_PI;
  chassis_spd_acc.ax = vel_acc->accel.linear.x*1000;
  chassis_spd_acc.ay = vel_acc->accel.linear.y*1000;
  chassis_spd_acc.wz = vel_acc->accel.angular.z * 1800.0 / M_PI;
  chassis_spd_acc.rotate_x_offset = 0;
  chassis_spd_acc.rotate_y_offset = 0;
  chassis_spd_acc_pub_->Publish(chassis_spd_acc);
}
void Chassis::ImuInfoCallback(const sensor_msgs::Imu::ConstPtr &imu_data)
{
  xsens = *imu_data;
}
}
