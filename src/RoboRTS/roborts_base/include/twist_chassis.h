#ifndef TWIST_CHASSIS_H
#define TWIST_CHASSIS_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "pre_generated_msgs/GyroEcdAngle.h"
#include "roborts_msgs/TwistAccel.h"
#include "roborts_msgs/GimbalAngle.h"
#include <cmath>
#include <iostream>
#include "utils/range.hpp"
using util::lang::range;

class TwistChassis
{
protected:
	ros::NodeHandle nh_;

	ros::ServiceServer server;

	ros::Subscriber gimbal_angle_sub;
	ros::Subscriber gimbal_cmd_sub;
	ros::Subscriber chassis_cmd_sub;
	ros::Publisher chassis_pub;
	ros::Publisher turret_pub;

	roborts_msgs::GyroEcdAngle angles_info;
	roborts_msgs::GimbalAngle gimbal_cmd;
	roborts_msgs::TwistAccel chassis_cmd;
	double absolute_angle;
	double body_rotate_speed;
	double angle_offset;

public:
	bool twist_enable;
	TwistChassis(ros::NodeHandle &nodehandle, std::string srv_name);
	void send_gimbal_cmd();
	void send_chassis_cmd(int i);

protected:
	void angleInfoCallback(const roborts_msgs::GyroEcdAngle::ConstPtr &msg);
	void chassisCmdCallback(const roborts_msgs::TwistAccel::ConstPtr &msg);
	void gimbalCmdCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg);
	bool switcherCallback(std_srvs::SetBool::Request &req,
						  std_srvs::SetBool::Response &res);
};

#endif //TWIST_CHASSIS_H