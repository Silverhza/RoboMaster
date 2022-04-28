#include "twist_chassis.h"

TwistChassis::TwistChassis(ros::NodeHandle &nodehandle, std::string srv_name) : nh_(nodehandle)
{
	server = nh_.advertiseService(srv_name, &TwistChassis::switcherCallback, this);
	gimbal_angle_sub = nh_.subscribe("gimbal_gyro_ecd_angle", 100, &TwistChassis::angleInfoCallback, this);
	gimbal_cmd_sub = nh_.subscribe("custom_cmd_gimbal_angle", 100, &TwistChassis::gimbalCmdCallback, this);
	chassis_cmd_sub = nh_.subscribe("custom_cmd_vel_acc", 100, &TwistChassis::chassisCmdCallback, this);
	chassis_pub = nh_.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
	turret_pub = nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
	absolute_angle = 0;
	gimbal_cmd.yaw_mode = true;
	body_rotate_speed = 2;
	twist_enable = false;
}

bool TwistChassis::switcherCallback(std_srvs::SetBool::Request &req,
									std_srvs::SetBool::Response &res)
{
	twist_enable = req.data;
	res.success = true;
	res.message = "sent";
	return true;
}

void TwistChassis::send_gimbal_cmd()
{
	gimbal_cmd.yaw_angle = sin(absolute_angle - angles_info.gyro_angle);
	turret_pub.publish(gimbal_cmd);
}

void TwistChassis::send_chassis_cmd(int i)
{
	if (i == 0 || i == 180)
	{
		// angle_offset = ((test.angles_info.ecd_angle - 0.72) * 1);
		angle_offset = angles_info.ecd_angle;
	}

	roborts_msgs::TwistAccel cmd;
	cmd.twist.linear.x = chassis_cmd.twist.linear.x * cos(angles_info.ecd_angle) + chassis_cmd.twist.linear.y * sin(-angles_info.ecd_angle);
	cmd.twist.linear.y = chassis_cmd.twist.linear.x * sin(angles_info.ecd_angle) + chassis_cmd.twist.linear.y * cos(-angles_info.ecd_angle);
	cmd.twist.angular.z = sin(3.1415926 / 180 * i) * body_rotate_speed + angle_offset;

	chassis_pub.publish(cmd);
}

void TwistChassis::angleInfoCallback(const roborts_msgs::GyroEcdAngle::ConstPtr &msg)
{
	angles_info = *msg;
}

void TwistChassis::chassisCmdCallback(const roborts_msgs::TwistAccel::ConstPtr &msg)
{
	chassis_cmd = *msg;
}

void TwistChassis::gimbalCmdCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg)
{
	if (true == msg->yaw_mode)
	{
		absolute_angle += msg->yaw_angle;
	}
	else
	{
		absolute_angle = msg->yaw_angle;
	}
	gimbal_cmd.pitch_mode = msg->pitch_mode;
	gimbal_cmd.pitch_angle = msg->pitch_angle;
}
