#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include "roborts_msgs/TwistAccel.h"
#include <cmath>
#include <iostream>

#include "modified_chassis/MoveMode.h"

class MoveSrv
{
	protected:

		ros::NodeHandle nh_;
		ros::ServiceServer server;
		ros::Subscriber cmd_sub;
		ros::Publisher smooth_chassis_pub;
	public:
		ros::Publisher shake_chassis_pub;

		roborts_msgs::TwistAccel shake_cmd;
		bool current_mode;

		MoveSrv(std::string name) 
		{
			server = nh_.advertiseService(name, &MoveSrv::changeMode,this);

			cmd_sub = nh_.subscribe("teb_cmd_vel_acc", 1000, &MoveSrv::cmdVelCallback,this);
			shake_chassis_pub = nh_.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 1000);
			smooth_chassis_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
			current_mode=false;
			ROS_INFO("START");
		}

		~MoveSrv(void)
		{
		}

		bool changeMode(modified_chassis::MoveMode::Request &req,
				modified_chassis::MoveMode::Response &res)
		{
			current_mode = req.mode;
			shake_cmd.twist.linear.x = 0;
			shake_cmd.twist.linear.y = 0;
			shake_cmd.twist.linear.z = 0;
			shake_cmd.twist.angular.x = 0;
			shake_cmd.twist.angular.y = 0;
			shake_cmd.twist.angular.z = 0;
			shake_cmd.accel.linear.x = 0;
			shake_cmd.accel.linear.y = 0;
			shake_cmd.accel.linear.z = 0;
			shake_cmd.accel.angular.x = 0;
			shake_cmd.accel.angular.y = 0;
			shake_cmd.accel.angular.z = 0;
			shake_chassis_pub.publish(shake_cmd); 
res.received = true;
			return true;
		}


		void cmdVelCallback(const roborts_msgs::TwistAccel::ConstPtr& msg)
		{
			//double scale=1;
			shake_cmd = *msg;
			//chassicCmd.twist.linear.x = msg->twist.linear.x * scale;
			//chassicCmd.twist.linear.y = msg->twist.linear.y * scale;
		}
};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "chassis_server_node");

	MoveSrv moveSrv("move_srv");

	ros::Rate loop_rate(100);

	while(ros::ok()){
		if(moveSrv.current_mode == true)
		{
			moveSrv.shake_chassis_pub.publish(moveSrv.shake_cmd); 
			//ROS_INFO("SHAKE!"); 
		}
		/*
		   else
		   {
		//			ROS_INFO("STOP");
		}
		 */
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
