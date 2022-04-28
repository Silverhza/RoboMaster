#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include "roborts_msgs/TwistAccel.h"
#include <cmath>
#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include "modified_chassis/MoveModeAction.h"

class MoveAction
{
	protected:

		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<modified_chassis::MoveModeAction> as_; 
		ros::Subscriber cmd_sub;
		ros::Publisher shake_chassis_pub;
		ros::Publisher smooth_chassis_pub;
		// create messages that are used to published feedback/result
		modified_chassis::MoveModeFeedback feedback_;
		modified_chassis::MoveModeResult result_;

		geometry_msgs::Twist smooth_cmd;
		roborts_msgs::TwistAccel shake_cmd;
	public:
		MoveAction(std::string name) :
			as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false)
	{
		cmd_sub = nh_.subscribe("/teb_cmd_vel_acc", 1000, &MoveAction::cmdVelCallback,this);
		shake_chassis_pub = nh_.advertise<roborts_msgs::TwistAccel>("/cmd_vel_acc", 1000);
		smooth_chassis_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		as_.start();
		ROS_INFO("START");
	}

		~MoveAction(void)
		{
		}

		void executeCB(const modified_chassis::MoveModeGoalConstPtr &goal)
		{
			ros::Rate loop_rate(300);

			if(goal->mode == true){
				while(!as_.isPreemptRequested() && ros::ok()){
					shake_chassis_pub.publish(shake_cmd); 
					ROS_INFO("SHAKE!"); 
					loop_rate.sleep();
				}
			}
			else{
result_.is_finished = true;
as_.setSucceeded(result_);
/*
				while(!as_.isPreemptRequested() && ros::ok()){
					smooth_cmd = shake_cmd.twist;
					smooth_chassis_pub.publish(smooth_cmd);
					ROS_INFO("SMOOTH!"); 
					loop_rate.sleep();
				}
*/
			}

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

	MoveAction moveAction("moveAction");
	ros::spin();
	return 0;
}
