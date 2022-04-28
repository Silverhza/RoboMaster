#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/server/simple_action_server.h>
#include "modified_chassis/RechargeAction.h"

class RobotRechargeAction
{
	protected:

		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<modified_chassis::RechargeAction> as_; 
		ros::Publisher recharge_vel;
		// create messages that are used to published feedback/result
		modified_chassis::RechargeFeedback feedback_;
		modified_chassis::RechargeResult result_;

		geometry_msgs::Twist vel_msg;

		tf2::Quaternion current_quat;
		tf2::Matrix3x3 tmp_m;
		double roll;
		double pitch;
		double yaw;
	public:

		RobotRechargeAction(std::string name) :
			as_(nh_, name, boost::bind(&RobotRechargeAction::executeCB, this, _1), false)
	{
		roll=0;
		pitch=0;
		yaw=0;

		recharge_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		as_.start();
ROS_INFO("START");
	}

		~RobotRechargeAction(void)
		{
		}

		void executeCB(const modified_chassis::RechargeGoalConstPtr &goal)
		{
ROS_INFO("cb");

			tf2_ros::Buffer tfBuffer(ros::Duration(1));
			tf2_ros::TransformListener tfListener(tfBuffer);
			ros::Rate rate(10.0);

			while (nh_.ok()){
				geometry_msgs::TransformStamped transformStamped;
				try{
					feedback_.percent_complete = 0.5;
					as_.publishFeedback(feedback_);
					transformStamped = tfBuffer.lookupTransform("base_link","gimbal_fixed",
							ros::Time::now(),ros::Duration(0.3));

					tf2::Quaternion goal_quat(0,0,0,1);

					tf2::convert(transformStamped.transform.rotation, current_quat);

					tf2::Matrix3x3 current_m(current_quat);
					tf2::Matrix3x3 goal_m(goal_quat);


					tmp_m = goal_m*current_m.inverse();

					tmp_m.getRPY(roll,pitch,yaw);
					vel_msg.angular.z = -1.2 * yaw;

					//ROS_INFO("roll:%f",roll);
					//ROS_INFO("pitch:%f",pitch);
					ROS_INFO("yaw:%f",yaw);
					//ROS_INFO("test:%f",2*current_quat.angle(goal_quat));

					vel_msg.linear.x = 0;
					vel_msg.linear.y = 0;

					//   vel_msg.linear.z = transformStamped.transform.rotation.angle(quat);
					//ROS_INFO("x:%f, y:%f, z:%f", vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);

					if(fabs(yaw) <0.1)
					{ 
						ROS_INFO("Arrived!");
						result_.is_finished = true;
						as_.setSucceeded(result_);
return;
					}
					else
					{
						recharge_vel.publish(vel_msg);
					}

				}
				catch (tf2::TransformException &ex) {
					ROS_WARN("%s",ex.what());
					ros::Duration(1.0).sleep();
					continue;
				}
				rate.sleep();
			}

		}

};



int main(int argc, char** argv){
	ros::init(argc, argv, "find_camera_server_node");
	RobotRechargeAction robotRechargeAction("findCamera");

	ros::spin();

	return 0;
};


