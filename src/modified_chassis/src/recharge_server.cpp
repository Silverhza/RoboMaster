#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/server/simple_action_server.h>
#include "modified_chassis/RechargeAction.h"
#include <time.h>

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

//ROS_INFO("cb");
						as_.setSucceeded(result_);
return;

			tf2_ros::Buffer tfBuffer(ros::Duration(1));
			tf2_ros::TransformListener tfListener(tfBuffer);
			ros::Rate rate(30.0);


time_t begin = time(NULL);

			while (nh_.ok()){
				geometry_msgs::TransformStamped transformStamped;
				try{
					feedback_.percent_complete = 0.5;
					as_.publishFeedback(feedback_);
					transformStamped = tfBuffer.lookupTransform("base_link","recharge",
//ros::Time(0),ros::Duration(0.3));
							ros::Time::now(),ros::Duration(0.3));


//ROS_INFO("x:%f",transformStamped.transform.translation.x);
//ROS_INFO("y:%f",transformStamped.transform.translation.y);
					tf2::Quaternion goal_quat(0.679, 0.302, 0.621, -0.248);

					tf2::convert(transformStamped.transform.rotation, current_quat);

					tf2::Matrix3x3 current_m(current_quat);
					tf2::Matrix3x3 goal_m(goal_quat);


					tmp_m = goal_m*current_m.inverse();

					tmp_m.getRPY(roll,pitch,yaw);
					vel_msg.angular.z = -0.5*yaw;

					//ROS_INFO("roll:%f",roll);
					//ROS_INFO("pitch:%f",pitch);
					ROS_INFO("yaw:%f",yaw);
					//ROS_INFO("test:%f",2*current_quat.angle(goal_quat));

float tx =  (transformStamped.transform.translation.x +0.55);
float ty =  (transformStamped.transform.translation.y + 0.4);

ROS_INFO("tx:%f",tx);
ROS_INFO("ty:%f",ty);
					vel_msg.linear.x = 0.7 * (tx-ty)*0.707;
vel_msg.linear.y = (tx+ty)*0.707;
					//vel_msg.linear.y = 2 * (transformStamped.transform.translation.y - 0);

					//   vel_msg.linear.z = transformStamped.transform.rotation.angle(quat);
					ROS_INFO("x:%f, y:%f, z:%f", vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);


					if(fabs(tx) < 0.1 && fabs(ty)< 0.12 && fabs(yaw) <0.25)
//if(0)
					{ 
						ROS_INFO("Arrived!");
						as_.setSucceeded(result_);
return;
					}
					else
					{
						recharge_vel.publish(vel_msg);
					}

time_t end = time(NULL);
if(end - begin > 5)
{
as_.setAborted(result_);
ROS_INFO("TIME:%ld",end-begin);
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
	ros::init(argc, argv, "robot_recharge_server_node");

	RobotRechargeAction robotRechargeAction("robotRecharge");

	ros::spin();

	return 0;
};


