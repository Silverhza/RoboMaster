#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "roborts_msgs/GimbalAngle.h"
#include <cmath>
#include <iostream>
#include <roborts_msgs/TwistAccel.h>


roborts_msgs::GimbalAngle gimbalCmd;

ros::Publisher  turret_pub;
ros::Publisher  chassis_pub;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{



    gimbalCmd.yaw_mode = true;
    gimbalCmd.yaw_angle =msg->angular.z*0.0546;
    turret_pub.publish(gimbalCmd); 


roborts_msgs::TwistAccel cmd;
cmd.twist = *msg;
    chassis_pub.publish(cmd); 
} 



int main(int argc, char **argv)
{

  ros::init(argc, argv, "teleop_node");
  ros::NodeHandle nh;

  ros::Subscriber cmd_sub = nh.subscribe("/logi_cmd_vel", 100, &cmdCallback);
turret_pub = nh.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle", 1000);
chassis_pub = nh.advertise<roborts_msgs::TwistAccel>("/cmd_vel_acc", 1000);

    ros::Rate loop_rate(300);



  while(ros::ok()){


    ros::spinOnce();    
    loop_rate.sleep();
  } 

  return 0;
}
