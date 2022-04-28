#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/ShootInfo.h"
#include "obstacle_detector/Obstacles.h"
#include <cmath>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "autofire/ShootSwitchAction.h"
#include "autofire/Lidar2Enemy.h"


bool enableAlternant=true;
class Lidar2EnemySrv
{
        protected:
                ros::ServiceServer server;
        public:
                bool current_mode;

                Lidar2EnemySrv(std::string name, ros::NodeHandle& nh_) 
                {
                        server = nh_.advertiseService(name, &Lidar2EnemySrv::changeMode,this);
                        current_mode=false;
                        ROS_INFO("START");
                }

                ~Lidar2EnemySrv(void)
                {
                }

                bool changeMode(autofire::Lidar2Enemy::Request &req,
                                autofire::Lidar2Enemy::Response &res)
                {
                        current_mode = req.setStatus;
                        enableAlternant=true;
                        res.received = true;
                        return true;
                }
};






tf::Point current_position;
tf::Quaternion current_quat;
tf::Quaternion first_quat;


void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        tf::Point pt(
                        msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z
                    );
        current_position = pt;

        tf::Quaternion quat(
                        msg->pose.orientation.x,
                        msg->pose.orientation.y,
                        msg->pose.orientation.z,
                        msg->pose.orientation.w
                        );
        current_quat = quat;
}


tf::Point target_point;
void targetCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{

        double minDist=100;
        double pointDist = 0;

        double distFilter = 4;

        bool isFind = false;
        for(auto it_c = msg->circles.begin() ; it_c!=msg->circles.end(); it_c++){
                tf::Point p(it_c->center.x, it_c->center.y,0); 

                pointDist = sqrt((current_position.x()-p.x())*(current_position.x()-p.x()) + (current_position.y()-p.y())*(current_position.y()-p.y()));

                if(pointDist < minDist)
                {
                        target_point = p;
                        minDist = pointDist;
                        isFind=true;
                }
        }

        if(isFind==false)
        {
                target_point.setX(0);
                target_point.setY(0);
        }

        if(minDist>distFilter)
        {
                isFind=false;
                target_point.setX(0);
                target_point.setY(0);
        }
}

roborts_msgs::ShootInfo current_bullet_info;
void buttleInfoCallback(const roborts_msgs::ShootInfo::ConstPtr& msg)
{
        current_bullet_info = *msg;
}




// 当action完成后会调用该回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
                const autofire::ShootSwitchResultConstPtr& result)
{
        if(result->is_finished == true)
        {
                enableAlternant=true;
        }

}
// 当action激活后会调用该回调函数一次
void activeCb(){return;}
// 收到feedback后调用该回调函数
void feedbackCb(const autofire::ShootSwitchFeedbackConstPtr& feedback){return;}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "lidar_shooter_alternant_server_node");

        ros::NodeHandle nh("~");
        Lidar2EnemySrv lidar2EnemySrv("lidar2EnemySrv", nh);
        std::string my_color;
        nh.param<std::string>("color", my_color, "green");
        std::cout<<my_color<<std::endl;

        //TODO use the fusion obstacle data
        ros::Subscriber sub = nh.subscribe("/obstacle_filtered", 1000, &targetCallback);

        ros::Subscriber current_pose_sub = nh.subscribe("/amcl_pose", 10, &currentPoseCallback);
        ros::Subscriber buttle_info_sub = nh.subscribe("/bullet", 10, &buttleInfoCallback);
        ros::Publisher turret_pub = nh.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle", 1000);

        actionlib::SimpleActionClient<autofire::ShootSwitchAction> ac("/shoot_switch", true);


        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer(); //will wait for infinite time
        ROS_INFO("ShootSwitch server connected.");

        ros::Rate loop_rate(30);

        while(ros::ok()){
                //ROS_INFO("current_mode:%d",lidar2EnemySrv.current_mode);
                //ROS_INFO("enableAlternant:%d",enableAlternant);
                if( (current_bullet_info.remain_bullet != 0 && lidar2EnemySrv.current_mode) || current_bullet_info.supply_status==2){


                        if(enableAlternant == true)
                        {
                                ROS_INFO("enable");
                                if(fabs(target_point.x()+target_point.y())>0.1)
                                {

                                        double yaw_angle = atan2(target_point.y()-current_position.y(), target_point.x()-current_position.x());

                                        first_quat = tf::createQuaternionFromYaw(yaw_angle);

                                        tf::Matrix3x3 goal_m(first_quat);
                                        tf::Matrix3x3 current_m(current_quat);
                                        tf::Matrix3x3 tmp_m(first_quat);

                                        tmp_m = goal_m*current_m.inverse();

                                        double roll,pitch,yaw;
                                        tmp_m.getRPY(roll,pitch,yaw);

                                        /*
                                           ROS_INFO("yaw:%f", yaw);
                                           ROS_INFO("quat.x:%f", first_quat.x());
                                           ROS_INFO("quat.y:%f", first_quat.y());
                                           ROS_INFO("quat.z:%f", first_quat.z());
                                           ROS_INFO("quat.w:%f", first_quat.w());
                                         */
                                        if(fabs(yaw)>0.1)
                                        {
                                                roborts_msgs::GimbalAngle gimbalCmd;
                                                gimbalCmd.yaw_mode = true;
                                                //gimbalCmd.yaw_angle =yaw*0.2546;
                                                gimbalCmd.pitch_mode =true;
                                                gimbalCmd.pitch_angle =0;
                                                gimbalCmd.yaw_angle =yaw;
                                                turret_pub.publish(gimbalCmd);
                                                ros::Duration(0.5).sleep();
                                                ROS_INFO("in");
                                        }

                                        ROS_INFO("target_x:%f",target_point.x());
                                        ROS_INFO("target_y:%f",target_point.y());
                                        ROS_INFO("self_x:%f",current_position.x());
                                        ROS_INFO("self_y:%f",current_position.y());

                                        ROS_INFO("Angle:%f", yaw);

                                        autofire::ShootSwitchGoal goal;
                                        goal.cmd_id = 1;
                                        ac.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);
                                        enableAlternant = false;
                                }
                        }
                }


                ros::spinOnce();    
                loop_rate.sleep();
        }
        return 0;
}

