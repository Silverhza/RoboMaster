#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd;

void cmdCB(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rotation_node");
    ros::NodeHandle nh("~");

    ros::Subscriber cmd_vel_sub = nh.subscribe("/green/cmd_vel", 10, &cmdCB);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        cmd_vel_pub.publish(cmd);
        loop_rate.sleep();
    }

    return 0;
}