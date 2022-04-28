#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "actionlib/client/simple_action_client.h"
#include "roborts_msgs/LocalPlannerAction.h"
// #include "vector"


std::string CAR_ID;
ros::Subscriber path_sub_, pose_sub_;



class send2localPlanner
{
    typedef actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction> LocalActionClient;

    public:
    
    send2localPlanner();
    void path_callback(const nav_msgs::Path::ConstPtr &path);
    void callback_test(const geometry_msgs::PoseStamped &pose);
    void updatePose(const geometry_msgs::PoseStamped::ConstPtr& goal) ;
    
    
    roborts_msgs::LocalPlannerGoal local_planner_goal_;
    actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction> local_planner_client_;
};

send2localPlanner::send2localPlanner() :local_planner_client_("/"+CAR_ID+"/local_planner_node_action", true)
{
    ros::NodeHandle node_("~");
    // pose_sub_ = node_.subscribe("/CAR2/amcl_pose", 100, &send2localPlanner::updatePose, this);

    // pose_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("/CAR2/amcl_pose", 1000, &send2localPlanner::updatePose, this);
    path_sub_ = node_.subscribe<nav_msgs::Path>("/"+CAR_ID+"/decision_global_path", 1000, &send2localPlanner::path_callback, this);


    ROS_INFO("[PATH RECEIVER] SUBSCRIBER STARTS !");

    local_planner_client_.waitForServer();
    ROS_INFO("[PATH RECEIVER] SEVER WAITING FINISHED !");

};



void send2localPlanner::updatePose(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    ROS_INFO("[PATH RECEIVER] TEST SIGNAL GOT !");
};


void send2localPlanner::path_callback(const nav_msgs::Path::ConstPtr &path)
{
    // ROS_WARN("[PATH RECEIVER] SIGNAL GOT !");
    local_planner_goal_.route = *path;
    local_planner_client_.sendGoal(local_planner_goal_);


};


int main(int argc, char** argv)
{

    if (const char* env_p = std::getenv("CAR_ID"))
        CAR_ID = env_p;
    else
        std::cout << "ENV CAR_ID NOT FOUND" << std::endl;


    ros::init(argc, argv, "path_receiver_c_node");

    // ros::NodeHandle node2_("~");
    // pose_sub_ = node2_.subscribe<geometry_msgs::PoseStamped>("/CAR2/amcl_pose", 1000, &updatePose);
    // path_sub_ = node2_.subscribe<nav_msgs::Path>("/CAR2/decision_global_path", 1000, &path_callback);



    send2localPlanner sender;


    // path_sub_ = node_.subscribe<nav_msgs::Path> ("/"+CAR_ID+"/decision_global_path", 100, &path_callback);

    ros::spin();

    return 0;
}

