#include "robot_layer_setting.pb.h"
#include "robot_layer.h"
#include "numeric"
#include <math.h>

namespace roborts_costmap
{

    void RobotLayer::OnInitialize()
    {
        ros::NodeHandle nh;
        ParaRobotLayer para_robot_layer;

        std::string robot_map = ros::package::getPath("roborts_costmap") +
                                 "/config/robot_layer_config.prototxt";
        roborts_common::ReadProtoFromTextFile(robot_map.c_str(), &para_robot_layer);
        global_frame_ = layered_costmap_->GetGlobalFrameID();
        map_topic_ = para_robot_layer.topic_name();

        if (const char* env_p = std::getenv("CAR_ID"))
        {
           car_id = env_p;
        }
        else{
           std::cout << "ENV CAR_ID NOT FOUND" << std::endl;
        }
        enemies_sub_ = nh.subscribe("/obstacle_preprocessed", 10, &RobotLayer::receiveEnemiesPoses, this);
        if (car_id[3] == '2')
            friend_sub_ = nh.subscribe("/CAR1/amcl_pose", 10, &RobotLayer::receiveFriendPose, this);
        if (car_id[3] == '1')
            friend_sub_ = nh.subscribe("/CAR2/amcl_pose", 10, &RobotLayer::receiveFriendPose, this);

        debuff_inflation_ = para_robot_layer.debuff_inflation();
        friend_radius_ = para_robot_layer.friend_radius();
        enemy_radius_ = para_robot_layer.enemy_radius();
        Costmap2D *master = layered_costmap_->GetCostMap();
        resolution_ = master->GetResolution();
        offset_x = -master->GetOriginX() * 50; // * 50 == / 0.02
        offset_y = -master->GetOriginY() * 50;

        force_clear_ = false;//para_debuff_layer.force_clear_costmap();
        ROS_INFO("[ROBOT LAYER] THE ROBOT LAYER IS LOADED");
        // ROS_ERROR("[ROBOT LAYER] offset_x: %d", offset_x);
        // ROS_ERROR("[ROBOT LAYER] offset_y: %d", offset_y);
/*
        game_zones[1].x = 0.23, game_zones[1].y = 3.12;
        game_zones[2].x = 1.63, game_zones[2].y = 1.695;
        game_zones[3].x = 3.73, game_zones[3].y = 4.35;
        game_zones[6].x = 7.33, game_zones[6].y = 1.5;
        game_zones[5].x = 5.93, game_zones[5].y = 2.925;
        game_zones[4].x = 3.83, game_zones[4].y = 0.27;
*/
        
 //* 新地图
        float zone_offset_x = 0.3;
        float zone_offset_y = 0.15;
    game_zones[1].x = 0.23+zone_offset_x,   game_zones[1].y = 2.55+zone_offset_y;
    game_zones[2].x = 1.63+zone_offset_x,   game_zones[2].y = 1.41+zone_offset_y;
    game_zones[3].x = 3.77+zone_offset_x,   game_zones[3].y = 4.275+zone_offset_y;
    game_zones[4].x = 7.31+zone_offset_x,   game_zones[4].y = 1.45+zone_offset_y;
    game_zones[5].x = 5.91+zone_offset_x,   game_zones[5].y = 2.59+zone_offset_y;
    game_zones[6].x = 3.77+zone_offset_x,   game_zones[6].y = 0.205+zone_offset_y;


        
    }

    void RobotLayer::receiveFriendPose(const geometry_msgs::PoseStamped& msg)
    {
        // ROS_INFO("[ROBOT LAYER] FRIEND RECEIVED");
        friend_ = msg;

    }


    void RobotLayer::receiveEnemiesPoses(const obstacle_detector::Obstacles::ConstPtr& msg)
    {
        // ROS_INFO("[ROBOT LAYER] ENEMIES RECEIVED");
        enemies_.clear();
        for(auto it_c = msg->circles.begin() ; it_c!=msg->circles.end(); it_c++){
            geometry_msgs::PoseStamped p;
            p.header.frame_id = "map";
            p.pose.position.x = it_c->center.x;
            p.pose.position.y = it_c->center.y;
            enemies_.push_back(p);
        }
    }

    // void RobotLayer::receiveEnemiesPoses(const roborts_msgs::GameZoneArray &new_buff)
    // {
    //     /*
    //      * new_buff 内容： 指明那几个debuff zone被激活, 中间用空格隔开
    //      * Eg. 被激活的是 1 2 4 6 区, 则new_buff的内容是：
    //      * index: 0 1 2 3 4 5 6
    //      *       "0 1 1 0 1 0 1"
    //     */
    // 	//ROS_INFO("[DEBUFF LAYER] RECEIVE DEBUFF SIGNAL !");

    //     debuff_.clear();
    //     for (int i = 0; i < 6; ++i)
    //     {
    //         // if(new_buff.data[2*i] == '1'){
    //         //     debuff_.push_back(i);
    //         // }
    //         if (new_buff.zone[i].active &&
    //             (new_buff.zone[i].type == roborts_msgs::GameZone::DISABLE_MOVEMENT ||
    //              new_buff.zone[i].type == roborts_msgs::GameZone::DISABLE_SHOOTING))
    //         {
    //             debuff_.push_back(i + 1);
    //             //ROS_INFO("[DEBUFF LAYER] BAD BUFF ZONE %d ADDED !", i);
    //         }

    //         // if (new_buff.zone[i].active && team_type == "RED" &&
    //         //     (new_buff.zone[i].type == roborts_msgs::GameZone::BLUE_BULLET_SUPPLY ||
    //         //      new_buff.zone[i].type == roborts_msgs::GameZone::BLUE_HP_RECOVERY))
    //         // {
    //         //     debuff_.push_back(i + 1);
    //         // }

    //         // if (new_buff.zone[i].active && team_type == "BLUE" &&
    //         //     (new_buff.zone[i].type == roborts_msgs::GameZone::RED_BULLET_SUPPLY ||
    //         //      new_buff.zone[i].type == roborts_msgs::GameZone::RED_HP_RECOVERY))
    //         // {
    //         //     debuff_.push_back(i + 1);
    //         // }
    //     }
    // }

    void RobotLayer::Activate()
    {
        OnInitialize();
    }

    void RobotLayer::Deactivate()
    {
        // delete cost_map_;
        // shut down the map topic message subscriber
        enemies_sub_.shutdown();
        friend_sub_.shutdown();
    }

    void RobotLayer::Reset()
    {
        OnInitialize();
    }

    void RobotLayer::UpdateBounds(double robot_x,
                                   double robot_y,
                                   double robot_yaw,
                                   double *min_x,
                                   double *min_y,
                                   double *max_x,
                                   double *max_y)
    {
        return;
    }

    void RobotLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {

        


        // static const double PI = 3.1415926535;
        // ROS_INFO("[ROBOT LAYER] RADIUS OF ENEMY AND FRIEND: (%f, %f)", enemy_radius_, friend_radius_);
        // double i, angle, x1, y1;

        for(auto &pose : enemies_){
            DrawCircle(master_grid, pose.pose.position.x, pose.pose.position.y, enemy_radius_);
        }

        DrawCircle(master_grid, friend_.pose.position.x, friend_.pose.position.y, friend_radius_);
        //ROS_INFO("[ROBOT LAYER] DRAWING (%f, %f)...", friend_.pose.position.x, friend_.pose.position.y);
    }

    void RobotLayer::DrawCircle(Costmap2D &master_grid, double x_set, double y_set, double r_set)
    {

        int x0 = x_set / resolution_, y0 = y_set / resolution_, radius = r_set / resolution_;
        // ROS_INFO("[ROBOT LAYER] resolution_: %f, (%d, %d)", resolution_, x0, y0);
        for(int y=y0-radius; y<=y0+radius; y++){
            for(int x=x0-radius; x<=x0+radius; x++){
                // ROS_INFO("[ROBOT LAYER] %d ^2 x %d ^2 = %d vs R^2: %d!", x, y,x*x+y*y, radius*radius);
                if(abs(x-x0)*abs(x-x0)+abs(y-y0)*abs(y-y0) <= radius*radius && x>0 && y>0)
                    master_grid.SetCost(x, y, LETHAL_OBSTACLE); 
                    // ROS_INFO("[ROBOT LAYER] SET (%d, %d)!", x, y);

            }
        }


    }

    void RobotLayer::DrawRectangular(Costmap2D &master_grid, double x_set, double y_set, double r_set)
    {

        int x0 = x_set / resolution_, y0 = y_set / resolution_, radius = r_set / resolution_;
        // ROS_INFO("[ROBOT LAYER] resolution_: %f, (%d, %d)", resolution_, x0, y0);
        for(int y=y0; y<=y0+radius; y++){
            for(int x=x0; x<=x0+radius; x++){
                // ROS_INFO("[ROBOT LAYER] %d ^2 x %d ^2 = %d vs R^2: %d!", x, y,x*x+y*y, radius*radius);
                    master_grid.SetCost(x, y, LETHAL_OBSTACLE); 
                    // ROS_INFO("[ROBOT LAYER] SET (%d, %d)!", x, y);

            }
        }


    }





        
    

} //namespace roborts_costmap
