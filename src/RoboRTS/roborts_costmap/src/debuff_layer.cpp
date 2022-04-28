#include "debuff_layer_setting.pb.h"
#include "debuff_layer.h"
#include "numeric"

namespace roborts_costmap
{

    void DebuffLayer::OnInitialize()
    {
        ros::NodeHandle nh;
        ParaDebuffLayer para_debuff_layer;

        std::string debuff_map = ros::package::getPath("roborts_costmap") +
                                 "/config/debuff_layer_config.prototxt";
        roborts_common::ReadProtoFromTextFile(debuff_map.c_str(), &para_debuff_layer);
        global_frame_ = layered_costmap_->GetGlobalFrameID();
        map_topic_ = para_debuff_layer.topic_name();
        map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &DebuffLayer::receiveDebuffSignal, this);

        debuff_inflation_ = para_debuff_layer.debuff_inflation();
        Costmap2D *master = layered_costmap_->GetCostMap();
        resolution_ = master->GetResolution();
        offset_x = -master->GetOriginX() * 50; // * 50 == / 0.02
        offset_y = -master->GetOriginY() * 50;

        force_clear_ = false;//para_debuff_layer.force_clear_costmap();
        ROS_INFO("[DEBUFF LAYER] THE DEBUFF LAYER IS LOADED");
        //    ROS_ERROR("offset_x: %d", offset_x);
        //    ROS_ERROR("offset_y: %d", offset_y);
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
        float offset_for_planning = 0.18;
    game_zones[1].x = 0.23+zone_offset_x-offset_for_planning,   game_zones[1].y = 2.55+zone_offset_y;
    game_zones[2].x = 1.63+zone_offset_x,   game_zones[2].y = 1.41+zone_offset_y;
    game_zones[3].x = 3.77+zone_offset_x,   game_zones[3].y = 4.275;
    game_zones[6].x = 7.31+zone_offset_x,   game_zones[6].y = 1.45+zone_offset_y+offset_for_planning;
    game_zones[5].x = 5.91+zone_offset_x,   game_zones[5].y = 2.59+zone_offset_y;
    game_zones[4].x = 3.77+zone_offset_x,   game_zones[4].y = 0.205+zone_offset_y;


        if (const char *env_p = std::getenv("TEAM_TYPE"))
        {
            team_type = env_p;
        }
        else
        {
            std::cout << "not find ENV TEAM_TYPE" << std::endl;
        }
    }

    void DebuffLayer::receiveDebuffSignal(const roborts_msgs::GameZoneArray &new_buff)
    {
        /*
     * new_buff 内容： 指明那几个debuff zone被激活, 中间用空格隔开
     * Eg. 被激活的是 1 2 4 6 区, 则new_buff的内容是：
     * index: 0 1 2 3 4 5 6
     *       "0 1 1 0 1 0 1"
    */
    	//ROS_INFO("[DEBUFF LAYER] RECEIVE DEBUFF SIGNAL !");

        debuff_.clear();
        for (int i = 0; i < 6; ++i)
        {
            // if(new_buff.data[2*i] == '1'){
            //     debuff_.push_back(i);
            // }
            if (new_buff.zone[i].active &&
                (new_buff.zone[i].type == roborts_msgs::GameZone::DISABLE_MOVEMENT ||
                 new_buff.zone[i].type == roborts_msgs::GameZone::DISABLE_SHOOTING))
            {
                debuff_.push_back(i + 1);
                //ROS_INFO("[DEBUFF LAYER] BAD BUFF ZONE %d ADDED !", i);
            }

            // if (new_buff.zone[i].active && team_type == "RED" &&
            //     (new_buff.zone[i].type == roborts_msgs::GameZone::BLUE_BULLET_SUPPLY ||
            //      new_buff.zone[i].type == roborts_msgs::GameZone::BLUE_HP_RECOVERY))
            // {
            //     debuff_.push_back(i + 1);
            // }

            // if (new_buff.zone[i].active && team_type == "BLUE" &&
            //     (new_buff.zone[i].type == roborts_msgs::GameZone::RED_BULLET_SUPPLY ||
            //      new_buff.zone[i].type == roborts_msgs::GameZone::RED_HP_RECOVERY))
            // {
            //     debuff_.push_back(i + 1);
            // }
        }
    }

    void DebuffLayer::Activate()
    {
        OnInitialize();
    }

    void DebuffLayer::Deactivate()
    {
        // delete cost_map_;
        // shut down the map topic message subscriber
        map_sub_.shutdown();
    }

    void DebuffLayer::Reset()
    {
        OnInitialize();
    }

    void DebuffLayer::UpdateBounds(double robot_x,
                                   double robot_y,
                                   double robot_yaw,
                                   double *min_x,
                                   double *min_y,
                                   double *max_x,
                                   double *max_y)
    {
        return;
    }

    void DebuffLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {

        std::vector<int> range6(6);
    	std::iota(range6.begin(), range6.end(), 1);
        // for (auto i : debuff_){
        //     ROS_INFO("[DEBUFF LAYER] %d",i);
        // }
    	not_debuff_.clear();
    	std::set_difference( range6.begin(), range6.end(), debuff_.begin(), debuff_.end(), std::inserter(not_debuff_, not_debuff_.begin()));
    	std::vector<std::vector<int>> two_sets = {debuff_, not_debuff_};
    	std::vector<unsigned char> obstacle_value = {LETHAL_OBSTACLE, 0};
        // ROS_WARN("[DEBUFF LAYER] SIZE OF BUFF: %d  NOT_BUFF: %d", debuff_.size(), not_debuff_.size());

    	for (int s=0; s<=int(force_clear_); s++)
    	{
    		for (auto k : two_sets[s])
	        {

	            int start_x = (int)((game_zones[k].x - debuff_inflation_) / resolution_) + offset_x;
	            int end_x = (int)((game_zones[k].x + debuff_width_ + debuff_inflation_) / resolution_) + offset_x;
	            int start_y = (int)((game_zones[k].y - debuff_inflation_) / resolution_) + offset_y;
	            int end_y = (int)((game_zones[k].y + debuff_height_ + debuff_inflation_) / resolution_) + offset_y;
	            //ROS_INFO("[DEBUFF LAYER] start_x = [ %f (game_zones[k].x) + %f (debuff_inflation_) ] / %f (resolution_) + %d (offset_x) = %d", 
	                //game_zones[k].x, debuff_inflation_, resolution_, offset_x, start_x);
	            if (s == 1)
	            	ROS_WARN("[DEBUFF LAYER] RETANGLE [%d, %d, %d, %d] IS DELETED ~!", start_x, end_x, start_y, end_y);

	            for (auto i = start_x; i < end_x; ++i)
	            {
	                for (auto j = start_y; j < end_y; ++j)
	                {
	                    master_grid.SetCost(i, j, obstacle_value[s]); 
	                }
	            }
	        }

    	}
        
    }

} //namespace roborts_costmap
