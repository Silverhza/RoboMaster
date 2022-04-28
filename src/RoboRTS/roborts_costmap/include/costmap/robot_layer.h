#ifndef ROBORTS_COSTMAP_ROBOT_LAYER_H
#define ROBORTS_COSTMAP_ROBOT_LAYER_H

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <vector>
#include "io/io.h"
#include "map_common.h"
#include "costmap_layer.h"
#include "roborts_msgs/GameZoneArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "obstacle_detector/Obstacles.h"

namespace roborts_costmap
{

  class RobotLayer : public CostmapLayer
  {

  public:
    RobotLayer() {}
    virtual ~RobotLayer() {}
    virtual void OnInitialize();
    virtual void Activate();
    virtual void Deactivate();
    virtual void Reset();
    virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                              double *max_x, double *max_y);

  private:
    void receiveEnemiesPoses(const obstacle_detector::Obstacles::ConstPtr& msg);
    void receiveFriendPose(const geometry_msgs::PoseStamped& msg);
    void DrawCircle(Costmap2D &master_grid, double x_set, double y_set, double r_set);
    void DrawRectangular(Costmap2D &master_grid, double x_set, double y_set, double r_set);
    std::string global_frame_;
    std::string map_frame_;
    std::string map_topic_;
    unsigned int debuff_layer_x_, debuff_layer_y_, width_, height_;
    ros::Subscriber enemies_sub_, friend_sub_;
    geometry_msgs::PoseStamped friend_;
    std::vector<geometry_msgs::PoseStamped> enemies_;
    std::vector<int> debuff_;
    std::vector<int> not_debuff_;
    bool force_clear_;

    
    
    struct DebuffZone
    {
      float x; // debuff zone 左下角 x
      float y; // debuff zone 左下角 y
    } game_zones[7];
    roborts_msgs::GameZoneArray buff_zones[7];
    float debuff_width_ = 0.54;
    float debuff_height_ = 0.48;
    float resolution_;
    float debuff_inflation_;
    float friend_radius_;
    float enemy_radius_;
    int offset_x, offset_y;
    std::string ns_;
    std::string team_type, car_id;
  };

} // namespace roborts_costmap
#endif