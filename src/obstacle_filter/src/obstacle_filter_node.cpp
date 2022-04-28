#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "obstacle_detector/Obstacles.h"
#include "obstacle_filter/arena_filter.h"
#include <cmath>
#include "../include/obstacle_filter/robot_pose.h"
#include <type_traits>
#include <algorithm>
#include <numeric>
#include "roborts_msgs/GameStatus.h"

obstacle_detector::Obstacles filter_msg;
obstacle_detector::Obstacles filter_msg2;
obstacle_detector::Obstacles car1_msg;
obstacle_detector::Obstacles merged_msg;
obstacle_detector::Obstacles merged_msg2;
obstacle_detector::Obstacles closest_msg;
geometry_msgs::PoseStamped current_start;
ros::Publisher  obstacle_filter_pub;
ros::Publisher  obstacle_merge_pub;
ros::Publisher  closest_enemy_pub;
geometry_msgs::PoseStamped self_pose;
ros::Subscriber self_pose_sub, state_sub;

float inflation_dist;
float INFLAT_BOX = 0.18;
float INFLAT_BOX_CENTRE = 0.19;
float INFLAT_WALL = 0.17;
float INFLAT_FRIEND = 0.95;
float INFLAT_SELF = 0.6;
float MIN_DIST = 0.6;
float LOCAL_MIN_DIST = 0.6;
float MAX_INFLAT_DIST = 1.0;

void stateCallback(const roborts_msgs::GameStatus& msg)
{
  if (msg.game_status != 4){
    INFLAT_BOX_CENTRE = 0.3;
  } else {
    INFLAT_BOX_CENTRE = 0.19;
  }

}

void updatePose(const geometry_msgs::PoseStamped &msg) {
  self_pose = msg;
}

float circle_dist(const obstacle_detector::CircleObstacle& A, const obstacle_detector::CircleObstacle& B){
  return std::sqrt(pow(A.center.x - B.center.x, 2) + pow(A.center.y - B.center.y, 2));
}

float circle_pose_dist(const obstacle_detector::CircleObstacle& A, const geometry_msgs::PoseStamped& B){
  return std::sqrt(pow(A.center.x - B.pose.position.x, 2) + pow(A.center.y - B.pose.position.y, 2));
}

// float circle_pose_dist(const int& A, const geometry_msgs::PoseStamped B){
//   return A;
// }
// float circle_pose_dist(const int& A, const int& B){
//   return A;
// }

obstacle_detector::CircleObstacle bigger_circle(const obstacle_detector::CircleObstacle& A, const obstacle_detector::CircleObstacle& B){
  if (A.radius < B.radius){
    return B;
  } else {
    return A;
  }
}

void delete_circle(std::vector<obstacle_detector::CircleObstacle>& circles, int idx)
{
  circles[idx] = circles.back();
  circles.pop_back();
}



void merge(){
  std::vector<int> merge_idx;

  merged_msg.circles.clear();
  merged_msg.segments.clear();
  merged_msg.header.frame_id = "map";
  merged_msg.header.stamp = ros::Time::now();

  // ROS_WARN("[FILTER] %d (GREEN) + %d (ORANGE) = %d ENEMIES LEFT!", filter_msg2.circles.size(), car1_msg.circles.size(), filter_msg2.circles.size()+car1_msg.circles.size());

  std::set<int> erase_idx_set;
  std::set<int> erase_idx_car1;
  std::set<int> erase_idx_car2;
  merged_msg2 = merged_msg;
  merged_msg2.circles.clear();


  for (auto it_car2 = filter_msg2.circles.begin() ; it_car2!=filter_msg2.circles.end(); it_car2++){
    for (auto it_car1 = car1_msg.circles.begin() ; it_car1!=car1_msg.circles.end(); it_car1++){
      float dist = circle_dist(*it_car2, *it_car1);
      // ROS_INFO("[FILTER] DISTANCE: %f", dist);
      if (dist < MIN_DIST){
        if ((*it_car2).radius > (*it_car1).radius){
          erase_idx_car1.insert(it_car1-car1_msg.circles.begin());
        } else {
          erase_idx_car2.insert(it_car2-filter_msg2.circles.begin());
        }
      }
    }
  }

  for (auto& i : erase_idx_car1){
    delete_circle(car1_msg.circles, i);
    // ROS_INFO("[FILTER] CIRCLE %d FROM CAR 1 IS DELETED ...", i);
  }
  for (auto& i : erase_idx_car2){
    delete_circle(filter_msg2.circles, i);
    // ROS_INFO("[FILTER] CIRCLE %d FROM CAR 2 IS DELETED ...", i);
  }
  erase_idx_car1.clear();
  erase_idx_car2.clear();

  //if (merged_msg.circles.size() >= 2){
    // merge all close pair
    // for (auto it_c = car1_msg.circles.begin() ; it_c!=car1_msg.circles.end(); it_c++){
    //   for (auto it_c2 = it_c+1 ; it_c2!=merged_msg.circles.end(); it_c2++){
    //     float dist = circle_dist(*it_c, *it_c2);
    //     if (dist < MIN_DIST){
    //       if ((*it_c).radius > (*it_c2).radius){
    //         erase_idx_set.insert(it_c2-merged_msg.circles.begin());
    //       } else {
    //         erase_idx_set.insert(it_c-merged_msg.circles.begin());
    //       }
    //     }
    //   }
    // }
  //ROS_WARN("[FILTER] NOW THE NUMBER OF ENEMIES IS %d !", merged_msg2.circles.size());
  //} 

  for(auto it_c1 = car1_msg.circles.begin() ; it_c1!=car1_msg.circles.end(); it_c1++){
    merged_msg.circles.push_back(*it_c1);
  }
  for(auto it_c2 = filter_msg2.circles.begin() ; it_c2!=filter_msg2.circles.end(); it_c2++){
    merged_msg.circles.push_back(*it_c2);
  }

  /*
  // ROS_WARN("[FILTER] AFTER 1ST CLEARNING, %d ENEMIES LEFT!",merged_msg.circles.size() );
  if (merged_msg.circles.size() > 2){
    // while the number of obsticles is still greater than 2 after merging -> 
    // delete the ones close to the wall 
    for (auto it_c = merged_msg.circles.begin() ; it_c!=merged_msg.circles.end(); it_c++){
      Point p(it_c->center.x, it_c->center.y);
      if (IsPointInAerna(p, inflation_dist*20)){
        erase_idx_set.insert(it_c-merged_msg.circles.begin());
        //ROS_WARN("[FILTER]  ++  THE MACHANISIM WORKS !");
      }
    }
    for (auto& i : erase_idx_set){
      delete_circle(merged_msg.circles, i);
    }
    erase_idx_set.clear();
  }

  */

  if (merged_msg.circles.size() > 2){
    // while the number of obsticles is still greater than 2 after merging -> 
    // delete the ones close to the wall 

    for (float new_inflation_dist = INFLAT_BOX+0.3; new_inflation_dist < MAX_INFLAT_DIST; new_inflation_dist+=0.1){

      for (auto it_c = merged_msg.circles.begin() ; it_c!=merged_msg.circles.end(); it_c++){
        Point p(it_c->center.x, it_c->center.y);
        if (IsPointInAerna(p, new_inflation_dist, new_inflation_dist+0.2)){
          erase_idx_set.insert(it_c-merged_msg.circles.begin());
          //ROS_WARN("[FILTER] THE MACHANISIM WORKS !");
        }
      }

      if ((merged_msg.circles.size() - erase_idx_set.size()) < 3 && (merged_msg.circles.size() - erase_idx_set.size()) > 0){
        for (auto& i : erase_idx_set){
          delete_circle(merged_msg.circles, i);
        }
        erase_idx_set.clear();
        break;
      } else if ((merged_msg.circles.size() - erase_idx_set.size()) == 0){
        break;
      }
    }

  }


  /*
  if (merged_msg.circles.size() > 2){
    // if the number is still incorrect, just publish the pair with longest distance 
    float max_dist = 0;
    obstacle_detector::CircleObstacle circle1;
    obstacle_detector::CircleObstacle circle2;

    for (auto it_c = merged_msg.circles.begin() ; it_c!=merged_msg.circles.end(); it_c++){
      for (auto it_c2 = it_c+1 ; it_c2!=merged_msg.circles.end(); it_c2++){
        float dist = circle_dist(*it_c, *it_c2);

        if (dist>max_dist){
          max_dist = dist;
          circle1 = *it_c;
          circle2 = *it_c2;
        }
      }
    }
    merged_msg2.circles.push_back(circle1);
    merged_msg2.circles.push_back(circle2);
  } else {
    merged_msg2 = merged_msg;
  }
  */

  if (merged_msg.circles.size() > 2){
    // if the number is still incorrect, just publish the pair with largest radius 
    float max_dist = 0;
    obstacle_detector::CircleObstacle circle1;
    obstacle_detector::CircleObstacle circle2;
    std::vector<float> radius_list;

    for (auto it_c = merged_msg.circles.begin() ; it_c!=merged_msg.circles.end(); it_c++){
      radius_list.push_back((*it_c).radius);
    }

    std::vector<int> circle_idx(radius_list.size());
    std::iota(circle_idx.begin(), circle_idx.end(), 0);
    auto comparator = [&radius_list](int a, int b){ return radius_list[a] > radius_list[b]; };
    std::sort(circle_idx.begin(), circle_idx.end(), comparator);

    merged_msg2.circles.push_back(merged_msg.circles[circle_idx[0]]);
    merged_msg2.circles.push_back(merged_msg.circles[circle_idx[1]]);
  } else {
    merged_msg2 = merged_msg;
  }



  /*

  if (merged_msg.circles.size() == 2){
    float dx = merged_msg.circles[0].center.x - merged_msg.circles[1].center.x;
    float dy = merged_msg.circles[0].center.y - merged_msg.circles[1].center.y;
    float dist = std::sqrt(dx*dx + dy*dy);
    if (dist < MIN_DIST){
      ROS_WARN("[FILTER] CLOSE PAIR FOUND [WHILE THERE ARE TOTALLY TWO CIRCLES] !");
      if (merged_msg.circles[0].radius <= merged_msg.circles[1].radius){
        merged_msg.circles.erase(merged_msg.circles.begin() + 0);
      } else {
        merged_msg.circles.erase(merged_msg.circles.begin() + 1);
      }
    }

  } else if (merged_msg.circles.size() > 2){
    std::vector<float> dist_list;
    std::vector<int> closed_pairs_idx;
    std::vector<int> small_circle_idx;
    for(auto it_c1 = merged_msg.circles.begin() ; it_c1!=merged_msg.circles.end(); it_c1++){
      for(auto it_c2 = it_c1+1 ; it_c2!=merged_msg.circles.end(); it_c2++){
        float dx = it_c1->center.x - it_c2->center.x;
        float dy = it_c1->center.y - it_c2->center.y;
        float dist = std::sqrt(dx*dx + dy*dy);

        dist_list.push_back(dist);
        if (dist < MIN_DIST){
          closed_pairs_idx.push_back(it_c1-merged_msg.circles.begin());
          closed_pairs_idx.push_back(it_c2-merged_msg.circles.begin());
        }
      }

      if ((*it_c1).true_radius < 0.01){
          small_circle_idx.push_back(it_c1-merged_msg.circles.begin());
      }  
    }

    if (dist_list.size() == 3){
      ROS_WARN("[FILTER] I GUESS THERE ARE 3 ENEMIES. ");
      


      if (closed_pairs_idx.size() == 6){ // three obstacles are close together -> choose two points with largest distance
        ROS_INFO("[FILTER] THERE ENEMIES ARE CLOSE TOGETHER. ");

        int dis_idx = std::max_element(dist_list.begin(),dist_list.end()) - dist_list.begin(); // which distance is largest
        int erase_idx = 2 - dis_idx; // which circle should be deleted
        merged_msg.circles.erase(merged_msg.circles.begin() + erase_idx);
        ROS_INFO("[FILTER] AFTER CHOOSING THE FARTEST PAIR, THERE ARE %d ENEMIES (SHOULD BE 2). ", merged_msg.circles.size());

      } else { // otherwise -> delete one in the close pair
        int dis_idx = std::min_element(dist_list.begin(),dist_list.end()) - dist_list.begin(); 
        ROS_INFO("[FILTER] A CLOSE PAIR IS CHOSEN, WHERE ONE POINT WILL BE DELETED. ");
        // 0->0,1  1->0,2  2->1,2 
        int p1_idx = max(dis_idx-1, 0);
        int p2_idx = min(dis_idx+1, 2);
        if (merged_msg.circles[p1_idx].radius <= merged_msg.circles[p2_idx].radius){
          merged_msg.circles.erase(merged_msg.circles.begin() + p1_idx);
        } else {
          merged_msg.circles.erase(merged_msg.circles.begin() + p2_idx);
        }
      }
      

    } else if (dist_list.size() == 6){
    // } else if (false){

      ROS_WARN("[FILTER] I GUESS THERE ARE 4 ENEMIES. ");
      

      std::vector<int> erase_idx;

      // erase 0, 1, or 2 enemies
      if (closed_pairs_idx.size() == 2 || closed_pairs_idx.size() == 4){
        for(auto p = closed_pairs_idx.begin() ; p!=closed_pairs_idx.end(); p+=2){
          if (merged_msg.circles[*p].radius <= merged_msg.circles[*p+1].radius){
            erase_idx.push_back(*p);
          } else {
            erase_idx.push_back(*(p+1));
          }
        }
      }
      ROS_INFO("[FILTER] %d ENEMIES ARE IN THE ERASE LIST SINCE THEY ARE CLOSE PAIRS. ", erase_idx.size());
      if (erase_idx.size() == 2){
        ROS_INFO("[FILTER] ENEMY %d AND ENEMY %d ARE DELETED.", erase_idx[0], erase_idx[1]);
      }
      

      if (small_circle_idx.size() > 0 && small_circle_idx.size() < 3 && erase_idx.size() < 1){
        ROS_INFO("[FILTER] %d SMALL CIRCLES ARE ADDED TO THE ERASE LIST. ", small_circle_idx.size());
        std::vector<int> temp_erase;
        std::set_union(erase_idx.begin(),erase_idx.end(),small_circle_idx.begin(),small_circle_idx.end(),inserter(temp_erase, temp_erase.begin()));
        erase_idx = temp_erase;
        ROS_INFO("[FILTER] NOW %d ENEMIES ARE IN THE ERASE LIST. ", erase_idx.size());
      } 

      if (erase_idx.size() == 0){
        ROS_INFO("[FILTER] ERASE LIST IS EMPTY. BEGIN TO CHOSE THE FARTHEST PAIR.");

        int dis_idx = std::max_element(dist_list.begin(),dist_list.end()) - dist_list.begin();
        switch(dis_idx){
        case 0  :
            // delete 2, 3
            merged_msg.circles.erase(merged_msg.circles.begin() + 2);
            merged_msg.circles.erase(merged_msg.circles.begin() + 2);
            break; 
        case 1  :
            // delete 1, 3
            merged_msg.circles.erase(merged_msg.circles.begin() + 1);
            merged_msg.circles.erase(merged_msg.circles.begin() + 2);
           break; 
        case 2  :
            // delete 1, 2
            merged_msg.circles.erase(merged_msg.circles.begin() + 1);
            merged_msg.circles.erase(merged_msg.circles.begin() + 1);
            break; 
        case 3  :
            // delete 0, 3
            merged_msg.circles.erase(merged_msg.circles.begin() + 0);
            merged_msg.circles.erase(merged_msg.circles.begin() + 2);
            break; 
        case 4  :
            // delete 0, 2
            merged_msg.circles.erase(merged_msg.circles.begin() + 0);
            merged_msg.circles.erase(merged_msg.circles.begin() + 1);
            break; 
        case 5  :
            // delete 0, 1
            merged_msg.circles.erase(merged_msg.circles.begin() + 0);
            merged_msg.circles.erase(merged_msg.circles.begin() + 0);
            break; 
        }
      } else if (erase_idx.size() == 1){
        ROS_INFO("[FILTER] ERASE LIST HAS A LENGTH OF 1. BEGIN TO CHOSE THE FARTHEST PAIR.");
        merged_msg.circles.erase(merged_msg.circles.begin() + erase_idx[0]);
        float max_dis = 0;
        obstacle_detector::CircleObstacle circle1;
        obstacle_detector::CircleObstacle circle2; 

        for(auto it_c1 = merged_msg.circles.begin() ; it_c1!=merged_msg.circles.end(); it_c1++){
          for(auto it_c2 = merged_msg.circles.begin() ; it_c2!=merged_msg.circles.end(); it_c2++){
            float dx = it_c1->center.x - it_c2->center.x;
            float dy = it_c1->center.y - it_c2->center.y;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist > max_dis){
              max_dis = dist;
              circle1 = *it_c1;
              circle2 = *it_c2;
            }

          }
        }
        merged_msg.circles.clear();
        merged_msg.circles.push_back(circle1);
        merged_msg.circles.push_back(circle2);

      } else if (erase_idx.size() == 2){

        ROS_INFO("[FILTER] ERASE LIST IS ENOUGH.");
        for (auto i : erase_idx){
          merged_msg.circles.erase(merged_msg.circles.begin() + i);
        }
      
      } else {
        ROS_ERROR("[FILTER] ERASE LIST IS WRONG.");

      }



    } else {
      ROS_ERROR("[FILTER] SOMETHING WRONG!");
    }



  }

  */

  obstacle_merge_pub.publish(merged_msg2);
  
  

}

// void car1Callback(const obstacle_detector::Obstacles& msg){
//   //car1_msg = *msg;
//   //auto x = car1_msg.circles[0].center.x;
//   ROS_INFO("SHIT... ");

// }

void car1Callback(const obstacle_detector::Obstacles::ConstPtr& msg){
  //car1_msg = *msg;
  //auto x = car1_msg.circles[0].center.x;

  car1_msg.circles.clear();
  car1_msg.segments.clear();
  car1_msg.header.frame_id = "map";
  car1_msg.header.stamp = ros::Time::now();
  for(auto it_c1 = msg->circles.begin() ; it_c1!=msg->circles.end(); it_c1++){
    car1_msg.circles.push_back(*it_c1);
  }
  merge();
  
}

bool centre_box(Point& centre, Point& seg_first, Point& seg_last) {
  int dy = seg_last.y - seg_first.y;
  int dx = seg_last.x - seg_first.x;
  // if (abs(abs(dx) - 0.141) < 0.02 && abs(abs(dx) - 0.141) < 0.02 && IsPointInB5(centre, 0.1)) {
  if (IsPointInB5(centre, 0.1)) {
    return true;
    // ROS_WARN("[FILTER] THERE IS A CENTRE BOX ! ");
  } else {
    return false;
  }
}


bool straight(Point& seg_first, Point& seg_last) {
  int dy = seg_last.y - seg_first.y;
  int dx = seg_last.x - seg_first.x;
  if (abs(abs(dx) - abs(dy)) < 0.05 && seg_first.x!=seg_first.y && seg_last.x != seg_last.y) {
    return true;
  } else {
    return false;
  }
}

void filterCallback(const obstacle_detector::Obstacles::ConstPtr& msg, robot_pose& get_robot_pose)
{
  get_robot_pose.GetRobotPose(current_start);
  filter_msg.circles.clear();
  filter_msg.segments.clear();
  filter_msg.header.frame_id = "map";
  filter_msg.header.stamp = ros::Time::now();
  for(auto it_c = msg->circles.begin() ; it_c!=msg->circles.end(); it_c++){
    Point p(it_c->center.x, it_c->center.y);
    Point seg_first(it_c->segment.first_point.x, it_c->segment.first_point.y);
    Point seg_last(it_c->segment.last_point.x, it_c->segment.last_point.y);
    float friend_mask = INFLAT_FRIEND;//0.5; //0.25;
    // if(abs(current_start.pose.position.x-it_c->center.x)<friend_mask&&abs(current_start.pose.position.y-it_c->center.y)<friend_mask){
    if(circle_pose_dist(*it_c, current_start)<friend_mask || circle_pose_dist(*it_c, self_pose)<INFLAT_SELF){ 
      // ROS_WARN("[FILTER] SELF: (%f, %f);  ENEMY: (%f,%f); FRIEND: (%f,%f)", self_pose.pose.position.x, self_pose.pose.position.y, it_c->center.x, it_c->center.y, current_start.pose.position.x, current_start.pose.position.y);
        continue;
    }
    // if (IsPointInAerna(p, inflation_dist) && straight(seg_first, seg_last))
    if (IsPointInAerna(p, INFLAT_BOX, INFLAT_WALL, INFLAT_BOX_CENTRE))
    {
      continue;
    }
    else
    {
      filter_msg.circles.push_back(*it_c);
    }
  }
  /*
  if (filter_msg.circles.size() == 2){
    float dx = filter_msg.circles[0].center.x - filter_msg.circles[1].center.x;
    float dy = filter_msg.circles[0].center.y - filter_msg.circles[1].center.y;
    float dist = std::sqrt(dx*dx + dy*dy);
    if (dist < 0.2){
      if (filter_msg.circles[0].radius < filter_msg.circles[1].radius){
        filter_msg.circles.erase(merged_msg.circles.begin() + 0);
      } else {
        filter_msg.circles.erase(merged_msg.circles.begin() + 1);
      }
      obstacle_filter_pub.publish(filter_msg);
    } 

  } else if (filter_msg.circles.size() > 2){
  */
  std::set<int> erase_idx_set;
  filter_msg2 = filter_msg;
  filter_msg2.circles.clear();
  
  obstacle_detector::CircleObstacle circle1;
  obstacle_detector::CircleObstacle circle2;

  if (filter_msg.circles.size() >= 2){
    // merge all close pair
    for (auto it_c = filter_msg.circles.begin() ; it_c!=filter_msg.circles.end(); it_c++){
      for (auto it_c2 = it_c+1 ; it_c2!=filter_msg.circles.end(); it_c2++){
        float dist = circle_dist(*it_c, *it_c2);
        if (dist < LOCAL_MIN_DIST){
          if ((*it_c).radius > (*it_c2).radius){
            erase_idx_set.insert(it_c2-filter_msg.circles.begin());
          } else {
            erase_idx_set.insert(it_c-filter_msg.circles.begin());
          }
        }
      }
    }

    for (auto& i : erase_idx_set){
      delete_circle(filter_msg.circles, i);
    }
    erase_idx_set.clear();
  } 
  // ROS_INFO("[FILTER] THE ORIGINAL NUMBER OF ENEMIES IS %d !", filter_msg.circles.size());
  
  if (filter_msg.circles.size() > 2){
    // while the number of obsticles is still greater than 2 after merging -> 
    // delete the ones close to the wall 

    for (float new_inflation_dist = INFLAT_BOX+0.3; new_inflation_dist < MAX_INFLAT_DIST; new_inflation_dist+=0.1){

      for (auto it_c = filter_msg.circles.begin() ; it_c!=filter_msg.circles.end(); it_c++){
        Point p(it_c->center.x, it_c->center.y);
        if (IsPointInAerna(p, new_inflation_dist, new_inflation_dist+0.2)){
          erase_idx_set.insert(it_c-filter_msg.circles.begin());
          //ROS_WARN("[FILTER] THE MACHANISIM WORKS !");
        }
      }

      if ((filter_msg.circles.size() - erase_idx_set.size()) < 3 && (filter_msg.circles.size() - erase_idx_set.size()) > 0){
        for (auto& i : erase_idx_set){
          delete_circle(filter_msg.circles, i);
        }
        erase_idx_set.clear();
        break;
      } else if ((merged_msg.circles.size() - erase_idx_set.size()) == 0){
        break;
      }
    }

  }
  // ROS_WARN("[FILTER] THE NUMBER OF ENEMIES IS %d AFTER THE CRAZY PROCESSING!", filter_msg.circles.size());

  /*
  if (filter_msg.circles.size() > 2){
    // if the number is still incorrect, just publish the pair with longest distance 
    float max_dist = 0;
    for (auto it_c = filter_msg.circles.begin() ; it_c!=filter_msg.circles.end(); it_c++){
      for (auto it_c2 = it_c+1 ; it_c2!=filter_msg.circles.end(); it_c2++){
        float dist = circle_dist(*it_c, *it_c2);

        if (dist>max_dist){
          max_dist = dist;
          circle1 = *it_c;
          circle2 = *it_c2;
        }
      }
    }
    filter_msg2.circles.push_back(circle1);
    filter_msg2.circles.push_back(circle2);
  } else {
    filter_msg2 = filter_msg;
  }
  */


  if (filter_msg.circles.size() > 2){
    // if the number is still incorrect, just publish the pair with largest radius 
    float max_dist = 0;
    obstacle_detector::CircleObstacle circle1;
    obstacle_detector::CircleObstacle circle2;
    std::vector<float> radius_list;

    for (auto it_c = filter_msg.circles.begin() ; it_c!=filter_msg.circles.end(); it_c++){
      radius_list.push_back((*it_c).radius);
    }

    std::vector<int> circle_idx(radius_list.size());
    std::iota(circle_idx.begin(), circle_idx.end(), 0);
    auto comparator = [&radius_list](int a, int b){ return radius_list[a] > radius_list[b]; };
    std::sort(circle_idx.begin(), circle_idx.end(), comparator);

    filter_msg2.circles.push_back(filter_msg.circles[circle_idx[0]]);
    filter_msg2.circles.push_back(filter_msg.circles[circle_idx[1]]);
  } else {
    filter_msg2 = filter_msg;
  }


  // ROS_INFO("[FILTER] THE PUBLISHED NUMBER OF ENEMIES IS %d!", filter_msg2.circles.size());


  obstacle_filter_pub.publish(filter_msg2);

  if (filter_msg2.circles.size() > 0){
    closest_msg = filter_msg;
    closest_msg.circles.clear();
    double closest_dist = 100;
    int closest_idx = 0;
    int circle_idx = 0;
    for (auto &c : filter_msg2.circles){
      double dist = std::sqrt(pow(current_start.pose.position.x - c.center.x, 2) + pow(current_start.pose.position.y - c.center.y, 2));
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_idx = circle_idx;
      }
      circle_idx++;
    }
    closest_msg.circles.push_back(filter_msg2.circles[closest_idx]);
  }
  closest_enemy_pub.publish(closest_msg);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_filter_node");
  ros::NodeHandle nh("~");

  nh.param<float>("inflation", inflation_dist, INFLAT_BOX);  // 0.05

  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
  {
    car_id = env_p;
  }
  else{
    std::cout << "ENV CAR_ID NOT FOUND" << std::endl;
  }

  // ROS_WARN("[FILTER] CAR ID IS NORMAL !");
  std::shared_ptr <tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  robot_pose get_robot_pose(*tf_ptr_, car_id);
  ROS_WARN("[FILTER] GETTING POSE IS NORMAL !");

  obstacle_filter_pub = nh.advertise<obstacle_detector::Obstacles>("/"+car_id+"/obstacle_filtered", 1000);
  ros::Subscriber sub = nh.subscribe<obstacle_detector::Obstacles>("/"+car_id+"/tracked_obstacles", 1000, boost::bind(&filterCallback, _1, get_robot_pose));
  closest_enemy_pub = nh.advertise<obstacle_detector::Obstacles>("/"+car_id+"/closest_enemy", 1000);
  self_pose_sub = nh.subscribe("/"+car_id+"/amcl_pose", 1000, &updatePose);
  state_sub = nh.subscribe("/" + car_id + "/game_status", 100, &stateCallback);
  // ROS_WARN("[FILTER] SUBSCRIBTION IS NORMAL !");

  ros::Subscriber sub_for_car1;
  if (car_id[3] == '2'){
    ROS_WARN("[FILTER] Listen to filtered obstracle from CAR1");
    // ros::Subscriber sub_for_car1;
    sub_for_car1 = nh.subscribe<obstacle_detector::Obstacles>("/CAR1/obstacle_filtered", 1000, &car1Callback);
    obstacle_merge_pub = nh.advertise<obstacle_detector::Obstacles>("/obstacle_merged", 1000);
  }
  

  ros::spin();
  return 0;
}

