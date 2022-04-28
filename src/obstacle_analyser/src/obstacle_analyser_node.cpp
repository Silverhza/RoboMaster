#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "obstacle_detector/Obstacles.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <cmath>
#include <type_traits>
#include "obstacle_analyser/arena_filter.h"
#include <tf/transform_listener.h>
#include <algorithm>

obstacle_detector::Obstacles new_msg;
geometry_msgs::PoseStamped current_start;
ros::Publisher  obstacle_preprocessed_pub;

std::vector<obstacle_detector::CircleObstacle> enemyA_stack;
std::vector<obstacle_detector::CircleObstacle> enemyB_stack;
obstacle_detector::CircleObstacle enemyA;
obstacle_detector::CircleObstacle enemyB;
std::vector<obstacle_detector::CircleObstacle> enemyA_stack_clean;
std::vector<obstacle_detector::CircleObstacle> enemyB_stack_clean;

ros::Publisher  goal_pub;
ros::Publisher  goal_pub2;
ros::Subscriber sub_for_brake;

float MF_THRESHOLD = 6;
float TELEPORT = 2;
float STACK_TAIL = 50;
float MIN_DIST = 0.6;

float circle_dist(const obstacle_detector::CircleObstacle& A, const obstacle_detector::CircleObstacle& B){

  return std::sqrt(pow(A.center.x - B.center.x, 2) + pow(A.center.y - B.center.y, 2));

}

std::vector<int> median_filtered(std::vector<float> signal, float threshold) {
  // ROS_INFO("[ANALYSER] MEDIAN FILTER STARTS !");
  // return a list of index to be removed
  std::vector<float> signal_sorted = signal;
  float median;
  std::vector<float> difference;
  std::vector<float> s;
  std::vector<int> erase_index;
  std::sort(signal_sorted.begin(), signal_sorted.end()); 
  if (signal_sorted.size() % 2 == 0){  //even
    // ROS_INFO("[ANALYSER] THE LENGTH OF SIGNAL IS EVEN !");
    median = (signal_sorted[signal_sorted.size()/2] + signal_sorted[signal_sorted.size()/2+1]) / 2;
  } else {
    median = (signal_sorted[signal_sorted.size()/2]);
  }
  for (auto& x : signal){
    difference.push_back(abs(x - median));
  }
  std::vector<float> difference_sorted = difference;
  std::sort(difference_sorted.begin(), difference_sorted.end()); 
  float difference_median;
  if (difference_sorted.size() % 2 == 0){  //even
    difference_median = (difference_sorted[difference_sorted.size()/2] + difference_sorted[difference_sorted.size()/2+1]) / 2;
  } else {
    difference_median = (difference_sorted[difference_sorted.size()/2]);
  }

  int n = 0;
  for (auto& diff : difference){
    if (difference_median == 0){
      s.push_back(0);
    } else {
      s.push_back(diff / difference_median);
      if (diff / difference_median > threshold){
        erase_index.push_back(n);
      }
    }
    n++;
  }
  // ROS_INFO("[ANALYSER] MEDIAN FILTER FINISHES !");
  return erase_index;
}

float median(const std::vector<float>& vec){
  std:vector<float> vec_sorted = vec;
  std::sort(vec_sorted.begin(), vec_sorted.end()); 
  float median_;
  if (vec_sorted.size() % 2 == 0){  //even
    median_ = (vec_sorted[vec_sorted.size()/2] + vec_sorted[vec_sorted.size()/2+1]) / 2;
  } else {
    median_ = (vec_sorted[vec_sorted.size()/2]);
  }
  return median_;
}


float enemy_stack_equiv(const obstacle_detector::CircleObstacle& c, const std::vector<obstacle_detector::CircleObstacle>& stack, const int n){

  std::vector<obstacle_detector::CircleObstacle> last_enemies(stack.end() - std::min<int>(stack.size(), n), stack.end());
  std::vector<float> distances;

  for (auto& enemy : last_enemies){
    distances.push_back(circle_dist(enemy, c));
  }

  return median(distances);

}

float stack_dist(const std::vector<obstacle_detector::CircleObstacle>& stack_A, const std::vector<obstacle_detector::CircleObstacle>& stack_B){

  std::vector<obstacle_detector::CircleObstacle> last_enemies_A(stack_A.end() - std::min<int>(stack_A.size(), STACK_TAIL/10), stack_A.end());
  std::vector<obstacle_detector::CircleObstacle> last_enemies_B(stack_B.end() - std::min<int>(stack_B.size(), STACK_TAIL/10), stack_B.end());

  std::vector<float> distances;

  for (auto& A : last_enemies_A){
    for (auto& B : last_enemies_B){
      distances.push_back(circle_dist(A, B));
    }
  }

  return median(distances);

}

void mergerCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{

  if (msg->circles.size() == 2){

    if (enemyA_stack.size() != 0 && enemyB_stack.size() != 0){

      //float dist_0A = std::sqrt(pow(msg->circles[0].center.x - enemyA_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyA_stack.back().center.y, 2));
      float dist_0A = enemy_stack_equiv(msg->circles[0], enemyA_stack, STACK_TAIL);
      //float dist_1A = std::sqrt(pow(msg->circles[1].center.x - enemyA_stack.back().center.x, 2) + pow(msg->circles[1].center.y - enemyA_stack.back().center.y, 2));
      float dist_1A = enemy_stack_equiv(msg->circles[1], enemyA_stack, STACK_TAIL);
      //float dist_0B = std::sqrt(pow(msg->circles[0].center.x - enemyB_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyB_stack.back().center.y, 2));
      float dist_0B = enemy_stack_equiv(msg->circles[0], enemyB_stack, STACK_TAIL);
      //float dist_1B = std::sqrt(pow(msg->circles[1].center.x - enemyB_stack.back().center.x, 2) + pow(msg->circles[1].center.y - enemyB_stack.back().center.y, 2));
      float dist_1B = enemy_stack_equiv(msg->circles[1], enemyB_stack, STACK_TAIL);

      if (dist_0A + dist_1B < dist_1A + dist_0B){
        // 0-A, 1-B

        if (dist_0A > 150){
          //enemyA_stack.clear();
          // ROS_INFO("[enemyA_stack] IS CLEARED BECAUSE ENEMY 0 IS FAR AWAY FROM THE LAST ENEMY A");
        }
        if (dist_1B > 150){
          //enemyB_stack.clear();
        }
        enemyA_stack.push_back(msg->circles[0]);
        enemyB_stack.push_back(msg->circles[1]);
      } else {
        // 1-A, 0-B
        if (dist_1A > 150){
          //enemyA_stack.clear();
          // ROS_INFO("[enemyA_stack] IS CLEARED BECAUSE ENEMY 1 IS FAR AWAY FROM THE LAST ENEMY A");

        }
        if (dist_0B > 150){
          //enemyB_stack.clear();
        }
        enemyA_stack.push_back(msg->circles[1]);
        enemyB_stack.push_back(msg->circles[0]);
      }
    } else if (enemyA_stack.size() == 0 && enemyB_stack.size() == 0){
      enemyA_stack.push_back(msg->circles[0]);
      enemyB_stack.push_back(msg->circles[1]);
    } else if (enemyA_stack.size() == 0 && enemyB_stack.size() != 0){
      //float dist_0B = std::sqrt(pow(msg->circles[0].center.x - enemyB_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyB_stack.back().center.y, 2));
      //float dist_1B = std::sqrt(pow(msg->circles[1].center.x - enemyB_stack.back().center.x, 2) + pow(msg->circles[1].center.y - enemyB_stack.back().center.y, 2));
      float dist_0B = enemy_stack_equiv(msg->circles[0], enemyB_stack, STACK_TAIL);
      float dist_1B = enemy_stack_equiv(msg->circles[1], enemyB_stack, STACK_TAIL);

      if (dist_0B < dist_1B){
        if (dist_0B > 150){
          //enemyB_stack.clear();
        } 
        enemyB_stack.push_back(msg->circles[0]);
        enemyA_stack.push_back(msg->circles[1]);
      } else {
        if (dist_1B > 150){
          //enemyB_stack.clear();
        } 
        enemyB_stack.push_back(msg->circles[1]);
        enemyA_stack.push_back(msg->circles[0]);
      } 
    } else if (enemyA_stack.size() != 0 && enemyB_stack.size() == 0){

      // float dist_0A = std::sqrt(pow(msg->circles[0].center.x - enemyA_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyA_stack.back().center.y, 2));
      // float dist_1A = std::sqrt(pow(msg->circles[1].center.x - enemyA_stack.back().center.x, 2) + pow(msg->circles[1].center.y - enemyA_stack.back().center.y, 2));
      float dist_0A = enemy_stack_equiv(msg->circles[0], enemyA_stack, STACK_TAIL);
      float dist_1A = enemy_stack_equiv(msg->circles[1], enemyA_stack, STACK_TAIL);

      if (dist_0A < dist_1A){
        if (dist_0A > 150){
          //enemyA_stack.clear();
          // ROS_INFO("[enemyA_stack] IS CLEARED BECAUSE ENEMY 0 IS FAR AWAY FROM THE LAST ENEMY A [B]");

        } 
        enemyA_stack.push_back(msg->circles[0]);
        enemyB_stack.push_back(msg->circles[1]);
      } else {
        if (dist_1A > 150){
          //enemyA_stack.clear();
          // ROS_INFO("[enemyA_stack] IS CLEARED BECAUSE ENEMY 1 IS FAR AWAY FROM THE LAST ENEMY A [B]");
        } 
        enemyA_stack.push_back(msg->circles[1]);
        enemyB_stack.push_back(msg->circles[0]);

      }

    }
  } else if (msg->circles.size() == 1){

    // ROS_INFO("ONE ENEMY IS DETECTED");

    if (enemyA_stack.size() != 0 && enemyB_stack.size() != 0){
      // float dist_0A = std::sqrt(pow(msg->circles[0].center.x - enemyA_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyA_stack.back().center.y, 2));
      // float dist_0B = std::sqrt(pow(msg->circles[0].center.x - enemyB_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyB_stack.back().center.y, 2));
      float dist_0A = enemy_stack_equiv(msg->circles[0], enemyA_stack, STACK_TAIL);
      float dist_0B = enemy_stack_equiv(msg->circles[0], enemyB_stack, STACK_TAIL);

      if (dist_0A < dist_0B){
        if (dist_0A > 150){
          //enemyA_stack.clear();
        }
        enemyA_stack.push_back(msg->circles[0]);
        enemyB_stack.erase(enemyB_stack.begin());
      } else {
        if (dist_0B > 150){
          //enemyB_stack.clear();
        }
        enemyB_stack.push_back(msg->circles[0]);
        enemyA_stack.erase(enemyA_stack.begin());
      }

    } else if (enemyA_stack.size() == 0 && enemyB_stack.size() == 0){
      enemyA_stack.push_back(msg->circles[0]);
    } else if (enemyA_stack.size() == 0 && enemyB_stack.size() != 0){
      // float dist_0B = std::sqrt(pow(msg->circles[0].center.x - enemyB_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyB_stack.back().center.y, 2));
      float dist_0B = enemy_stack_equiv(msg->circles[0], enemyB_stack, STACK_TAIL);
      if (dist_0B > 150) {
        enemyA_stack.push_back(msg->circles[0]);
      } else {
        enemyB_stack.push_back(msg->circles[0]);
      }
    } else if (enemyA_stack.size() != 0 && enemyB_stack.size() == 0){
      // float dist_0A = std::sqrt(pow(msg->circles[0].center.x - enemyA_stack.back().center.x, 2) + pow(msg->circles[0].center.y - enemyA_stack.back().center.y, 2));
      float dist_0A = enemy_stack_equiv(msg->circles[0], enemyA_stack, STACK_TAIL);
      if (dist_0A > 150) {
        enemyB_stack.push_back(msg->circles[0]);
      } else {
        enemyA_stack.push_back(msg->circles[0]);
      }
    }
  } else if (msg->circles.size() == 0){
    if (enemyA_stack.size() != 0){
      enemyA_stack.erase(enemyA_stack.begin());
    }
    if (enemyB_stack.size() != 0){
      enemyB_stack.erase(enemyB_stack.begin());
    }

  }

  if (enemyA_stack.size() > 100){
    enemyA_stack.erase(enemyA_stack.begin());
  }
  if (enemyB_stack.size() > 100){
    enemyB_stack.erase(enemyB_stack.begin());
  }

  std::vector<float> enemyA_stack_x;
  std::vector<float> enemyA_stack_y;
  std::vector<float> enemyB_stack_x;
  std::vector<float> enemyB_stack_y;
  for (auto& enemy : enemyA_stack){
    enemyA_stack_x.push_back(enemy.center.x);
    enemyA_stack_y.push_back(enemy.center.y);
  }
  for (auto& enemy : enemyB_stack){
    enemyB_stack_x.push_back(enemy.center.x);
    enemyB_stack_y.push_back(enemy.center.y);
  }

  std::vector<int> enemyA_stack_x_erase_index;
  std::vector<int> enemyA_stack_y_erase_index;
  std::vector<int> enemyB_stack_x_erase_index;
  std::vector<int> enemyB_stack_y_erase_index;
  if (!enemyA_stack.empty()){
    enemyA_stack_x_erase_index = median_filtered(enemyA_stack_x, MF_THRESHOLD);
    enemyA_stack_y_erase_index = median_filtered(enemyA_stack_y, MF_THRESHOLD);
  }
  if (!enemyB_stack.empty()){
    enemyB_stack_x_erase_index = median_filtered(enemyB_stack_x, MF_THRESHOLD);
    enemyB_stack_y_erase_index = median_filtered(enemyB_stack_y, MF_THRESHOLD);
  }

  std::vector<int> enemyA_stack_erase_index;
  std::set_union(enemyA_stack_x_erase_index.begin(),enemyA_stack_x_erase_index.end(),enemyA_stack_y_erase_index.begin(),enemyA_stack_y_erase_index.end(),inserter(enemyA_stack_erase_index, enemyA_stack_erase_index.begin()));
  std::sort(enemyA_stack_erase_index.begin(), enemyA_stack_erase_index.end(), greater<int>()); 
  enemyA_stack_clean = enemyA_stack;
  for (auto& i : enemyA_stack_erase_index){
    enemyA_stack_clean.erase(enemyA_stack_clean.begin() + i);
  }
  std::vector<int> enemyB_stack_erase_index;
  std::set_union(enemyB_stack_x_erase_index.begin(),enemyB_stack_x_erase_index.end(),enemyB_stack_y_erase_index.begin(),enemyB_stack_y_erase_index.end(),inserter(enemyB_stack_erase_index, enemyB_stack_erase_index.begin()));
  std::sort(enemyB_stack_erase_index.begin(), enemyB_stack_erase_index.end(), greater<int>()); 
  enemyB_stack_clean = enemyB_stack;
  for (auto& i : enemyB_stack_erase_index){
    enemyB_stack_clean.erase(enemyB_stack_clean.begin() + i);
  }
  

  // ROS_WARN("[ANALYSER] %d ENEMIES IN A ARE ERASED; %d ENEMIES IN B ARE ERASED", enemyA_stack_erase_index.size(), enemyB_stack_erase_index.size());


  new_msg.circles.clear();
  new_msg.segments.clear();
  new_msg.header.frame_id = "map";
  new_msg.header.stamp = ros::Time::now();
  // new_msg.circles.push_back
  enemyA.center.x = enemyB.center.x = 0;
  enemyA.center.y = enemyB.center.y = 0;
  enemyA.center.z = enemyB.center.z = 0;
  enemyA.velocity.x = enemyB.velocity.x = 0;
  enemyA.velocity.y = enemyB.velocity.y = 0;
  enemyA.velocity.z = enemyB.velocity.z = 0;
  enemyA.radius = enemyB.radius = 0;
  enemyA.true_radius = enemyB.true_radius = 0;

  
  int avg_idx = 0;
  for (auto& enemy : enemyA_stack_clean){
    avg_idx += 1;
    enemyA.center.x = enemyA.center.x + (enemy.center.x - enemyA.center.x)/avg_idx;
    enemyA.center.y = enemyA.center.y + (enemy.center.y - enemyA.center.y)/avg_idx;
    enemyA.velocity.x = enemyA.velocity.x + (enemy.velocity.x - enemyA.velocity.x)/avg_idx;
    enemyA.velocity.y = enemyA.velocity.y + (enemy.velocity.y - enemyA.velocity.y)/avg_idx;
    enemyA.radius = enemyA.radius + (enemy.radius - enemyA.radius)/avg_idx;
    enemyA.true_radius = enemyA.true_radius + (enemy.true_radius - enemyA.true_radius)/avg_idx;
  }

  avg_idx = 0;
  for (auto& enemy : enemyB_stack_clean){
    avg_idx += 1;
    enemyB.center.x = enemyB.center.x + (enemy.center.x - enemyB.center.x)/avg_idx;
    enemyB.center.y = enemyB.center.y + (enemy.center.y - enemyB.center.y)/avg_idx;
    enemyB.velocity.x = enemyB.velocity.x + (enemy.velocity.x - enemyB.velocity.x)/avg_idx;
    enemyB.velocity.y = enemyB.velocity.y + (enemy.velocity.y - enemyB.velocity.y)/avg_idx;
    enemyB.radius = enemyB.radius + (enemy.radius - enemyB.radius)/avg_idx;
    enemyB.true_radius = enemyB.true_radius + (enemy.true_radius - enemyB.true_radius)/avg_idx;
  }

  enemyA.true_radius = enemyA.true_radius*1.2;
  enemyB.true_radius = enemyB.true_radius*1.2;

  if (circle_dist(enemyA, enemyB) > MIN_DIST){
    if (!(enemyA.center.x == 0 && enemyA.center.y ==0))
      new_msg.circles.push_back(enemyA);
    if (!(enemyB.center.x == 0 && enemyB.center.y ==0))
      new_msg.circles.push_back(enemyB);
  } else {
    if (!(enemyA.center.x == 0 && enemyA.center.y ==0))
      new_msg.circles.push_back(enemyA);
  }

  

  // ROS_WARN(" %d ENEMIES IN A STACK;  %d ENEMIES IN B STACK", enemyA_stack.size(), enemyB_stack.size());



  // for(auto it_c = msg->circles.begin() ; it_c!=msg->circles.end(); it_c++){

  //   if (enemyA_stack.size() == 0){
  //     enemyA_stack.push_back(*it_c);
  //   } else if (enemyB_stack.size() == 0){  // enemyA_stack is not empty but 
  //     enemyB_stack.push_back(*it_c);
  //   } else {
  //     float distA = std::sqrt(pow(it_c->center.x - enemyA_stack.back().center.x, 2) + pow(it_c->center.y - enemyA_stack.back().center.y, 2));
  //     float distB = std::sqrt(pow(it_c->center.x - enemyB_stack.back().center.x, 2) + pow(it_c->center.y - enemyB_stack.back().center.y, 2));
  //     if (distA < distB){
  //       enemyA_stack.push_back(*it_c);
  //       ROS_INFO("ENEMIE [A] CAPTURED !")
  //     } else {
  //       enemyB_stack.push_back(*it_c);
  //       ROS_INFO("ENEMIE [B] CAPTURED !")
  //     }
  //   }
  // }






  obstacle_preprocessed_pub.publish(new_msg);


}


void brakeCallback(const geometry_msgs::PoseStamped& msg)
{
  ROS_WARN("[ANALYSER] SUCESSFULLY CALLED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  goal_pub.publish(msg);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_filter_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");


  // nh.param<float>("inflation", inflation_dist, 0.05);

  std::string car_id;

  if (const char* env_p = std::getenv("CAR_ID"))
  {
    car_id = env_p;
  }
  else{
    std::cout << "not find ENV CAR_ID" << std::endl;
  }

  std::shared_ptr <tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  ros::Subscriber sub_for_merger;

  if (car_id[3] == '2'){
    sub_for_merger = nh.subscribe<obstacle_detector::Obstacles>("/obstacle_merged", 1000, &mergerCallback);
    obstacle_preprocessed_pub = nh.advertise<obstacle_detector::Obstacles>("/obstacle_preprocessed", 1000);
  }

  // sub_for_brake = nh.subscribe("/CAR2/brake", 100, &brakeCallback);
  // goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + car_id + "/move_base_simple/goal", 5);
  goal_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/debug_goal", 5);
  // goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/test_goal", 5);
  

  ros::spin();
  return 0;
}

