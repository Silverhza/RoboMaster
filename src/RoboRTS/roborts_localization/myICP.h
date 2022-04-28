/*
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
class myICP
{
    private:
        //pcl::PointCloud<pcl::PointXYZ>::Ptr map;
        pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
        // lastTransforation
        void print4x4Matrix(const Eigen::Matrix<float,4,4> & matrix);
        
public:
        myICP();
        void getFinal(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr, Eigen::Vector3d &pose);
        Eigen::Matrix<float,4,4> Transformation;
        void laserScan2pclCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr);
        void pose2Transforation(Eigen::Vector3d pose, Eigen::Matrix<float,4,4> &transfoation);
        void transforation2Pose(const Eigen::Matrix<float,4,4> &transfoation, Eigen::Vector3d &pose);

};
