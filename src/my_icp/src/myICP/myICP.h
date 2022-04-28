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
#include <pcl/console/time.h>
class myICP
{
    private:
        //pcl::PointCloud<pcl::PointXYZ>::Ptr map;
        pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
        // lastTransforation
        void print4x4Matrix(const Eigen::Matrix4d & matrix);
        bool isFirst;
public:
        myICP(pcl::PointCloud<pcl::PointXYZ>::Ptr _map);
        pcl::PointCloud<pcl::PointXYZ>::Ptr getFinal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);
        Eigen::Matrix<float,4,4> Transformation;

};