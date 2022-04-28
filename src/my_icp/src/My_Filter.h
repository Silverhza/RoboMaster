    #include <cmath>
    #include <iostream>
    #include <ros/ros.h>
    #include <tf/transform_listener.h>
    #include <tf/transform_broadcaster.h>
    #include <laser_geometry/laser_geometry.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/LaserScan.h>
    #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
    #include <tf2_ros/transform_listener.h>
    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl_ros/point_cloud.h>
    #include <geometry_msgs/TransformStamped.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/io/pcd_io.h>
    #include <nav_msgs/GetMap.h>
    #include "myICP/myICP.h"
    #include <pcl/visualization/cloud_viewer.h>
    #include <sensor_msgs/point_cloud_conversion.h>
    class My_Filter 
    {
         public:
            My_Filter(std::string carNum="CAR2");

            void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

            void publishTF(Eigen::Matrix4d transformation_matrix);

            tf2_ros::Buffer tfBuffer;

            tf2_ros::TransformListener* tfListener_;

            tf::TransformBroadcaster tf_broadcaster;


    private:
            myICP* icp;

            ros::NodeHandle node_;

            ros::Publisher point_cloud_publisher_;

            ros::Publisher test_laser_point_cloud_publisher_;

            ros::Publisher test_map_point_cloud_publisher_;

            std::string carNumber;

            ros::Publisher correctionPosePublisher;

            ros::Subscriber scan_sub_;

            pcl::PointCloud<pcl::PointXYZ> map_;

            pcl::PointCloud<pcl::PointXYZ> laserScan2pclPointCloud(sensor_msgs::LaserScan::ConstPtr scan, const geometry_msgs::TransformStamped tf_);

            sensor_msgs::PointCloud2 pclPointCloud2rosPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud);

            pcl::PointCloud<pcl::PointXYZ> map2pclPointCloud(nav_msgs::OccupancyGrid map);

            void testPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr scanPointCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr mapPointCloud);
    };

