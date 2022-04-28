#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include "myICP.h"

double getAbsoluteDiff2Angles(const double x, const double y, const double c)
{
    // c can be PI (for radians) or 180.0 (for degrees);
    return c - fabs(fmod(fabs(x - y), 2*c) - c);
}

void myICP::print4x4Matrix(const Eigen::Matrix<float,4,4> & matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void myICP::getFinal(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr, Eigen::Vector3d &pose)
{
    Eigen::Vector3d pose_icp = pose;
    // ROS_INFO("[LOCALISATION] ORIGINAL POSE: (%f, %f, %f) ",pose[0], pose[1], pose[2]);
    pcl::PointCloud<pcl::PointXYZ> input;
    laserScan2pclCloud(input,laser_scan_msg_ptr);
    myICP::icp.setInputSource(input.makeShared());
    pcl::PointCloud<pcl::PointXYZ> _final;
    pose2Transforation(pose_icp,Transformation);
    myICP::icp.align(_final,Transformation);
    // ROS_WARN("[LOCALISATION] ICP POSE: (%f, %f, %f) ",pose_icp[0], pose_icp[1], pose_icp[2]);
    // ROS_WARN("[LOCALISATION] ICP has converged, score is %f",icp.getFitnessScore());
    transforation2Pose(Transformation,pose_icp);
    Transformation = myICP::icp.getFinalTransformation().cast<float>();
    // ROS_INFO("[LOCALISATION] CHECK THAT THE ORIGINAL POSE IS THE SAME : (%f, %f, %f) ",pose[0], pose[1], pose[2]);
    transforation2Pose(Transformation,pose_icp);
    if (getAbsoluteDiff2Angles(pose_icp[2], pose[2], 3.14159)<0.03 && abs(pose_icp[0] - pose[0])<0.5 && abs(pose_icp[1] - pose[1])<0.5)
    {
        pose = pose_icp;
    } else{
        // ROS_INFO("[LOCALISATION] THE ICP POSE : (%f, %f, %f) ",pose_icp[0], pose_icp[1], pose_icp[2]);
        // ROS_INFO("[LOCALISATION] THE FINAL POSE : (%f, %f, %f) ",pose[0], pose[1], pose[2]);
        ROS_WARN("[LOCALISATION] THE ICP RESULT IS ABANDONED. YAW DIFFERENCE: %f, X DIFFERENCE: %f, Y DIFFERENCE: %f",
                getAbsoluteDiff2Angles(pose_icp[2], pose[2], 3.14159), abs(pose_icp[0] - pose[0]), abs(pose_icp[1] - pose[1]));
    }   
    

}

myICP::myICP()
{
    pcl::PointCloud<pcl::PointXYZ> map;
    
    std::string car_id;

    if (const char* env_p = std::getenv("CAR_ID"))
    {
        car_id = env_p;
    }
    else{
        std::cout << "not find ENV CAR_ID" << std::endl;
    }
    if(car_id == "CAR1")
        pcl::io::loadPCDFile<pcl::PointXYZ>("/home/one/roborts_ws/map_pcd.pcd", map);
    else
        pcl::io::loadPCDFile<pcl::PointXYZ>("/home/two/roborts_ws/map_pcd.pcd", map);
    Transformation = Eigen::Matrix<float, 4, 4>::Identity();
    icp.setInputTarget(map.makeShared());
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setTransformationEpsilon(1e-9);
    icp.setEuclideanFitnessEpsilon(0.03);
    icp.setMaximumIterations (1000);
}
void myICP::laserScan2pclCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr)
{
    sensor_msgs::PointCloud2 rawcloud;
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*laser_scan_msg_ptr, rawcloud);
    pcl::fromROSMsg(rawcloud, cloud);
}
void myICP::pose2Transforation(Eigen::Vector3d pose, Eigen::Matrix<float,4,4> &transfoation)
{
    if(std::isnan(pose[2]))
        pose[2] = 0;
    double sinYaw = sin(pose[2]);
    double cosYaw = cos(pose[2]);
    transfoation(0,0) = cosYaw;
    transfoation(0,1) = -sinYaw;
    transfoation(1,0) = sinYaw;
    transfoation(1,1) = cosYaw;

    transfoation(0,3) = pose[0];
    transfoation(1,3) = pose[1];
    transfoation(2,2) = 1;
}
void myICP::transforation2Pose(const Eigen::Matrix<float,4,4> &transfoation, Eigen::Vector3d &pose)
{
    pose[0] = transfoation(0,3);
    pose[1] = transfoation(1,3);
    pose[2] = acos(transfoation(0,0));
    if(std::isnan(pose[2]))
        pose[2] = 0;
    if(transfoation(1,0)<0)
        pose[2] = -pose[2];

}
