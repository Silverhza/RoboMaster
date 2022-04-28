#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include "myICP.h"

void myICP::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr myICP::getFinal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{

    pcl::PointCloud<pcl::PointXYZ> inputWithLastTransformation;
    myICP::icp.setInputSource(input);
    pcl::PointCloud<pcl::PointXYZ> _final;
    myICP::icp.align(_final,Transformation);
    if (icp.hasConverged())
    {
        // std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        if(icp.getFitnessScore()>0.1)
            Transformation = Eigen::Matrix<float, 4, 4>::Identity();
        else
            Transformation = myICP::icp.getFinalTransformation().cast<float>();
        //print4x4Matrix(Transformation);
    }
    return _final.makeShared();
}

myICP::myICP(pcl::PointCloud<pcl::PointXYZ>::Ptr _map)
{
    isFirst = true;
    Transformation = Eigen::Matrix<float, 4, 4>::Identity();
    icp.setInputTarget(_map);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.03);
    icp.setMaximumIterations (250);
}

