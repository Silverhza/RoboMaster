    #include "My_Filter.h"
    My_Filter::My_Filter(std::string carNum)
    {
        carNumber = carNum;
        ROS_WARN_STREAM(carNumber);
        tfListener_ = new tf2_ros::TransformListener(tfBuffer);
        ros::ServiceClient static_map_srv_ = node_.serviceClient<nav_msgs::GetMap>("/static_map");
        ros::service::waitForService("/static_map", -1);
        nav_msgs::GetMap::Request req;
        nav_msgs::GetMap::Response res;

        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> (carNumber+"/ResultCloud", 100, false);

        //test_map_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/mapCloud2", 100, false);

        //test_laser_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/ScanCloud2", 100, false);

        //correctionPosePublisher = node_.advertise<geometry_msgs::PoseStamped> ("/correctionOfPose",2,true);

        if(static_map_srv_.call(req, res)) 
        {
            std::cout<<"get map!"<<std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud = map2pclPointCloud(res.map).makeShared();
            map_ = *mapCloud;
            std::cout<<*mapCloud<<std::endl;
            icp = new myICP(mapCloud);
        }
        //发布icp后的数据
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/"+carNumber+"/scan", 100, &My_Filter::scanCallback, this);
    }

    void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {


        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tfBuffer.lookupTransform("map", carNumber+"/base_link",
                                                 scan->header.stamp,ros::Duration(0.5));
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        //std::cout<<"get Scan!"<<std::endl;
        pcl::PointCloud<pcl::PointXYZ> laderCloud = laserScan2pclPointCloud(scan,transform);
        //testPublish(laderCloud.makeShared(),map_.makeShared());
        //ros::Time startTime = ros::Time::now();
        //Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        pcl::PointCloud<pcl::PointXYZ>::Ptr modifiedCloud = icp->getFinal(laderCloud.makeShared());
        //ros::Duration spendTime = ros::Time::now() - startTime;
        //std::cout<<"icp costs "<<spendTime<<" second"<<std::endl;
        sensor_msgs::PointCloud2 cloud2 = pclPointCloud2rosPointCloud(modifiedCloud);
        //std::cout<<cloud.header<<endl;
        // static bool sensor_msgs::convertPointCloud2ToPointCloud(const sensor_msgs::PointCloud2 & input,sensor_msgs::PointCloud & output);
        sensor_msgs::PointCloud cloud1;
        convertPointCloud2ToPointCloud(cloud2, cloud1);
/*
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.header.frame_id = "map";
        poseMsg.pose.position.x = transformation_matrix(0,3);
        poseMsg.pose.position.y = transformation_matrix(1,3);
        poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw(acos(transformation_matrix(0,0)));
        correctionPosePublisher.publish(poseMsg);
*/
        //publishTF(transformation_matrix);
        if(cloud1.header.frame_id == "map")
            point_cloud_publisher_.publish(cloud1);

    }


    void My_Filter::publishTF(Eigen::Matrix4d transformation_matrix)
    {
        tf::Transform latest_tf;
        latest_tf.setOrigin(tf::Vector3(transformation_matrix(0,3),transformation_matrix(1,3),0));
        latest_tf.setRotation(tf::createQuaternionFromYaw(acos(transformation_matrix(0,0))));
        tf::StampedTransform tmp_tf_stamped(latest_tf.inverse(),
                                            ros::Time::now(),
                                            "map",
                                            "icp");
        try
        {
            tf_broadcaster.sendTransform(tmp_tf_stamped);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }


    pcl::PointCloud<pcl::PointXYZ> My_Filter::laserScan2pclPointCloud(sensor_msgs::LaserScan::ConstPtr scan, const geometry_msgs::TransformStamped tf_)
    {
        sensor_msgs::PointCloud2 cloud,cloudUnderMap;
        laser_geometry::LaserProjection projector_;
        projector_.projectLaser(*scan, cloud);
        tf2::doTransform (cloud, cloudUnderMap, tf_);
        pcl::PointCloud<pcl::PointXYZ> rawCloud;
        pcl::fromROSMsg(cloudUnderMap, rawCloud);
        //for (size_t i = 0; i < rawCloud.points.size (); ++i) //显示所有的点
            //for (size_t i = 0; i < cloud->size(); ++i) // 为了方便观察，只显示前5个点
        //    std::cout << "    " << rawCloud.points[i].x
        //              << " " << rawCloud.points[i].y
        //              << " " << rawCloud.points[i].z << std::endl;
        //pcl::visualization::CloudViewer viewer("pcd viewer");
        //viewer.showCloud(rawCloud.makeShared());
        //system("pause");
        return rawCloud;
    }

    sensor_msgs::PointCloud2 My_Filter::pclPointCloud2rosPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud)
    {
        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg(*pclCloud, cloud);
        //std::cout<<cloud.header<<endl;
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ> My_Filter::map2pclPointCloud(nav_msgs::OccupancyGrid map)
    {
        //std::cout<<map<<std::endl;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        int width = map.info.width;
        int height = map.info.height;
        //int width = map.info.width;
        for(int i=0; i<map.data.size();i++)
        {
            int value = map.data[i];
            int y = i / width;
            int x = i % width;
            bool flag = (((x+1)<width)&&(map.data[i+1] != 100)) ||
                        (((x-1)>0)&&(map.data[i-1] != 100))     ||
                        ((y+1)<height)&&(map.data[i+width] != 100)  ||
                        ((y-1)>0)&&(map.data[i-width] != 100);
            if((value == 100)&&flag)
            {
                pcl::PointXYZ p;
                p.x = 4.8*(x / (float)height);
                p.y = 8.5*(y / (float)width);
                p.z = 0;
                cloud.push_back(p);
            }
        }
        //for (size_t i = 0; i < cloud.points.size (); ++i) //显示所有的点
            //for (size_t i = 0; i < cloud->size(); ++i) // 为了方便观察，只显示前5个点
        //    std::cout << "    " << cloud.points[i].x
        //              << " " << cloud.points[i].y
        //              << " " << cloud.points[i].z << std::endl;
        //pcl::visualization::CloudViewer viewer("pcd viewer");
        //viewer.showCloud(cloud.makeShared());
        //std::cout<<cloud<<std::endl;
        //system("pause");
        return cloud;
    }

    void My_Filter::testPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr scanPointCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr mapPointCloud)
    {
            sensor_msgs::PointCloud2 scanCloud = pclPointCloud2rosPointCloud(scanPointCloud);
            test_laser_point_cloud_publisher_.publish(scanCloud);
            sensor_msgs::PointCloud2 mapCloud = pclPointCloud2rosPointCloud(mapPointCloud);
            mapCloud.header.frame_id = "map";
            mapCloud.header.seq = scanCloud.header.seq;
            mapCloud.header.stamp = scanCloud.header.stamp;
            test_map_point_cloud_publisher_.publish(mapCloud);
    }
