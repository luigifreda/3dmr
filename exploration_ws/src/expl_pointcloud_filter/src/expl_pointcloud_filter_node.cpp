/**
* This file is part of the ROS package expl_pointcloud_filter which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>  
* For more information see <https://github.com/luigifreda/3dmr>
*
* 3DMR is free software: you can redistribute it and/or modify
* it under the terms of the MIT License.
*
* 3DMR is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. *
*/

#include <expl_pointcloud_filter/utils.h>
#include <expl_pointcloud_filter/ExplPclFilter.h>

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <nifti_pcl_common/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <limits>
#include <pcl/point_types.h>



/*
 * main function
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "expl_pointcloud_filter");

    ExplPclFilter<pcl::PointXYZ> expl_pointcloud_filter(PreFilterType::kNone);

    ros::spin();

    return 0;
}

