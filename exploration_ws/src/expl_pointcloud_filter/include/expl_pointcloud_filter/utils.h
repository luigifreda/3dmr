/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>  
* For more information see <https://github.com/luigifreda/3dmr>
*
* 3DMR is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DMR is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DMR. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once 

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud2.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <limits>
#include <pcl/point_types.h>


void downsamplePcl(const float leaf_size,
                   const sensor_msgs::PointCloud2& cloud_in,
                   sensor_msgs::PointCloud2& cloud_out)
{ 
    // NOTE: the following commented code does not work!
    // pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    // sor.setInputCloud(cloud); // does not work even with `const sensor_msgs::PointCloud2ConstPtr& cloud`
    // sor.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
    // sor.filter(*cloud_filtered);

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
    
    pcl_conversions::toPCL(cloud_in, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*cloud_filtered);

    pcl_conversions::moveFromPCL(*cloud_filtered, cloud_out);              
}

void downsamplePcl(const float leaf_size,
                   const sensor_msgs::PointCloud2& cloud_in,
                   pcl::PCLPointCloud2::Ptr& cloud_filtered)
{ 
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(cloud_in, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*cloud_filtered);         
}


void statisticOutliersFilterPcl(const int mean_k,
                                const double stddev_mul_thresh,
                                const sensor_msgs::PointCloud2& cloud_in,
                                sensor_msgs::PointCloud2& cloud_out)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(cloud_in, *cloud);

	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(mean_k);
	sor.setStddevMulThresh(stddev_mul_thresh);
    sor.filter (*cloud_filtered);

    pcl_conversions::moveFromPCL(*cloud_filtered, cloud_out);   
}

void statisticOutliersFilterPcl(const int mean_k,
                                const double stddev_mul_thresh,
                                const sensor_msgs::PointCloud2& cloud_in,
                                pcl::PCLPointCloud2::Ptr& cloud_filtered)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(cloud_in, *cloud);

	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(mean_k);
	sor.setStddevMulThresh(stddev_mul_thresh);
    sor.filter (*cloud_filtered);
}


void radiusOutliersFilterPcl(const int min_neighbors,
                             const double radius_search,
                             const sensor_msgs::PointCloud2& cloud_in,
                             sensor_msgs::PointCloud2& cloud_out)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(cloud_in, *cloud);

	pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setRadiusSearch(radius_search);
	sor.setMinNeighborsInRadius(min_neighbors);
    sor.filter (*cloud_filtered);

    pcl_conversions::moveFromPCL(*cloud_filtered, cloud_out);   
}

void radiusOutliersFilterPcl(const int min_neighbors,
                             const double radius_search,
                             const sensor_msgs::PointCloud2& cloud_in,
                             pcl::PCLPointCloud2::Ptr& cloud_filtered)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(cloud_in, *cloud);

	pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setRadiusSearch(radius_search);
	sor.setMinNeighborsInRadius(min_neighbors);
    sor.filter (*cloud_filtered);  
}