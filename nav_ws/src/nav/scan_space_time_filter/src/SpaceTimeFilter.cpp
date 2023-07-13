/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Tiago Novo
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

#include "SpaceTimeFilter.h"

#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_xml.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <boost/operators.hpp>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iomanip>
#include <memory>
#include <limits>
#include <sstream>

#include <iostream>
#include <fstream>

#define VERBOSE // entry level of verbosity

SpaceTimeFilter::SpaceTimeFilter(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    init();
}

// initialize the vars
void SpaceTimeFilter::init()
{
    robot_id_ = 0;

    if (!setParams())
    {
        ROS_ERROR("Could not start the filter. Parameters missing!");
    }

    SpaceTimeFilterBase::init();

    // N.B.: this must stay here since we set params_.inspectionPath_!
    setPubsAndSubs();
}

void SpaceTimeFilter::setPubsAndSubs()
{
    // < Publishers
    filteredPointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud", 1);

    // < Subscribers
    pointcloud_sub_ = nh_.subscribe("dynamic_point_cloud", 1, &SpaceTimeFilter::processPointcloudWithTf, this);
}

SpaceTimeFilter::~SpaceTimeFilter()
{
    std::cout << "SpaceTimeFilter::~SpaceTimeFilter() - start " << std::endl;

    std::cout << "SpaceTimeFilter::~SpaceTimeFilter() - end " << std::endl;
}

void SpaceTimeFilter::processPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr &pointcloud)
{
    //std::cout << "SpaceTimeFilter::processPointcloudWithTf() - start " << std::endl;
    if(insertPointCloud(pointcloud))
    {
        const bool do_processing = params_.do_downsampling_;
        if(do_processing)
        {
            sensor_msgs::PointCloud2 cloud_out; 
            if(params_.do_downsampling_)
            {
                pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
                pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
                
                pcl_conversions::toPCL(*pointcloud, *cloud);

                pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
                sor.setInputCloud (cloud);
                const float leaf_size = params_.downsample_leaf_size_;
                sor.setLeafSize (leaf_size, leaf_size, leaf_size);
                sor.filter (*cloud_filtered);


                pcl_conversions::moveFromPCL(*cloud_filtered, cloud_out);            
                filteredPointCloudPub_.publish(cloud_out);
                std::cout << "SpaceTimeFilter::processPointcloudWithTf() - publishing downsampled and filtered Pointcloud " << std::endl;     
            }
        }
        else
        {
            filteredPointCloudPub_.publish(pointcloud);
            std::cout << "SpaceTimeFilter::processPointcloudWithTf() - publishing filtered Pointcloud " << std::endl;     
        }
    }

    //std::cout << "SpaceTimeFilter::processPointcloudWithTf() - end " << std::endl;
}

void SpaceTimeFilter::setRobotId(int id)
{
    robot_id_ = id;
    std::stringstream ss;
    ss << "ugv" << id + 1;
    str_robot_name_ = ss.str();
}
