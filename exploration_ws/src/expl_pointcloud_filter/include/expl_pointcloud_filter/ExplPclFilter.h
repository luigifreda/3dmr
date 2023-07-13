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

#include <ros/ros.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <expl_pointcloud_filter/utils.h>
#include <expl_pointcloud_filter/SphericalTransform.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>

enum class PreFilterType {kNone, kDeterministicFilter, kStatisticFilter}; 

template<typename PointOut>
class ExplPclFilter
{
private:
	typedef pcl::PointCloud<PointOut> PointCloud_Out;

	void point_cloud_cb(const sensor_msgs::PointCloud2& point_cloud);

    //! /tf listener
    tf::TransformListener tf_listener;

    //! public NodeHandle
    ros::NodeHandle n;

    //! private NodeHandle
    ros::NodeHandle n_;

    //! Name of the laser frame (default: "/laser")
    std::string laser_frame;

    //! Name of the robot frame (default: "/base_link")
    std::string robot_frame;

    //! Name of the reference world frame (default: "/odom")
    std::string world_frame;

    //! prefilter type  		
	PreFilterType prefilter_type;

    //! Subscriber to input scans (default topic: "/scan_point_cloud")
    ros::Subscriber point_cloud_sub;	

    //! Publisher for the ouput point cloud (default topic: "/expl_point_cloud")
    ros::Publisher expl_point_cloud_pub;	

    SphericalTransform spherical_transform;

public:
	explicit ExplPclFilter(const PreFilterType& type=PreFilterType::kNone);
	~ExplPclFilter();

};
