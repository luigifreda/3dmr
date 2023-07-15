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

#include <expl_pointcloud_filter/ExplPclFilter.h>
#include <cmath>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("[ExplPclFilter] Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("[ExplPclFilter] Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}


template<typename PointOut>
ExplPclFilter<PointOut>::ExplPclFilter(const PreFilterType& type):n_("~"), prefilter_type(type)
{	
    // frame names
    laser_frame = getParam<std::string>(n_, "laser_frame", "laser");
    robot_frame = getParam<std::string>(n_, "robot_frame", "base_link");
    world_frame = getParam<std::string>(n_, "world_frame", "odom");

    const float max_reading = getParam<float>(n_, "max_reading", max_reading);    
    const float min_angle_v = getParam<float>(n_, "min_angle_v", -90.0); // min elevation in degs 
    const float max_angle_v = getParam<float>(n_, "max_angle_v",  90.0); // max elevation in degs 		 

    const int num_subdivisions_phi = getParam<int>(n_, "num_subdivisions_phi", num_subdivisions_phi);    
    const int num_subdivisions_theta = getParam<int>(n_, "num_subdivisions_theta", num_subdivisions_theta);    	

    // laser scan subscriber
    point_cloud_sub = n.subscribe("point_cloud", 1, &ExplPclFilter::point_cloud_cb, this);	

    // point cloud publisher
    expl_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("expl_point_cloud", 1);

	const double minTheta = 0.5*M_PI - max_angle_v*M_PI/180.0; // min inclination 
	const double maxTheta = 0.5*M_PI - min_angle_v*M_PI/180.0; // max inclination 
	spherical_transform.Init(num_subdivisions_phi, num_subdivisions_theta, minTheta, maxTheta, max_reading);


    if (!tf_listener.waitForTransform(laser_frame, world_frame, ros::Time(0), ros::Duration(30.)))
    {
        ROS_WARN_STREAM("[ExplPclFilter] Timeout (30s) while waiting between " << laser_frame <<
                        " and " << world_frame << " at startup.");
    }	
}

template<typename PointOut>
ExplPclFilter<PointOut>::~ExplPclFilter()
{
}

/*
 * laser scan callback
 */
template<typename PointOut>
void ExplPclFilter<PointOut>::point_cloud_cb(const sensor_msgs::PointCloud2& cloud)
{
	const ros::Time time = cloud.header.stamp;
	const std::string frame_id = cloud.header.frame_id;
    tf::StampedTransform tmp_tf;
    if (!tf_listener.waitForTransform(frame_id, laser_frame, time, ros::Duration(1.)))
    {
        ROS_WARN_STREAM("[ExplPclFilter] Timeout (1s) while waiting between "<< laser_frame<< " and "<< frame_id);
    }
    tf_listener.lookupTransform(frame_id, laser_frame, time, tmp_tf);

	pcl::PointXYZ laser_center;
	laser_center.x = tmp_tf.getOrigin().getX();
	laser_center.y = tmp_tf.getOrigin().getY();
	laser_center.z = tmp_tf.getOrigin().getZ();

	spherical_transform.offsetX = laser_center.x;
	spherical_transform.offsetY = laser_center.y;
	spherical_transform.offsetZ = laser_center.z;		

	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

	const int mean_k = 9;
    const double stddev_mul_thresh = 3.0;
	const int min_neighbors = 8;
	const double radius_search = 0.5;

	// remove outliers from scan
	if(prefilter_type == PreFilterType::kStatisticFilter)
	{
 		statisticOutliersFilterPcl(mean_k,stddev_mul_thresh, cloud, cloud_filtered);
	}
	else if(prefilter_type == PreFilterType::kDeterministicFilter)
	{
		radiusOutliersFilterPcl(min_neighbors, radius_search, cloud, cloud_filtered);
	}
	else 
	{
		pcl_conversions::toPCL(cloud, *cloud_filtered);
	}

	// init the output pointcloud with max readings 
	PointCloud_Out pcl_out;
	spherical_transform.initCloud(pcl_out);

	// create a spherical laser scan according to phi, theta
	spherical_transform.sphericalCloud(*cloud_filtered, pcl_out);

    sensor_msgs::PointCloud2 expl_point_cloud;	
	pcl::toROSMsg(pcl_out, expl_point_cloud);
	expl_point_cloud.header = cloud.header; 	
	expl_point_cloud_pub.publish(expl_point_cloud);
}

template class ExplPclFilter<pcl::PointXYZ>;
