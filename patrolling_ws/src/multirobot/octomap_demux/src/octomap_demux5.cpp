/**
* This file is part of the ROS package octomap_demux which belongs to the framework 3DMR. 
*
* Copyright (C) 2017-2019 Luigi Freda <luigifreda at gmail dot com>  
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

#include "octomap_demux/octomap_demux5.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/impl/transforms.hpp>

OctomapDeMux5::OctomapDeMux5(ros::NodeHandle& n) :
nh_(n),
n_("~"),
tf_() {
    global_ref = getParam<std::string>(n_, "global_ref", "map");

    pcl_sub_name = getParam<std::string>(n_, "pcl_sub_name", "/volumetric_mapping/octomap_pcl");
    //pcl_sub_.subscribe(nh_, pcl_sub_name, kSubMessageQueueSize);
    //pcl_tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pcl_sub_, tf_, global_ref, 20);
    //pcl_tf_filter_->registerCallback(&OctomapDeMux5::pointMapCallback, this);
    pcl_normal_sub_ = nh_.subscribe(pcl_sub_name, 10, &OctomapDeMux5::pointMapCallback, this);

    pcl1_pub_name = getParam<std::string>(n_, "pcl1_pub_name", "/vrep/ugv1/local_map");
    pcl2_pub_name = getParam<std::string>(n_, "pcl2_pub_name", "/vrep/ugv2/local_map");
    pcl3_pub_name = getParam<std::string>(n_, "pcl3_pub_name", "/vrep/ugv3/local_map");
    pcl4_pub_name = getParam<std::string>(n_, "pcl4_pub_name", "/vrep/ugv4/local_map");
    pcl5_pub_name = getParam<std::string>(n_, "pcl5_pub_name", "/vrep/ugv5/local_map");

    pcl1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl1_pub_name, kPubMessageQueueSize);
    pcl2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl2_pub_name, kPubMessageQueueSize);
    pcl3_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl3_pub_name, kPubMessageQueueSize);
    pcl4_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl4_pub_name, kPubMessageQueueSize);
    pcl5_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl5_pub_name, kPubMessageQueueSize);


}

void OctomapDeMux5::pointMapCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
    std::cout << "OctomapDeMux Volumetric mapping Octomap PCD" << cloud_msg_in.header.frame_id << std::endl;

        try {

            pcl1_pub_.publish(cloud_msg_in);
            pcl2_pub_.publish(cloud_msg_in);
            pcl3_pub_.publish(cloud_msg_in);
            pcl4_pub_.publish(cloud_msg_in);
            pcl5_pub_.publish(cloud_msg_in);

        } catch (tf::ExtrapolationException e) {
            printf("Failure %s\n", e.what()); //Print exception which was caught
        }
    
}
