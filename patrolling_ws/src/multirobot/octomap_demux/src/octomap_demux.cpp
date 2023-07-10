/**
* This file is part of the ROS package octomap_demux which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2017-2019 Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#include "octomap_demux/octomap_demux.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/impl/transforms.hpp>

OctomapDeMux::OctomapDeMux(ros::NodeHandle& n) :
nh_(n),
n_("~"),
tf_() {
    global_ref = getParam<std::string>(n_, "global_ref", "map");
    ref_frame_1 = getParam<std::string>(n_, "ref_frame_1", "ugv1/map");
    ref_frame_2 = getParam<std::string>(n_, "ref_frame_2", "ugv2/map");

    pcl_sub_name = getParam<std::string>(n_, "pcl_sub_name", "/volumetric_mapping/octomap_pcl");
    pcl_sub_.subscribe(nh_, pcl_sub_name, kSubMessageQueueSize);
    pcl_tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pcl_sub_, tf_, global_ref, 20);
    pcl_tf_filter_->registerCallback(&OctomapDeMux::pointMapCallback, this);

    pcl1_pub_name = getParam<std::string>(n_, "pcl1_pub_name", "/vrep/ugv1/local_map");
    pcl2_pub_name = getParam<std::string>(n_, "pcl2_pub_name", "/vrep/ugv2/local_map");

    pcl1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl1_pub_name, kPubMessageQueueSize);
    pcl2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl2_pub_name, kPubMessageQueueSize);


}

void OctomapDeMux::pointMapCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
    std::cout << "OctomapDeMux Volumetric mapping Octomap PCD" << cloud_msg_in.header.frame_id << std::endl;

    if (cloud_msg_in.header.frame_id.compare(ref_frame_1) == 0) {
        sensor_msgs::PointCloud2 pcl_out2;

        try {
            pcl_ros::transformPointCloud(ref_frame_2, cloud_msg_in, pcl_out2, tf_);

            pcl1_pub_.publish(cloud_msg_in);
            pcl2_pub_.publish(pcl_out2);

        } catch (tf::ExtrapolationException e) {
            printf("Failure %s\n", e.what()); //Print exception which was caught
        }
    } else if (cloud_msg_in.header.frame_id.compare(ref_frame_2) == 0) {
        sensor_msgs::PointCloud2 pcl_out1;

        try {
            pcl_ros::transformPointCloud(ref_frame_1, cloud_msg_in, pcl_out1, tf_);

            pcl1_pub_.publish(pcl_out1);
            pcl2_pub_.publish(cloud_msg_in);

        } catch (tf::ExtrapolationException e) {
            printf("Failure %s\n", e.what()); //Print exception which was caught
        }

    } else {
        sensor_msgs::PointCloud2 pcl_out1;
        sensor_msgs::PointCloud2 pcl_out2;

        try {
            pcl_ros::transformPointCloud(ref_frame_1, cloud_msg_in, pcl_out1, tf_);
            pcl_ros::transformPointCloud(ref_frame_2, cloud_msg_in, pcl_out2, tf_);

            pcl1_pub_.publish(pcl_out1);
            pcl2_pub_.publish(pcl_out2);

        } catch (tf::ExtrapolationException e) {
            printf("Failure %s\n", e.what()); //Print exception which was caught
        }
    }



}
