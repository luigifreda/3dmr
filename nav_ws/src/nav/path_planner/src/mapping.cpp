/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Alcor Lab (La Sapienza University)
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

#include <ros/ros.h>
#include <DynamicJoinPcl.h>
#include <NormalEstimationPcl.h>
#include <ColorNormalsPcl.h>
#include <ConversionPcl.h>
#include <MakeNormalsMarkers.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

#include "KdTreeFLANN.h"


DynamicJoinPcl<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> dynjoinpcl;
NormalEstimationPcl<pcl::PointXYZRGBNormal> normal_estimator;
ConversionPcl<pcl::PointXYZ> conv_pcl;

ros::Publisher pcl_normal_pub;
ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub;

pcl::PointCloud<pcl::PointXYZRGBNormal> map_pcl;

std::string laser_frame_name;
std::string global_frame_name;

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}

void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
{
    geometry_msgs::PoseArray poseArray;
    makeNormalsMarkers(pcl_norm, poseArray);
    ROS_INFO("normals poseArray size: %ld", poseArray.poses.size());
    marker_normal_pub.publish(poseArray);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& scan_msg)
{
    ROS_INFO("Received a new PointCloud2 message");

    DynamicJoinPclConfig dynjoinpcl_config = dynjoinpcl.getConfig();
    NormalEstimationPclConfig normal_config = normal_estimator.getConfig();

    // downsample
    pcl::PointCloud<pcl::PointXYZ> scan_pcl;
    pcl::PointCloud<pcl::PointXYZRGBNormal> scan_norm_pcl;
    conv_pcl.transform(scan_msg, scan_pcl);
    scan_norm_pcl.header = scan_pcl.header;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(scan_pcl.makeShared());
    sor.setLeafSize(dynjoinpcl_config.leaf_size, dynjoinpcl_config.leaf_size, dynjoinpcl_config.leaf_size);
    sor.filter(scan_pcl);
    pcl::copyPointCloud(scan_pcl, scan_norm_pcl);

    // normal estimation
    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> scan_norm_kdtree;
    scan_norm_kdtree.setInputCloud(scan_norm_pcl.makeShared());
    tf::Transform t;
    pcl::PointXYZ laser_center;
    conv_pcl.getLastTransform(t);
    conv_pcl.getFrameOrigin(dynjoinpcl_config.laser_frame, laser_center);
    normal_estimator.computeNormals(scan_norm_pcl, scan_norm_kdtree, laser_center);

    // dynamic join
    pcl::PointCloud<pcl::PointXYZRGBNormal> map_new_pcl;
    map_new_pcl.header = map_pcl.header;
    dynjoinpcl.joinPCL(scan_norm_pcl, map_pcl, map_new_pcl, laser_center);
    map_pcl.swap(map_new_pcl);
    //colorNormalsPCL(map_pcl);
    visualizeNormals(map_pcl);
    sensor_msgs::PointCloud2 map_msg_out;
    pcl::toROSMsg(map_pcl, map_msg_out);
    pcl_pub.publish(map_msg_out);
}

void dynjoinpclConfigCallback(DynamicJoinPclConfig& config, uint32_t level)
{
    dynjoinpcl.setConfig(config);
    conv_pcl.setOutputFrame(config.global_frame);
    map_pcl.header.frame_id = config.global_frame;
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
    normal_estimator.setConfig(config);
}

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "test_dynjoinpcl_with_normal");
    ros::init(argc, argv, "mapping_node");

    tf::TransformListener tf_listener(ros::Duration(10.0));

    ros::NodeHandle n("~");
    
    laser_frame_name = getParam<std::string>(n, "laser_frame_name", std::string());
    if(!laser_frame_name.empty())
    {
        NormalEstimationPclConfig& normal_config = normal_estimator.getConfig();
        normal_config.laser_frame = laser_frame_name;
    }

    dynamic_reconfigure::Server<DynamicJoinPclConfig> dynjoinpcl_config_server(ros::NodeHandle("~/DynamicJoinPcl"));
    dynjoinpcl_config_server.setCallback(boost::bind(&dynjoinpclConfigCallback, _1, _2));

    dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
    normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

    conv_pcl.setTFListener(tf_listener);

    ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);

    marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker", 1);
    pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/dynjoinpcl", 1, true);

    ros::spin();
    return 0;
}

