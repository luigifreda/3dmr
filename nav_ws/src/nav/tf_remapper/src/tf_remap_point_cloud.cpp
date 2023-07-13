/**
* This file is part of the ROS package tf_remapper which belongs to the framework 3DMR. 
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

#include <ros/ros.h>
#include <tf/tf.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


std::string robot_name;
std::string prefix; 

ros::Publisher pcl_pub;

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
    {
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
}

void pointCloudCallback(const sensor_msgs::PointCloud2& pcl_msg)
{
    sensor_msgs::PointCloud2 pcl_out_msg = pcl_msg;
    pcl_out_msg.header.frame_id = prefix + pcl_msg.header.frame_id;
    pcl_pub.publish(pcl_out_msg);
}


/// < this runs on robot, it receives a point cloud and changes its header so at to have a robot_name prefix
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_remap_point_cloud"); 
    
    ros::NodeHandle n("~");
    
    /// < get parameters
    robot_name = getParam<std::string>(n, "robot_name", "ugv1");
    prefix = "/" + robot_name + "/";
    
    /// < subscribers 
    ros::Subscriber pcl_sub = n.subscribe("/input_topic", 1, &pointCloudCallback);

    /// < publishers 
    pcl_pub =  n.advertise<sensor_msgs::PointCloud2>("/output_topic", 1, true);
    
    ros::spin();
    return 0;
};
