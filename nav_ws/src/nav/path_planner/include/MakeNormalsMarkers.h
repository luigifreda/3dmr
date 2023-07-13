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

#ifndef MAKE_NORMAL_MARKERS_H_
#define MAKE_NORMAL_MARKERS_H_

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <set>

#include <ros/ros.h>

#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <VoxelBinaryKey.h>


#define VERBOSE_MAKE_NORMALS 0


// X axis
static const tf::Vector3 xAxis = tf::Vector3(1, 0, 0);
// Z axis
static const tf::Vector3 zAxis = tf::Vector3(0, 0, 1);    

template<typename PointT>
void makeNormalsMarkers(const pcl::PointCloud<PointT>& pcl_norm, geometry_msgs::PoseArray& markers, double leaf_size = 0.1)
{
    // fill header
    markers.header.frame_id = pcl_norm.header.frame_id;
    markers.header.stamp = ros::Time::now();

    std::set<uint64_t> k;

    for (int i = 0; i < pcl_norm.size(); i++)
    {
        uint64_t ki = voxelBinaryKey(pcl_norm[i], leaf_size);
        if (k.count(ki)) continue;

        if (std::isfinite(pcl_norm.points[i].normal_x) && std::isfinite(pcl_norm.points[i].normal_y) && std::isfinite(pcl_norm.points[i].normal_z) )        
        {
            geometry_msgs::Pose pose;
            pose.position.x = pcl_norm.points[i].x;
            pose.position.y = pcl_norm.points[i].y;
            pose.position.z = pcl_norm.points[i].z;            
            
            tf::Quaternion q; 
                    
            tf::Vector3 v = tf::Vector3(pcl_norm.points[i].normal_x, pcl_norm.points[i].normal_y, pcl_norm.points[i].normal_z);
            
            if(v.length2() < 1e-3) 
            {
#if VERBOSE_MAKE_NORMALS                
                std::cout << "makeNormalsMarkers() - v length is smaller than 1e-3: " << v.length2() << std::endl;
#endif              
                continue;
            } 
            
            tf::Vector3 cross = tf::tfCross(xAxis, v);            
            const double cross_length = cross.length(); 
            if(cross_length > 1e-3)
            {           
                // q = axis_vector*sin(theta/2) + 1*cos(theta/2) =~ axis_vector + cos(theta/2)/sin(theta/2) =~ axis_vector*sin(theta) + (1+cos(theta))  ONLY IF sin(theta) != 0 
                // cross = axis_vector 
                q = tf::Quaternion(cross.x(), cross.y(), cross.z(), cross_length*(1. + tf::tfDot(xAxis, v)));
            }
            else
            {
                // q = axis_vector*sin(theta/2) + 1*cos(theta/2) =~ axis_vector + cos(theta/2)/sin(theta/2) =~ axis_vector*sin(theta) + (1+cos(theta))  ONLY IF sin(theta) != 0 
                // cross = axis_vector 
                cross = tf::tfCross(zAxis, v); 
                q = tf::Quaternion(cross.x(), cross.y(), cross.z(), cross.length()*(1. + tf::tfDot(zAxis, v)));
            }
            q.normalize();            
            tf::quaternionTFToMsg(q, pose.orientation);
            markers.poses.push_back(pose);
            k.insert(ki);            
        }
#if VERBOSE_MAKE_NORMALS                
        else
        {
            std::cout << "makeNormalsMarkers() - v has some infinite component!" << std::endl;
        }
#endif             
    }
}

#endif // MAKE_NORMAL_MARKERS_H_
