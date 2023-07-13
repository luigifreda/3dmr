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

#ifndef CONVERSIONPCL_H
#define CONVERSIONPCL_H

#include <ros/ros.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

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

#include <pcl_conversions/pcl_conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>


///	\class ConversionPcl
///	\author Luigi Freda (2016-Present) and Alcor (2016)
///	\brief A class for transforming point clouds from one frame to another
///	\note 
/// 	\todo 
///	\date
///	\warning
template<typename PointT>
class ConversionPcl
{
private:
    typedef pcl::PointCloud<PointT> PointCloudT;

    // /tf listener
    const tf::TransformListener* p_tf_listener_;

    // Name of the output frame
    std::string output_frame_;

    // Last transform from local to global frame
    tf::StampedTransform input_output_tf_;

public:
    ConversionPcl();
    ~ConversionPcl();

    inline void setOutputFrame(const std::string& f)
    {
        output_frame_ = f;
    }

    inline void setTFListener(const tf::TransformListener& tfl)
    {
        p_tf_listener_ = &tfl;
    }

    void transform(const sensor_msgs::PointCloud2& msg_in, sensor_msgs::PointCloud2& msg_out);
    void transform(const sensor_msgs::PointCloud2& msg_in, PointCloudT& pcl_out);
    void transform(const PointCloudT& pcl_in, sensor_msgs::PointCloud2& msg_out);
    void transform(const PointCloudT& pcl_in, PointCloudT& pcl_out);

    void getLastTransform(tf::Transform& t);
    void getLastTransform(tf::StampedTransform& t);

    void getFrameOrigin(const std::string& frame_id, pcl::PointXYZ& p);
    
    tf::StampedTransform getTransform(const std::string& parent, const std::string& child);
};
#endif // CONVERSIONPCL_H

