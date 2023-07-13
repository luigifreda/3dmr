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

#include <ConversionPcl.h>
#include <stdio.h>

template<typename PointT>
ConversionPcl<PointT>::ConversionPcl() :
output_frame_("map"),
p_tf_listener_(0L)
{
}

template<typename PointT>
ConversionPcl<PointT>::~ConversionPcl()
{
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const sensor_msgs::PointCloud2& msg_in, sensor_msgs::PointCloud2& msg_out)
{
    ros::Time scan_time = msg_in.header.stamp;
    tf::StampedTransform input_output_tf;

    if (!p_tf_listener_)
    {
        ROS_ERROR("ConversionPcl::transform: tf_listener is not set");
        return;
    }

    try
    {
        //tf_listener.lookupTransform(laser_frame, global_frame, scan_time, map_laser_tf);
        p_tf_listener_->lookupTransform(output_frame_, msg_in.header.frame_id, scan_time, input_output_tf);
        sensor_msgs::PointCloud2 msg_new_scan;
        pcl_ros::transformPointCloud(output_frame_, msg_in, msg_out, *p_tf_listener_);
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR("ConversionPcl::transform: tf::LookupException&: %s", ex.what());
        return;
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR("ConversionPcl::transform: tf::ExtrapolationException&: %s", ex.what());
        return;
    }
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const sensor_msgs::PointCloud2& msg_in, PointCloudT& pcl_out)
{
    sensor_msgs::PointCloud2 msg_out;
    transform(msg_in, msg_out);
    pcl::fromROSMsg(msg_out, pcl_out);

    pcl_out.header = pcl_conversions::toPCL(msg_in.header);
    pcl_out.header.frame_id = output_frame_;
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const PointCloudT& pcl_in, sensor_msgs::PointCloud2& msg_out)
{
    sensor_msgs::PointCloud2 msg_in;
    pcl::toROSMsg(pcl_in, msg_in);
    transform(msg_in, msg_out);
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const PointCloudT& pcl_in, PointCloudT& pcl_out)
{
    sensor_msgs::PointCloud2 msg_in;
    pcl::toROSMsg(pcl_in, msg_in);
    transform(msg_in, pcl_out);

    pcl_out.header = pcl_conversions::toPCL(msg_in.header);
    pcl_out.header.frame_id = output_frame_;
}

template<typename PointT>
void ConversionPcl<PointT>::getLastTransform(tf::Transform& t)
{
    t.setOrigin(input_output_tf_.getOrigin());
    t.setRotation(input_output_tf_.getRotation());
}

template<typename PointT>
void ConversionPcl<PointT>::getLastTransform(tf::StampedTransform& t)
{
    t = input_output_tf_;
}

template<typename PointT>
void ConversionPcl<PointT>::getFrameOrigin(const std::string& frame_id, pcl::PointXYZ& p)
{
    tf::Stamped<tf::Vector3> origin, point;
    origin.frame_id_ = frame_id;
    origin.setZero();
    point.frame_id_ = output_frame_;

    try
    {
        p_tf_listener_->transformPoint(output_frame_, origin, point);
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR("ConversionPcl::getFrameOrigin: tf::LookupException&: %s", ex.what());
        return;
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR("ConversionPcl::getFrameOrigin: tf::ExtrapolationException&: %s", ex.what());
        return;
    }

}

template<typename PointT>
tf::StampedTransform ConversionPcl<PointT>::getTransform(const std::string& parent, const std::string& child)
{
    tf::StampedTransform transform;

    if( p_tf_listener_->waitForTransform(parent,child,ros::Time(),ros::Duration(1.0)) )
    {
	try
	{
	    p_tf_listener_->lookupTransform(parent,child,ros::Time(),transform);
	}
	catch( tf::LookupException& ex )
	{
	    ROS_WARN("no transform available: %s\n",ex.what());
	    return tf::StampedTransform();
	}
	catch( tf::ConnectivityException& ex )
	{
	    ROS_WARN("connectivity error: %s\n",ex.what());
	    return tf::StampedTransform();
	}
	catch( tf::ExtrapolationException& ex )
	{
	    ROS_WARN("extrapolation error: %s\n",ex.what());            
	    return tf::StampedTransform();
	}

	return transform;
    }

    std::string error_message = "transformation not available between " + parent +" and " + child;
    ROS_ERROR_STREAM(error_message.c_str());

    return tf::StampedTransform();
}

template class ConversionPcl<pcl::PointXYZ>;
template class ConversionPcl<pcl::PointXYZRGBNormal>;

