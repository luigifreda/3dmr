/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
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

#include "SpaceTimeFilterBase.h"

#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_xml.h>

#include <boost/operators.hpp>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iomanip>
#include <memory>
#include <limits>
#include <sstream>

#include <iostream>
#include <fstream>

#define VERBOSE // entry level of verbosity

namespace explplanner
{

SpaceTimeFilterBase::SpaceTimeFilterBase():pcl_pose_(new pcl::PointCloud<pcl::PointNormal>()),oldest_pose_idx_(0)
{
}

SpaceTimeFilterBase::~SpaceTimeFilterBase()
{
    std::cout << "SpaceTimeFilterBase::~SpaceTimeFilterBase() - start " << std::endl;
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    std::cout << "SpaceTimeFilterBase::~SpaceTimeFilterBase() - end " << std::endl;
}

void SpaceTimeFilterBase::init()
{
    p_tf_listener_ = boost::make_shared<tf::TransformListener>(ros::Duration(params_.tf_cache_length_));
}

bool SpaceTimeFilterBase::setParams(const std::string& prefix)
{
    std::string ns = ros::this_node::getName() + prefix;
    bool ret = true;

    if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_))
    {
        ROS_WARN("No navigation frame specified. Looking for %s. Default is %s.",
                 (ns + "/tf_frame").c_str(), params_.navigationFrame_.c_str());
    }

    if (!ros::param::get(ns + "/tf_cache_length", params_.tf_cache_length_))
    {
        ROS_WARN(
            "No tf cached length specified. Looking for %s. Default is %f.",
            (ns + "/tf_cache_length").c_str(),params_.tf_cache_length_);
    }

    if (!ros::param::get(ns + "/pose_cache_time_length", params_.pose_cache_time_length_))
    {
        ROS_WARN(
            "No pose_cache_time_length specified. Looking for %s. Default is %f.",
            (ns + "/pose_cache_time_length").c_str(),params_.pose_cache_time_length_);
    }

    if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_))
    {
        ROS_WARN(
            "No throttle time constant for the point cloud insertion specified. Looking for %s. Default is %f.",
            (ns + "/pcl_throttle").c_str(),params_.pcl_throttle_);
    }


    if (!ros::param::get(ns + "/dist_pose_thres", params_.dist_pose_thres_))
    {
        ROS_WARN(
            "No Distance threshold for spatial filter specified. Looking for %s. Default is %f",
            (ns + "/dist_pose_thres").c_str(), params_.dist_pose_thres_);
    }

    if (!ros::param::get(ns + "/rot_pose_thres", params_.rot_pose_thres_))
    {
        ROS_WARN(
            "No Distance_orientation threshold for spatial filter specified. Looking for %s. Default is %f",
            (ns + "/rot_pose_thres").c_str(),params_.rot_pose_thres_);
    }

    if (!ros::param::get(ns + "/do_downsampling", params_.do_downsampling_))
    {
        ROS_WARN(
            "No flag do_downsampling provided for spatial filter specified. Looking for %s. Default is %d",
            (ns + "/do_downsampling").c_str(),(int)params_.do_downsampling_);
    }
    if (!ros::param::get(ns + "/downsample_leaf_size", params_.downsample_leaf_size_))
    {
        ROS_WARN(
            "No downsample_leaf_size_ provided for spatial filter specified. Looking for %s. Default is %f",
            (ns + "/use_downsampling").c_str(),params_.downsample_leaf_size_);
    }

    return ret;
}

bool SpaceTimeFilterBase::filterPointCloud(const sensor_msgs::PointCloud2::ConstPtr &pointcloud, const ros::Time &pcl_stamp, SpaceTimeFilterBase::Transformation::Position &q_p_i, Eigen::Vector3d &q_n_i)
{
    //std::cout << "SpaceTimeFilterBase::filterPointCloud() from " << pointcloud->header.frame_id << " - start " << std::endl;

    //boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    SpaceTimeFilterBase::Transformation tf_sensor_to_world;
    if (!lookupTransform(pointcloud->header.frame_id, GetWorldFrame(), pointcloud->header.stamp, &tf_sensor_to_world))
    {
        std::cout << "ERROR getting transformation" << std::endl;
        return false;
    }

    // extract q_i / n_i / t_i
    q_p_i = tf_sensor_to_world.getPosition();
    q_n_i = tf_sensor_to_world.getRotationMatrix().normalized().col(2);
    q_n_i.normalize(); // normalize in place

    //std::cout << "pose from pointcloud: (" << q_p_i.x() << ", " << q_p_i.y() << ", " << q_p_i.z() << ") and normal - (" <<  q_n_i(0) << ", " << q_n_i(1) << ", " << q_n_i(2) << ")" << std::endl;

    pcl::PointNormal search_pose;
    search_pose.x = q_p_i.x();
    search_pose.y = q_p_i.y();
    search_pose.z = q_p_i.z();

    // If Lf is empty
    if (pcl_pose_->empty())
    {
        std::cout << "SpaceTimeFilterBase::filterPointCloud() - Pushing initial pose - (" << q_p_i.x() << ", " << q_p_i.y() << ", " << q_p_i.z() << ") with normal - (" << q_n_i(0) << ", " << q_n_i(1) << ", " << q_n_i(2) << ")" << std::endl;
        return true; // continue
    }

    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    int num_founds = kdtree_pose_.nearestKSearch(search_pose, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    if (num_founds == 0)
    {
        std::cout << "WARNING: getting transformation" << std::endl;
        return false;
    }

#if 0        
    Eigen::Vector3d q_pf;
    q_pf(0) = pcl_pose_->points[pointIdxNKNSearch[0]].x;
    q_pf(1) = pcl_pose_->points[pointIdxNKNSearch[0]].y;
    q_pf(2) = pcl_pose_->points[pointIdxNKNSearch[0]].z;
    std::cout << " closest pose found:  (" << q_pf(0) << ", " << q_pf(1) << ", " << q_pf(2) << ") and normal - (" <<  q_nf(0) << ", " << q_nf(1) << ", " << q_nf(2) << ")" << std::endl;
#endif

    Eigen::Vector3d q_nf;
    q_nf(0) = pcl_pose_->points[pointIdxNKNSearch[0]].normal_x;
    q_nf(1) = pcl_pose_->points[pointIdxNKNSearch[0]].normal_y;
    q_nf(2) = pcl_pose_->points[pointIdxNKNSearch[0]].normal_z;
    q_nf.normalize(); // normalize in place

    double dist_poses = sqrt(pointNKNSquaredDistance[0]);

    double cos_theta = q_n_i.dot(q_nf);            // avoid division by a quantity which risks to be zero or ill-conditioned
    double theta = acos(cos_theta) * (180 / M_PI); //  get angle in degs
    //std::cout << "q_nf= " << q_nf << " theta= " << theta << std::endl;

    ros::Time pcl_pose_time;
    fromUint64ToRosTime(pcl_pose_->header.stamp, pcl_pose_time);

    const ros::Duration timestamp_age = pcl_stamp - pcl_pose_time;
    if ((dist_poses > params_.dist_pose_thres_) || 
        (theta > params_.rot_pose_thres_) || 
        (timestamp_age.toSec() > params_.pcl_throttle_))
    {
        std::cout << "SpaceTimeFilterBase::filterPointCloud() - New pose to store - time: " << timestamp_age 
                  << " theta: " << theta << " dist: " << dist_poses 
                  << " (pose cache size: "<< pcl_pose_->size() << ")" << std::endl;
        return true;
    }
    return false;
}

bool SpaceTimeFilterBase::insertPointCloud(const sensor_msgs::PointCloud2::ConstPtr &pointcloud, const uint32_t& index)
{
    ROS_ASSERT(sizeof(uint32_t) == sizeof(float));

    //std::cout << "SpaceTimeFilterBase::insertPointCloud() - start " << std::endl;

    const ros::Time& pcl_stamp = pointcloud->header.stamp;
    SpaceTimeFilterBase::Transformation::Position q_p_i;
    Eigen::Vector3d q_n_i;

    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    if (filterPointCloud(pointcloud, pcl_stamp, q_p_i, q_n_i)) //, pcl_stamp, q_pose)
    {
        pcl::PointNormal pose;
        pose.x = q_p_i(0);
        pose.y = q_p_i(1);
        pose.z = q_p_i(2);
        *(uint32_t *)&(pose.data[3]) = index;  // store the index the in the 4th unused data field

        pose.normal_x = q_n_i(0);
        pose.normal_y = q_n_i(1);
        pose.normal_z = q_n_i(2);

        pcl_pose_->header.stamp = fromRosTimeToUint64(pcl_stamp);
        pcl_pose_->push_back(pose);
        pcl_pose_stamp_.push_back(pcl_stamp);

        ROS_ASSERT_MSG(pcl_pose_->size()==pcl_pose_stamp_.size(),"pcl_pose_ and pcl_pose_stamp_ should have equal size");      

        // control pcl_pose_ size: remove old poses 
        while(pcl_pose_->size()>2)
        {
            const ros::Duration timestamp_age = ros::Time::now() - pcl_pose_stamp_[oldest_pose_idx_];
            if(timestamp_age.toSec() > params_.pose_cache_time_length_)
            {
                auto& oldest_pose = (*pcl_pose_)[oldest_pose_idx_];
                auto& oldest_time = pcl_pose_stamp_[oldest_pose_idx_];

                std::swap(oldest_pose, pcl_pose_->back());
                pcl_pose_->points.pop_back();
                pcl_pose_->width = pcl_pose_->size();            
                ROS_ASSERT_MSG(pcl_pose_->height==1,"pcl_pose_ should have height=1!");  

                std::swap(oldest_time, pcl_pose_stamp_.back());
                pcl_pose_stamp_.pop_back();

                oldest_pose_idx_ = (oldest_pose_idx_+1) % pcl_pose_->size();            
            }
            else
            {
                break; 
            }
        }

        kdtree_pose_.setInputCloud(pcl_pose_);     

        return true; 
    }
    else 
    {
        return false; 
    }
    //std::cout << "SpaceTimeFilterBase::insertPointCloud() - end " << std::endl;
}

bool SpaceTimeFilterBase::lookupTransform(const std::string &from_frame,
                                     const std::string &to_frame,
                                     const ros::Time &timestamp,
                                     Transformation *transform)
{
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
}

bool SpaceTimeFilterBase::lookupTransformTf(const std::string &from_frame,
                                       const std::string &to_frame,
                                       const ros::Time &timestamp,
                                       Transformation *transform)
{
    tf::StampedTransform tf_transform;

    ros::Time time_to_lookup = timestamp;

    // If this transform isn't possible at the time, then try to just look up
    // the latest (this is to work with bag files and static transform publisher,
    // etc).
    if (!p_tf_listener_->canTransform(to_frame, from_frame, time_to_lookup))
    {
        const ros::Duration timestamp_age = ros::Time::now() - time_to_lookup;      
        if (timestamp_age < p_tf_listener_->getCacheLength())
        {
#if 0              
            time_to_lookup = ros::Time(0);
            ROS_WARN("Using latest TF transform instead of timestamp match.");
        }
        else  
        {
#endif                   
            ROS_ERROR("Requested transform time older than cache limit.");
            return false;
        }
    }

    try
    {
        p_tf_listener_->lookupTransform(to_frame, from_frame, time_to_lookup,
                                        tf_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR_STREAM(
            "Error getting TF transform from sensor data: " << ex.what());
        return false;
    }

    tf::transformTFToKindr(tf_transform, transform);
    return true;
}


void SpaceTimeFilterBase::getPoseArrayMessage(geometry_msgs::PoseArray& message)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    message.header.stamp = ros::Time::now();

    message.poses.resize(pcl_pose_->size());
    for(int i=0;i<pcl_pose_->size();i++)
    {
        const auto& pose = (*pcl_pose_)[i];
        message.poses[i].position.x = pose.x;
        message.poses[i].position.y = pose.y;
        message.poses[i].position.z = pose.z;

        // TODO: fill if needed also the quaternion part
        //message.poses[i].orientation     
    }
}

bool SpaceTimeFilterBase::getClosestScanInfo(const pcl::PointNormal& searchPoint, uint32_t& scanIndex, float& scanSquaredDistance)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    bool res = true; 
    const int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if(pcl_pose_->empty())
    {
        scanIndex = 0;
        scanSquaredDistance = -1;
        res = false;        
        ROS_WARN_STREAM("SpaceTimeFilterBase::getClosestScanIndex() - no cloud stored yet!");
    }
    else
    {
        int num_founds = kdtree_pose_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        if (num_founds == 0)
        {
            ROS_WARN_STREAM("SpaceTimeFilterBase::getClosestScanIndex() - could not get closest point");
            scanIndex = 0;
            scanSquaredDistance = -1;
            res = false;
        }
        else
        { 
            scanIndex = *(uint32_t*)&(pcl_pose_->points[pointIdxNKNSearch[0]].data[3]);
            scanSquaredDistance = pointNKNSquaredDistance[0];
        }
    }
    return res; 
}

} // namespace 