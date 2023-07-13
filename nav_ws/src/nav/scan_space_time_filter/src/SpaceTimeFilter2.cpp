/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
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

#include "SpaceTimeFilter2.h"

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
#include <chrono>

#define VERBOSE // entry level of verbosity

SpaceTimeFilter2::SpaceTimeFilter2(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    init();
}

SpaceTimeFilter2::~SpaceTimeFilter2()
{
    std::cout << "SpaceTimeFilter2::~SpaceTimeFilter2() - start " << std::endl;

    stopPubThread();
    
    std::cout << "SpaceTimeFilter2::~SpaceTimeFilter2() - end " << std::endl;
}

// initialize the vars
void SpaceTimeFilter2::init()
{
    robot_id_ = 0;

    std::string ns = ros::this_node::getName();

    if (!ros::param::get(ns + "/pubThread/sleep_time", pubThredSleepTimeInMs))
    {
        ROS_WARN_STREAM("No pub thread sleep time specified. Looking for " << ns << "/pubThread/sleep_time. Default is " << pubThredSleepTimeInMs);
    }

    if (!ros::param::get(ns + "/pubThread/no_sleep_max_queue_size", noSleepQueueSize))
    {
        ROS_WARN_STREAM("No value specified for no_sleep_max_queue_size. Looking for " << ns << "/pubThread/no_sleep_max_queue_size. Default is " << noSleepQueueSize);
    }

    pFilter1_.reset( new SpaceTimeFilterBase()); 
    if (!pFilter1_->setParams("/scan_filter1"))
    {
        ROS_ERROR("Could not start the filter1. Parameters missing!");
    }

    pFilter2_.reset( new SpaceTimeFilterBase()); 
    if (!pFilter2_->setParams("/scan_filter2"))
    {
        ROS_ERROR("Could not start the filter2. Parameters missing!");
    }

    pFilter1_->init();
    pFilter2_->init();

    // N.B.: this must stay here since we set params_.inspectionPath_!
    setPubsAndSubs();
}

void SpaceTimeFilter2::setPubsAndSubs()
{
    // < Publishers
    filteredPointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud", 1);

    // < Subscribers
    pointcloud_sub1_ = nh_.subscribe("dynamic_point_cloud1", 1, &SpaceTimeFilter2::processPointcloudWithTf1, this);
    pointcloud_sub2_ = nh_.subscribe("dynamic_point_cloud2", 1, &SpaceTimeFilter2::processPointcloudWithTf2, this);
}

void SpaceTimeFilter2::processPointcloudWithTf1(const sensor_msgs::PointCloud2::ConstPtr &pointcloud)
{
    //std::cout << "SpaceTimeFilter2::processPointcloudWithTf1() - start " << std::endl;
    if(pFilter1_->insertPointCloud(pointcloud))
    {
        std::cout << "SpaceTimeFilter2::processPointcloudWithTf1() " << std::endl;           
        sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2()); 
        *cloud = *pointcloud;
        queue_.push(cloud);     
    }

    //std::cout << "SpaceTimeFilter2::processPointcloudWithTf1() - end " << std::endl;
}

void SpaceTimeFilter2::processPointcloudWithTf2(const sensor_msgs::PointCloud2::ConstPtr &pointcloud)
{
    //std::cout << "SpaceTimeFilter2::processPointcloudWithTf2() - start " << std::endl;
    if(pFilter2_->insertPointCloud(pointcloud))
    {
        std::cout << "SpaceTimeFilter2::processPointcloudWithTf2() " << std::endl;        
        sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2()); 
        *cloud = *pointcloud;  
        queue_.push(cloud);    
    }

    //std::cout << "SpaceTimeFilter2::processPointcloudWithTf2() - end " << std::endl;
}

void SpaceTimeFilter2::setRobotId(int id)
{
    robot_id_ = id;
    std::stringstream ss;
    ss << "ugv" << id + 1;
    str_robot_name_ = ss.str();
}

void SpaceTimeFilter2::startPubThread()
{
    pPubThread_.reset( new std::thread(&SpaceTimeFilter2::pubThreadLoop,this) );
}

void SpaceTimeFilter2::stopPubThread()
{
    shouldStopThread_ = true; 
    if(pPubThread_) pPubThread_->join();
}

void SpaceTimeFilter2::pubThreadLoop()
{
    while(!shouldStopThread_)
    {
        sensor_msgs::PointCloud2::Ptr pointcloud; 
        queue_.pop(pointcloud);
        filteredPointCloudPub_.publish(pointcloud);
        std::cout << "SpaceTimeFilter2::pubThreadLoop() - publishing out" << std::endl;   

        if(queue_.size() < noSleepQueueSize)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(pubThredSleepTimeInMs));
        }
        else
        {
            // let's sleep for a minimum amount of time
            std::this_thread::sleep_for(std::chrono::milliseconds(pubThredMinSleepTimeInMs));            
        }
        
    }
}