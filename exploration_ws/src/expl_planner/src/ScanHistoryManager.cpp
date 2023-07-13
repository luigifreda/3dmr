/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
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

#include "ScanHistoryManager.h"
#include "SpaceTimeFilterBase.h"
#include "ScanFileManager.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iomanip>
#include <memory>
#include <limits> 
#include <sstream> 

#include <iostream>
#include <fstream>
#include <random>

bool randomBool() 
{
    static auto gen = std::bind(std::uniform_int_distribution<>(0,1),std::default_random_engine());
    return gen();
}

namespace explplanner
{

const float ScanHistoryManager::kCacheTimeInSec = 10; // [sec]
const float ScanHistoryManager::kMaxScanDistance = 1; // [m]
const float ScanHistoryManager::kMaxScanSquaredDistance = ScanHistoryManager::kMaxScanDistance * ScanHistoryManager::kMaxScanDistance; // [m^2]

ScanHistoryManager::ScanHistoryManager(const int& robotId)
{
    robot_id_ = robotId;
    //init();
}

void ScanHistoryManager::init()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    isInit_ = true;     

    p_space_time_filter_.reset( new SpaceTimeFilterBase());
    p_space_time_filter_->setParams("/scan_history");
    p_space_time_filter_->init();

    p_scan_file_manager_.reset( new ScanFileManager(robot_id_));
}

void ScanHistoryManager::insert(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    if(!isInit_) return; 

    // let's select proper time-spatially separated clouds 
    if (p_space_time_filter_->insertPointCloud(cloud_msg, counter_))
    {
        ROS_INFO_STREAM("ScanHistoryManager::insert() - inserting scan " << counter_);

        // convert to pointcloud2 to more comfortably manage it with ScanFileManager 
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2); 
        pcl_conversions::toPCL(*cloud_msg, *cloud);

        auto it = mapCloudPtrToScanInfoPtr_.find(cloud);
        if( it == mapCloudPtrToScanInfoPtr_.end())
        {
            ScanInfo::Ptr scan(new ScanInfo(counter_, ros::Time::now(), cloud));
            mapCloudPtrToScanInfoPtr_[cloud] = scan; 
            mapIndexToScanInfoPtr_[counter_] = scan;             
            recent_scans_.insert(scan);

            counter_++; // increment the counter once we have actually inserted the pointcloud 
        }
        else
        {
            ROS_ERROR("ScanHistoryManager::insert() - You already inserted the scan!");
        }

        // now update the cache history 
        updateHistory();
    }    

}

void ScanHistoryManager::updateHistory()
{
    ROS_INFO_STREAM("ScanHistoryManager::updateHistory()");
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);    
    ros::Time now = ros::Time::now();
    int savedClouds = 0; 
    for(auto it=recent_scans_.begin(), itEnd=recent_scans_.end(); it != itEnd; it++)
    {
        // release and save unused scans
        ScanInfo::Ptr scanInfo = *it; 

        // elapsed time since the last usage 
        ros::Duration elapsedTime = now - scanInfo->timestampLastReload;
        if(elapsedTime.toSec() > kCacheTimeInSec)
        {
            ROS_INFO_STREAM("ScanHistoryManager::updateHistory() - saving cloud " << scanInfo->index);

            // save cloud to file 
            p_scan_file_manager_->save(scanInfo->index, *(scanInfo->cloud_ptr));

            // release cloud memory 
            scanInfo->cloud_ptr.reset();

            // erase the scan from recent history
            it = recent_scans_.erase(it);
            savedClouds++;
        }
    }
    ROS_INFO_STREAM("ScanHistoryManager::updateHistory() - saved " << savedClouds);

}

void ScanHistoryManager::setRobotId(int id)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);    

    robot_id_ = id;
    p_scan_file_manager_->setRobotId(id);
    init();
}

void ScanHistoryManager::getPoseArrayMessage(geometry_msgs::PoseArray& message)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    p_space_time_filter_->getPoseArrayMessage(message);
    std::stringstream ss; 
    ss << robot_id_; 
    message.header.frame_id = ss.str();
} 

bool ScanHistoryManager::getCloudMessage(sensor_msgs::PointCloud2::Ptr &cloud_msg, const uint32_t& index)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    bool res = true; 
    auto it = mapIndexToScanInfoPtr_.find(index);
    if( it != mapIndexToScanInfoPtr_.end())
    {
        ScanInfo::Ptr& scan = it->second;  
        if(!scan->cloud_ptr)
        {
            ROS_INFO_STREAM("ScanHistoryManager::getCloudMessage() - reloading back cloud " << index);
            scan->cloud_ptr.reset(new pcl::PCLPointCloud2());
            p_scan_file_manager_->read(index, *(scan->cloud_ptr));

            //pcl_conversions::fromPCL(*(scan->cloud_ptr), *cloud_msg);
            scan->timestampLastReload = ros::Time::now();
            recent_scans_.insert(scan);
        }

        pcl_conversions::fromPCL(*(scan->cloud_ptr), *cloud_msg);
        // scan->timestampLastReload = ros::Time::now();
        // recent_scans_.insert(scan);
    }
    else
    {
        ROS_ERROR_STREAM("ScanHistoryManager::getPoseArrayMessage() didnt find cloud with index" << index);
        res = false;
    }
    return res;
}

void ScanHistoryManager::mapMessageOverlapCheck(const geometry_msgs::PoseArray& message, std::vector<sensor_msgs::PointCloud2::Ptr>& cloudsToSend)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    ROS_INFO_STREAM("ScanHistoryManager::mapMessageOverlapCheck() - checking message with " << message.poses.size() << " poses");

    cloudsToSend.clear();
    for(size_t i=0; i<message.poses.size();i++)
    {
        pcl::PointNormal searchPoint; 
        uint32_t scanIndex;   
        float scanSquaredDistance; 

        searchPoint.x = message.poses[i].position.x;
        searchPoint.y = message.poses[i].position.y;
        searchPoint.z = message.poses[i].position.z;

        if( p_space_time_filter_->getClosestScanInfo(searchPoint, scanIndex, scanSquaredDistance) )
        {
            //if(randomBool()) // this just for testing 
            if( scanSquaredDistance > kMaxScanSquaredDistance )            
            {
                cloudsToSend.emplace_back(new sensor_msgs::PointCloud2());
                if( !getCloudMessage( cloudsToSend.back(), scanIndex) )
                {
                    ROS_ERROR_STREAM("ScanHistoryManager::mapMessageOverlapCheck() - we could not find the cloud!");
                    cloudsToSend.pop_back(); // remove last added element since for some reason we didn't fin the cloud
                }
            }
            else
            {
                //ROS_INFO_STREAM("ScanHistoryManager::mapMessageOverlapCheck() - found close point to pose " << i);   
            }
        }
        else
        {
            ROS_ERROR_STREAM("ScanHistoryManager::mapMessageOverlapCheck() - could not find closest scan");
        }

    }    
}

} // namespace explplanner
