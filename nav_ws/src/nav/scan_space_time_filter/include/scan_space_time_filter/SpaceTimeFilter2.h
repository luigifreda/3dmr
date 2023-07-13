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

#ifndef SPACETIME_FILTERN_H_
#define SPACETIME_FILTERN_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include "SpaceTimeFilterBase.h"
#include "Queue.h"

#include <thread>
#include <memory>


///	\class SpaceTimeFilter2
///	\author Luigi Freda
///	\brief Space-timer filter for downsampling the scans continuously acquired by 2 cameras
///	\note
/// \todo
///	\date
///	\warning
class SpaceTimeFilter2
{
public: 

    SpaceTimeFilter2(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~SpaceTimeFilter2();

public: // setters
    void setRobotId(int id);
    void startPubThread(); 
    void stopPubThread(); 

private: // private data

    //ROS node handle
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //Publisher for filtered point clouds
    ros::Publisher filteredPointCloudPub_;

    // input point cloud to filter 
    ros::Subscriber pointcloud_sub1_;
    ros::Subscriber pointcloud_sub2_; 

    SpaceTimeFilterBase::Ptr pFilter1_;
    SpaceTimeFilterBase::Ptr pFilter2_;

private: 

    // initialize the vars
    void init();

    // set the topic name of the publishers
    void setPubsAndSubs();

    void processPointcloudWithTf1(const sensor_msgs::PointCloud2::ConstPtr &pointcloud);
    void processPointcloudWithTf2(const sensor_msgs::PointCloud2::ConstPtr &pointcloud);

    void pubThreadLoop();

private:

    int robot_id_;
    std::string str_robot_name_;

    Queue<sensor_msgs::PointCloud2::Ptr> queue_;

    std::unique_ptr<std::thread> pPubThread_;
    std::atomic<bool> shouldStopThread_{false};
    std::atomic<bool> isStopThread_{false};      
        
    int pubThredSleepTimeInMs = 100; 
    int pubThredMinSleepTimeInMs = 5; 
    int noSleepQueueSize = 6;

};

#endif //
