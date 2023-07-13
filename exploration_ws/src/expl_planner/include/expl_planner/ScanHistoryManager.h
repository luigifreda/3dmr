/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
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

#ifndef SCAN_HISTORY_MANAGER_H_
#define SCAN_HISTORY_MANAGER_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <math.h>
#include <limits>
#include <algorithm>
#include <map>
#include <set>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/core/noncopyable.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


namespace explplanner{

class SpaceTimeFilterBase;
class ScanFileManager; 

///	\class ScanInfo
///	\author Luigi Freda 
///	\brief 
///	\note
/// \todo
///	\date
///	\warning
struct ScanInfo
{
    typedef std::shared_ptr<ScanInfo> Ptr;

    ScanInfo(const uint32_t& indexIn=0, const ros::Time& timestampIn = ros::Time::now(), const pcl::PCLPointCloud2::Ptr& ptr=nullptr)
        :index(indexIn), timestamp(timestampIn), timestampLastReload(timestampIn), cloud_ptr(ptr)
    {}
    uint32_t index; 
    ros::Time timestamp;
    ros::Time timestampLastReload;
    pcl::PCLPointCloud2::Ptr cloud_ptr;
};

///	\class ScanHistoryManager
///	\author Luigi Freda 
///	\brief 
///	\note
/// \todo
///	\date
///	\warning
class ScanHistoryManager: private boost::noncopyable
{
public: 

    static const float kCacheTimeInSec; // [sec]
    static const float kMaxScanDistance; // [m]
    static const float kMaxScanSquaredDistance; // [m]

    //typedef pcl::PointXYZINormal Point;
    //typedef pcl::PointCloud<Point> PointCloud;

public:

    ScanHistoryManager(const int& robotId=0);

    void init();

    void insert(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg); 

    uint32_t getCurrentCounter() const { return counter_; }

    void setRobotId(int id);

    void getPoseArrayMessage(geometry_msgs::PoseArray& message);    
    bool getCloudMessage(sensor_msgs::PointCloud2::Ptr &cloud_msg, const uint32_t& index);

    void mapMessageOverlapCheck(const geometry_msgs::PoseArray& message, std::vector<sensor_msgs::PointCloud2::Ptr>& cloudsToSend);

private: 

    void updateHistory();

private: 

    // let's store here the history of pointclouds
    //std::map<PointCloud::Ptr, ScanInfo> map_;
    std::map<pcl::PCLPointCloud2::Ptr, ScanInfo::Ptr> mapCloudPtrToScanInfoPtr_;
    std::map<uint32_t, ScanInfo::Ptr> mapIndexToScanInfoPtr_;    

    std::set<ScanInfo::Ptr> recent_scans_;

private: 

    // interaction mutex: to be locked every time a public method is called (setters, getters and planning)
    boost::recursive_mutex interaction_mutex;

    std::shared_ptr<SpaceTimeFilterBase> p_space_time_filter_; // to spatially filter and select the configuration to store 

    std::shared_ptr<ScanFileManager> p_scan_file_manager_; // to manage history cache: save and release the scan, and reload it at need 

    //! Point cloud counter.
    uint32_t counter_ = 0;

    int robot_id_;    

    bool isInit_ = false; 
};

} // namespace explplanner


#endif //EXPLORATION_PLANNER_H_
