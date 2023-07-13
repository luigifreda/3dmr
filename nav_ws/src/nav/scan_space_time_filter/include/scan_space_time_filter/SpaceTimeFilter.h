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

#ifndef SPACETIME_FILTER_H_
#define SPACETIME_FILTER_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include "SpaceTimeFilterBase.h"


///	\class SpaceTimeFilter
///	\author Luigi Freda and Tiago Novo
///	\brief Space-timer filter for downsampling the scans continuously acquired by the RGBD camera
///	\note
/// \todo
///	\date
///	\warning
class SpaceTimeFilter: private SpaceTimeFilterBase
{
public:
    SpaceTimeFilter(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~SpaceTimeFilter();

public: // setters
    void setRobotId(int id);


private: // private data

    //ROS node handle
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //Publisher for filtered point clouds
    ros::Publisher filteredPointCloudPub_;

    // input point cloud to filter 
    ros::Subscriber pointcloud_sub_;

private:

    int robot_id_;
    std::string str_robot_name_;

private: 

    // initialize the vars
    void init();

    // set the topic name of the publishers
    void setPubsAndSubs();

    void processPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr &pointcloud);

};

#endif //
