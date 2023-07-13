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

#ifndef SCAN_FILE_MANAGER_H_
#define SCAN_FILE_MANAGER_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <math.h>
#include <limits>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <path_planner/KdTreeFLANN.h>
#include <path_planner/CostFunction.h>


namespace explplanner{

///	\class ScanFileManager
///	\author Luigi Freda 
///	\brief 
///	\note
/// \todo
///	\date
///	\warning
class ScanFileManager
{
    static std::string kBaseFolderPath; 

public: 
    typedef pcl::PointXYZINormal Point;
    typedef pcl::PointCloud<Point> PointCloud;

public:

    ScanFileManager(const int& robotId=0);

    bool save(uint32_t index, PointCloud::Ptr& cloud);
    bool save(uint32_t index, const pcl::PCLPointCloud2 & cloud);

    bool read(uint32_t index, PointCloud::Ptr& pclCloud);
    bool read(uint32_t index, pcl::PCLPointCloud2 & cloud);    

    void setRobotId(int id);
    
private:

    std::string getFilePath(uint32_t index);

private: 
    //! Path to the point cloud folder.
    std::string folderPath_;

    //! Point cloud file prefix.
    std::string filePrefix_;

    //! Point cloud file ending.
    std::string fileEnding_;

    int robot_id_;
    std::string robotName_;     
};

} // namespace explplanner


#endif //EXPLORATION_PLANNER_H_
