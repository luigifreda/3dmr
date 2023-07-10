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
