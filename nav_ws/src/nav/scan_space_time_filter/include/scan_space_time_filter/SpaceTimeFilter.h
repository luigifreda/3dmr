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
