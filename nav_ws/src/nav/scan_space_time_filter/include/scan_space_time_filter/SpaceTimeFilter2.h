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
