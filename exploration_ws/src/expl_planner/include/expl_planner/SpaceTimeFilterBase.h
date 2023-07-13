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

#ifndef SPACETIME_FILTER_BASE_H_
#define SPACETIME_FILTER_BASE_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <math.h>
#include <limits>
#include <algorithm>

#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Dense>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/core/noncopyable.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/search/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp> // this solves a strange undefined reference which appear with optimizations
                                            // from http://www.pcl-users.org/Linking-problem-for-user-defined-point-type-td2414744.html

#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <path_planner/KdTreeFLANN.h>

#include <tf/transform_listener.h>
#include <kindr/minimal/quat-transformation.h>

namespace explplanner
{

// from https://nasa.github.io/astrobee/html/pcl__conversions_8h_source.html
inline std::uint64_t fromRosTimeToUint64(const ros::Time &stamp) 
{
    return stamp.toNSec() / 1000ull;  // Convert from ns to us
}
inline void fromUint64ToRosTime(const std::uint64_t &pcl_stamp, ros::Time &stamp) 
{
   stamp.fromNSec(pcl_stamp * 1000ull);  // Convert from us to ns
}


///	\class SpaceTimeFilterParams
///	\author Luigi Freda 
///	\brief 
///	\note
/// \todo
///	\date
///	\warning
struct SpaceTimeFilterParams
{
    std::string navigationFrame_ = "map";
    double pcl_throttle_         = 1;    // [s]
    double dist_pose_thres_      = 0.2;  // [m]
    double rot_pose_thres_       = 30;   // [degrees]
     
    double tf_cache_length_         = 20;   // [s]
    double pose_cache_time_length_  = 20;   // [s]

    bool   do_downsampling_      = false;      
    double downsample_leaf_size_ = 0.075; // [m]

};

///	\class SpaceTimeFilterBase
///	\author Luigi Freda and Tiago Novo
///	\brief Space-timer filter for downsampling the scans continuously acquired by the RGBD camera
///	\note
/// \todo
///	\date
///	\warning
class SpaceTimeFilterBase : private boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public: // static constants

public: // typedefs

    typedef kindr::minimal::QuatTransformation Transformation;

    //typedef pcl::KdTreeFLANN<Point,::flann::L2_Simple<float> > KdTreeFLANN;
    //typedef pp::KdTreeFLANN<pcl::PointXYZI, ::flann::L2_3D<float>> KdTreeFLANN; // it is necessary to define above PCL_NO_PRECOMPILE
    //typedef pp::KdTreeFLANN<pcl::PointXYZINormal> ScanCloudKdTree;
    //typedef pcl::PointCloud<pcl::PointXYZINormal> PCLPointCloud;

    typedef std::shared_ptr<SpaceTimeFilterBase> Ptr; 

public:
    SpaceTimeFilterBase();
    virtual ~SpaceTimeFilterBase();

    virtual void init();

    bool insertPointCloud(const sensor_msgs::PointCloud2::ConstPtr &pointcloud, const uint32_t& index=0);
    bool filterPointCloud(const sensor_msgs::PointCloud2::ConstPtr &pointcloud, const ros::Time &pcl_stamp, Transformation::Position &q_p_i, Eigen::Vector3d &q_n_i);

public: // setters 

    bool setParams(const std::string& prefix="/scan_filter"); // read from ros workspace 
    void setParams(const SpaceTimeFilterParams& params) { params_ = params; }

public: // getters
    const SpaceTimeFilterParams &getParams() const { return params_; }
    const std::string &GetWorldFrame() const { return params_.navigationFrame_; }

    void getPoseArrayMessage(geometry_msgs::PoseArray& message);

    bool getClosestScanInfo(const pcl::PointNormal& searchPoint, uint32_t& scanIndex, float& scanSquaredDistance);

protected: // private data
    // interaction mutex: to be locked every time a public method is called (setters, getters and planning)
    boost::recursive_mutex interaction_mutex;

    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_pose_;                      
    std::vector<ros::Time> pcl_pose_stamp_;
    int oldest_pose_idx_;

    pp::KdTreeFLANN<pcl::PointNormal, ::flann::L2_3D<float>> kdtree_pose_; // to search closest pose in spatial-time filter

    SpaceTimeFilterParams params_;

protected:
    bool lookupTransform(const std::string &from_frame,
                         const std::string &to_frame, const ros::Time &timestamp,
                         Transformation *transform);
    bool lookupTransformTf(const std::string &from_frame,
                           const std::string &to_frame,
                           const ros::Time &timestamp, Transformation *transform);

    boost::shared_ptr<tf::TransformListener> p_tf_listener_;
};

} // namespace 

#endif //
