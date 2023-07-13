/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Alcor Lab (La Sapienza University)
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

#ifndef NORMALESTIMATIONPCL_H
#define NORMALESTIMATIONPCL_H

#include <ros/ros.h>
#include <ros/console.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <NormalEstimationPclConfig.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <nav_msgs/Path.h>
#include <tf/tf.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <deque>
#include <atomic>

#include "KdTreeFLANN.h"
#include "Transform.h"
#include "MultiConfig.h"


using namespace path_planner;

typedef std::atomic<bool> atom_bool;
typedef std::vector<atom_bool> atom_bool_vec; 

///	\class NormalEstimationPcl
///	\author Luigi Freda and Alcor Lab
///	\brief Class for computing normals 
///	\note 
/// 	\todo 
///	\date
///	\warning
template<typename PointT>
class NormalEstimationPcl
{

    static const float kLaserZOffset; // [m] how much each laser view point is pushed higher
    static const float kLaserZWrtBody; // [m] actual delta_z between laser view point and base_link
    static const int kMinNumNeighboursForComputingNormal; // minimum number of neighbours for computing the normal 
    static const int kMaxNumNeighboursForComputingNormal; // minimum number of neighbours for computing the normal 

    static const float kMinPercentDoneWithDirectPropagation; 

    static const int kNumInitialClosePointsForStartingPropagation;     
    
    static const float kTrajDownsampleDistance;
    static const float kTrajDownsampleDistance2;
    
    static const float kMinCosToPropagate; 
    
public:     
    
    typedef pcl::PointCloud<PointT> PointCloudT;

    typedef pp::KdTreeFLANN<PointT> KdTreeT;
    
public:
    
    NormalEstimationPcl();
    ~NormalEstimationPcl();
    
    void computeNormals(PointCloudT& pcl, KdTreeT& kdtree, const pcl::PointXYZ& center);

    void computeNormalsStandard(PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center);  
    void computeNormalsByDirectionPropagation(PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center);    

public: ///  < setters     
    
    inline void setConfig(const NormalEstimationPclConfig& new_config)
    {
        config_ = new_config;
    }

    // Set previously built map base_link trajectory 
    void setPrevMapTraj(const nav_msgs::Path& base_link_traj);
    
    // Set laser trajectory 
    void setLaserTraj(const nav_msgs::Path& base_link_traj);
    
    void setRobotId(const int id) {robot_id_ = id; }
    void setNumberOfRobots(const int num) { number_of_robots_ = std::min(num,kMaxNumberOfRobots); }     
        
    void setTeammateBaseFrame(const std::string& frame, int id) { teammate_base_link_frame_[id] = frame; } 
    
    void initTransformListener()
    {
        p_transform_teammate_.reset(new Transform());
    }
    
public:  /// < getters 
    
    inline NormalEstimationPclConfig& getConfig()
    {
        return config_;
    }
    
    
protected:
    
    // Kernel between two vectors
    inline float kernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
    inline float gaussianKernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
    inline float cosineKernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);


    // Compute the covariance matrix between the point center and his neighbors. The covariance matrix is weighted with kernel function
    void computeCovarianceMatrix(const PointCloudT& neighbors, const pcl::PointXYZ& center, Eigen::Matrix3f& covariance_matrix);

    // Compute one normal for a given point i
    bool computeNormal(const size_t i, const size_t begin, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center);

    // Compute normals in a given range
    void computeNormalsInRange(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center);

    void computeNormalsInQueue(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center);
    
    // Add a new laser center to the laser trajectory 
    void addPointToLaserTraj(const pcl::PointXYZ& center, const int robot_id = 0); 
    
    // Compute closest laser point to each point of the cloud 
    void computeClosestLaserPoints(PointCloudT& pcl); 
        
    void checkTeammatePositionsFromTransform();

protected:
    
    NormalEstimationPclConfig config_;
    
    // Minimum value of the weight in the computation of the weighted covariance matrix
    const float threshold_;
    
    boost::recursive_mutex pcl_laser_traj_mutex_;
    typename PointCloudT::Ptr pcl_laser_trajectory_;     // laser trajectory
    PointCloudT pcl_prev_map_trajectory_;  // previously built map trajectory
    KdTreeT kdtree_laser_centers_;
    
    std::vector<PointT> last_laser_center_;
    
    std::vector<size_t> closest_laser_point_idx_; 
    
    int robot_id_;
    int number_of_robots_;     
        
    std::string map_frame_; 
    std::string teammate_base_link_frame_[kMaxNumberOfRobots]; 
    
    boost::shared_ptr<Transform> p_transform_teammate_;    
    
    boost::recursive_mutex kdtree_mutex_;    
    
    std::deque<size_t> queue_; 
    boost::recursive_mutex queue_mutex_;      
    
protected:
    
    template<class Point1, class Point2>
    double distSquared(const Point1& p1, const Point2& p2)
    {
        return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
    }

};

#endif //NORMALESTIMATIONPCL_H
