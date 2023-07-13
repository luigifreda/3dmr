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

#ifndef TRAV_ANALIZER_H_
#define TRAV_ANALIZER_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <cmath>
#include <set>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <trajectory_control_msgs/MultiRobotPath.h>
#include <trajectory_control_msgs/MultiRobotPose.h>

#include <TravAnalyzerConfig.h>

#include "ClusterPcl.h"
#include "Transform.h"
#include "KdTreeFLANN.h"
#include "MultiConfig.h"


///	\class TravAnalyzer
///	\author Luigi Freda (2016-Present) and Alcor (2016)
///	\brief A class for implementing traversability analysis 
///	\note 
/// 	\todo Move all the numerical constants into static class parameters 
///	\date
///	\warning
class TravAnalyzer
{
public:
   
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    
    static const float kRobotRadiusDefault;  // [m] robot radius for computing the clearance    
    static const float kClearanceCollisionValue; // value of the clearance when robot the robot is in collision 
    static const float kClearanceDoCareRange; // [m] value of the maximum clearance for which robot cares in computing clearance cost
    static const float kClearanceDoCareRangeSquared;
    static const float kRobotToAvoidRadiusDefault;  // [m] robot radius for computing the clearance   
    static const float kPathToAvoidCropDist; // [m] crop distance for the path of the robot to avoid
    static const float kPathToAvoidCropDist2; // [m^2] squared kPathToAvoidCropDist
    static const float kPathToAvoidResolution; // [m] resolution of the path to avoid
    static const float kPathToAvoidResolution2; // [m^2] squared kPathToAvoidResolution
    static const float kTraversabilityMaxCost; // max cost for a traversability point 
    
    static const float kMinDistRobotPathToConsiderPathToAvoid; // [m] for considering or not a path to avoid 
    static const float kMinDistRobotPathToConsiderPathToAvoidSquared;
    
    static const float kClearanceLambda; // 
    static const float kNeighborhoodDeltaZ; // [m]
    
    static const float kRobotZOffset; // [m] how much each point is pushed higher from the map in the robot direction (TODO: we should use here local normals!) 
    
    static const double kObstPclResetTime; // [s] if elapsed time from last obst pcl check is > kObstPclResetTime then obst_pcl is not used anymore
    static const double kObstDistDiscardIfCloseToNoWall; // [m] distance threshold between (obstacle point, closest noWall point) to discard obstacle point 
    static const double kObstDist2DiscardIfCloseToNoWall; // squared distance     
    
    static const double kTeammatePositionResetTime; // [s]
    static const double kTeammatePathResetTime; // [s]
    
    typedef pcl::PointXYZI PointOutI;
    typedef pcl::PointCloud<PointOutI> PointCloudI;
    typedef pp::KdTreeFLANN<PointOutI> KdTreeInI;
   

public:
    
    TravAnalyzer();
    ~TravAnalyzer();
    
    void setInput(std::vector<int>& clusters_info, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_pcl, pcl::PointCloud<pcl::PointXYZRGBNormal>& input_pcl);

    void computeTrav(PointCloudI& traversabilty_pcl);
        
public: /// < setters     

    inline void setConfig(TravAnalyzerConfig& new_config)
    {
        config_ = new_config;
    }
    
    void initTransformListener()
    {
        p_transform_teammate_.reset(new Transform());
    }

    // enable/disable robot to avoid 
    void setEnableOtherRobotAvoidance(bool val) {b_enable_team_avoidance_ = val; }
    
    void setOtherRobotPathToAvoid(const nav_msgs::Path& path);
    void setRobotToAvoidPosition(const geometry_msgs::TransformStamped& msg);
        
    void setObstPcl(const sensor_msgs::PointCloud2& obst_pcl_msg);
        
    void setNumberOfRobots(const int num) { number_of_robots_ = std::min(num,kMaxNumberOfRobots); } 
    
    void setRobotName(const std::string& name) { str_robot_name_ = name; }
    void setRobotId(const int id) {robot_id_ = id; }
    void setRobotPose(const tf::StampedTransform& robot_pose, bool valid);

    void setRobotRadius(const float radius);
    void setRobotToAvoidRadius(const float radius);    
    
    void setMultiRobotPose(const trajectory_control_msgs::MultiRobotPose& msg);
    void setMultiRobotPath(const trajectory_control_msgs::MultiRobotPath& msg);
    
    void setTeammateBaseFrame(const std::string& frame, int id) { teammate_base_link_frame_[id] = frame; } 

public: /// < getters 
    
    void getPcl(PointCloudI& clearence_pcl, PointCloudI& density_pcl, PointCloudI& label_pcl, PointCloudI& roughnes_pcl);
        
    inline TravAnalyzerConfig getConfig()
    {
        return config_;
    }
    
    nav_msgs::Path& getTeammatePath(int id) 
    {
        boost::recursive_mutex::scoped_lock wall_locker(teammate_path_mutex_[id]); 
        return teammate_path_[id];
    }
    
protected:
    
    template<class Point1, class Point2>
    static double distSquared(const Point1& p1, const Point2& p2);
    
    void downsampleTeamPaths();
    
    //double computeDistSquaredFromPath(const int point_index, const double safety_radius); 
    
    bool updateTeammatePositions(); 
    bool updateTeammatePositionsFromTransform(); 
    
    void checkObstPclUpdate(); 
    
    void buildFutureTrailsPcl(); 
    
protected: // sub-methods for computing traversability contributions 
    
    double computeRough(int point_index, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& nh);
    inline double computeDensity(int point_index, pcl::PointCloud<pcl::PointXYZRGBNormal>& nh);
    inline double computeDensity(int point_index, pcl::PointCloud<pcl::PointXYZRGBNormal>& nh, double pow_config_leaf_size_3, double volume_local_sphere);
    inline double computePlanarDensity(int point_index, pcl::PointCloud<pcl::PointXYZRGBNormal>& nh, double pow_config_leaf_size_3, double volume_local_sphere);
        
    double computeClearance(int point_index);
    double computeLabel(int point_index);
    
    void downSamplePath(const nav_msgs::Path& in, const PointOutI& start_point, nav_msgs::Path& out); 
    void addPathToPcl(const nav_msgs::Path& in, pcl::PointCloud<pcl::PointXYZRGBNormal>& point_cloud);
    
protected: 

    TravAnalyzerConfig config_;
        
    std::vector<int> clusters_info_;
    
    boost::recursive_mutex input_pcl_mutex_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wall_pcl_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr noWall_pcl_;
    
    PointCloudI traversabilty_pcl_, density_pcl_, clearence_pcl_, label_pcl_, roughness_pcl_;

    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> wall_kdtree_;
    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> noWall_kdtree_; 
    
    bool b_ready_; // ready to compute 
    bool b_empty_wall_; 
    
    bool b_enable_team_avoidance_; // true if we must avoid robot teammates
    volatile bool b_updated_teammate_positions_;
    volatile bool b_updated_teammate_position_[kMaxNumberOfRobots];  
    volatile bool b_updated_teammate_position_from_msg_[kMaxNumberOfRobots];
    boost::shared_ptr<Transform> p_transform_teammate_;
    
    boost::recursive_mutex teammate_position_mutex_[kMaxNumberOfRobots];
    PointOutI teammate_position_[kMaxNumberOfRobots];
    ros::Time teammate_pos_last_time_[kMaxNumberOfRobots]; // time the last position msg was received 
    
    boost::recursive_mutex teammate_path_mutex_[kMaxNumberOfRobots];
    nav_msgs::Path teammate_path_[kMaxNumberOfRobots]; 
    bool b_teammate_path_downsampled_[kMaxNumberOfRobots];
    ros::Time teammate_path_last_time_[kMaxNumberOfRobots]; // time the last path msg was received 
    
    boost::recursive_mutex obst_pcl_mutex_;
    volatile bool b_set_ost_pcl_;  
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr obst_pcl_;
    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> obst_pcl_kdtree_;
    ros::Time obst_pcl_last_time_; // time the last laser proximity msg was received 
    double obst_plc_radius_;
    Eigen::Vector4f obst_plc_centroid_;
    
    boost::recursive_mutex team_future_trails_pcl_mutex_;
    volatile bool b_set_team_future_trails_pcl_;  
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr future_trails_pcl_;
    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> future_trails_pcl_kdtree_;
    
    std::string str_robot_name_;
    int robot_id_;
    int number_of_robots_; 
    
    std::string map_frame_; 
    std::string teammate_base_link_frame_[kMaxNumberOfRobots]; 
    
    tf::StampedTransform robot_pose_;
    bool is_valid_robot_pose_;

    float robot_radius_; //  [m] robot radius for computing the (self) clearance 
    float robot_radius_squared_; 

    float robot_to_avoid_radius_; // [m] other robot radius for computing the clearance (multi-robot traversability)  
    float robot_to_avoid_radius_squared_;     
};

template<class Point1, class Point2>
inline double TravAnalyzer::distSquared(const Point1& p1, const Point2& p2)
{
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
}

inline double point_plane_distance(const pcl::PointXYZRGBNormal& p, const pcl::ModelCoefficients& coefficients)
{
    double num = abs(p.x * coefficients.values[0] + p.y * coefficients.values[1] + p.z * coefficients.values[2] + coefficients.values[3]);
    double den = sqrt(pow(coefficients.values[0], 2) + pow(coefficients.values[1], 2) + pow(coefficients.values[2], 2));
    return num / den;
}

inline double point_point_distance(const pcl::PointXYZRGBNormal& p1, const pcl::PointXYZRGBNormal& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

#endif // TRAV_ANAL_H_
