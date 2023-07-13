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

#ifndef EXPLORATION_PLANNER_H_
#define EXPLORATION_PLANNER_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <math.h>
#include <limits>
#include <algorithm>

#include <eigen3/Eigen/StdVector>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/core/noncopyable.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/search/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>  // this solves a strange undefined reference which appear with optimizations 
                                             // from http://www.pcl-users.org/Linking-problem-for-user-defined-point-type-td2414744.html

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

#include <path_planner/KdTreeFLANN.h>
#include <path_planner/CostFunction.h>

#include <octomap_world/octomap_manager.h>
#include <kdtree/kdtree.h>
#include "Tree.h"


#define USE_MAP_SYNC 1

namespace explplanner{

class RrtTree;
class ExplorationTree; 
class FrontierTree;
class NavigationTree; 
class SpaceTimeFilterBase; 
class ScanHistoryManager;

///	\class ExplorationPlanner
///	\author Luigi Freda 
///	\brief Exploration planner 
///	\note 
/// 	\todo 
///	\date
///	\warning
class ExplorationPlanner: private boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
public: // static constants 

    static const float kRobotSizeHorizontal;  // robot size horizontal   
    static const float kRobotSizeVertical;  // robot size vertical
    
    static const float kBiasAcceptCheckThreshold;   // if the distance of a the input bias to the traversability cloud is smaller than this threshold than the node is valid 

    static const float kPlanningTimeoutSec; // maximum time in seconds for planning 
    static const float kMinNeighborsRadius; // minimum radius where to search neighbors during each node expansion 
    
    static const float kMaxRobotStep; // maximum radius where to search neighbors during each node expansion 
    static const float kMaxRobotStepDeltaZ; 
    static const float kMinStepExpansion;  // [m] min step for in the tree expansion 
    static const float kMinStepExpansion2; // squared   kMinStepExpansion  
    
    static const float kExpansionMaxCount; 
    
    static const float kRobotSensorZ; // height of the sensor w.r.t. robot body 
    
    static const float kTraversabilityProbabilitySmootherFactor; 
    
    static const float kBiasPlanningCheckThreshold; // if the distance of a path node to the bias point is smaller than this threshold than the node is valid as final 
    
    
public: // typedefs
    
    //typedef pcl::KdTreeFLANN<Point,::flann::L2_Simple<float> > KdTreeFLANN;
    typedef pp::KdTreeFLANN<pcl::PointXYZI,::flann::L2_3D<float> > KdTreeFLANN; // it is necessary to define above PCL_NO_PRECOMPILE
    typedef pp::KdTreeFLANN<pcl::PointXYZINormal> ScanCloudKdTree;
    typedef pcl::PointCloud<pcl::PointXYZINormal> PCLPointCloud;

public: // custom structs 
    
    struct PointPlanning
    {
        PointPlanning():parent_id(0),id(0),point_idx(0),cost(0.){}
        
        size_t parent_id;
        size_t id;
        int point_idx;
        double cost;
        std::vector<size_t> child_id;
    };
    
    struct IdxProbability
    {
        size_t point_idx;
        float probability;
    };

    enum ExplorationStepType
    {
        kNone             = 0,
        kForwarding,
        kBacktracking,
        kRotating,
        kCompleted,
        kNoInformation,
        kNumExplorationStepTypes
    };
    static const std::string kExplorationStepTypeStrings[]; /// < keep this coherent with ExplorationStepType
    
public: 

    ExplorationPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~ExplorationPlanner();

    //Compute a new target point
    bool planning(nav_msgs::Path& path_out, bool bEnableBacktracking = true);
    
    void insertOtherUgvPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud); 
    
    void mapMessageOverlapCheck(const geometry_msgs::PoseArray& message, std::vector<sensor_msgs::PointCloud2::Ptr>& cloudsToSend);

public: // setters     
    
    // set the input
    // N.B: first setInput(), then you can set setGoal() and set2DUtility()
    void setInput(pcl::PointCloud<pcl::PointXYZI>& noWall_in, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_in,
                   pp::KdTreeFLANN<pcl::PointXYZRGBNormal>& wallKdTree_in, KdTreeFLANN& noWallKdTree_in, int start_point_idx_in);

    void set2DUtility(pcl::PointCloud<pcl::PointXYZI>& utility_pcl);
        
    //set the exploration bias position
    bool setExplorationBias(pcl::PointXYZI& bias_in, bool bUseIt);

    // set the abort flag for possibly aborting current planning 
    void setAbort(bool val) { b_abort_ = val; }
    
    void setCostFunction(BaseCostFunction* new_cost, float lamda_trav = 1, float lambda_aux_utility = 1, float tau_exp_decay = std::numeric_limits<float>::max());
    
    void setCostFunctionType(int type, float lamda_trav = 1, float lambda_aux_utility = 1, float tau_exp_decay = std::numeric_limits<float>::max());
        
    // set absolute robot position (w.r.t. /map frame)
    void setRobotPosition(const double& x, const double& y, const double& z);
    
    void setRobotId(int id);
    
    void setParamExplorationBoxXY(const double minX, const double maxX, const double minY, const double maxY);
    
    void addSelectedNodeToExplorationTree();
    void removeCurrentExplorationNode();      
    void backTrackOnExplorationTree();
    
    void manageConflict();    
    
    void resetSelectedBackTrackingCluster(); 
    
public: // getters 

    //get the goal position
    pcl::PointXYZI& getExplorationBias(void);    
    
    bool isAbort() const { return b_abort_; }
    
    ExplorationStepType getExplorationStepType() const { return explorationStepType_; }
    
    const ExplParams& getParams() const { return params_; }

    void getMapSyncMessage(geometry_msgs::PoseArray& message);
    
public: // other utils        
    
    void publishPlannedExplorationNode();    
    
public: // static functions 
    
    //Return the value of the euclidian distance between p1 and p2
    template<class Point>
    static double dist(const Point& p1, const Point& p2);
    template<class Point1, class Point2>
    static double dist(const Point1& p1, const Point2& p2);
    template<class Point>
    static double dist2D(const Point& p1, const Point& p2);
    
    static int getClosestNodeIdx(pcl::PointXYZI& position, KdTreeFLANN& kdtree, float radius);

private: // private data 
    
    // interaction mutex: to be locked every time a public method is called (setters, getters and planning)
    boost::recursive_mutex interaction_mutex; 
    
    //The search tree
    std::vector<PointPlanning> nodes_;

    //List of leaf node indexes in the explored graph
    std::vector<size_t> leaf_nodes_idxs_;
    
    std::vector<bool> visited_points_flag_;

    size_t current_node_idx_;

    //ROS node handle
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;    

    //List of the point composing the path
    std::vector<size_t> path_;

    //Point Clouds of segmented part: wall and noWall, the intensity is the traversability cost of each point
    pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_wall_;
    //Kd tree representation of the point cloud wall
    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree_wall_;
    
    pcl::PointCloud<pcl::PointXYZI> pcl_traversability_;
    //Kd tree representation of the point cloud noWall
    KdTreeFLANN kdtree_traversability_;
    
    bool b_utility_2d_available_; 
    pcl::PointCloud<pcl::PointXYZI> pcl_utility_2d_; // [x,y,u(x,y),var(x,y)] we assume that pcl_2d_utility_ must contain info about the points in pcl_traversability_

    //exploration point bias 
    bool b_use_expl_bias_;
    pcl::PointXYZI expl_point_bias_;

    int start_point_idx_;

    //Count iteration
    int count_;

    //Get the markerArray of visited node
    visualization_msgs::MarkerArray markerArr_;

    //Publisher for merkerArr
    ros::Publisher markerArrPub_;

    //Publisher for path
    ros::Publisher explPathPub_;
    
    // flag for aborting the planning 
    volatile bool b_abort_;  // set it true if you want to abort current planning
    
    boost::shared_ptr<BaseCostFunction> p_cost_; 

    // input point cloud for the exploration 
    ros::Subscriber pointcloud_sub_;
      
    ExplParams params_;
    std::shared_ptr<volumetric_mapping::OctomapManager> p_octomap_manager_; 
    
    
    std::shared_ptr<RrtTree> p_search_tree_; // this data-structure is used for 
                                             // (i)  locally expanding the search trees for NBV selection 
                                             // (ii) store and reuse the found best branches 
                                             // N.B.: when a conflict occurs we need to back track on its history!
    
    std::shared_ptr<ExplorationTree> p_expl_tree_; // this data-structure is used for organizing/planning view configurations in a tree data structure 
                                                   // and hence guiding forwarding/backtracking steps 
    size_t n_expl_nodes_; 
    
    std::shared_ptr<FrontierTree> p_frontier_tree_; // this data-structure is used for collecting all the frontier nodes, organized in a tree data structure (these can be reused for backtracking)
    
    std::shared_ptr<NavigationTree> p_nav_tree_; // this data-structure is used for reorganizing the frontier tree in the form of a navigation tree rooted at the current robot position 
    
    std::shared_ptr<SpaceTimeFilterBase> p_space_time_filter_; 

    std::shared_ptr<ScanHistoryManager> p_scan_history_manager_; 

    ExplorationStepType explorationStepType_; 
    
private:     
    
    Eigen::Vector3d robot_position_; // (current) absolute robot position w.r.t. (/map frame)
    Eigen::Vector3d robot_bounding_box_size_; // experimental: not used at the moment 
    
    int robot_id_;
    std::string str_robot_name_; 

    // a copy of the last integrated point cloud (rotated in the world frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_integrated_pointcloud_;     

    double last_pcl_stamp_ = std::numeric_limits<double>::lowest();
        
private: // private functions 
    
    // initialize the vars 
    void init(); 
    
    // set the topic name of the publishers
    void setPubsAndSubs(); 
    
    bool setParams();
    
    void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);    

    // Find the best node
    bool findNextNode();

    // Find neighbors to current point and compute probability
    void findNeighbors(std::vector<IdxProbability>& neighbors, double& radius); 

    bool visitedPoint(size_t pointIdx);

    // Sample the followers from the current point p
    void sampleFollowers();

    //Back track when different expansion is chosen
    void backtracking(int parentIdx);
    
    bool checkCloseToBias();

    void computeZMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in);
    void computeIMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in);
    
    void cutPathWithinRange(const std::vector<geometry_msgs::Pose>& path_in, const Eigen::Vector3d& robot_position, const double range, std::vector<geometry_msgs::Pose>& path_out);   
    
    void filterNeighborhoodByDistAndDz(const pcl::PointCloud<pcl::PointXYZI>& pcl_wall, const pcl::PointXYZI& p, double squaredDist, double deltaZ, std::vector<int>& pointIdx, std::vector<float>& pointSquaredDistance);
    
};

template<class Point>
inline double ExplorationPlanner::dist(const Point& p1, const Point& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

template<class Point1, class Point2>
inline double ExplorationPlanner::dist(const Point1& p1, const Point2& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

template<class Point>
inline double ExplorationPlanner::dist2D(const Point& p1, const Point& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

inline bool ExplorationPlanner::checkCloseToBias()
{
    return (dist(pcl_traversability_[nodes_[current_node_idx_].point_idx], expl_point_bias_) < kBiasPlanningCheckThreshold);
}

} // namespace explplanner


#endif //EXPLORATION_PLANNER_H_
