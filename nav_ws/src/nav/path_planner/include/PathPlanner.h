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


#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/search/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>  // this solves a strange undefined reference which appear with optimizations 
                                             // from http://www.pcl-users.org/Linking-problem-for-user-defined-point-type-td2414744.html

#include "KdTreeFLANN.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <math.h>
#include <limits>
#include <algorithm>

#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "CostFunction.h"


///	\class PathPlanner
///	\author Luigi Freda (2016-Present) and Alcor (2016)
///	\brief Path planner for a single segment (start point, goal point)
///	\note 
/// 	\todo 
///	\date
///	\warning
class PathPlanner
{
    
public: // static constants 

    static const float kRobotSize;  // robot size  
    
    static const float kGoalPlanningCheckThreshold; // if the distance of a path node to the goal is smaller than this threshold than the node is valid as final 
    static const float kGoalAcceptCheckThreshold;   // if the distance of a the input goal to the traversability cloud is smaller than this threshold than the node is valid 

    static const float kPlanningTimeoutSec; // maximum time in seconds for planning 
    static const float kMinNeighborsRadius; // minimum radius where to search neighbors during each node expansion 
    
    static const float kMaxRobotStep; // maximum radius where to search neighbors during each node expansion 
    static const float kMaxRobotStepDeltaZ; 
    
    const static double kPathSmoothingKernel3[3];
    
public: // typedefs
    
    //typedef pcl::KdTreeFLANN<Point,::flann::L2_Simple<float> > KdTreeFLANN;
    typedef pp::KdTreeFLANN<pcl::PointXYZI,::flann::L2_3D<float> > KdTreeFLANN; // it is necessary to define above PCL_NO_PRECOMPILE

public: // custom structs 
    
    struct PointPlanning
    {
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

    
public: 

    PathPlanner(ros::NodeHandle n_in);
    PathPlanner();
    ~PathPlanner();

    // set the input
    // N.B: first setInput(), then you can set setGoal() and set2DUtility()
    void setInput(pcl::PointCloud<pcl::PointXYZI>& noWall_in, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_in,
                   pp::KdTreeFLANN<pcl::PointXYZRGBNormal>& wallKdTree_in, KdTreeFLANN& noWallKdTree_in, int start_point_idx_in);

    void set2DUtility(pcl::PointCloud<pcl::PointXYZI>& utility_pcl);
        
    //set the goal position
    bool setGoal(pcl::PointXYZI& goal_in);

    //Compute the path if it is exist
    bool planning(nav_msgs::Path& path_out);
    
    // set the abort flag for possibly aborting current planning 
    void setAbort(bool val) { b_abort_ = val; }
    
    void setCostFunction(BaseCostFunction* new_cost, float lamda_trav = 1, float lambda_aux_utility = 1, float tau_exp_decay = std::numeric_limits<float>::max());
    
    void setCostFunctionType(int type, float lamda_trav = 1, float lambda_aux_utility = 1, float tau_exp_decay = std::numeric_limits<float>::max());
        
public: // getters 

    //get the goal position
    pcl::PointXYZI& getGoal(void);    
    
    bool isAbort() const { return b_abort_; }
    
public: // static functions 
    
    //Return the value of the euclidian distane between p1 and p2
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
    
    //The graph
    std::vector<PointPlanning> nodes_;

    //List of leaf node indexes in the explored graph
    std::vector<size_t> leaf_nodes_idxs_;

    //List of visited nodes indexes 
    std::vector<size_t> visited_nodes_idxs_;
    
    std::vector<bool> visited_points_flag_;

    size_t current_node_idx_;

    //Ros node handle
    ros::NodeHandle n_;

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

    //Robot pose and goal
    pcl::PointXYZI goal_;

    int start_point_idx_;

    //Count iteration
    int count_;

    //Get the markerArray of visited node
    visualization_msgs::MarkerArray markerArr_;

    //Publisher for merkerArr
    ros::Publisher markerArrPub_;

    //Publisher for path
    ros::Publisher localPathPub_;
    
    // flag for aborting the planning 
    volatile bool b_abort_;  // set it true if you want to abort current planning
    
    boost::shared_ptr<BaseCostFunction> p_cost_; 

private: // private functions 
    
    // initialize the vars 
    void initVars(); 
    
    // set the topic name of the publishers
    void setPublishers(); 
        
    // Find the best node
    bool findNextNode();

    // Find neighbors to current point and compute probability
    void findNeighbors(std::vector<IdxProbability>& neighbors, double& radius); 

    bool visitedPoint(size_t pointIdx);

    // Sample the followers from the current point p
    void sampleFollowers();

    //Back track when different expansion is chosen
    void backtracking(int parentIdx);

    bool checkGoal();
    
    void computeZMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in);
    void computeIMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in);
    
    void filterNeighborhoodByDz(const pcl::PointCloud<pcl::PointXYZI>& pcl_wall, const pcl::PointXYZI& p, double deltaZ, std::vector<int>& pointIdx, std::vector<float>& pointSquaredDistance);
    
    nav_msgs::Path smoothPath3(const nav_msgs::Path& path);
};

template<class Point>
inline double PathPlanner::dist(const Point& p1, const Point& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

template<class Point1, class Point2>
inline double PathPlanner::dist(const Point1& p1, const Point2& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

template<class Point>
inline double PathPlanner::dist2D(const Point& p1, const Point& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

inline bool PathPlanner::checkGoal()
{
    return (dist(pcl_traversability_[nodes_[current_node_idx_].point_idx], goal_) < kGoalPlanningCheckThreshold);
}

#endif //PATH_PLANNING_H_
