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

#include "PathPlanner.h"
#include "TravAnalyzer.h"

#include <limits>       // std::numeric_limits

//#define VERBOSE  // entry level of verbosity 
//#define VERBOSE2 // additional level of verbosity (the entry level is independent!)

#define NO_OLD_VISITED_STRUCT

#define ENABLE_TRAVERSABILITY_BIAS_IN_RANDOM_SAMPLE_GENERATOR 0

// this soft function starts from 0.9 for radius=0m and then approximatively vanishes near radius=3m
inline double weight(double radius)
{
    return (0.8 / (1 + exp((radius - 1) / 0.5)) + 0.2);
}

const float PathPlanner::kRobotSize = 0.5; // robot size  
const float PathPlanner::kGoalPlanningCheckThreshold = 0.1; // if the distance of a path node to the goal is smaller than this threshold than the node is valid as final 
const float PathPlanner::kGoalAcceptCheckThreshold = PathPlanner::kRobotSize; // if the distance of a the input goal to the traversability cloud is smaller than this threshold than the node is valid 
const float PathPlanner::kPlanningTimeoutSec = 100; // maximum time in seconds for planning 
const float PathPlanner::kMinNeighborsRadius = 0.1; // minimum radius where to search neighbors during each node expansion 

const float PathPlanner::kMaxRobotStep = 0.4; // maximum radius where to search neighbors during each node expansion 
const float PathPlanner::kMaxRobotStepDeltaZ = 0.2; // 0.2

const double PathPlanner::kPathSmoothingKernel3[3] = {0.3, 0.4, 0.3};

PathPlanner::PathPlanner(ros::NodeHandle n_in)
{
    initVars();

    //markerArrPub_ = n_.advertise<visualization_msgs::MarkerArray>("PathPlanner/visitedNodes", 1);
    //pathPub_ = n_.advertise<nav_msgs::Path>("PathPlanner/localPath", 1);
    setPublishers();
}

PathPlanner::PathPlanner()
{
    initVars();

    //markerArrPub_ = n_.advertise<visualization_msgs::MarkerArray>("PathPlanner/visitedNodes", 1);
    //pathPub_ = n_.advertise<nav_msgs::Path>("PathPlanner/localPath", 1);
    setPublishers();
}

// initialize the vars 

void PathPlanner::initVars()
{
    count_ = 0;
    n_ = ros::NodeHandle("~");

    b_abort_ = false;
    b_utility_2d_available_ = false;

    /// < set the default cost function 
    p_cost_.reset(new SimpleCostFunction());
}

/// < DESTRUCTOR

PathPlanner::~PathPlanner()
{
    std::cout << "PathPlanner::~PathPlanner() - start " << std::endl;
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    //count_ = 0;
    //n = n_;
    //markerArrPub_ = n_.advertise<visualization_msgs::MarkerArray>("PathPlanner/visitedNodes", 1);
    //pathPub_ = n_.advertise<nav_msgs::Path>("PathPlanner/localPath", 1);

    // publish empty stuff to reset rviz
    //nav_msgs::Path local_path;
    //localPathPub_.publish(local_path);

    //markerArr_.markers.clear(); 
    //markerArrPub_.publish(markerArr_); 
    std::cout << "PathPlanner::~PathPlanner() - end " << std::endl;
}

void PathPlanner::setPublishers()
{
    markerArrPub_ = n_.advertise<visualization_msgs::MarkerArray>("/path_planner/visited_nodes", 1);
    localPathPub_ = n_.advertise<nav_msgs::Path>("/path_planner/localPath", 1);
}

bool sortProbFunction(const PathPlanner::IdxProbability& i, const PathPlanner::IdxProbability& j)
{
    return (i.probability < j.probability);
}

void PathPlanner::filterNeighborhoodByDz(const pcl::PointCloud<pcl::PointXYZI>& pcl_trav, const pcl::PointXYZI& p, const double deltaZ, std::vector<int>& pointIdx, std::vector<float>& pointSquaredDistance)
{
    std::vector<float>::iterator it_dist = pointSquaredDistance.begin();
    std::vector<int>::iterator it_id = pointIdx.begin();

    while ((it_dist != pointSquaredDistance.end()) && (it_id != pointIdx.end()))
    {
        const pcl::PointXYZI& p_it = pcl_trav[*it_id];
        if (fabs(p_it.z - p.z) > deltaZ)
        {
            it_dist = pointSquaredDistance.erase(it_dist);
            it_id = pointIdx.erase(it_id);
        }
        else
        {
            ++it_dist;
            ++it_id;
        }
    }
}

//Create the neighborhood for the current node and the probability associate with each neighbor
void PathPlanner::findNeighbors(std::vector<IdxProbability>& neighbors, double& radius)
{
    /// < Find the nearest point labeled as wall and set the search radius according to this distance
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1, std::numeric_limits<float>::max());
    const pcl::PointXYZI& point_noWall = pcl_traversability_[nodes_[current_node_idx_].point_idx];
    pcl::PointXYZRGBNormal point_noWall_RGBN;
    point_noWall_RGBN.x = point_noWall.x;
    point_noWall_RGBN.y = point_noWall.y;
    point_noWall_RGBN.z = point_noWall.z;

    /// < NOTE: This function returns squared distances
    int num_wall_neighbors = kdtree_wall_.nearestKSearch(point_noWall_RGBN, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    if (num_wall_neighbors < 1)
    {
        ROS_WARN("could not find wall neighbors");
    }
    /*double checkradius = sqrt(pointNKNSquaredDistance[0]);
    if( fabs(checkradius-dist(point_wall,pcl_wall_.points[pointIdxNKNSearch[0]])) > 1e-6) 
    {
        std::cout << "PathPlanner::findNeighbors() - error - radius: " << checkradius << ", dist: " << dist(point_wall,pcl_wall_.points[pointIdxNKNSearch[0]]) << std::endl;
        quick_exit(-1);
    }*/

    //radius = sqrt(pointNKNSquaredDistance[0]);
    //radius = std::max(sqrt(pointNKNSquaredDistance[0]) - robot_radius_, kMinNeighborsRadius);
    //radius = std::min(std::max(sqrt(pointNKNSquaredDistance[0]) - robot_radius_, kMinNeighborsRadius), kMaxRobotStep);

    // fix the maximum step of the robot to the set related parameter 
    radius = std::min(sqrt(pointNKNSquaredDistance[0]), kMaxRobotStep);


#ifdef VERBOSE2    
    ROS_INFO("PathPlanner::findNeighbors() - radius: %f", radius);
    std::cout << "PathPlanner::find_neighbors() - current point : " << pcl_traversability_[nodes_[current_node_idx_].point_idx] << ", radius " << radius << std::endl;
#endif

    /// < Find traversability neighbors of the current node within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int num_close_points = kdtree_traversability_.radiusSearch(point_noWall, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    // eliminate points which have Dz w.r.t. point_noWall higher than kMaxRobotStepDeltaZ
    filterNeighborhoodByDz(pcl_traversability_, point_noWall, kMaxRobotStepDeltaZ, pointIdxRadiusSearch, pointRadiusSquaredDistance);

#ifdef VERBOSE2     
    std::cout << "PathPlanner::find_neighbors() - num_close_points : " << num_close_points << std::endl;
#endif
    if (num_close_points < 1)
    {
        ROS_WARN("could not find point close to current node");
    }



    /// < Extract the traversability neighbors and compute probability associated with each point
    /// < Point with low cost will have higher probability	  
    float normalization_factor = 0;
    for (size_t i = 0, iEnd = pointIdxRadiusSearch.size(); i < iEnd; i++)
    {
        if (dist(pcl_traversability_[pointIdxRadiusSearch[i]], goal_) < 0.1 * kGoalPlanningCheckThreshold) // we want to be more precise in the expansion
        {
#ifdef VERBOSE
            std::cout << "PathPlanner::find_neighbors() - found point close to goal" << std::endl;
#endif            
            // set this point as the unique point in the neighborhood 
            neighbors.clear();
            IdxProbability point;
            point.point_idx = pointIdxRadiusSearch[i];
            point.probability = 1;
            neighbors.push_back(point);
            normalization_factor = 1;
            break;
        }
        else if (pcl_traversability_[pointIdxRadiusSearch[i]].intensity < std::numeric_limits<double>::infinity())
        {
#if ENABLE_TRAVERSABILITY_BIAS_IN_RANDOM_SAMPLE_GENERATOR            
            float prob = 1 / pcl_traversability_[pointIdxRadiusSearch[i]].intensity;
#else
            float prob = 1.f;
#endif
            normalization_factor += prob;
            IdxProbability point;
            point.point_idx = pointIdxRadiusSearch[i];
            point.probability = prob;
            neighbors.push_back(point);
        }

    }
    //ROS_INFO("neighbors size: %d", neighbors.size());

#ifdef VERBOSE2 
    std::cout << "PathPlanner::find_neighbors() - neighbors size : " << neighbors.size() << std::endl;
#endif

    if (neighbors.size() > 0)
    {

        //Normalize the probability of the neighbors 
        for (size_t i = 0; i < neighbors.size(); i++)
        {
            neighbors[i].probability /= normalization_factor;
        }

        //Sort the neighbors by probability
        std::sort(neighbors.begin(), neighbors.end(), sortProbFunction);


        //Compute cumulative probability
        for (int i = 1; i < neighbors.size() - 1; i++)
        {
            neighbors[i].probability += neighbors[i - 1].probability;
        }
        neighbors.back().probability = 1;
    }

}

bool PathPlanner::visitedPoint(size_t pointIdx)
{
    //    for (std::vector<size_t>::iterator it = visited_nodes_idxs_.begin(); it != visited_nodes_idxs_.end(); ++it)
    //    {
    //        if (nodes_[*it].point_idx == pointIdx) 
    //            return true;
    //    }
    //    return false;
    return visited_points_flag_[pointIdx];
}

//Sample the followers from the current point p
void PathPlanner::sampleFollowers()
{
    //Build the neighborhood of current node and set the probability of each point on the basis of the traversability 
    std::vector<IdxProbability> neighbors;
    double radius;
    findNeighbors(neighbors, radius);

    //Sample the child from the neighbors if the neighbors are more then a threshold, else use all the neighbors
    int num_generated_followers = 0;
    int num_random_samples = 0;
    //double w = weight(radius);
    double w = std::min(weight(radius), 1.d);
    int weighted_neighbors_size = lrint(w * neighbors.size());
    
    bool b_publish_markers = markerArrPub_.getNumSubscribers()>0;
    
    while ((num_generated_followers < weighted_neighbors_size) && (num_random_samples < neighbors.size()))
    {
        // Generate a random sample by using traversability-based probability 
        float r = ((double) rand() / (RAND_MAX));
        int i = 0;
        while (neighbors[i].probability < r)
        {
            i++;
        }
        num_random_samples++;

        if (!visitedPoint(neighbors[i].point_idx))
        {
            // Create new child
            PointPlanning child;

            child.id = nodes_.size();


            //double heuristic = dist(pcl_traversability_.points[neighbors[i].point_idx], goal_); // A* heuristic             
            //child.cost = dist(pcl_traversability_[nodes_[current_node_idx_].point_idx], pcl_traversability_[neighbors[i].point_idx])
            //        + pcl_traversability_[neighbors[i].point_idx].intensity + heuristic;

            double aux_2d_cost = 0.;
            double aux_2d_confidence = 1.;
            if (!pcl_utility_2d_.empty())
            {
                aux_2d_cost = pcl_utility_2d_[neighbors[i].point_idx].z;
                aux_2d_confidence = pcl_utility_2d_[neighbors[i].point_idx].intensity;
            }
            //double cost(const pcl::PointXYZI& current, const pcl::PointXYZI& next, const pcl::PointXYZI& goal, double traversability, double aux_utility = 0, double aux_conf = 0)    
            child.cost = p_cost_->cost(pcl_traversability_[nodes_[current_node_idx_].point_idx], pcl_traversability_[neighbors[i].point_idx], goal_, pcl_traversability_[neighbors[i].point_idx].intensity, aux_2d_cost, aux_2d_confidence);

            child.parent_id = current_node_idx_;
            child.point_idx = neighbors[i].point_idx;
            nodes_[current_node_idx_].child_id.push_back(child.id);
            nodes_.push_back(child);

            leaf_nodes_idxs_.push_back(child.id);
#ifndef NO_OLD_VISITED_STRUCT            
            visited_nodes_idxs_.push_back(child.id);
#endif
            visited_points_flag_[child.point_idx] = true;

            if(b_publish_markers)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.pose.position.x = pcl_traversability_[neighbors[i].point_idx].x;
                marker.pose.position.y = pcl_traversability_[neighbors[i].point_idx].y;
                marker.pose.position.z = pcl_traversability_[neighbors[i].point_idx].z;
                marker.lifetime = ros::Duration(5);
                marker.id = markerArr_.markers.size() + 1;
                markerArr_.markers.push_back(marker);
            }
            
            num_generated_followers++;
            
#ifdef VERBOSE2            
            std::cout << num_generated_followers << std::endl;
#endif
        }
    }

#ifdef VERBOSE2 
    std::cout << "PathPlanner::sample_followers() - num_generated_followers : " << num_generated_followers << std::endl;
#endif

    // Remove current node from leaf nodes structure
    for (std::vector<size_t>::iterator it = leaf_nodes_idxs_.begin(), itEnd = leaf_nodes_idxs_.end(); it != itEnd; ++it)
    {
        if (*it == current_node_idx_)
        {
            std::swap(*it, leaf_nodes_idxs_.back());
            leaf_nodes_idxs_.pop_back(); // no problem with invalidation of begin()-end() since we exit from the loop!
            break; // exit from loop!
        }
    }

#ifdef VERBOSE2  
    ROS_INFO("num visited nodes: %ld", markerArr_.markers.size());
#ifndef NO_OLD_VISITED_STRUCT 
    ROS_INFO("num leaf nodes: %ld", visited_nodes_idxs_.size());
#endif
#endif

    if(b_publish_markers) markerArrPub_.publish(markerArr_);

    //Add the current point to the path
}

//Find the best node from all the leaf node
bool PathPlanner::findNextNode()
{
    //double minCost = INFINITY;
    double minCost = std::numeric_limits<double>::max();

    size_t minCostNodeIdx = 0;
    bool b_found = false;

    //Find the best expansion within the leaf nodes.
    for (std::vector<size_t>::iterator it = leaf_nodes_idxs_.begin(); it != leaf_nodes_idxs_.end(); ++it)
    {
        if (nodes_[*it].cost < minCost)
        {
            minCost = nodes_[*it].cost;
            minCostNodeIdx = *it;
            b_found = true;
        }
    }

    if (b_found)
    {
        current_node_idx_ = minCostNodeIdx;
    }
    return b_found;
}

void PathPlanner::setInput(pcl::PointCloud<pcl::PointXYZI>& traversability_pcl_in, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_pcl_in,
                           pp::KdTreeFLANN<pcl::PointXYZRGBNormal>& wall_kdtree_in, KdTreeFLANN& traversability_kdtree_in, int start_point_idx_in)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    b_utility_2d_available_ = false; // reset the utility function, we are required to add new information synched with traversability cloud 

    std::cout << "PathPlanner::setInput() - traversability pcl size: " << traversability_pcl_in.size() << std::endl;

    pcl_traversability_ = traversability_pcl_in;

    computeIMinMaxRange(pcl_traversability_, p_cost_->GetMinTrav(), p_cost_->GetMaxTrav(), p_cost_->GetRangeTrav());

    pcl_wall_ = wall_pcl_in;

    kdtree_wall_ = wall_kdtree_in;
    kdtree_traversability_ = traversability_kdtree_in;

    start_point_idx_ = start_point_idx_in;

    // clear the nodes and set the actual robot position as start 
    nodes_.clear();
    PointPlanning node;
    node.id = nodes_.size();
    node.cost = 0;
    node.parent_id = 0; //node.id;
    node.point_idx = start_point_idx_;
    current_node_idx_ = node.id;
    nodes_.push_back(node);

#ifndef NO_OLD_VISITED_STRUCT 
    // set the starting node as visited 
    visited_nodes_idxs_.push_back(node.id);
#endif

    // reset visited points 
    visited_points_flag_ = std::vector<bool>(pcl_traversability_.size(), false);
    // set the starting node as visited 
    visited_points_flag_[start_point_idx_] = true;
}

void PathPlanner::set2DUtility(pcl::PointCloud<pcl::PointXYZI>& utility_pcl)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    std::cout << "PathPlanner::set2DUtility() - utility  pcl size: " << utility_pcl.size() << std::endl;

    pcl_utility_2d_ = utility_pcl;

    /// < check if it has the same size of the traversability pcl 
    if ((pcl_utility_2d_.header.stamp != pcl_traversability_.header.stamp) || (pcl_utility_2d_.size() != pcl_traversability_.size()))
    {
        ROS_WARN("**********************************************************************************************************");
        ROS_WARN("PathPlanner::set2DUtility() - traversability and utility pcls have different sizes or stamp");
        ROS_WARN("**********************************************************************************************************");
        std::cout << "trav stamp: " << pcl_traversability_.header.stamp << ", utility stamp: " << pcl_utility_2d_.header.stamp << std::endl;
        std::cout << "trav size: " << pcl_traversability_.size() << ", utility size: " << pcl_utility_2d_.size() << std::endl;
        b_utility_2d_available_ = false;
        return; /// < EXIT POINT 
    }

    if (!pcl_utility_2d_.empty())
    {
        b_utility_2d_available_ = true;
        computeZMinMaxRange(utility_pcl, p_cost_->GetMinAuxUtility(), p_cost_->GetMaxAuxUtility(), p_cost_->GetRangeAuxUtility());
    }
}

void PathPlanner::setCostFunction(BaseCostFunction* new_cost, float lamda_trav, float lambda_aux_utility, float tau_exp_decay)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    p_cost_.reset(new_cost);
    p_cost_->SetLambdaTrav(lamda_trav);
    p_cost_->SetLambdaAuxUtility(lambda_aux_utility);
    p_cost_->SetTauExpDecay(tau_exp_decay);
}

void PathPlanner::setCostFunctionType(int type, float lamda_trav, float lambda_aux_utility, float tau_exp_decay)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    type = std::max(std::min(type, (int) BaseCostFunction::kNumCostFunctions - 1), 0);

    switch (type)
    {
    case BaseCostFunction::kTraversabilityCost:
        std::cout << "PathPlanner::setCostFunctionType()- setting TraversabilityCostFunction()" << std::endl;
        setCostFunction(new TraversabilityCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        break;
    case BaseCostFunction::kTraversabilityProdCost:
        setCostFunction(new TraversabilityProdCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        std::cout << "PathPlanner::setCostFunctionType()- setting TraversabilityProdCostFunction()" << std::endl;
        break;
    case BaseCostFunction::kSimpleCost:
        setCostFunction(new SimpleCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        std::cout << "PathPlanner::setCostFunctionType()- setting OriginalCostFunction()" << std::endl;
        break;
    case BaseCostFunction::kBaseCost:
    default:
        setCostFunction(new BaseCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        std::cout << "PathPlanner::setCostFunctionType()- setting BaseCostFunction()" << std::endl;
    }
}

bool PathPlanner::setGoal(pcl::PointXYZI& goal_in)
{
    bool res = true;

    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    if (pcl_traversability_.empty())
    {
        ROS_WARN("PathPlanner::set_goal() - cannot  set goal if traversability kdtree is empty");
        return false;
    }

    leaf_nodes_idxs_.clear();

#ifndef NO_OLD_VISITED_STRUCT 
    //visited_nodes_idxs_.clear();
    visited_nodes_idxs_.resize(1); // leave the starting node as visited as we set in the function set_input()
#endif 

    // reset visited points 
    visited_points_flag_ = std::vector<bool>(pcl_traversability_.size(), false);
    // set the starting node as visited 
    visited_points_flag_[start_point_idx_] = true;

    //nodes_.erase(nodes_.begin() + 1, nodes_.end());
    nodes_.resize(1); // leave the starting node we set with the function set_input()

    // Find the goal nearest point in noWall point cloud
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    //int found = kdtree_traversability_.nearestKSearch(goal_in, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    int found = kdtree_traversability_.radiusSearch(goal_in, kGoalAcceptCheckThreshold, pointIdxNKNSearch, pointNKNSquaredDistance);

    if (found < 1)
    {
        ROS_WARN("PathPlanner::setGoal() - cannot find a close node ");
        res = false;
    }
    {
        goal_ = pcl_traversability_[pointIdxNKNSearch[0]];
    }

    for (size_t i = 0, iEnd = markerArr_.markers.size(); i < iEnd; i++)
    {
        markerArr_.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    return res;
}

pcl::PointXYZI& PathPlanner::getGoal()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    return goal_;
}

bool PathPlanner::planning(nav_msgs::Path& path_out)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

#ifdef VERBOSE     
    std::cout << "PathPlanner::planning() - start position: " << pcl_traversability_[start_point_idx_] << std::endl;
    std::cout << "PathPlanner::planning() - goal position: " << goal_ << std::endl;
#endif

    b_abort_ = false;

    if (pcl_traversability_.empty())
    {
        ROS_WARN("PathPlanner::planning() - cannot plan if traversability kdtree is empty");
        return false;
    }

    std::cout << "PathPlanner::planning() - using cost function: " << p_cost_->getName() << std::endl;

    path_out.poses.clear();
    path_out.header.frame_id = "map";

    leaf_nodes_idxs_.clear();

#ifndef NO_OLD_VISITED_STRUCT 
    //visited_nodes_idxs_.clear();
    visited_nodes_idxs_.resize(1); // leave the starting node as visited as we set in the function set_input()
#endif

    // reset visited points 
    visited_points_flag_ = std::vector<bool>(pcl_traversability_.size(), false);
    // set the starting node as visited 
    visited_points_flag_[start_point_idx_] = true;

    //nodes_.erase(nodes_.begin() + 1, nodes_.end());
    nodes_.resize(1); // leave the starting node we set with the function set_input()

    nav_msgs::Path local_path;
    local_path.header.frame_id = "map";

    markerArr_.markers.clear();

    bool is_found_path = false;
    bool is_exist_path = true;
    bool is_timeout = false;

    ros::Time time_start = ros::Time::now();
    count_ = 0;

    p_cost_->initTime();

    while (!is_found_path && is_exist_path && !is_timeout && !b_abort_)
    {
        count_++;
        sampleFollowers();
        is_exist_path = findNextNode();
        is_found_path = false;
        if (is_exist_path)
        {
            is_found_path = checkGoal();
        }

        ros::Duration elapsed_time = ros::Time::now() - time_start;
        if (elapsed_time.toSec() > kPlanningTimeoutSec)
        {
            ROS_WARN("PathPlanner::planning() - timeout **********************");
            is_timeout = true;
        }
    }

#ifdef VERBOSE
    std::cout << "PathPlanner::planning() - found path: " << is_found_path << ", exist path: " << is_exist_path << std::endl;
    if (b_abort_) std::cout << "PathPlanner::planning() - planning  aborted" << std::endl;
#endif

    // build path:
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = goal_.x;
    pose.pose.position.y = goal_.y;
    pose.pose.position.z = goal_.z;
    local_path.poses.push_back(pose);
    size_t node_idx = current_node_idx_;

    bool bDone = false;
    do
    {

#ifdef VERBOSE2
        std::cout << "parent node id: " << node_idx << std::endl;
#endif        
        const pcl::PointXYZI& point = pcl_traversability_[nodes_[node_idx].point_idx];
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = point.z;
        local_path.poses.push_back(pose);
        if (node_idx == nodes_[node_idx].parent_id) // we got the root!
            bDone = true;
        node_idx = nodes_[node_idx].parent_id;

    }
    while (!bDone);

    if (is_found_path)
    {
        // reverse the path  
        for (int i = local_path.poses.size(); i > 0; i--)
        {
            path_out.poses.push_back(local_path.poses[i - 1]);
        }

        // smooth the path 
        path_out = smoothPath3(path_out);

        //ROS_INFO("PathPlanner::planning() - local path length: %ld ", local_path.poses.size());
        ROS_INFO("path length: %ld", path_out.poses.size());
    }
    else
    {
        ROS_INFO("PathPlanner::planning() - attempted local path length: %ld ", local_path.poses.size());
        ROS_WARN("PathPlanner::planning() - NO PATH");
    }


    //localPathPub_.publish(path_in);
    localPathPub_.publish(local_path);
    //return is_exist_path;
    return is_found_path;
}

/// static functions 

int PathPlanner::getClosestNodeIdx(pcl::PointXYZI& position, KdTreeFLANN& kdtree, float radius)
{
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    // k-nearest neighborhood 
    int found = kdtree.nearestKSearch(position, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

    // radius search 
    //int found = kdtree.radiusSearch(position, radius, pointIdxNKNSearch, pointNKNSquaredDistance);

    if (found < 1)
    {
        ROS_WARN("PathPlanner::get_closest_node_index() - cannot find a close node ");
        return -1;
    }
    else
    {
        ROS_INFO("distance to starting node %f", sqrt(pointNKNSquaredDistance[0]));
    }

    return pointIdxNKNSearch[0];
}

bool pointXYZIcompare(int i, int j)
{
    return i<j;
}

bool compareZ(const pcl::PointXYZI& a, const pcl::PointXYZI& b)
{
    return a.z < b.z;
}

void PathPlanner::computeZMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in)
{
    if (pcl_in.empty())
    {
        ROS_ERROR("PathPlanner::computeZMinMaxRange() - empty pcl as input");
        return; /// < EXIT POINT 
    }

    pcl::PointCloud<pcl::PointXYZI>::const_iterator min_elem_it = std::min_element(pcl_in.begin(), pcl_in.end(), compareZ);
    pcl::PointCloud<pcl::PointXYZI>::const_iterator max_elem_it = std::max_element(pcl_in.begin(), pcl_in.end(), compareZ);

    min_in = (*min_elem_it).z;
    max_in = (*max_elem_it).z;
    range_in = max_in - min_in;

    std::cout << "PathPlanner::computeZMinMaxRange() - min: " << min_in << ", max: " << max_in << std::endl;
}

bool compareI(const pcl::PointXYZI& a, const pcl::PointXYZI& b)
{
    return a.intensity < b.intensity;
}

void PathPlanner::computeIMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in)
{
    pcl::PointCloud<pcl::PointXYZI>::const_iterator min_elem_it = std::min_element(pcl_in.begin(), pcl_in.end(), compareI);
    pcl::PointCloud<pcl::PointXYZI>::const_iterator max_elem_it = std::max_element(pcl_in.begin(), pcl_in.end(), compareI);

    min_in = (*min_elem_it).intensity;
    max_in = (*max_elem_it).intensity;
    range_in = max_in - min_in;


    std::cout << "PathPlanner::computeIMinMaxRange() - min: " << min_in << ", max: " << max_in << std::endl;
}

nav_msgs::Path PathPlanner::smoothPath3(const nav_msgs::Path& path)
{
    nav_msgs::Path path_out = path;  
    for (int i = 1; i < path.poses.size() - 1; i++)
    {
        path_out.poses[i].pose.position.x = kPathSmoothingKernel3[0] * path.poses[i - 1].pose.position.x + kPathSmoothingKernel3[1] * path.poses[i].pose.position.x + kPathSmoothingKernel3[2] * path.poses[i + 1].pose.position.x;
        path_out.poses[i].pose.position.y = kPathSmoothingKernel3[0] * path.poses[i - 1].pose.position.y + kPathSmoothingKernel3[1] * path.poses[i].pose.position.y + kPathSmoothingKernel3[2] * path.poses[i + 1].pose.position.y;
        path_out.poses[i].pose.position.z = kPathSmoothingKernel3[0] * path.poses[i - 1].pose.position.z + kPathSmoothingKernel3[1] * path.poses[i].pose.position.z + kPathSmoothingKernel3[2] * path.poses[i + 1].pose.position.z;
    }
    return path_out;
}
