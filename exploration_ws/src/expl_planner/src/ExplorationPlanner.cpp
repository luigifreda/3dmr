/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
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

#include "ExplorationPlanner.h"

#include <boost/operators.hpp>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iomanip>
#include <memory>
#include <limits> 
#include <sstream>

#include <iostream>
#include <fstream>

#include "RrtTree.h"
#include "ExplorationTree.h"
#include "FrontierTree.h"
#include "TeamModel.h"
#include "NavigationTree.h"
#include "SpaceTimeFilterBase.h"
#include "ScanHistoryManager.h"

#include "octomap_world/octomap_world.h"
#include "octomap_world/octomap_manager.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>




#define VERBOSE  // entry level of verbosity 
//#define VERBOSE2 // additional level of verbosity (the entry level verbosity is independent!)

#define ENABLE_TRAVERSABILITY_BIAS_IN_RANDOM_SAMPLE_GENERATOR 0

#define USE_SCAN_FILTER_TO_DOWNSAMPLE_SCANS 0  // not needed anymore here since we have a global scan filter outside 


static double dist2(const double& x, const double& y, const double& z)
{
    return (x*x + y*y + z*z);
}


namespace explplanner
{

// this soft function starts from 0.9 for radius=0m and then approximatively vanishes near radius=3m
inline double weight(double radius)
{
    return (0.8 / (1 + exp((radius - 1) / 0.5)) + 0.2);
}

template<typename T>
static T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
    {
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
}

const float ExplorationPlanner::kRobotSizeHorizontal = 0.45; // robot size horizontal  (NOT USED)
const float ExplorationPlanner::kRobotSizeVertical = 0.3;    // robot size vertical    (NOT USED)

const float ExplorationPlanner::kBiasAcceptCheckThreshold = 0.45; // for prioritizing exploration:if the distance of the input (bias) goal to the traversability cloud is smaller than this threshold than the node is valid 
const float ExplorationPlanner::kPlanningTimeoutSec = 60; // maximum time in seconds for planning 
const float ExplorationPlanner::kMinNeighborsRadius = 0.1; // minimum radius where to search neighbors during each node expansion 

const float ExplorationPlanner::kMaxRobotStep = 0.4; // maximum radius where to search neighbors during each node expansion 
const float ExplorationPlanner::kMaxRobotStepDeltaZ = 0.2;

const float ExplorationPlanner::kMinStepExpansion = 0.0; // [m] min step for in the tree expansion 
const float ExplorationPlanner::kMinStepExpansion2 = ExplorationPlanner::kMinStepExpansion *ExplorationPlanner::kMinStepExpansion; // squared   kMinStepExpansion  

const float ExplorationPlanner::kExpansionMaxCount = 20000; // max num expanded nodes 

const float ExplorationPlanner::kRobotSensorZ = 0.2; // 0.07  height of the sensor w.r.t. robot body (do not put it to zero otherwise with a rough octomap resolution there will be troubles)
    
const float ExplorationPlanner::kTraversabilityProbabilitySmootherFactor = 1000.; 
    
const std::string ExplorationPlanner::kExplorationStepTypeStrings[] = {"None", "Forwarding", "Backtracking", "Rotating", "Completed", "kNoInformation", "NumExplorationStepTypes"};

const float ExplorationPlanner::kBiasPlanningCheckThreshold = 0.2; // if the distance of a path node to the bias point is smaller than this threshold than the node is valid as final 


ExplorationPlanner::ExplorationPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
: nh_(nh), nh_private_(nh_private)
{
    init();
}


// initialize the vars 
void ExplorationPlanner::init()
{
    robot_id_ = 0;
             
    count_ = 0;
    
    n_expl_nodes_ = 0;

    b_abort_ = false;
    b_utility_2d_available_ = false;

    if (!setParams())
    {
        ROS_ERROR("Could not start the planner. Parameters missing!");
    }
    
    /// < set the default cost function 
    p_cost_.reset(new SimpleCostFunction()); //OriginalCostFunction());
 
    //std::cout << "initVars - I'm here" << std::endl; 
    p_octomap_manager_.reset(new volumetric_mapping::OctomapManager(nh_, nh_private_));    
    {       
    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);        
    
    // N.B.: this must stay here since we set params_.inspectionPath_! 
    setPubsAndSubs();
    
    // < Initialize the search tree instance.
    p_search_tree_.reset( new RrtTree(p_octomap_manager_) );
    p_search_tree_->setParams(params_);
    
    // < Initialize the exploration tree instance.
    p_expl_tree_.reset( new ExplorationTree(p_octomap_manager_) );
    p_expl_tree_->setParams(params_);    
    p_expl_tree_->initialize(); 
    
    // < Initialize the exploration frontier tree instance.    
    p_frontier_tree_.reset( new FrontierTree(p_octomap_manager_) );
    p_frontier_tree_->setParams(params_);    
    p_frontier_tree_->initialize();   
    }
    
    // < Initialize the navigation frontier tree instance.    
    p_nav_tree_.reset( new NavigationTree() );
    p_nav_tree_->setParams(params_);           

#if USE_SCAN_FILTER_TO_DOWNSAMPLE_SCANS
    p_space_time_filter_.reset( new SpaceTimeFilterBase());
    p_space_time_filter_->setParams();
    p_space_time_filter_->init();
#endif 

#if USE_MAP_SYNC
    p_scan_history_manager_.reset( new ScanHistoryManager());
    p_scan_history_manager_->init();
#endif 

    explorationStepType_ = kNone; 
    
    robot_bounding_box_size_[0] = robot_bounding_box_size_[1] = kRobotSizeHorizontal;
    robot_bounding_box_size_[2] = kRobotSizeVertical;       
    
    b_use_expl_bias_ = false;
}


void ExplorationPlanner::setPubsAndSubs()
{
    // < Publishers
    markerArrPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/expl_planner/visited_nodes", 1);
    explPathPub_ = nh_.advertise<nav_msgs::Path>("/expl_planner/path", 1);

    params_.inspectionPathPub_ = nh_.advertise<visualization_msgs::Marker>("/expl_planner/inspection_path", 10000);  

    params_.explorationPathPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/expl_planner/exploration_path", 100); 
    params_.explorationNodesPub_ = nh_.advertise<visualization_msgs::Marker>("/expl_planner/exploration_nodes", 100);             
    params_.explorationFrontierTreePub_ = nh_.advertise<visualization_msgs::MarkerArray>("/expl_planner/frontier_tree", 100, true);  // latched 
    params_.explorationFrontierClustersPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/expl_planner/frontier_clusters", 100, true);  // latched 
    params_.explorationNavTreePub_ = nh_.advertise<visualization_msgs::MarkerArray>("/expl_planner/back_nav_tree", 100);     
    
    // < Subscribers 
    
    pointcloud_sub_ = nh_.subscribe("/expl_planner/exploration_pointcloud", 1, &ExplorationPlanner::insertPointcloudWithTf, this);
}


/// < DESTRUCTOR
ExplorationPlanner::~ExplorationPlanner()
{
    std::cout << "ExplorationPlanner::~ExplorationPlanner() - start " << std::endl;
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    //count_ = 0;
    //n = n_;
    //markerArrPub_ = n_.advertise<visualization_msgs::MarkerArray>("expl_planner/visitedNodes", 1);

    //markerArr_.markers.clear(); 
    //markerArrPub_.publish(markerArr_); 
    std::cout << "ExplorationPlanner::~ExplorationPlanner() - end " << std::endl;
}


void ExplorationPlanner::insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
    double pcl_stamp = pointcloud->header.stamp.now().toSec();
    
#if USE_SCAN_FILTER_TO_DOWNSAMPLE_SCANS        
    if (p_space_time_filter_->insertPointCloud(pointcloud))
#else 
    if (pcl_stamp >= last_pcl_stamp_ + params_.pcl_throttle_)
#endif     
    {
        //std::cout << "ExplorationPlanner::insertPointcloudWithTf() - start " << std::endl;         
        boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);

        std::cout << "ExplorationPlanner::insertPointcloudWithTf() - inserting Pointcloud " << std::endl;
        // count the number map points before
        // allocate visualization_msgs::MarkerArray* occupied_nodes, visualization_msgs::MarkerArray* free_nodes
        visualization_msgs::MarkerArray occupied_nodes, free_nodes;

        //p_octomap_manager_->generateMarkerArray(pointcloud->header.frame_id, &occupied_nodes, &free_nodes);
        p_octomap_manager_->generateMarkerArray(p_octomap_manager_->GetWorldFrame(), &occupied_nodes, &free_nodes);
        double occ_i = occupied_nodes.markers[p_octomap_manager_->getTreeDepth()].points.size();
        double free_i = free_nodes.markers[p_octomap_manager_->getTreeDepth()].points.size();

        std::cout << "ExplorationPlanner::insertPointcloudWithTf() - inserting... " << std::endl;
        p_octomap_manager_->insertPointcloudWithTf(pointcloud);
        std::cout << "\nExplorationPlanner::insertPointcloudWithTf() - added new point cloud with tf " << pointcloud->header.frame_id << std::endl;
        last_pcl_stamp_ = pcl_stamp;

        last_integrated_pointcloud_ = p_octomap_manager_->getLastIntegratedPointcloud();

#if 0
        Eigen::Vector3d robot_box_position_ = robot_position_;
        //robot_box_position_[2] += 0.5 * robot_bounding_box_size_[2] + 0.5 * p_octomap_manager_->getResolution();
        robot_box_position_[2] += 0.5 * robot_bounding_box_size_[2] + p_octomap_manager_->getResolution();
        p_octomap_manager_->setFree(robot_box_position_, robot_bounding_box_size_);
#endif

        // count the number map points after
        // allocate visualization_msgs::MarkerArray* occupied_nodes, visualization_msgs::MarkerArray* free_nodes
        visualization_msgs::MarkerArray occupied, free;

        //p_octomap_manager_->generateMarkerArray(pointcloud->header.frame_id, &occupied, &free);
        p_octomap_manager_->generateMarkerArray(p_octomap_manager_->GetWorldFrame(), &occupied, &free);
        double occ_f = occupied.markers[p_octomap_manager_->getTreeDepth()].points.size();
        double free_f = free.markers[p_octomap_manager_->getTreeDepth()].points.size();

        // compute the deltas number of points => delta volume
        double d_occ = occ_f - occ_i;
        double d_free = free_f - free_i;

        // write values in a correspondent file =========================================================
        std::string path = "3dmr_devel/" + pointcloud->header.frame_id;
        int res_command = system(("mkdir -p ~/.ros/" + path).c_str());
        std::string filename = path + "/points_map.txt";
        std::ofstream file;
        file.open(filename, std::ios_base::app);
        file << pointcloud->header.stamp << "\t" << occ_i << "\t" << occ_f << "\t" << d_occ << "\t" << free_i << "\t" << free_f << '\t' << d_free << std::endl;
        file.close();

        std::ofstream doc;
        doc.open("3dmr_devel/explored.txt", std::ios_base::out);
        doc << "OCC=" << occ_f << std::endl;
        doc.close();

#if USE_MAP_SYNC
        p_scan_history_manager_->insert(pointcloud);
#endif 

    }
}

void ExplorationPlanner::insertOtherUgvPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
    double pcl_stamp = pointcloud->header.stamp.now().toSec();
    
#if USE_SCAN_FILTER_TO_DOWNSAMPLE_SCANS    
    if (p_space_time_filter_->insertPointCloud(pointcloud))
#endif     
    {

    //std::cout << "ExplorationPlanner::insertOtherUgvPointcloudWithTf() - start " << std::endl;    
    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);

    visualization_msgs::MarkerArray occupied_nodes, free_nodes;

    //p_octomap_manager_->generateMarkerArray(pointcloud->header.frame_id, &occupied_nodes, &free_nodes);
    p_octomap_manager_->generateMarkerArray(p_octomap_manager_->GetWorldFrame(), &occupied_nodes, &free_nodes);

    double occ_i = occupied_nodes.markers[p_octomap_manager_->getTreeDepth()].points.size();
    double free_i = free_nodes.markers[p_octomap_manager_->getTreeDepth()].points.size();

    std::cout << "ExplorationPlanner::insertOtherUgvPointcloudWithTf() - start " << std::endl;
    p_octomap_manager_->insertPointcloudWithTf(pointcloud);
    std::cout << "\nExplorationPlanner::insertOtherUgvPointcloudWithTf() - added new point cloud with tf " << pointcloud->header.frame_id << std::endl;

#if 0
    Eigen::Vector3d robot_box_position_ = robot_position_;
    //robot_box_position_[2] += 0.5* robot_bounding_box_size_[2] + 0.5 * p_octomap_manager_->getResolution();
    robot_box_position_[2] += 0.5* robot_bounding_box_size_[2] + p_octomap_manager_->getResolution();
    p_octomap_manager_->setFree(robot_box_position_, robot_bounding_box_size_);
#endif

    // count the number map points after
    // allocate visualization_msgs::MarkerArray* occupied_nodes, visualization_msgs::MarkerArray* free_nodes
    visualization_msgs::MarkerArray occupied, free;

    //p_octomap_manager_->generateMarkerArray(pointcloud->header.frame_id, &occupied, &free);
    p_octomap_manager_->generateMarkerArray(p_octomap_manager_->GetWorldFrame(), &occupied, &free);

    double occ_f = occupied.markers[p_octomap_manager_->getTreeDepth()].points.size();
    double free_f = free.markers[p_octomap_manager_->getTreeDepth()].points.size();

    // compute the deltas number of points => delta volume
    double d_occ = occ_f - occ_i;
    double d_free = free_f - free_i;

    // write values in a correspondent file =========================================================
    std::string path = "3dmr_devel/" + pointcloud->header.frame_id;
    int res_command = system(("mkdir -p ~/.ros/" + path).c_str());
    std::string filename = path + "/points_map_rec.txt";
    std::ofstream file;
    file.open(filename, std::ios_base::app);
    file << pointcloud->header.stamp << "\t" << occ_i << "\t" << occ_f << "\t" << d_occ << "\t" << free_i << "\t" << free_f << '\t' << d_free << std::endl;
    file.close();

#if USE_MAP_SYNC
        p_scan_history_manager_->insert(pointcloud);
#endif 

    }
}


bool ExplorationPlanner::setParams()
{
    std::string ns = ros::this_node::getName();
    bool ret = true;
    params_.v_max_ = 0.25;
    if (!ros::param::get(ns + "/system/v_max", params_.v_max_))
    {
        ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.",
                 (ns + "/system/v_max").c_str());
    }
    params_.dyaw_max_ = 0.5;
    if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_))
    {
        ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
                 (ns + "/system/yaw_max").c_str());
    }
    params_.camPitch_ = {15.0}; // NOTE: not used here!
    if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_))
    {
        ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
                 (ns + "/system/camera/pitch").c_str());
    }
    params_.camHorizontal_ = {90.0}; // NOTE: not used here!
    if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_))
    {
        ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.",
                 (ns + "/system/camera/horizontal").c_str());
    }
    params_.camVertical_ = {60.0}; // NOTE: not used here!
    if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_))
    {
        ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60deg.",
                 (ns + "/system/camera/vertical").c_str());
    }
    if (params_.camPitch_.size() != params_.camHorizontal_.size() || params_.camPitch_.size() != params_.camVertical_.size())
    {
        ROS_WARN("Specified camera fields of view unclear: Not all parameter vectors have same length! Setting to default.");
        params_.camPitch_.clear();
        params_.camPitch_ = {15.0};
        params_.camHorizontal_.clear();
        params_.camHorizontal_ = {90.0};
        params_.camVertical_.clear();
        params_.camVertical_ = {60.0}; 
    }
    params_.cameraVerticalFov_ = 180.0;
    if (!ros::param::get(ns + "/system/camera/verticalFov", params_.cameraVerticalFov_))
    {
        ROS_WARN_STREAM(
            "No camera vertical FOV specified. Looking for " << 
            (ns + "/system/camera/verticalFov").c_str() << ". Default is " << params_.cameraVerticalFov_);
    }        
    params_.robotSensorHeight = kRobotSensorZ;
    if (!ros::param::get(ns + "/system/robot/robotSensorHeight", params_.robotSensorHeight))
    {
        ROS_WARN_STREAM(
            "No robot-sensor height specified. Looking for " << 
            (ns + "/system/robot/robotSensorHeight").c_str() << ". Default is " << params_.robotSensorHeight);
    }     

    params_.igProbabilistic_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/probabilistic", params_.igProbabilistic_))
    {
        ROS_WARN(
                 "No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/probabilistic").c_str());
    }
    params_.igFree_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/free", params_.igFree_))
    {
        ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/free").c_str());
    }
    params_.igOccupied_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/occupied", params_.igOccupied_))
    {
        ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/occupied").c_str());
    }
    params_.igUnmapped_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/unmapped", params_.igUnmapped_))
    {
        ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
                 (ns + "/nbvp/gain/unmapped").c_str());
    }
    params_.igArea_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/area", params_.igArea_))
    {
        ROS_WARN("No gain coefficient for mesh area specified. Looking for %s. Default is 1.0.",
                 (ns + "/nbvp/gain/area").c_str());
    }
    params_.degressiveCoeff_ = 0.25;
    if (!ros::param::get(ns + "/nbvp/gain/degressive_coeff", params_.degressiveCoeff_))
    {
        ROS_WARN(
                 "No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.",
                 (ns + "/nbvp/gain/degressive_coeff").c_str());
    }
    params_.extensionRange_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/tree/extension_range", params_.extensionRange_))
    {
        ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
                 (ns + "/nbvp/tree/extension_range").c_str());
    }
    params_.frontierClusteringRadius_ = 1.0; // default value 
    if (!ros::param::get(ns + "/nbvp/tree/frontier_clustering_radius", params_.frontierClusteringRadius_))
    {
        ROS_WARN("No value for frontier clustering radius specified. Looking for %s. Default is 1.0m.",
                 (ns + "/nbvp/tree/frontier_clustering_radius").c_str());
    }   
    if(params_.frontierClusteringRadius_ < params_.extensionRange_ )
    {
        ROS_WARN_STREAM("Forcing frontier clustering radius to be bigger than extension range");
        params_.frontierClusteringRadius_ = 1.5 * params_.extensionRange_;
    }
        
    params_.numEdgeSteps_ = 1; 
    if (!ros::param::get(ns + "/nbvp/tree/num_edge_steps", params_.numEdgeSteps_))
    {
        ROS_WARN("No value for num edge steps specified. Looking for %s. Default is 1.",
                 (ns + "/nbvp/tree/num_edge_steps").c_str());
    }    
    
    params_.explStep_ = 1.0; 
    if (!ros::param::get(ns + "/nbvp/tree/expl_step", params_.explStep_))
    {
        ROS_WARN("No value for exploration step specified. Looking for %s. Default is 1.",
                 (ns + "/nbvp/tree/expl_step").c_str());
    }    
    params_.explStep_ = std::max(params_.extensionRange_, params_.explStep_);   // at least >= params_.extensionRange_ 
    std::cout << "expl_step: " << params_.explStep_ << std::endl; 
    
    params_.explStepBacktracking_ = params_.explStep_; 
    if (!ros::param::get(ns + "/nbvp/tree/expl_step_backtracking", params_.explStepBacktracking_))
    {
        ROS_WARN("No value for exploration step specified. Looking for %s. Default is equal to /nbvp/tree/expl_step.",
                 (ns + "/nbvp/tree/expl_step_backtracking").c_str());
    }    
    params_.explStepBacktracking_ = std::max(params_.extensionRange_, params_.explStepBacktracking_);   // at least >= params_.extensionRange_ 
    std::cout << "expl_step: " << params_.explStepBacktracking_ << std::endl;     
    
    params_.initIterations_ = 15;
    if (!ros::param::get(ns + "/nbvp/tree/initial_iterations", params_.initIterations_))
    {
        ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
                 (ns + "/nbvp/tree/initial_iterations").c_str());
    }
    params_.dt_ = 0.1;
    if (!ros::param::get(ns + "/nbvp/dt", params_.dt_))
    {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                 (ns + "/nbvp/dt").c_str());
    }
    params_.gainRange_ = 5.0;
    if (!ros::param::get(ns + "/nbvp/gain/range", params_.gainRange_))
    {
        ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
                 (ns + "/nbvp/gain/range").c_str());
    }
    if (!ros::param::get(ns + "/bbx/minX", params_.minX_))
    {
        ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minY", params_.minY_))
    {
        ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_))
    {
        ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_))
    {
        ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_))
    {
        ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_))
    {
        ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
        ret = false;
    }
    params_.softBounds_ = false;
    if (!ros::param::get(ns + "/bbx/softBounds", params_.softBounds_))
    {
        ROS_WARN(
                 "Not specified whether scenario bounds are soft or hard. Looking for %s. Default is false",
                 (ns + "/bbx/softBounds").c_str());
    }
    params_.boundingBox_[0] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0]))
    {
        ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
                 (ns + "/system/bbx/x").c_str());
    }
    params_.boundingBox_[1] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1]))
    {
        ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
                 (ns + "/system/bbx/y").c_str());
    }
    params_.boundingBox_[2] = 0.3;
    if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2]))
    {
        ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
                 (ns + "/system/bbx/z").c_str());
    }
    params_.cuttoffIterations_ = 200;
    if (!ros::param::get(ns + "/nbvp/tree/cuttoff_iterations", params_.cuttoffIterations_))
    {
        ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
                 (ns + "/nbvp/tree/cuttoff_iterations").c_str());
    }
    params_.zero_gain_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/zero", params_.zero_gain_))
    {
        ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/zero").c_str());
    }
    params_.dOvershoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_))
    {
        ROS_WARN(
                 "No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
                 (ns + "/system/bbx/overshoot").c_str());
    }
    params_.bLog_ = false;
    if (!ros::param::get(ns + "/nbvp/log/on", params_.bLog_))
    {
        ROS_WARN("Logging is off by default. Turn on with %s: true", (ns + "/nbvp/log/on").c_str());
    }
    params_.log_throttle_ = 0.5;
    if (!ros::param::get(ns + "/nbvp/log/throttle", params_.log_throttle_))
    {
        ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
                 (ns + "/nbvp/log/throttle").c_str());
    }
    params_.navigationFrame_ = "map";
    if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_))
    {
        ROS_WARN("No navigation frame specified. Looking for %s. Default is 'map'.",
                 (ns + "/tf_frame").c_str());
    }
    params_.pcl_throttle_ = 0.1;
    if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_))
    {
        ROS_WARN(
                 "No throttle time constant for the point cloud insertion specified. Looking for %s. Default is 0.333.",
                 (ns + "/pcl_throttle").c_str());
    }
    params_.inspection_throttle_ = 0.1;
    if (!ros::param::get(ns + "/inspection_throttle", params_.inspection_throttle_))
    {
        ROS_WARN(
                 "No throttle time constant for the inspection view insertion specified. Looking for %s. Default is 0.1.",
                 (ns + "/inspection_throttle").c_str());
    }
    params_.exact_root_ = true;
    if (!ros::param::get(ns + "/nbvp/tree/exact_root", params_.exact_root_))
    {
        ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.",
                 (ns + "/nbvp/tree/exact_root").c_str());
    }
    

    params_.bDownSampleTraversability_ = true;
    params_.bDownSampleTraversability_ = getParam<bool>(nh_private_,ns + "/nbvp/downsample_traversability", params_.bDownSampleTraversability_);    
    
    params_.downsampleTravResolution_ = 0.1; 
    params_.downsampleTravResolution_ = getParam<double>(nh_private_,ns + "/nbvp/downsample_trav_resolution", params_.downsampleTravResolution_);   
    
    params_.conflictDistance_ = TeamModel::kNodeConflictDistance; // default value 
    params_.conflictDistance_ = getParam<double>(nh_private_,ns + "/team/conflict_distance", params_.conflictDistance_);   
   
    params_.probHit_ = 0.75; 
    params_.probHit_ = getParam<double>(nh_private_, ns + "/probability_hit", params_.probHit_);
    
    params_.probMiss_ = 0.4;
    params_.probMiss_ = getParam<double>(nh_private_,ns + "/probability_miss", params_.probMiss_);
    
    params_.clampingThresMin_ = 0.12;
    params_.clampingThresMin_ = getParam<double>(nh_private_,ns + "/threshold_min", params_.clampingThresMin_);
    
    params_.clampingThresMax_ = 0.97;
    params_.clampingThresMax_ = getParam<double>(nh_private_,ns + "/threshold_max", params_.clampingThresMax_);
    
    params_.occupancyThres_ = 0.7;
    params_.occupancyThres_ = getParam<double>(nh_private_,ns + "/threshold_occupancy", params_.occupancyThres_);
    
    return ret;
}


bool sortProbFunction(const ExplorationPlanner::IdxProbability& i, const ExplorationPlanner::IdxProbability& j)
{
    return (i.probability < j.probability);
}

void ExplorationPlanner::filterNeighborhoodByDistAndDz(const pcl::PointCloud<pcl::PointXYZI>& pcl_trav, const pcl::PointXYZI& p, double squaredDist, const double deltaZ, std::vector<int>& pointIdx, std::vector<float>& pointSquaredDistance)
{
    std::vector<float>::iterator it_dist = pointSquaredDistance.begin();
    std::vector<int>::iterator it_id = pointIdx.begin();

    bool bRemove = false;

    while ((it_dist != pointSquaredDistance.end()) && (it_id != pointIdx.end()))
    {
        pcl::PointXYZI p_it = pcl_trav[*it_id];

        bRemove = (fabs(p_it.z - p.z) > deltaZ)  // do not take new point over big Z-steps 
                    || (*it_dist < squaredDist); // do not take new point too close 
        if (bRemove)
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
void ExplorationPlanner::findNeighbors(std::vector<IdxProbability>& neighbors, double& radius)
{
    /// < Find the nearest point labeled as wall and set the search radius according to this distance
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1, std::numeric_limits<float>::max());
    pcl::PointXYZI point_noWall = pcl_traversability_[nodes_[current_node_idx_].point_idx];
    pcl::PointXYZRGBNormal point_noWall_RGBN;
    point_noWall_RGBN.x = point_noWall.x;
    point_noWall_RGBN.y = point_noWall.y;
    point_noWall_RGBN.z = point_noWall.z;

    /// < NOTE: This function returns squared distances
    int num_wall_neighbors = kdtree_wall_.nearestKSearch(point_noWall_RGBN, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    if (num_wall_neighbors < 1)
    {
        ROS_WARN("ExplorationPlanner::findNeighbors() - could not find wall neighbors");
    }
    /*double checkradius = sqrt(pointNKNSquaredDistance[0]);
    if( fabs(checkradius-dist(point_wall,pcl_wall_.points[pointIdxNKNSearch[0]])) > 1e-6) 
    {
        std::cout << "ExplorationPlanner::findNeighbors() - error - radius: " << checkradius << ", dist: " << dist(point_wall,pcl_wall_.points[pointIdxNKNSearch[0]]) << std::endl;
        quick_exit(-1);
    }*/

    //radius = sqrt(pointNKNSquaredDistance[0]);
    //radius = std::max(sqrt(pointNKNSquaredDistance[0]) - kRobotRadius, kMinNeighborsRadius);
    //radius = std::min(std::max(sqrt(pointNKNSquaredDistance[0]) - kRobotRadius, kMinNeighborsRadius), kMaxRobotStep);

    // fix the maximum step of the robot to the set related parameter 
    radius = std::min((float) sqrt(pointNKNSquaredDistance[0]), kMaxRobotStep);


#ifdef VERBOSE2    
    ROS_INFO("ExplorationPlanner::findNeighbors() - radius: %f", radius);
    std::cout << "ExplorationPlanner::find_neighbors() - current point : " << pcl_traversability_[nodes_[current_node_idx_].point_idx] << ", radius " << radius << std::endl;
#endif

    /// < Find traversability neighbors of the current node within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int num_close_points = kdtree_traversability_.radiusSearch(point_noWall, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    // eliminate points which have Dz w.r.t. point_noWall higher than kMaxRobotStepDeltaZ
    filterNeighborhoodByDistAndDz(pcl_traversability_, point_noWall, kMinStepExpansion2, kMaxRobotStepDeltaZ, pointIdxRadiusSearch, pointRadiusSquaredDistance);

#ifdef VERBOSE2     
    std::cout << "ExplorationPlanner::find_neighbors() - num_close_points : " << num_close_points << std::endl;
#endif
    if (num_close_points < 1)
    {
        ROS_WARN("ExplorationPlanner::findNeighbors() - could not find point close to current node");
    }


    /// < Extract the traversability neighbors and compute probability associated with each point
    /// < Point with low cost will have higher probability	  
    float normalization_factor = 0;
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
    {
        if( b_use_expl_bias_ && (dist(pcl_traversability_[pointIdxRadiusSearch[i]], expl_point_bias_) < kBiasPlanningCheckThreshold) ) // we want to be more precise in the expansion; the threshold was 0.001
        {
#ifdef VERBOSE
            std::cout << "ExplorationPlanner::findNeighbors() - found point close to bias" << std::endl;
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
        //else if (pcl_traversability_[pointIdxRadiusSearch[i]].intensity < INFINITY)
        else if (pcl_traversability_[pointIdxRadiusSearch[i]].intensity < std::numeric_limits<double>::infinity())
        {
#if ENABLE_TRAVERSABILITY_BIAS_IN_RANDOM_SAMPLE_GENERATOR
            float prob = (pcl_traversability_[pointIdxRadiusSearch[i]].intensity + kTraversabilityProbabilitySmootherFactor);
#else
            float prob = 1.;
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
    std::cout << "ExplorationPlanner::find_neighbors() - neighbors size : " << neighbors.size() << std::endl;
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

bool ExplorationPlanner::visitedPoint(size_t pointIdx)
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
void ExplorationPlanner::sampleFollowers()
{
    //Build the neighborhood of current node and set the probability of each point on the basis of the traversability 
    std::vector<IdxProbability> neighbors;
    double radius;
    findNeighbors(neighbors, radius);

    //Sample the child from the neighbors if the neighbors are more then a threshold, else use all the neighbors
    int num_generated_followers = 0;
    int num_random_samples = 0;
    
    //double w = weight(radius);
#if 0
    double w = std::min(weight(radius), 1.d);
    int weighted_neighbors_size = lrint(w * neighbors.size());
#else
    int weighted_neighbors_size = lrint(0.9 * neighbors.size());
#endif
    
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

            double aux_2d_cost = 0;
            double aux_2d_confidence = 1;
            if (!pcl_utility_2d_.empty())
            {
                aux_2d_cost = pcl_utility_2d_[neighbors[i].point_idx].z;
                aux_2d_confidence = pcl_utility_2d_[neighbors[i].point_idx].intensity;
            }

            //double cost(const pcl::PointXYZI& current, const pcl::PointXYZI& next, const pcl::PointXYZI& goal, double traversability, double aux_utility = 0, double aux_conf = 0)    
            //child.cost = p_cost_->cost(pcl_traversability_[nodes_[current_node_idx_].point_idx], pcl_traversability_[neighbors[i].point_idx], goal_, pcl_traversability_[neighbors[i].point_idx].intensity, aux_2d_cost, aux_2d_confidence);
            
            if(b_use_expl_bias_)
            {
                // A* expansion 
                child.cost = dist(pcl_traversability_[nodes_[current_node_idx_].point_idx], pcl_traversability_[neighbors[i].point_idx]) +
                             dist(pcl_traversability_[nodes_[current_node_idx_].point_idx], expl_point_bias_);
            }
            else
            {
                // normal breadth first expansion 
                child.cost = nodes_[current_node_idx_].cost + dist(pcl_traversability_[nodes_[current_node_idx_].point_idx], pcl_traversability_[neighbors[i].point_idx]);
            }

            child.parent_id = current_node_idx_;
            child.point_idx = neighbors[i].point_idx;
            nodes_[current_node_idx_].child_id.push_back(child.id);
            nodes_.push_back(child);

            leaf_nodes_idxs_.push_back(child.id);

            visited_points_flag_[child.point_idx] = true;
            
            /// < add the node to the exploration tree 
            const pcl::PointXYZI& neighbor_point = pcl_traversability_[neighbors[i].point_idx];
            p_search_tree_->addNode(neighbor_point.x, neighbor_point.y, neighbor_point.z + params_.robotSensorHeight);
            
            if(b_publish_markers)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.pose.position.x = pcl_traversability_[neighbors[i].point_idx].x;
                marker.pose.position.y = pcl_traversability_[neighbors[i].point_idx].y;
                marker.pose.position.z = pcl_traversability_[neighbors[i].point_idx].z;
                marker.lifetime = ros::Duration(5);
                marker.id = markerArr_.markers.size() + 1;
                markerArr_.markers.push_back(marker);
            }            
            
            num_generated_followers++;

//#ifdef VERBOSE2            
//          std::cout << "num_generated_followers: "<< num_generated_followers << std::endl;
//#endif
        }
    }

#ifdef VERBOSE2 
    std::cout << "ExplorationPlanner::sample_followers() - num_generated_followers : " << num_generated_followers << std::endl;
#endif

    // Remove current node from leaf nodes structure
    for (std::vector<size_t>::iterator it = leaf_nodes_idxs_.begin(); it != leaf_nodes_idxs_.end(); ++it)
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
#endif

    if(b_publish_markers)   markerArrPub_.publish(markerArr_);

    //Add the current point to the path
}

//Find the best node from all the leaf node
bool ExplorationPlanner::findNextNode()
{
    //double minCost = INFINITY;
    double minCost = std::numeric_limits<double>::max();

    size_t minCostNodeIdx = 0;
    bool b_found = false;

    //Find the best expansion within the leaf nodes
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

void ExplorationPlanner::setInput(pcl::PointCloud<pcl::PointXYZI>& traversability_pcl_in, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_pcl_in,
                                  pp::KdTreeFLANN<pcl::PointXYZRGBNormal>& wall_kdtree_in, KdTreeFLANN& traversability_kdtree_in, int start_point_idx_in)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    b_utility_2d_available_ = false; // reset the utility function, we are required to add new information synched with traversability cloud 

    std::cout << "ExplorationPlanner::setInput() - traversability pcl size: " << traversability_pcl_in.size() << std::endl;

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

    // reset visited points 
    visited_points_flag_ = std::vector<bool>(pcl_traversability_.size(), false);
    // set the starting node as visited 
    visited_points_flag_[start_point_idx_] = true;
}

void ExplorationPlanner::set2DUtility(pcl::PointCloud<pcl::PointXYZI>& utility_pcl)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    std::cout << "ExplorationPlanner::set2DUtility() - utility  pcl size: " << utility_pcl.size() << std::endl;

    pcl_utility_2d_ = utility_pcl;

    /// < check if it has the same size of the traversability pcl 
    if ((pcl_utility_2d_.header.stamp != pcl_traversability_.header.stamp) || (pcl_utility_2d_.size() != pcl_traversability_.size()))
    {
        ROS_WARN("**********************************************************************************************************");
        ROS_WARN("ExplorationPlanner::set2DUtility() - traversability and utility pcls have different sizes or stamp");
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

void ExplorationPlanner::setCostFunction(BaseCostFunction* new_cost, float lamda_trav, float lambda_aux_utility, float tau_exp_decay)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    p_cost_.reset(new_cost);
    p_cost_->SetLambdaTrav(lamda_trav);
    p_cost_->SetLambdaAuxUtility(lambda_aux_utility);
    p_cost_->SetTauExpDecay(tau_exp_decay);
}

void ExplorationPlanner::setCostFunctionType(int type, float lamda_trav, float lambda_aux_utility, float tau_exp_decay)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    type = std::max(std::min(type, (int) BaseCostFunction::kNumCostFunctions - 1), 0);

    switch (type)
    {
    case BaseCostFunction::kTraversabilityCost:
        std::cout << "ExplorationPlanner::setCostFunctionType()- setting TraversabilityCostFunction()" << std::endl;
        setCostFunction(new TraversabilityCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        break;
    case BaseCostFunction::kTraversabilityProdCost:
        setCostFunction(new TraversabilityProdCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        std::cout << "ExplorationPlanner::setCostFunctionType()- setting TraversabilityProdCostFunction()" << std::endl;
        break;
    case BaseCostFunction::kSimpleCost:
        setCostFunction(new SimpleCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        std::cout << "ExplorationPlanner::setCostFunctionType()- setting OriginalCostFunction()" << std::endl;
        break;
    case BaseCostFunction::kBaseCost:
    default:
        setCostFunction(new BaseCostFunction(), lamda_trav, lambda_aux_utility, tau_exp_decay);
        std::cout << "ExplorationPlanner::setCostFunctionType()- setting BaseCostFunction()" << std::endl;
    }
}

void ExplorationPlanner::setRobotPosition(const double& x, const double& y, const double& z)
{
    robot_position_[0] = x; 
    robot_position_[1] = y; 
    robot_position_[2] = z;     
}

void ExplorationPlanner::setRobotId(int id)
{
    robot_id_ = id;
    std::stringstream ss; 
    ss << "ugv" << id+1; 
    str_robot_name_ = ss.str();
    
    if(p_search_tree_) p_search_tree_->setRobotId(id);
    
    if(p_expl_tree_) p_expl_tree_->setRobotId(id);
    
    if(p_frontier_tree_) p_frontier_tree_->setRobotId(id);    
    
    if(p_nav_tree_) p_nav_tree_->setRobotId(id);

    if(p_scan_history_manager_) p_scan_history_manager_->setRobotId(id);
}

void ExplorationPlanner::setParamExplorationBoxXY(const double minX, const double maxX, const double minY, const double maxY)
{
    params_.minX_ = minX; params_.maxX_ = maxX;    
    params_.minY_ = minY; params_.maxY_ = maxY;
    
    p_search_tree_->setParams(params_);
}

void ExplorationPlanner::addSelectedNodeToExplorationTree()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    
    if( explorationStepType_ == kForwarding ) 
    {
        Node<ExplorationTree::StateVec>* selectedNode = p_search_tree_->getSelectedNode();
        if(selectedNode) 
        {
            p_expl_tree_->addNode(selectedNode);
            n_expl_nodes_++;
               
        }
    }          
}

void ExplorationPlanner::publishPlannedExplorationNode()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    
    if( explorationStepType_ == kForwarding ) 
    {
        Node<ExplorationTree::StateVec>* selectedNode = p_search_tree_->getSelectedNode();
        if(selectedNode) 
        {
            p_expl_tree_->publishSelectedNode(selectedNode); 
        }
    }      
}
    
void ExplorationPlanner::removeCurrentExplorationNode()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);    
    
    p_expl_tree_->removeCurrentNode();
    n_expl_nodes_++;    
    
}

void ExplorationPlanner::backTrackOnExplorationTree()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);    
    
    p_expl_tree_->backTrack();
    
}

void ExplorationPlanner::manageConflict()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);    
    
    // Return to previous point on search tree manager
    p_search_tree_->getPathBackToPrevious(params_.navigationFrame_); // we do not need the output here 
    p_search_tree_->resetBestBranch();    
}

void ExplorationPlanner::resetSelectedBackTrackingCluster()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);    
    p_frontier_tree_->resetSelectedBackTrackingCluster();
}

bool ExplorationPlanner::setExplorationBias(pcl::PointXYZI& bias_in, bool bUseIt)
{
    b_use_expl_bias_ = bUseIt;
    if(!bUseIt) return false; 
    
    bool res = true;

    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    if (pcl_traversability_.empty())
    {
        ROS_WARN("ExplorationPlanner::setExplorationBias() - cannot  set exploration bias if traversability kdtree is empty");
        b_use_expl_bias_ = false; 
        return false;
    }

    // Find the goal nearest point in traversability point cloud
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    //int found = kdtree_traversability_.nearestKSearch(goal_in, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    int found = kdtree_traversability_.radiusSearch(bias_in, kBiasAcceptCheckThreshold, pointIdxNKNSearch, pointNKNSquaredDistance);
    if (found < 1)
    {
        ROS_WARN("ExplorationPlanner::setExplorationBias() - cannot find a node close to bias ");
        res = false;
        b_use_expl_bias_ = false; 
    }
    {
        expl_point_bias_ = pcl_traversability_[pointIdxNKNSearch[0]];
        b_use_expl_bias_ = true; 
    }

//    for (int i = 0; i < markerArr_.markers.size(); i++)
//    {
//        markerArr_.markers[i].action = visualization_msgs::Marker::DELETE;
//    }

    return res;
}

pcl::PointXYZI& ExplorationPlanner::getExplorationBias()
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

    return expl_point_bias_;
}

bool ExplorationPlanner::planning(nav_msgs::Path& path_out, bool bEnableBacktracking)
{
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);

#ifdef VERBOSE 
    std::cout << "------------------------------------------------"<< std::endl;    
    std::cout << "ExplorationPlanner::planning() - start position: " << pcl_traversability_[start_point_idx_] << std::endl;
    //std::cout << "ExplorationPlanner::planning() - bias position: " << expl_bias_ << std::endl;
#endif

    explorationStepType_ = kNone; 
        
    b_abort_ = false;

    if (pcl_traversability_.empty())
    {
        ROS_WARN("ExplorationPlanner::planning() - cannot plan if traversability kdtree is empty");
        return false;
    }

    std::cout << "ExplorationPlanner::planning() - using cost function: " << p_cost_->getName() << std::endl;

    path_out.poses.clear();
    path_out.header.frame_id = "map";

    leaf_nodes_idxs_.clear();

    // reset visited points 
    visited_points_flag_ = std::vector<bool>(pcl_traversability_.size(), false);
    // set the starting node as visited 
    visited_points_flag_[start_point_idx_] = true;

    //nodes_.erase(nodes_.begin() + 1, nodes_.end());
    nodes_.resize(1); // leave the starting node we set with the function set_input()

    //path_out.header.frame_id = "map";

    markerArr_.markers.clear();

    bool is_found_path = false;
    bool is_close_to_bias = false;    
    bool is_exist_path = true;
    bool is_timeout = false;

    ros::Time time_start = ros::Time::now();
    count_ = 0;

    p_cost_->initTime();
    
    // Clear old search exploration tree and reinitialize.
    p_search_tree_->clear();
    
    // Clear old navigation tree and reinitialize.    
    p_nav_tree_->clear();

    // First set root then init 
    const pcl::PointXYZI& start_point = pcl_traversability_[start_point_idx_];
    p_search_tree_->setRoot(start_point.x, start_point.y, start_point.z);
    
    if(n_expl_nodes_ == 0)
    {
        p_expl_tree_->setRoot(robot_position_[0], robot_position_[1], robot_position_[2]);   
        n_expl_nodes_++;
     
        p_frontier_tree_->setRoot(robot_position_[0], robot_position_[1], robot_position_[2]);                    
    }    
    
    p_nav_tree_->setRoot(robot_position_[0], robot_position_[1], robot_position_[2]);      
    
    if(b_use_expl_bias_) p_search_tree_->resetBestBranch(); // < if we bias the exploration forget about the last best branch!
    p_search_tree_->initialize(); // < N.B: here we re-insert last best branch (if any) and perform collision checking on it!
            
    std::cout << "ExplorationPlanner::planning() - using cost function: " << p_cost_->getName() << std::endl;
    
    // expand the exploration tree 
    while( (count_ < kExpansionMaxCount) && is_exist_path && !is_close_to_bias && !is_timeout && !b_abort_ )
    {
        count_++;
        sampleFollowers();
        is_exist_path = findNextNode();
        is_close_to_bias = false;
        if(b_use_expl_bias_ && is_exist_path)
        {
            is_close_to_bias = checkCloseToBias();
        }

        ros::Duration elapsed_time = ros::Time::now() - time_start;
        if (elapsed_time.toSec() > kPlanningTimeoutSec)
        {
            ROS_WARN("ExplorationPlanner::planning() - timeout **********************");
            is_timeout = true;
        }
    }

#ifdef VERBOSE
    //std::cout << "ExplorationPlanner::planning() - found path: " << is_found_path << ", exist path: " << is_exist_path << std::endl;
    if (b_abort_) std::cout << "ExplorationPlanner::planning() - planning  aborted" << std::endl;
#endif
           
    std::vector<Node<ExplorationTree::StateVec>*>& frontierNodes = p_search_tree_->getFrontierNodes();       
    p_frontier_tree_->updateFrontierNodes();      
    p_frontier_tree_->addFrontierNodes(frontierNodes);    
    p_frontier_tree_->clusterFrontiers();  
    
    p_nav_tree_->setInputKdtree(p_frontier_tree_->getKdTree());  // set the updated kdtree from the frontier tree  
    //p_nav_tree_->setExtensionDistance(p_frontier_tree_->getMaxNeighborDistance()*1.05); // let's give it a margin 
    p_nav_tree_->initialize();       
    p_nav_tree_->expand();
    std::cout << "#nodes in frontier tree: " << p_frontier_tree_->getNumNodes() << std::endl;
    
    p_frontier_tree_->publishTree();      
    p_expl_tree_->publishTree();    
    p_nav_tree_->publishTree();    
        
    std::vector<geometry_msgs::Pose> tree_path;

    // check if there is a minimum information gain 
    if(p_search_tree_->gainFound())  
    {
        ROS_INFO_STREAM("ExplorationPlanner::planning() - gain: "  << p_search_tree_->getBestGain());
        
        // Extract the best edges.
        //tree_path = p_search_tree_->getBestEdge(params_.navigationFrame_);        
        //tree_path = p_search_tree_->getBestEdges(params_.navigationFrame_,params_.numEdgeSteps_);
        tree_path = p_search_tree_->getBestEdgesWithinRange(params_.navigationFrame_,params_.explStep_);
        
        p_search_tree_->memorizeBestBranch();       
#ifdef VERBOSE
        std::cout << "ExplorationPlanner::planning() - forwarding" << std::endl;
#endif           
        explorationStepType_ = kForwarding; 
    }
    else
    {
        p_search_tree_->resetBestBranch();   
        
        if (bEnableBacktracking)
        {                        
#if 0    
            // Return to previous point on search tree manager
            /*tree_path =*/ p_search_tree_->getPathBackToPrevious(params_.navigationFrame_); 
            
            // actually take the backtrack path on the exploration tree             
            tree_path = p_expl_tree_->getPathBackToPrevious();
#else
            // select the best node on the frontier tree 
            tree_path = p_frontier_tree_->getPathToBestFrontierCentroid(p_nav_tree_, robot_position_);
            
            // in order to avoid a long journey to a possible far backtracking node, cut down the maximum traveled distance in order to check again information gain
            
#if 0            
            // N.B.: this is problematic: all the poses of the tree_path may be too far 
            std::vector<geometry_msgs::Pose> tree_path_cut;
            cutPathWithinRange(tree_path, robot_position_, params_.explStepBacktracking_, tree_path_cut);    
            tree_path = tree_path_cut;
#endif 
            
#endif 
            
            if( tree_path.empty() )
            {
#ifdef VERBOSE                
                std::cout << "ExplorationPlanner::planning() - no information!" << std::endl;
#endif 
                explorationStepType_ = kNoInformation;                
            }
            else
            {
#ifdef VERBOSE
                std::cout << "ExplorationPlanner::planning() - backtracking" << std::endl;
#endif                      
                explorationStepType_ = kBacktracking;
            }
        }
    }

    // build path:
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    
    is_found_path = !tree_path.empty();
    if(is_found_path)
    {
        pose.pose = tree_path.front();
        path_out.poses.push_back(pose);
        pose.pose = tree_path.back();
        path_out.poses.push_back(pose);       
        
        /*if( explorationStepType_ == kForwarding ) 
        {
            Node<SimpleTree::StateVec>* selectedNode = p_search_tree_->getSelectedNode();
            if(selectedNode) 
            {
                p_expl_tree_->addNode(selectedNode);
                n_expl_nodes_++;
            }
        }*/        
    }
        
#ifdef VERBOSE
    std::cout << "ExplorationPlanner::planning() - is_found_path: " << is_found_path << std::endl;
#endif       
    
    explPathPub_.publish(path_out);
    
    return is_found_path;
}

/// static functions 

int ExplorationPlanner::getClosestNodeIdx(pcl::PointXYZI& position, KdTreeFLANN& kdtree, float radius)
{
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    // k-nearest neighborhood 
    int found = kdtree.nearestKSearch(position, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

    // radius search 
    //int found = kdtree.radiusSearch(position, radius, pointIdxNKNSearch, pointNKNSquaredDistance);

    if (found < 1)
    {
        ROS_WARN("ExplorationPlanner::get_closest_node_index() - cannot find a close node ");
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

void ExplorationPlanner::computeZMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in)
{
    if (pcl_in.empty())
    {
        ROS_ERROR("ExplorationPlanner::computeZMinMaxRange() - empty pcl as input");
        return; /// < EXIT POINT 
    }

    pcl::PointCloud<pcl::PointXYZI>::const_iterator min_elem_it = std::min_element(pcl_in.begin(), pcl_in.end(), compareZ);
    pcl::PointCloud<pcl::PointXYZI>::const_iterator max_elem_it = std::max_element(pcl_in.begin(), pcl_in.end(), compareZ);

    min_in = (*min_elem_it).z;
    max_in = (*max_elem_it).z;
    range_in = max_in - min_in;

    std::cout << "ExplorationPlanner::computeZMinMaxRange() - min: " << min_in << ", max: " << max_in << std::endl;
}

bool compareI(const pcl::PointXYZI& a, const pcl::PointXYZI& b)
{
    return a.intensity < b.intensity;
}

void ExplorationPlanner::computeIMinMaxRange(const pcl::PointCloud<pcl::PointXYZI>& pcl_in, float& min_in, float& max_in, float& range_in)
{
    pcl::PointCloud<pcl::PointXYZI>::const_iterator min_elem_it = std::min_element(pcl_in.begin(), pcl_in.end(), compareI);
    pcl::PointCloud<pcl::PointXYZI>::const_iterator max_elem_it = std::max_element(pcl_in.begin(), pcl_in.end(), compareI);

    min_in = (*min_elem_it).intensity;
    max_in = (*max_elem_it).intensity;
    range_in = max_in - min_in;


    std::cout << "ExplorationPlanner::computeIMinMaxRange() - min: " << min_in << ", max: " << max_in << std::endl;
}

void ExplorationPlanner::cutPathWithinRange(const std::vector<geometry_msgs::Pose>& path_in, const Eigen::Vector3d& robot_position, const double range, std::vector<geometry_msgs::Pose>& path_out)
{
    std::cout << "ExplorationPlanner::cutPathWithinRange - cutting backtracking path" << std::endl; 
    path_out.clear();
    
    const double range2 = range*range;
    
    for(size_t ii=0, iiEnd=path_in.size(); ii<iiEnd; ii++)
    {
        const geometry_msgs::Point& path_point = path_in[ii].position;
        double distance2 = dist2(robot_position[0]-path_point.x, robot_position[1]-path_point.y, robot_position[2]-path_point.z);
        if (distance2 < range2) path_out.push_back(path_in[ii]);  
    }
    
}

void ExplorationPlanner::getMapSyncMessage(geometry_msgs::PoseArray& message)
{
    p_scan_history_manager_->getPoseArrayMessage(message);
}

void ExplorationPlanner::mapMessageOverlapCheck(const geometry_msgs::PoseArray& message, std::vector<sensor_msgs::PointCloud2::Ptr>& cloudsToSend)
{
    p_scan_history_manager_->mapMessageOverlapCheck(message, cloudsToSend);
}

} // namespace explplanner
