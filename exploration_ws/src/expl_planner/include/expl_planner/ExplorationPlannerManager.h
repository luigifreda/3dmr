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

#ifndef EXPLORATION_PLANNER_MANAGER_H_
#define EXPLORATION_PLANNER_MANAGER_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/filters/crop_box.h>
#include <std_msgs/Bool.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/core/noncopyable.hpp>

#include <path_planner/Transform.h>

#include <exploration_msgs/ExplorationRobotMessage.h>

#include "ExplorationPlanner.h"
#include "PriorityQueue.h"
#include "TeamModel.h"

namespace explplanner{


///	\class ExplorationPlannerManager
///	\author Luigi Freda
///	\brief Path planner for a single segment (start point,goal point)
///	\note 
/// \todo 
///	\date
///	\warning
class ExplorationPlannerManager: private boost::noncopyable
{
public:

    static const int kExplPlannerMaxNumAttempts; // max number of times path planner try to compute the path 
    
    static const float kMinXSizeCropBox; // [m] minimum size along X axis of the crop box for planning 
    static const float kMinYSizeCropBox; // [m] minimum size along Y axis of the crop box for planning 
    static const float kMinZSizeCropBox; // [m] minimum size along Z axis of the crop box for planning 
    static const float kGainCropBox; 
    static const float kGainCropBox2;
    static const float kGainCropBox3;
    static const float kGainCropBox4;      
    
    static const float kGainXCropBoxPathAligned; // crop box extends from (-kGainXCropBoxPathAligned* delta.norm(), -0.5*kSizeYCropBoxPathAligned, -0.5*kSizeZCropBoxPathAligned, 1) to (kGainXCropBoxPathAligned * delta.norm(), 0.5*kSizeYCropBoxPathAligned, 0.5*kSizeZCropBoxPathAligned, 1)
    static const float kSizeYCropBoxPathAligned; // 
    static const float kSizeZCropBoxPathAligned; // 
    
    static const double kDistanceThForRemovingPriorityPoint; // [m]
    
    //static const float kTaskCallbackPeriod;  // [s] the duration period of the task callback 
    
    enum CropBoxMethod
    {
        //kCropBoxMethodPathAligned = 0, /*crop box in path-aligned frame*/
        kCropBoxMethodWorldAligned = 0,  /*crop box in world-aligned frame*/
        kCropBoxMethodWorldAligned2,     /*crop box in a larger world-aligned frame*/     
        kCropBoxMethodWorldAligned3,     /*crop box in a larger world-aligned frame*/
        kCropBoxMethodWorldAligned4,     /*crop box in a larger world-aligned frame*/                  
        kNumCropBoxMethod,               // after this the enums are disabled 
        kCropBoxTakeAll,             
        //kCropBoxTakeAll2,/*do not crop, take all - second chance */                
    };
    
    
    enum ExplorationPlannerStatus
    {
        kNone = 0,
        kNotReady,
        kInputFailure,
        kFailure,
        kSuccess,
        kAborted,
        kNoInformation,
        kCompleted,
        kTransformFailure,
    };

public:
    ExplorationPlannerManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~ExplorationPlannerManager();

public:
    void traversabilityCloudCallback(const sensor_msgs::PointCloud2& traversability_msg);
    void wallCloudCallback(const sensor_msgs::PointCloud2& wall_msg);
    void utility2DCloudCallback(const sensor_msgs::PointCloud2& utility_msg);
    
    //void goalSelectionCallback(geometry_msgs::PoseStamped goal_msg);
    void abortCallback(std_msgs::Bool msg);
    
    // compute path
    ExplorationPlannerStatus doPlanning();
    
    void insertOtherUgvPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
    
    void insertPriorityPoint(int id, const geometry_msgs::Point& point, int priority);
    void removePriorityPoint(int id);    
    void clearPriorityPointQueue(); 
    
public: /// < setters 
    
    
   // set the abort flag for possibly aborting current planning 
    void setAbort(bool val) 
    { 
        b_abort_ = val; 
        if(p_expl_planner_) 
        {
            p_expl_planner_->setAbort(val);
            expl_planning_status_ = kAborted; 
        }
    }
    
    // set (parent) map frame and (child) robot frame
    void setFramesRobot(const std::string& parent_map, const std::string& child_robot)
    {
        transform_robot_.set(parent_map,child_robot);
    }
    
    void setParamExplorationBoxXY(const double minX, const double maxX, const double minY, const double maxY);    
    
    void setNo2DUtility()
    {
        boost::recursive_mutex::scoped_lock wall_locker(utility_2d_mutex_);
        b_utility_2d_info_available_ = false;
    }
    
    void setCostFunctionType(int type, double lamda_trav = 1, double lambda_aux_utility = 1);
    
    void setMyRobotId(int my_robot_id);
    
    void setRobotMessage(const exploration_msgs::ExplorationRobotMessage::ConstPtr msg, bool isMsgDirect = false);
    
    void setRobotData(int robot_id, const Eigen::Vector3d& goal, const nav_msgs::Path& path, const double path_cost, const ros::Time& timestamp);
    
    void addSelectedNodeToExplorationTree() { p_expl_planner_->addSelectedNodeToExplorationTree(); }  
    void removeCurrentExplorationNode() { p_expl_planner_->removeCurrentExplorationNode(); }
    void backTrackOnExplorationTree() { p_expl_planner_->backTrackOnExplorationTree(); }    
    
    void manageConflict() { p_expl_planner_->manageConflict();}
    
    void cleanExpiredTeamData() { team_model_.CleanExpiredData(); }     
    
    void resetSelectedBackTrackingCluster() { p_expl_planner_->resetSelectedBackTrackingCluster(); }

    void mapMessageOverlapCheck(const geometry_msgs::PoseArray& message, std::vector<sensor_msgs::PointCloud2::Ptr>& cloudsToSend);
    
public: /// < getters 
    
    ExplorationPlannerStatus getPlanningStatus() const { return expl_planning_status_;} 
    
    bool isReady() const;
    
    nav_msgs::Path& getPath() {return path_;}
    double& getPathCost() {return path_cost_;}
    
    bool getRobotPosition(pcl::PointXYZI& robot_position); 
    
    ExplorationPlanner::ExplorationStepType getExplorationStepType() const { return p_expl_planner_->getExplorationStepType(); }    
    
    PriorityPointQueue& getPriorityQueue() {return priority_queue_; }
    
    std::vector<PriorityPointQueue::PriorityPoint>& getRemovedPriorityPoints() { return removed_priority_points_; }
    
    bool IsNodeConflict();
    
    Eigen::Vector3d getMyGoal() { return team_model_.getMyGoal(); }
    nav_msgs::Path getMyPath() { return team_model_.getMyPath(); }   
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getConflictingNodes() { return team_model_.getConflictingNodes(); }    
    double getConflictingDistance() { return team_model_.getConflictingDistance(); }   
        
    void getMapSyncMessage(geometry_msgs::PoseArray& message) { p_expl_planner_->getMapSyncMessage(message); }

public:
    
    void publishPlannedExplorationNode() { p_expl_planner_->publishPlannedExplorationNode(); }   
    
protected:    
   
    static void cropPcl(const CropBoxMethod& crop_box_method, const pcl::PointXYZI& start, 
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in, 
                        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out);
    static bool cropTwoPcls(const CropBoxMethod& crop_box_method, const pcl::PointXYZI& start, 
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in, 
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in2,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out, 
                            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out2);
    
protected:
    
    double computePathLength(nav_msgs::Path& path);
    
protected: 
    
    //ROS node handle
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;    
    
    volatile bool b_wall_info_available_;
    volatile bool b_traversability_info_available_;
    volatile bool b_utility_2d_info_available_;
        
    volatile bool b_abort_; // true if we want to abort it 
    
    Transform transform_robot_;
    tf::StampedTransform robot_pose_;
    
    nav_msgs::Path path_; // last planned path 
    double path_cost_; // cost of the last planned path 

    boost::recursive_mutex wall_mutex_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wall_pcl_;
    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> wall_kdtree_;

    boost::recursive_mutex traversability_mutex_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr traversability_pcl_;
    
    boost::recursive_mutex utility_2d_mutex_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr utility_2d_pcl_; // [x,y,u(x,y),var(x,y)] we assume that utility_2d_pcl_ contains info about the points in traversability_pcl_
    
    boost::shared_ptr<ExplorationPlanner> p_expl_planner_; // the used path planner instance
    boost::recursive_mutex expl_planner_mutex_;

    BaseCostFunction::CostFunctionType cost_function_type_; 
    
    ExplorationPlannerStatus expl_planning_status_; 
    
    PriorityPointQueue priority_queue_; // this is thread-safe
    std::vector<PriorityPointQueue::PriorityPoint> removed_priority_points_;
    
    TeamModel team_model_;
};


}

#endif //EXPLORATION_PLANNER_MANAGER_H_
