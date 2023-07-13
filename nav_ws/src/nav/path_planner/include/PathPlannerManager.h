/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> (La Sapienza University)
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

#ifndef PATH_PLANNING_MANAGER_H_
#define PATH_PLANNING_MANAGER_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include "PathPlanner.h"  // stay before any pcl include, it contains PCL_NO_PRECOMPILE directive 

#include <pcl/filters/crop_box.h>
#include <std_msgs/Bool.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "Transform.h"
#include "MarkerController.h"

///	\class PathPlannerManager
///	\author Luigi Freda
///	\brief Path planner for a single segment (start point,goal point)
///	\note 
/// 	\todo 
///	\date
///	\warning
class PathPlannerManager
{
public:

    static const int kPathPlannerMaxNumAttempts = 10; // max number of times path planner try to compute the path 
    static const float k3DDistanceArrivedToGoal; // [m]
    static const float k2DDistanceArrivedToGoal; // [m]
    static const float k3DDistanceCloseToGoal;// [m]
    
    static const float kMinXSizeCropBox; // [m] minimum size along X axis of the crop box for planning 
    static const float kMinYSizeCropBox; // [m] minimum size along Y axis of the crop box for planning 
    static const float kMinZSizeCropBox; // [m] minimum size along Z axis of the crop box for planning 
    static const float kGainCropBox; 
    static const float kGainCropBox2;
    
    static const float kGainXCropBoxPathAligned; // crop box extends from (-kGainXCropBoxPathAligned* delta.norm(), -0.5*kSizeYCropBoxPathAligned, -0.5*kSizeZCropBoxPathAligned, 1) to (kGainXCropBoxPathAligned * delta.norm(), 0.5*kSizeYCropBoxPathAligned, 0.5*kSizeZCropBoxPathAligned, 1)
    static const float kSizeYCropBoxPathAligned; // 
    static const float kSizeZCropBoxPathAligned; // 
    
    //static const float kTaskCallbackPeriod;  // [s] the duration period of the task callback 
    
    enum CropBoxMethod
    {
        kCropBoxMethodPathAligned = 0, /*crop box in path-aligned frame*/
        kCropBoxMethodWorldAligned, /*crop box in world-aligned frame*/
        kCropBoxMethodWorldAligned2, /*crop box in a larger world-aligned frame*/
        kCropBoxTakeAll, /*do not crop, take all*/
        //kCropBoxTakeAll2,/*do not crop, take all - second chance */
        kNumCropBoxMethod
    };
    
    // N.B.: this must be kept in synch with trajectory_control_msgs::PlanningStatus
    enum PlannerStatus
    {
        kNone             = 0,
        kNotReady         = 1,
        kInputFailure     = 2,
        kFailure          = 3,
        kSuccess          = 4,
        kArrived          = 5,
        kAborted          = 6,
        kTransformFailure = 7,
        kFirstSuccess     = 8
    };

    static const std::string kPlannerStatusStrings[]; 

public:
    PathPlannerManager();
    ~PathPlannerManager();

public:
    void traversabilityCloudCallback(const sensor_msgs::PointCloud2& traversability_msg);
    void setTraversabilityCloud(const pcl::PointCloud<pcl::PointXYZI>& traversability_pcl_in);
    
    void wallCloudCallback(const sensor_msgs::PointCloud2& wall_msg);
    void utility2DCloudCallback(const sensor_msgs::PointCloud2& utility_msg);
    
    void goalSelectionCallback(geometry_msgs::PoseStamped goal_msg);
    void goalAbortCallback(std_msgs::Bool msg);
    
    PlannerStatus pathPlanningServiceCallback(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped end, nav_msgs::Path& path, double& path_cost);

    // compute path
    PlannerStatus doPathPlanning();
    
    bool checkIfWeArrived();    
    
public: /// < setters 
    
    void setGoal(const pcl::PointXYZI& goal)
    {
        boost::recursive_mutex::scoped_lock goal_locker(goal_mutex_);
        goal_position_ = goal;
        if(!b_goal_selected_) b_goal_selected_ = true; 
        b_is_close_to_goal_ = false; 
        b_found_a_solution_once_ = false;
        std::cout << "PathPlannerManager::setGoal() - goal: (" << goal_position_.x << "," << goal_position_.y << "," << goal_position_.z << ")" << std::endl;
    }
    
    void setNoGoal()
    {
        boost::recursive_mutex::scoped_lock goal_locker(goal_mutex_);
        if(b_goal_selected_) b_goal_selected_ = false;   
        b_is_close_to_goal_ = false; 
        b_found_a_solution_once_ = false; 
    }
    
   // set the abort flag for possibly aborting current planning 
    void setAbort(bool val) 
    { 
        b_abort_ = val; 
        if(p_path_planner_) 
        {
            p_path_planner_->setAbort(val);
            planning_status_ = kAborted; 
        }
        if(val) setNoGoal();
    }
    
    // set (parent) map frame and (child) robot frame
    void setFramesRobot(const std::string& parent_map, const std::string& child_robot)
    {
        transform_robot_.set(parent_map,child_robot);
    }
    
    void setNo2DUtility()
    {
        boost::recursive_mutex::scoped_lock wall_locker(utility_2d_mutex_);
        b_utility_2d_info_available_ = false;
    }
    
    void setCostFunctionType(int type, double lamda_trav = 1, double lambda_aux_utility = 1);
    
public: /// < getters 
    
    PlannerStatus getPlanningStatus() const { return planning_status_;} 
    
    bool isReady() const;
    bool isReadyForService() const; 
    
    bool isSetGoal() const { return b_goal_selected_; }
    pcl::PointXYZI& getGoal() { return goal_position_;} 
    
    nav_msgs::Path& getPath() {return path_;}
    double& getPathCost() {return path_cost_;}
    
    bool getRobotPosition(pcl::PointXYZI& robot_position); 
    
    bool isCloseToGoal() const {return b_is_close_to_goal_;}
    
    bool isSolutionFoundOnce() const { return b_found_a_solution_once_; }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr GetTraversabilityPcl() { return traversability_pcl_;}
        
public:
   
    static void cropPcl(const CropBoxMethod& crop_box_method, 
                        const pcl::PointXYZI& start, const pcl::PointXYZI& goal, 
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in, 
                        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out);
    static bool cropTwoPcls(const CropBoxMethod& crop_box_method, 
                            const pcl::PointXYZI& start, const pcl::PointXYZI& goal, 
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in, 
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in2,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out, 
                            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out2);
    
protected:
    
    double computePathLength(nav_msgs::Path& path);
    
protected: 
    
    volatile bool b_wall_info_available_;
    volatile bool b_traversability_info_available_;
    volatile bool b_utility_2d_info_available_;
        
    volatile bool b_abort_; // true if we want to abort it 
    volatile bool b_goal_selected_;
    volatile bool b_is_close_to_goal_; // true when close to the goal 
    volatile bool b_found_a_solution_once_; // for this goal. did you find a solution at least once?
    
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
    
    boost::recursive_mutex goal_mutex_;
    pcl::PointXYZI goal_position_;

    boost::shared_ptr<PathPlanner> p_path_planner_; // the used path planner instance
    boost::recursive_mutex path_planner_mutex_;

    BaseCostFunction::CostFunctionType cost_function_type_; 
    
    PlannerStatus planning_status_; 
    
};


#endif //PATH_PLANNING_MANAGER_H_
