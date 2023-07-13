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

#include "QueuePathPlanner.h"

const float QueuePathPlanner::kMinXSizeCropBox = 4; // [m] minimum size along X axis of the crop box for planning 
const float QueuePathPlanner::kMinYSizeCropBox = 4; // [m] minimum size along Y axis of the crop box for planning 
const float QueuePathPlanner::kMinZSizeCropBox = 4; // [m] minimum size along Z axis of the crop box for planning 
const float QueuePathPlanner::kGainCropBox = 1.5; 
const float QueuePathPlanner::kGainCropBox2 = 3; 

const float QueuePathPlanner::kGainXCropBoxPathAligned = 1; // crop box extends from (-kGainXCropBoxPathAligned* delta.norm(), -0.5*kSizeYCropBoxPathAligned, -0.5*kSizeZCropBoxPathAligned, 1) to (kGainXCropBoxPathAligned * delta.norm(), 0.5*kSizeYCropBoxPathAligned, 0.5*kSizeZCropBoxPathAligned, 1)
const float QueuePathPlanner::kSizeYCropBoxPathAligned = 3; // 
const float QueuePathPlanner::kSizeZCropBoxPathAligned = 2; // 
    
const float QueuePathPlanner::kTaskCallbackPeriod = 0.5;  // [s] the period of the task callback 
    

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
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


QueuePathPlanner::~QueuePathPlanner(void)
{
    trajectory_control_msgs::PlanningTask task_msg;

    // remove/stop all planning tasks from the planner node queue
    task_msg.name = "ALL";
    task_msg.header.stamp = ros::Time::now();
    task_remove_pub_.publish(task_msg);
}

QueuePathPlanner::QueuePathPlanner(void)
{
    traversability_pcl_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    utility_2d_pcl_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    wall_pcl_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        
    crop_box_method_ = PathPlannerManager::kCropBoxMethodPathAligned; // default method 
    //crop_box_method_ = kCropBoxMethodWorldAligned; // default method 
    
    // path planner library stuff:
    wall_flag_ = false;
    traversability_flag_ = false;
    utility_2d_flag_ = false;
            
    // ros stuff
    param_node_ = ros::NodeHandle("~");
    //node_ = ros::NodeHandle("~");
    
    reference_frame_ = "map";
    
    //robot_frame_ = "/base_link";
    robot_frame_ = getParam<std::string>(param_node_, "robot_frame_name", "/base_link");
    
    cost_function_type_ = getParam<int>(param_node_, "cost_function_type", (int)BaseCostFunction::kSimpleCost);
    lambda_trav_ = getParam<float>(param_node_, "lambda_trav", 1.0);
    lambda_utility_2d_ = getParam<float>(param_node_, "lambda_utility_2d", 1.); 
    tau_exp_decay_ = getParam<float>(param_node_, "tau_exp_decay", std::numeric_limits<float>::max());
    
    // ros stuff: path planner library stuff
    traversability_sub_ = node_.subscribe("/trav/traversability", 1, &QueuePathPlanner::traversabilityCallback, this);
    wall_sub_ = node_.subscribe("/clustered_pcl/wall", 1, &QueuePathPlanner::wallCallback, this);

    controller_ready_flag_ = true; // we moved it here: this will tell the taskTimerCallback() that the controller is ready 
    
    // ros stuff: tasks handling
    task_path_pub_ = node_.advertise<nav_msgs::Path>("/planner/tasks/path", 1);
    task_global_path_pub_ = node_.advertise<trajectory_control_msgs::PlanningGlobalPath>("/planner/tasks/global_path", 1); 
    task_timer_ = node_.createTimer(ros::Duration(kTaskCallbackPeriod), &QueuePathPlanner::taskTimerCallback, this); // the timer will automatically fire at startup 

    // ros stuff: tasks feedback handling
    //task_feedback_flag_ = true;
    task_feedback_pub_ = node_.advertise<trajectory_control_msgs::PlanningFeedback>("/planner/tasks/feedback", 20);
    task_feedback_sub_ = node_.subscribe("/planner/tasks/feedback", 20, &QueuePathPlanner::feedbackCallback, this);

    task_append_sub_ = node_.subscribe("/planner/tasks/append", 1, &QueuePathPlanner::appendCallback, this);
    
    task_remove_sub_ = node_.subscribe("/planner/tasks/remove", 1, &QueuePathPlanner::removeCallback, this);
    task_remove_pub_ = node_.advertise<trajectory_control_msgs::PlanningTask>("/planner/tasks/remove",1);
    
    
    utility_2d_sub_ = node_.subscribe("/planner/utility_2d", 1, &QueuePathPlanner::utility2DCloudCallback, this);
    
    // cloud cropping visualization
#ifdef VERBOSE
    cropbox_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/planner/waypoints/cropbox", 1);
#endif

}

void QueuePathPlanner::traversabilityCallback(const sensor_msgs::PointCloud2& traversability_msg)
{
    boost::recursive_mutex::scoped_lock locker_utility(utility_2d_mutex_); 
    boost::recursive_mutex::scoped_lock locker_traversability(traversability_pcl_mutex_); 
        
    // cloud having traversability labels
    pcl::fromROSMsg(traversability_msg, *traversability_pcl_);

    traversability_flag_ = true;
    utility_2d_flag_     = false; // reset utility 
    
    // enable path planning only if the cloud is not empty
    if (traversability_pcl_->size() > 0)
    {
        traversability_flag_ = true;
    }
}

void QueuePathPlanner::wallCallback(const sensor_msgs::PointCloud2& wall_msg)
{
    boost::recursive_mutex::scoped_lock locker(wall_pcl_mutex_); 
        
    // cloud of the segmented walls of the scene
    pcl::fromROSMsg(wall_msg, *wall_pcl_);
    
    // this cloud can be empty
    if(wall_pcl_->size() == 0)
    {
        /// < HACK: add a single far point 
        pcl::PointXYZRGBNormal far_point;
        far_point.x = std::numeric_limits<float>::max(); 
        far_point.y = std::numeric_limits<float>::max(); 
        far_point.z = std::numeric_limits<float>::max(); 
        wall_pcl_->push_back(far_point);
    }

    wall_flag_ = true;
}

void QueuePathPlanner::utility2DCloudCallback(const sensor_msgs::PointCloud2& utility_msg)
{
    boost::recursive_mutex::scoped_lock locker_utility(utility_2d_mutex_); 
    boost::recursive_mutex::scoped_lock locker_traversability(traversability_pcl_mutex_); 
        
    // cloud of an aux utility 2d
    pcl::fromROSMsg(utility_msg, *utility_2d_pcl_);
    
    utility_2d_flag_ = false;  
    
    /// < check if it has the same size of the traversability pcl 
    if( (utility_2d_pcl_->header.stamp != traversability_pcl_->header.stamp) || (utility_2d_pcl_->size() != traversability_pcl_->size()) )
    {
        ROS_WARN("**********************************************************************************************************");
        ROS_WARN("QueuePathPlanner::utility2DCloudCallback() - traversability and utility pcls have different sizes or stamp");
        ROS_WARN("**********************************************************************************************************");
        std::cout << "trav stamp: " << traversability_pcl_->header.stamp << ", utility stamp: " << utility_2d_pcl_->header.stamp << std::endl; 
        std::cout << "trav size: " << traversability_pcl_->size() << ", utility size: " << utility_2d_pcl_->size() << std::endl; 
        utility_2d_flag_ = false; 
        return; /// < EXIT POINT 
    }

    if (utility_2d_pcl_->size() > 0)
    {
        utility_2d_flag_ = true;
    }
  
}
