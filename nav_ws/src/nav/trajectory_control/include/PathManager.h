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

#pragma once 

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include <boost/thread/recursive_mutex.hpp>

#include <path_planner/KdTreeFLANN.h>

#include "SignalUtils.h"
#include "VelRamp.h"
#include "PathSmoother.h"

///	\class PathManager
///	\author Luigi Freda
///	\brief 
///	\note
/// \todo 
///	\date
///	\warning
class PathManager
{
public:
 
    typedef pcl::PointXYZL PointNode;
    static const double kDistanceToRecomputeIndex;

public:
    
    PathManager();
    virtual ~PathManager();

    /// Init
    virtual void init(double Ts, double vel, double rise_time, const nav_msgs::Path& path_in, const PathSmoother::PathSmootherType& smoother_type=PathSmoother::kNoSmoother);

    /// basic step for generating next ref point 
    /// return true when done 
    virtual bool step(const double current_time);
    
    
    void sendMarkers(); 

public: // setters 
    
    void setTs(double Ts) { d_Ts_ = Ts; }
    void setVel(double vel) { d_vel_lin_ = vel; }
    
    void setRobotPosition(double x, double y, double z);
    
public: // getters
    
    nav_msgs::Path& getPathOut() {return path_out_; }
    nav_msgs::Path& getPathIn() {return path_in_; }
    
    double getEstimatedDistance() const { return d_estimated_distance_;}
    double getEstimatedTime() const { return d_estimated_time_;}
    
    bool isPathEnd() const {return b_end_; }
    
    const geometry_msgs::PoseStamped& getCurrentPose() const {return current_pose_;}
    
    const geometry_msgs::PoseStamped& getFinalPose() const {return final_pose_;}
    
    double getLinearVel() const {return d_vel_lin_; }
    double getAngularVel() const {return d_vel_ang_;}

    double getCurrentVel() const {return d_vel_curr_; }
    
    double getCurrentYaw() const {return d_yaw_last_;}

protected:
    
    boost::recursive_mutex mutex_;    
    
    bool b_init_; // has object been initialized?
    bool b_end_; // is path ended? 
    bool b_jump_point_; // 
    
    double d_Ts_; // nominal time step
    double d_vel_lin_; // nominal linear velocity 
    double d_vel_ang_; // reference angular velocity (approximated numerically)  
    double d_yaw_last_; // last yaw
    
    double d_estimated_time_; // nominal estimated time 
    double d_estimated_distance_; // nominal estimated distance 
   
    nav_msgs::Path path_in_; // input path (smoothed if required)
    
    nav_msgs::Path path_out_; // output path 
    
    geometry_msgs::PoseStamped current_pose_; // current pose 
    geometry_msgs::PoseStamped jumped_pose_; // jumped pose during the resampling  
    geometry_msgs::PoseStamped final_pose_; // final pose 
    
    PointNode robot_position_;  
    
    size_t i_index_; // index of the current pose 
    
    double d_step_offset_; // step offset used for generating new points on the trajectory 
    
    PathSmoother::PathSmootherType smoother_type_;
    PathSmoother smoother_; 

    VelRamp vel_ramp_; 
    double d_vel_curr_; // nominal linear velocity 

protected:
    
    //Ros node handle
    ros::NodeHandle node_;      
};


///	\class PathManagerKdt
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning
class PathManagerKdt: public PathManager
{
public:
    
    typedef pcl::PointCloud<PointNode> PointCloudNodes;
    typedef pp::KdTreeFLANN<PointNode> KdTreeNodes;  
    
public:
    
    PathManagerKdt(){}
    
    /// Init
    void init(double Ts, double vel, double rise_time, const nav_msgs::Path& path_in);

    /// basic step for generating next ref point 
    /// return true when done 
    bool step(const double current_time);
    
protected: 
      
    PointCloudNodes plc_points_;
    KdTreeNodes     kdtree_points_;         
};
