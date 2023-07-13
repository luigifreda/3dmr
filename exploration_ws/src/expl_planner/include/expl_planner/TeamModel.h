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

#ifndef TEAM_MODEL_H_
#define TEAM_MODEL_H_

#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <boost/thread/recursive_mutex.hpp>

#include <exploration_msgs/ExplorationRobotMessage.h>


///\class RobotPlanningState 
///\brief 
///\author Luigi Freda
class RobotPlanningState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public: 
    RobotPlanningState();
    
    void Reset();
    
public:  // planning information 
    Eigen::Vector3d goal;    
    nav_msgs::Path path; 
    double path_length;  // nav cost 
    ros::Time timestamp;
    
    bool bNoInformation;     
    bool bCompleted; 
    
public: // deployment information 
    Eigen::Vector3d position;  // current robot position   
    bool bValidPosition;       // position info is valid?
    ros::Time timestampPosition;            
};


///\class TeamModel 
///\brief 
///\author Luigi Freda
class TeamModel
{
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const double kExpirationTime; // [sec]
    static const double kNodeConflictDistance; // [m]
    static const int kNumRobotsDefault;

public:

    TeamModel();
    
    // update team model 
    void Update(); 
    
    // check expired states 
    void CleanExpiredData();
    
    bool IsNodeConflict();
    
public: // setters 
    
    void setMyRobotId(const int& my_robot_id) { my_robot_id_ = my_robot_id; }
    
    void setNumRobots(const int& num_robots);
    
    void setRobotPlanData(const int& robot_id, const Eigen::Vector3d& goal, const ros::Time& timestamp);
    void setRobotPlanData(const int& robot_id, const Eigen::Vector3d& goal, const nav_msgs::Path& path, const double path_cost, const ros::Time& timestamp);
    
    void setRobotPositionData(const int& robot_id, const Eigen::Vector3d& position, const ros::Time& timestamp);    
    
    void setRobotMessage(const exploration_msgs::ExplorationRobotMessage::ConstPtr msg, bool isMsgDirect = false);
    
    void setConflictDistance(double distance) { conflict_distance_ = distance; }

public: 
    
    Eigen::Vector3d getMyGoal(); // get my goal 
    nav_msgs::Path getMyPath();    
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getConflictingNodes();
    double getConflictingDistance();
    
    bool isValid(const int& robot_id);
    
protected:
    
    void checkForRobotId(int robot_id); 
    
    double computePathLength(const nav_msgs::Path& path);

protected:

    boost::recursive_mutex mutex_; 

    std::vector<RobotPlanningState, Eigen::aligned_allocator<RobotPlanningState> > robot_states_;
    int num_robots_;
    int my_robot_id_; 
    
    double conflict_distance_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > conflicting_nodes_; 
};


#endif // TEAM_MODEL_H_
