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

#include "TeamModel.h"

//    uint8 kNone            = 0
//    uint8 kReached         = 1
//    uint8 kPlanned         = 2  
//    uint8 kSelected        = 3 
//    uint8 kAborted         = 4
//    uint8 kPosition        = 5            # to share own position 
//    uint8 kNoInformation   = 6
//    uint8 kCompleted       = 7
const std::string kActionString[]={"None", "Reached", "Planned", "Selected", "Aborted", "Position", "No Information", "Completed"}; // keep this coherent with the content of ExplorationRobotMessage.msg in exploration_msgs

RobotPlanningState::RobotPlanningState():path_length(-1.),bValidPosition(false),bNoInformation(false),bCompleted(false)
{
}

void RobotPlanningState::Reset()
{
    path.poses.clear();
    path_length = -1.; 
    timestamp = ros::Time::now();
    
    bValidPosition = false; 
    timestampPosition = ros::Time::now();
    
    bNoInformation = false;
    bCompleted = false;
}


const double TeamModel::kExpirationTime = 10; // [sec]
const double TeamModel::kNodeConflictDistance = 3; // [m]
const int TeamModel::kNumRobotsDefault = 2;
    
TeamModel::TeamModel()
{
    my_robot_id_ = 0;
    num_robots_ = 0;
    conflict_distance_ = kNodeConflictDistance;
    setNumRobots(kNumRobotsDefault);
}

void TeamModel::setNumRobots(const int& num_robots)
{
    std::cout << "setNumRobots" << std::endl; 
    boost::recursive_mutex::scoped_lock locker(mutex_);
    num_robots_ = num_robots;
    robot_states_.resize(num_robots);
}

// we assume robot_id are zero-based 
void TeamModel::checkForRobotId(int robot_id)
{
    //std::cout << "checkForRobotId: in (" << robot_id << ")" << std::endl; 
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    if( (robot_id+1) >= num_robots_)
    {
        setNumRobots(robot_id+1);
    }    
    //std::cout << "checkForRobotId: out " << std::endl; 
}

void TeamModel::setRobotPlanData(const int& robot_id, const Eigen::Vector3d& goal, const ros::Time& timestamp)
{
    boost::recursive_mutex::scoped_lock locker(mutex_);
    checkForRobotId(robot_id);
    
    robot_states_[robot_id].goal = goal; 
    robot_states_[robot_id].path.poses.clear();
    //robot_states_[robot_id].path_length = -1;   
    robot_states_[robot_id].timestamp = timestamp; 
}
    
void TeamModel::setRobotPlanData(const int& robot_id, const Eigen::Vector3d& goal, const nav_msgs::Path& path, const double path_cost, const ros::Time& timestamp)
{
    boost::recursive_mutex::scoped_lock locker(mutex_);
    setRobotPlanData(robot_id, goal, timestamp);
            
    robot_states_[robot_id].path = path; 
    //robot_states_[robot_id].path_length = computePathLength(path);
    robot_states_[robot_id].path_length = path_cost;  // this path cost is set by the path planner feedback 
}

void TeamModel::setRobotPositionData(const int& robot_id, const Eigen::Vector3d& position, const ros::Time& timestamp)
{
    boost::recursive_mutex::scoped_lock locker(mutex_);
    checkForRobotId(robot_id);
    
    robot_states_[robot_id].position = position; 
    robot_states_[robot_id].bValidPosition = true;
    robot_states_[robot_id].timestampPosition = timestamp;     
}

double TeamModel::computePathLength(const nav_msgs::Path& path)
{
    double d_estimated_distance_ = 0;
    int input_path_length = path.poses.size();
    for (int ii = 0; ii < (input_path_length - 1); ii++)
    {
        d_estimated_distance_ += sqrt(pow(path.poses[ii + 1].pose.position.x - path.poses[ii].pose.position.x, 2) +
                                      pow(path.poses[ii + 1].pose.position.y - path.poses[ii].pose.position.y, 2) +
                                      pow(path.poses[ii + 1].pose.position.z - path.poses[ii].pose.position.z, 2));
    }
    return d_estimated_distance_;
}

void TeamModel::Update()
{
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    
    CleanExpiredData();
}

void TeamModel::CleanExpiredData()
{
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    
    ros::Time time = ros::Time::now();
    
    for(size_t jj=0; jj< robot_states_.size(); jj++)
    {
        ros::Duration elapsedTime = robot_states_[jj].timestamp - time;
        if(elapsedTime.toSec() > kExpirationTime)
        {
            robot_states_[jj].Reset(); 
        }
        
        ros::Duration elapsedTimePosition = robot_states_[jj].timestampPosition - time;
        if(elapsedTimePosition.toSec() > kExpirationTime)
        {
            robot_states_[jj].bValidPosition = false; 
        }        
    }
}


bool TeamModel::IsNodeConflict()
{
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    
    bool res = false;
    
    Eigen::Vector3d my_goal = robot_states_[my_robot_id_].goal;
    //Eigen::Vector3d my_position = robot_states_[my_robot_id_].position;    
    double my_path_length = robot_states_[my_robot_id_].path_length;
    
    if(my_path_length < 0) 
    {        
        return false;
    }
    
    conflicting_nodes_.clear();
    
    for(size_t jj=0; jj< robot_states_.size(); jj++)
    {    
        if(jj == my_robot_id_) continue; 
        
        std::cout << "TeamModel::IsNodeConflict() - robot " << jj << ": path_length: " << robot_states_[jj].path_length << std::endl;  
        
        // check my goal - teammate goal conflict 
        if(robot_states_[jj].path_length > -1)  // not invalid goal 
        {        
            double distance = (my_goal - robot_states_[jj].goal).norm(); 
            std::cout << "TeamModel::IsNodeConflict() - (my_id, other_id): (" << my_robot_id_ << ", " << jj << ") - goal-goal distance: " << distance << std::endl;        
            if( 
                (distance < conflict_distance_) &&
                (my_path_length > robot_states_[jj].path_length)
              )
            {
                std::cout << "TeamModel::IsNodeConflict() - conflict detected (my_id, other_id): (" << my_robot_id_ << ", " << jj << ")" << std::endl;       
                res = true;
                conflicting_nodes_.push_back(robot_states_[jj].goal); 
                //break; // N.B.: at the present time, we just take the first one and then we break the loop 
            }
        }
        
        // check my goal - teammate position conflict         
        if(robot_states_[jj].bValidPosition) // valid position received 
        {
            double distance = (my_goal - robot_states_[jj].position).norm(); 
            std::cout << "TeamModel::IsNodeConflict() - (my_id, other_id): (" << my_robot_id_ << ", " << jj << ") - goal-position distance: " << distance << std::endl;        
            if(distance < conflict_distance_)
            {
                std::cout << "TeamModel::IsNodeConflict() - conflict detected (my_id, other_id): (" << my_robot_id_ << ", " << jj << ")" << std::endl;       
                res = true;
                conflicting_nodes_.push_back(robot_states_[jj].position); 
                //break; // N.B.: at the present time, we just take the first one and then we break the loop 
            }            
        }
    }
    
    return res; 
}


void TeamModel::setRobotMessage(const exploration_msgs::ExplorationRobotMessage::ConstPtr msg, bool isMsgDirect )
{
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    
    // avoid setting message coming from my self through topic (these can arrive late with possible negative/contradicting effects)
    // TODO: one could even compare the incoming timestamp with the last stored one 
    if( (!isMsgDirect) && ( msg->robot_id == my_robot_id_ ) ) return; 
    
    ROS_INFO_STREAM("TeamModel::setRobotMessage() - id: " << (int)msg->robot_id << ", action: " << kActionString[msg->action]);
    
    checkForRobotId(msg->robot_id);

    //ros::Time timestamp = ros::Time::now();
    ros::Time timestamp = msg->header.stamp;
    robot_states_[msg->robot_id].timestamp = timestamp;
                
    switch(msg->action)
    {
    case exploration_msgs::ExplorationRobotMessage::kAborted:
        robot_states_[msg->robot_id].path_length = -1;
        break;
        
    case exploration_msgs::ExplorationRobotMessage::kPlanned:
        robot_states_[msg->robot_id].path_length = -1;
        break;
        
    case exploration_msgs::ExplorationRobotMessage::kReached:
        robot_states_[msg->robot_id].path_length = -1;
        break;
        
    case exploration_msgs::ExplorationRobotMessage::kSelected:
        robot_states_[msg->robot_id].goal[0] = msg->goal.x;
        robot_states_[msg->robot_id].goal[1] = msg->goal.y;
        robot_states_[msg->robot_id].goal[2] = msg->goal.z;    
        robot_states_[msg->robot_id].path = msg->path;             // path on the exploration tree as computed by the exploration planner  
        robot_states_[msg->robot_id].path_length = msg->path_cost; // navigation path cost as computed by the path planner            
        //robot_states_[msg->robot_id].path_length = computePathLength(msg->path);  
        ROS_INFO_STREAM("TeamModel::setRobotMessage() - path_length: " << robot_states_[msg->robot_id].path_length );
        break;  
        
    case exploration_msgs::ExplorationRobotMessage::kPosition:
        robot_states_[msg->robot_id].position[0] = msg->position.x;
        robot_states_[msg->robot_id].position[1] = msg->position.y;
        robot_states_[msg->robot_id].position[2] = msg->position.z;            
        robot_states_[msg->robot_id].bValidPosition = true;
        robot_states_[msg->robot_id].timestampPosition = timestamp;         
        break; 
  
    case exploration_msgs::ExplorationRobotMessage::kNoInformation:
        robot_states_[msg->robot_id].path_length = -1;        
        robot_states_[msg->robot_id].bNoInformation = true;        
        break; 
        
    case exploration_msgs::ExplorationRobotMessage::kCompleted:
        robot_states_[msg->robot_id].path_length = -1;        
        robot_states_[msg->robot_id].bCompleted = true;        
        break; 
        
    default:
        ROS_ERROR_STREAM("TeamModel::setRobotMessage() - unknown message type");
    }
}

Eigen::Vector3d TeamModel::getMyGoal()
{
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    return robot_states_[my_robot_id_].goal;
}

nav_msgs::Path TeamModel::getMyPath()
{
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    return robot_states_[my_robot_id_].path;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > TeamModel::getConflictingNodes()
{
    boost::recursive_mutex::scoped_lock locker(mutex_);    
    return conflicting_nodes_;    
}

double TeamModel::getConflictingDistance() 
{ 
    boost::recursive_mutex::scoped_lock locker(mutex_);     
    return conflict_distance_; 
}

    
bool TeamModel::isValid(const int& robot_id)
{
    boost::recursive_mutex::scoped_lock locker(mutex_);
    checkForRobotId(robot_id);
    
    return (robot_states_[robot_id].path_length != -1);
}    
    
