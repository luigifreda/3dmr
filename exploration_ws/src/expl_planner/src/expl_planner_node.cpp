
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

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 


#include <ros/ros.h>
#include <signal.h>

#include <limits>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/common/geometry.h>

#include <trajectory_control_msgs/PlanningFeedback.h>
#include <trajectory_control_msgs/PlanningTask.h>
#include <trajectory_control_msgs/PlanningGlobalPath.h>
#include <trajectory_control_msgs/PlanningStatus.h>
#include <trajectory_control_msgs/PathPlanning.h>

#include <exploration_msgs/ExplorationPriorityPoint.h>
#include <exploration_msgs/ExplorationPriorityActions.h>
#include <exploration_msgs/ExplorationRobotMessage.h>
#include <exploration_msgs/ExplorationPlanningStatus.h>

#include <path_planner/Transform.h>

#include "ExplorationMarkerController.h"
#include "ExplorationPlannerManager.h"


/// < PARAMETERS 


const double kExplorationTaskCallbackPeriod =  1.0;  // [s] the period of the task callback 
const double kMapSyncCallbackPeriod =  10.0;  // [s] the period of the task callback 

const double kSleepTimeAfterRotation = 3.0; // [s] sleep time after rotation towards best informed orientation (this is to give time to the traversability map to update with new perceived information)
const double kSleepTimeAfterAbort = 1.0; // [s] sleep time after abort
const double kMinExplBoundingBoxDimension = 1.0; // [m]
const int kOnCompleteExplorationSubrate = 15; 
const int kMaxNumConsecutiveBacktrackingFailures = 3; 
const double kMaxBacktrackingTime = 8.0; // [s] maximum time to complete backtracking (check the note below when it is used)

static const int kMaxNumberOfRobots = 16; 

/// < GLOBAL VARS (in this node)

boost::shared_ptr<explplanner::ExplorationPlannerManager> p_expl_planner_manager;
boost::recursive_mutex expl_planner_manager_mutex;

boost::shared_ptr<ExplorationMarkerController> p_marker_controller;

ros::Publisher expl_plan_stat_pub;

ros::Publisher goal_pub;
ros::Publisher goal_abort_pub;
//ros::Publisher trajectory_control_abort_pub;
ros::Publisher robot_rotation_pub;
ros::Publisher priority_actions_pub;

ros::Publisher exploration_messages_pub;
boost::recursive_mutex exploration_messages_pub_mutex;

ros::Publisher dynamic_cloud_router_pub;

ros::Publisher map_sync_pub;

ros::Publisher missing_point_cloud_pub[kMaxNumberOfRobots];
boost::recursive_mutex map_sync_messages_pub_mutex;

ros::Subscriber other_dynamic_point_cloud_sub[kMaxNumberOfRobots];

ros::Time time_last_path_msg = ros::TIME_MIN;

ros::Time time_backtracking_start = ros::TIME_MIN;

std::string str_robot_name;
std::string robot_frame_id;

ros::Timer expl_task_timer;
ros::Timer map_sync_timer;
    
boost::recursive_mutex expl_planner_ready_mutex;
volatile bool bExplPlannerReady = false;  // am I ready to plan a new goal?
volatile bool bPathToGoalCompleted = false;
bool bGoalRotationCompleted = false; 
volatile int goalState = exploration_msgs::ExplorationRobotMessage::kReached; 

int number_of_robots = 2; 
int robot_id         = 0;

boost::recursive_mutex expl_planner_paused_mutex;
volatile bool bExplPlannerPaused = false;

trajectory_control_msgs::PlanningStatus path_planner_status;
boost::recursive_mutex path_planner_status_mutex;
bool path_planner_status_message_updated; 

boost::recursive_mutex abort_mutex;

//ros::ServiceServer pathPlannerService;
ros::ServiceClient path_planner_service_client;

int numExplorationStep = 0; 
int numConsecutiveBacktrackingFailures = 0;

/// < declarations

void sendExplorationMessage(const int id, const int action, const Eigen::Vector3d& goal, const nav_msgs::Path& path, const double path_cost);
void sendExplorationMessage(const int id, const int action, const Eigen::Vector3d& position);
void sendExplorationMessage(const int id, const int action);

/// < functions 

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

void publishGoal(geometry_msgs::PoseStamped goal_pose)
{
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose.position.x = goal_pose.pose.position.x; 
    goal_msg.pose.position.y = goal_pose.pose.position.y; 
    goal_msg.pose.position.z = goal_pose.pose.position.z;

    geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);
    goal_msg.pose.orientation = angle_quat;
    goal_pub.publish(goal_msg);
}

void publishGoalAbort()
{
    boost::recursive_mutex::scoped_lock locker(abort_mutex);
    std_msgs::Bool msg_abort;
    msg_abort.data = true;
    goal_abort_pub.publish(msg_abort);
}

void publishRemovedPriorityPoints(const std::vector<PriorityPointQueue::PriorityPoint>& removed_priority_points)
{
    if(removed_priority_points.empty()) return; 
    
    exploration_msgs::ExplorationPriorityActions msg; 
    
//    # sender type
//    uint8 kSenderExplorer = 0
//    uint8 kSenderGUI      = 1
//
//    # A reference coordinate frame and timestamp
//    Header header
//
//    int32 num_points	                          # number of points
//    ExplorationPriorityPoint[] priority_points  # list of priority points with actions
//    uint8 sender                                # sender type
    
    msg.header.stamp = ros::Time::now();
    msg.sender = exploration_msgs::ExplorationPriorityActions::kSenderExplorer;
    msg.num_points = removed_priority_points.size();
   
    for(int ii=0; ii<removed_priority_points.size(); ii++)
    {
        exploration_msgs::ExplorationPriorityPoint priorityPoint;
        priorityPoint.id       = removed_priority_points[ii].id; 
        priorityPoint.position = removed_priority_points[ii].data;
        priorityPoint.priority = removed_priority_points[ii].priority;
        priorityPoint.action   = exploration_msgs::ExplorationPriorityPoint::kActionRemove;
        msg.priority_points.push_back(priorityPoint);
    }
    
    priority_actions_pub.publish(msg);
}


void setExplorationReady(bool val)
{
    boost::recursive_mutex::scoped_lock locker(expl_planner_ready_mutex);
    std::cout << "setExplorationReady() " << val << std::endl;    
    bExplPlannerReady = val;
}

bool isExplorationReady()
{
    boost::recursive_mutex::scoped_lock locker(expl_planner_ready_mutex);
    return bExplPlannerReady;
}


void setExplorationPaused(bool val)
{
    boost::recursive_mutex::scoped_lock locker(expl_planner_paused_mutex);
    std::cout << "setExplorationPaused() " << val << std::endl;    
    bExplPlannerPaused = val;
}

bool isExplorationPaused()
{
    boost::recursive_mutex::scoped_lock locker(expl_planner_paused_mutex);
    return bExplPlannerPaused;
}
    
bool checkConnectivityAndPathLengthOnTraversability(const Eigen::Vector3d& goal, const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& conflictingNodes, const double maxPathLength)
{
    std::cout << "checkConnectivityAndPathLength" << std::endl; 
           
    bool checkConflict = false;
    if(conflictingNodes.empty())
    {
        ROS_ERROR_STREAM("empty conflicting nodes!");
        return checkConflict;
    }
    
    if (!path_planner_service_client.waitForExistence(ros::Duration(5.0)))  
    {
        ROS_ERROR("checkConnectivityAndPathLength() - path planning service is not UP!");       
        return checkConflict;
    }    
    
    for(size_t ii=0, iiEnd=conflictingNodes.size(); ii<iiEnd; ii++)
    {
        const Eigen::Vector3d& conflictingNode = conflictingNodes[ii];

        /// < call path planner service to check the cost and traversability of the edge 

        trajectory_control_msgs::PathPlanning srv_msg;
        srv_msg.request.start.header.stamp = ros::Time::now();
        srv_msg.request.start.header.frame_id = "map";
        srv_msg.request.start.pose.position.x = goal[0];
        srv_msg.request.start.pose.position.y = goal[1];
        srv_msg.request.start.pose.position.z = goal[2];

        srv_msg.request.goal.header.stamp = srv_msg.request.start.header.stamp;
        srv_msg.request.goal.header.frame_id = srv_msg.request.start.header.frame_id;    
        srv_msg.request.goal.pose.position.x = conflictingNode[0];
        srv_msg.request.goal.pose.position.y = conflictingNode[1];
        srv_msg.request.goal.pose.position.z = conflictingNode[2];

        if (path_planner_service_client.call(srv_msg))
        {
            if (srv_msg.response.success)
            {
                std::cout << "checkConnectivityAndPathLength() - goals can be connected with a path cost " << srv_msg.response.path_cost  << std::endl;
                // points can be connected! Let's check the length of the path 
                if( srv_msg.response.path_cost < maxPathLength )
                {
                    checkConflict = true; 
                }
            }
            else
            {
                std::cout << "checkConnectivityAndPathLength() - goals cannot be connected on local traversability map" << std::endl; 
            }
        }
        else
        {
            ROS_ERROR("checkConnectivityAndPathLength() - Failed to call service path planning service");
        }     
        
        if(checkConflict) return checkConflict;
    }
       
    return checkConflict;
}

void broadcastMyPosition()
{
    boost::recursive_mutex::scoped_lock expl_planner_manager_locker(expl_planner_manager_mutex);            
    pcl::PointXYZI robot_position;
    if(p_expl_planner_manager->getRobotPosition(robot_position))
    {
        Eigen::Vector3d position_vec; 
        position_vec << robot_position.x,robot_position.y,robot_position.z; 
        // send robot message to teammates
        sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kPosition, position_vec);
    }            
}

// main exploraion loop (which is triggered asynchronously by a ros::Timer)
void doExplorationPlanning(const ros::TimerEvent& timer_msg)
{        
    std::cout << "========================================" << std::endl;
    std::cout << "doExplorationPlanning()\n" << std::endl;
    
    numExplorationStep++;
    
    /// -------------------------------------------------------------------------
    /// < manage current goal 
    /// -------------------------------------------------------------------------    
    
    // check if exploration is ready, i.e. we can plan a new goal                
    if(!isExplorationReady()) 
    {
        /// < we are managing the current goal 
        
        broadcastMyPosition();
        
        bool doSleepAndSetExplorationReady = false; 
        
        boost::recursive_mutex::scoped_lock tasks_locker(path_planner_status_mutex);          
        if( path_planner_status.success && (goalState == exploration_msgs::ExplorationRobotMessage::kSelected) )
        {
            // check if there is a conflict on the currently selected goal 
            bool bIsConflict = false; 
            boost::recursive_mutex::scoped_lock expl_planner_manager_locker(expl_planner_manager_mutex);
            bIsConflict =  p_expl_planner_manager->IsNodeConflict();
            
            explplanner::ExplorationPlanner::ExplorationStepType explorationStepType = p_expl_planner_manager->getExplorationStepType(); 
            
            Eigen::Vector3d goal  = p_expl_planner_manager->getMyGoal();
            
            if( !bIsConflict )
            {
                std::cout << "ExplorationPlannerManager - node OK => keeping on toward current goal\n";   
                
                nav_msgs::Path path = p_expl_planner_manager->getMyPath();   

                // send robot message to teammates
                sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kSelected, goal, path, path_planner_status.path_cost); 
                
                p_expl_planner_manager->publishPlannedExplorationNode();
                
                // check if backtracking is taking too long; if this is the case, check information gain again and plan a new goal;
                // a long journey towards the goal (without checking world map, in "open loop" mode) is not a smart strategy
                if( explorationStepType == explplanner::ExplorationPlanner::kBacktracking )
                {
                    double elapsed_time_since_backtracking_start = (ros::Time::now()-time_backtracking_start).toSec();
                    if(elapsed_time_since_backtracking_start>kMaxBacktrackingTime)
                    {
                        std::cout << "ExplorationPlannerManager - Long backtracking - checking information gain again" << std::endl;                           
                        p_marker_controller->setMarkerColor(Colors::Orange(), "Planning again \n while backtracking");
                        ros::Duration(2.0).sleep();

#if 0                        
                        // abort and plan a new goal 
                        goalState = exploration_msgs::ExplorationRobotMessage::kAborted; 

                        // send robot message to teammates
                        sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kAborted);      

                        publishGoalAbort();   
#else
                        // do not abort, let's keep going towards the current goal while checking information gain and planning a new goal
#endif                        

                        doSleepAndSetExplorationReady = true;                         
                    }
                }
            }
            else
            {
                std::cout << "ExplorationPlannerManager - node conflict => aborting\n";    
                
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > conflictingNodes = p_expl_planner_manager->getConflictingNodes();
                double conflictingDistance = p_expl_planner_manager->getConflictingDistance();
                bool check = checkConnectivityAndPathLengthOnTraversability(goal, conflictingNodes, conflictingDistance);
                if( check )
                {
                    p_expl_planner_manager->manageConflict();

                    p_marker_controller->setMarkerColor(Colors::Red(), "Node conflict - aborting");

                    goalState = exploration_msgs::ExplorationRobotMessage::kAborted; 

                    // send robot message to teammates
                    sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kAborted);      

                    publishGoalAbort();   

                    doSleepAndSetExplorationReady = true; 
                }
            }            
        }
        
        p_expl_planner_manager->cleanExpiredTeamData(); // N.B.: team model has its own mutex 
                
        if(doSleepAndSetExplorationReady)
        {
            ros::Duration(kSleepTimeAfterAbort).sleep();
            setExplorationReady(true); 
        }
        
        return; /// < EXIT 
    }
 
    /// -------------------------------------------------------------------------
    /// < plan a new goal 
    /// -------------------------------------------------------------------------

    if( goalState == exploration_msgs::ExplorationRobotMessage::kNoInformation )
    {
        if( numExplorationStep % kOnCompleteExplorationSubrate != 0 ) return;
    }

    // lock the exploration planner manager      
    boost::recursive_mutex::scoped_lock expl_planner_manager_locker(expl_planner_manager_mutex);
    
    setExplorationReady(false);
    
    bPathToGoalCompleted = false;
    bGoalRotationCompleted = false; 
    goalState = exploration_msgs::ExplorationRobotMessage::kNone; 
    
    broadcastMyPosition();

    // do we have all the information?
    if (p_expl_planner_manager->isReady())
    {
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::LightYellow(), "Planning");
    }
    
    // < plan the next goal 
    explplanner::ExplorationPlannerManager::ExplorationPlannerStatus planner_status = p_expl_planner_manager->doPlanning();
    
    // < check if we have removed (hence visited) priority points and share the information with teammates 
    std::vector<PriorityPointQueue::PriorityPoint>& removed_priority_points = p_expl_planner_manager->getRemovedPriorityPoints();
    publishRemovedPriorityPoints(removed_priority_points);    

    exploration_msgs::ExplorationPlanningStatus msg_plan_status;
    msg_plan_status.path_cost = -1; // smaller than 0 means invalid 
    
    explplanner::ExplorationPlanner::ExplorationStepType explorationStepType = p_expl_planner_manager->getExplorationStepType();        
    
    /// < check the exploration planner status and take actions 
    switch (planner_status)
    {
    case explplanner::ExplorationPlannerManager::kNotReady:
        
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::LightYellow(), "Planner not ready");
        setExplorationReady(true);
        
        break;

    case explplanner::ExplorationPlannerManager::kFailure:
    case explplanner::ExplorationPlannerManager::kInputFailure:
    case explplanner::ExplorationPlannerManager::kTransformFailure:
        {
        std::string message; 
        if(planner_status == explplanner::ExplorationPlannerManager::kFailure) message = "Planning Failure: cannot find a new view point";
        if(planner_status == explplanner::ExplorationPlannerManager::kInputFailure) message = "Input Failure: cannot find a close starting node";
        if(planner_status == explplanner::ExplorationPlannerManager::kTransformFailure) message = "Transform Failure: cannot receive a valid transform!";
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Red(), message);
        }
                
        // send expl planner status
        {
        msg_plan_status.success = false;
        msg_plan_status.status  = explplanner::ExplorationPlannerManager::kFailure;
        expl_plan_stat_pub.publish(msg_plan_status);
        }
   
        setExplorationReady(true);     
        
        break;
        
    case explplanner::ExplorationPlannerManager::kCompleted:
        
        // send expl planner status
        {
        msg_plan_status.success = true;
        msg_plan_status.status  = explplanner::ExplorationPlannerManager::kCompleted;
        expl_plan_stat_pub.publish(msg_plan_status);
        }
            
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Magenta(), "Exploration Completed!");  

        goalState = exploration_msgs::ExplorationRobotMessage::kCompleted; 
        sendExplorationMessage(robot_id, goalState);   
                    
        numExplorationStep = 0;
                
        setExplorationPaused(true);  
        
        break;        
        
        case explplanner::ExplorationPlannerManager::kNoInformation:
        
        // send expl planner status
        {
        msg_plan_status.success = true;
        msg_plan_status.status  = explplanner::ExplorationPlannerManager::kNoInformation;
        expl_plan_stat_pub.publish(msg_plan_status);
        }
            
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Magenta(), "No Information Gain!");  

        goalState = exploration_msgs::ExplorationRobotMessage::kNoInformation; 
        sendExplorationMessage(robot_id, goalState);   
                    
        numExplorationStep = 0;
                
        setExplorationReady(true);  
        
        break;            


    case explplanner::ExplorationPlannerManager::kSuccess:
        
        // update marker 
        if(p_marker_controller) 
        {
            Color color = Colors::Green();
            if( explorationStepType == explplanner::ExplorationPlanner::kBacktracking ) color = Colors::Cyan();
            p_marker_controller->setMarkerColor(color, explplanner::ExplorationPlanner::kExplorationStepTypeStrings[explorationStepType]);
        }
        
        // send exploration planner status
        {      
        msg_plan_status.success   = true;
        msg_plan_status.status    = explplanner::ExplorationPlannerManager::kSuccess;
        msg_plan_status.path_cost = p_expl_planner_manager->getPathCost();
        expl_plan_stat_pub.publish(msg_plan_status);
        }

        {
        nav_msgs::Path& path = p_expl_planner_manager->getPath();        
        geometry_msgs::PoseStamped goal_pose = path.poses.back();
        double path_cost = -1; // now invalid (this must be computed by the path planner)

        Eigen::Vector3d goal_vec;
        goal_vec[0] = goal_pose.pose.position.x;
        goal_vec[1] = goal_pose.pose.position.y;
        goal_vec[2] = goal_pose.pose.position.z;
        p_expl_planner_manager->setRobotData(robot_id, goal_vec, path, path_cost, ros::Time::now());

        // send goal to path planner 
        publishGoal(goal_pose);        

        goalState = exploration_msgs::ExplorationRobotMessage::kPlanned; 
                
        // send robot message to teammates
        sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kPlanned, goal_vec, path, path_cost); 

        if( explorationStepType == explplanner::ExplorationPlanner::ExplorationStepType::kForwarding )
        {
            std::cout << "ExplorationPlannerManager - adding current exploration node\n"; 
            p_expl_planner_manager->publishPlannedExplorationNode();
        }           

        if( explorationStepType == explplanner::ExplorationPlanner::ExplorationStepType::kBacktracking )
        {
            std::cout << "ExplorationPlannerManager - back tracking\n";
            p_expl_planner_manager->backTrackOnExplorationTree();        
            time_backtracking_start = ros::Time::now();
        }               

        }
        
        break;

    case explplanner::ExplorationPlannerManager::kAborted:
        
        if(p_marker_controller) 
        {        
            p_marker_controller->setMarkerColor(Colors::Red(), "Abort");
        }
        
        // send expl planner status
        {
        msg_plan_status.success = true;
        msg_plan_status.status  = explplanner::ExplorationPlannerManager::kAborted;
        expl_plan_stat_pub.publish(msg_plan_status);
        }        
                
        ROS_INFO("ExplorationPlannerManager abort");
        
        goalState = exploration_msgs::ExplorationRobotMessage::kAborted;     
        sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kAborted); /// < send robot message to teammates  
        setExplorationReady(true);            
        
        break;

    default:
        
        ROS_ERROR("unknown ExplorationPlannerManager status");
        setExplorationReady(true);        
    }
    //===========================================================================================================
    std::string path = "3dmr_devel/expl_status";
    int res_command = system(("mkdir -p ~/.ros/" + path).c_str());
    std::string filename = path + "/status_" + std::to_string(robot_id+1) + ".txt";
    std::ofstream file;
    file.open(filename, std::ios_base::out);
    file << "status_" << robot_id+1 << "=" << explorationStepType << std::endl;
    file.close();
    //===========================================================================================================
}

void traversabilityCloudCallback(const sensor_msgs::PointCloud2& traversability_msg)
{
    p_expl_planner_manager->traversabilityCloudCallback(traversability_msg);
    
}

void wallCloudCallback(const sensor_msgs::PointCloud2& wall_msg)
{
    p_expl_planner_manager->wallCloudCallback(wall_msg);
}

void utilityCloudCallback(const sensor_msgs::PointCloud2& utility_msg)
{
    ROS_INFO("Got Utility PC");
    p_expl_planner_manager->utility2DCloudCallback(utility_msg);
}


void explPauseCallback(std_msgs::Bool msg)
{
    std::cout << "explPauseCallback()" << std::endl;
    
    //boost::recursive_mutex::scoped_lock expl_planner_manager_locker(expl_planner_manager_mutex);
        
    if(msg.data == true)
    {
        std::cout << "explPauseCallback() - pause" << std::endl;
        p_expl_planner_manager->abortCallback(msg);
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Yellow(), "Pause");
        
        setExplorationPaused(true);  
        setExplorationReady(false);  
        
        publishGoalAbort();
        
        sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kAborted); /// < send robot message
    }
    else
    {
        std::cout << "explPauseCallback() - restart" << std::endl;        
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Green(), "Restart");        
        
        setExplorationPaused(false);
        setExplorationReady(true);    
         
    }

    //std::cout << "explPauseCallback() - end " << std::endl;
}

// feedback from the path planner 
void pathPlanningFeedbackCallback(const trajectory_control_msgs::PlanningStatus::ConstPtr& msg)
{        
    if(isExplorationPaused()) return; 

    bool bManageFinalGoalRotation = false;
        
    explplanner::ExplorationPlanner::ExplorationStepType explorationStepType; 
    bool bGoalPathEmpty = true;
    Eigen::Vector3d goal;
    nav_msgs::Path path;  
    
    { // - start lock area -
        
    // update internal path planner status from message
    boost::recursive_mutex::scoped_lock tasks_locker(path_planner_status_mutex);        
    path_planner_status = *msg; 
    path_planner_status_message_updated = true; 
        
    // lock the exploration planner manager in order to get information about the exploration planner status      
    boost::recursive_mutex::scoped_lock expl_planner_manager_locker(expl_planner_manager_mutex);
    explorationStepType = p_expl_planner_manager->getExplorationStepType();   
    bGoalPathEmpty = p_expl_planner_manager->getPath().poses.empty();  
    
    if(path_planner_status.success)
    {
        /// < Path-planner success 
        
        numConsecutiveBacktrackingFailures = 0;
        
        goal  = p_expl_planner_manager->getMyGoal();
        path = p_expl_planner_manager->getMyPath();            

        if(path_planner_status.status == trajectory_control_msgs::PlanningStatus::kArrived)
        {
            std::cout << " pathPlanningFeedbackCallback() - goal reached\n";
            
            goalState = exploration_msgs::ExplorationRobotMessage::kReached; 
            
            bPathToGoalCompleted = true; 
            
            /// < now we have to manage the final rotation 
            bManageFinalGoalRotation = true;              
            
            sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kReached);       
                 
            // add new node to the exploration tree 
            p_expl_planner_manager->addSelectedNodeToExplorationTree(); 
        }        
        else
        {
            std::cout << " pathPlanningFeedbackCallback() - success - status:  " << path_planner_status.status << "\n";
                        
            goalState = exploration_msgs::ExplorationRobotMessage::kSelected; 
            
            sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kSelected, goal, path, path_planner_status.path_cost);             
        }
    }
    else
    {
        /// < Path-planner failure 
        
        std::cout << " pathPlanningFeedbackCallback() - goal not reached\n";
        
        goalState = exploration_msgs::ExplorationRobotMessage::kAborted; 
           
        if(path_planner_status.status == trajectory_control_msgs::PlanningStatus::kFailure)
        {
            std::cout << " pathPlanningFeedbackCallback() - path planning failure\n";            
            p_marker_controller->setMarkerColor(Colors::Red(), "Path Planning Failure");
                        
            sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kAborted);
            
            if( explorationStepType == explplanner::ExplorationPlanner::ExplorationStepType::kBacktracking )
            {
                numConsecutiveBacktrackingFailures++;
                std::cout << " pathPlanningFeedbackCallback() - number of consecutive backtracking failures: " <<   numConsecutiveBacktrackingFailures << std::endl;               
                if(numConsecutiveBacktrackingFailures>=kMaxNumConsecutiveBacktrackingFailures)
                {
                    // N.B.: here the robot is trying to backtrack to a cluster of nodes which became non-reachable => we reset the frontier node cluster 
                    std::cout << " pathPlanningFeedbackCallback() - resetting back-tracking cluster " << std::endl;     
                    p_expl_planner_manager->resetSelectedBackTrackingCluster();
                    numConsecutiveBacktrackingFailures = 0;
                }
            }
                     
            setExplorationReady(true); 
        }  
        
        if(path_planner_status.status == trajectory_control_msgs::PlanningStatus::kAborted)
        {
            std::cout << " pathPlanningFeedbackCallback() - path planning aborted\n";            
            p_marker_controller->setMarkerColor(Colors::Red(), "Goal Aborted");
                       
            sendExplorationMessage(robot_id, exploration_msgs::ExplorationRobotMessage::kAborted);
        }          
    }
    
    } // - end lock area -
    
    
    if(bManageFinalGoalRotation)
    {
        if(bGoalPathEmpty)
        {
            std::cout << "pathPlanningFeedbackCallback() - path emtpy " << std::endl;         
        }
            
        /// < if path is not empty and we are forwarding, then we have to manage the final rotation  
        if(!bGoalPathEmpty  && ( explorationStepType == explplanner::ExplorationPlanner::ExplorationStepType::kForwarding ) )
        {
            std::cout << "pathPlanningFeedbackCallback() - path completed, sending rotation " << std::endl; 
            // send rotation command to trajectory control     
            geometry_msgs::PoseStamped goal_pose = path.poses.back();
            nav_msgs::Path rotation_msg;
            rotation_msg.header = path.header;
            rotation_msg.poses.push_back(goal_pose);
            robot_rotation_pub.publish(rotation_msg);

            if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Green(), "Rotating");
        }    
        else
        {
            std::cout << "pathPlanningFeedbackCallback() - no need to rotate - setting exploration ready" << std::endl; 
            setExplorationReady(true);   
        }
    }
}

// feedback from trajectory control 
void feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg)
{
    if(isExplorationPaused()) return; 
        
    std::cout << "feedbackCallback() " << std::endl; 
    
    // if the message has been published by the tool node, then
    if ( (feedback_msg.node == "control") && (feedback_msg.task == "rotation") )
    {
        bGoalRotationCompleted = true; 

        std::cout << "rotation completed, ready again " << std::endl;         
        // once rotation is also performed we are ready 
        
        ros::Duration(kSleepTimeAfterRotation).sleep();
        
        std::cout << "pathPlanningFeedbackCallback() - setting exploration ready" << std::endl;         
        setExplorationReady(true);
    }
}

void otherUgvDynamicPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
    //boost::recursive_mutex::scoped_lock expl_planner_manager_locker(expl_planner_manager_mutex);
    // < N.B.: don't need to lock a mutex since there is the internal lock of the octomap manager 
    p_expl_planner_manager->insertOtherUgvPointcloudWithTf(pointcloud_msg);   
}

void missingPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{    
    std::cout << "missingPointCloudCallback() - got missing pointcloud" << std::endl;         

    otherUgvDynamicPointCloudCallback(pointcloud_msg); 

    // route teammate missing point cloud toward local 'dense' traversability octomap  
    dynamic_cloud_router_pub.publish(pointcloud_msg);
  
}

void xyBoundinBoxCallback(const nav_msgs::Path::ConstPtr path)
{
    std::cout << "*********************************" << std::endl;     
    std::cout << "xyBoundinBoxCallback()  - got BB " << std::endl;     
    
    double minX =  std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double minY =  std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    for(size_t i=0; i < path->poses.size(); i++)
    {
        const geometry_msgs::PoseStamped& pose = path->poses[i];
        
        if(minX > pose.pose.position.x) minX = pose.pose.position.x;
        if(maxX < pose.pose.position.x) maxX = pose.pose.position.x;
        
        if(minY > pose.pose.position.y) minY = pose.pose.position.y;
        if(maxY < pose.pose.position.y) maxY = pose.pose.position.y;        
    }
    
    if( ((maxX-minX)>kMinExplBoundingBoxDimension) && ((maxY-minY)>kMinExplBoundingBoxDimension) )
    {
        p_expl_planner_manager->setParamExplorationBoxXY( minX, maxX, minY, maxY);
        std::cout << "min: (" <<minX<<", "<<maxX<<"),  max: ("<<minY<<", "<<maxY<< ")" << std::endl;          
    }
    else
    {
        ROS_ERROR_STREAM("got improper BB!!!"); 
    }   
    std::cout << "*********************************" << std::endl; 
    
}

void priorityPointCallback(const exploration_msgs::ExplorationPriorityPoint::ConstPtr msg)
{
    std::cout << "*****************************************" << std::endl;        
//    std::cout << "msg:  " << std::endl; 
//    std::cout << " id: " << msg->id << std::endl;
//    std::cout << " " << msg->position << std::endl; 
//    std::cout << " priority: " << msg->priority << std::endl;
    
    switch(msg->action)
    {
    case exploration_msgs::ExplorationPriorityPoint::kActionAdd:
        std::cout << "priorityPointCallback()  - got new point " << std::endl;  
        p_expl_planner_manager->insertPriorityPoint(msg->id, msg->position, msg->priority);
        break;
            
    case exploration_msgs::ExplorationPriorityPoint::kActionRemove:
        std::cout << "priorityPointCallback()  - removing point " << msg->id << std::endl;  
        p_expl_planner_manager->removePriorityPoint(msg->id);        
        break;        
        
    case exploration_msgs::ExplorationPriorityPoint::kActionRemoveAll:  
        std::cout << "priorityPointCallback()  - removing all points " << std::endl;          
        p_expl_planner_manager->clearPriorityPointQueue();          
        break;
            
    default:
        ROS_ERROR_STREAM("priorityPointCallback() - unknown message type");
    }
    
    std::cout << std::endl;
    std::cout << "Queue content" << std::endl;
    std::cout << p_expl_planner_manager->getPriorityQueue() << std::endl; 
}


void priorityActionsCallback(const exploration_msgs::ExplorationPriorityActions::ConstPtr msg)
{
    std::cout << "*****************************************" << std::endl;        

//    switch(msg->action)
//    {
//    case exploration_msgs::ExplorationPriorityAction::kActionAdd:
//        std::cout << "priorityActionCallback()  - got new list of points " << std::endl;  
//        //p_expl_planner_manager->insertPriorityPoint(msg->id, msg->position, msg->priority);
//        break;
//            
//    case exploration_msgs::ExplorationPriorityAction::kActionRemove:
//        std::cout << "priorityActionCallback()  - removing point " << msg->id << std::endl;  
//        //p_expl_planner_manager->removePriorityPoint(msg->id);        
//        break;        
//            
//    default:
//        ROS_ERROR_STREAM("priorityActionCallback() - unknown message type");
//    }
}


void explorationMessagesCallback(const exploration_msgs::ExplorationRobotMessage::ConstPtr msg)
{
    p_expl_planner_manager->setRobotMessage(msg, false/*isMsgDirect*/);
}

void sendExplorationMessage(const int id, const int action, const Eigen::Vector3d& goal, const nav_msgs::Path& path, const double path_cost)
{
    boost::recursive_mutex::scoped_lock locker(exploration_messages_pub_mutex);

    exploration_msgs::ExplorationRobotMessage::Ptr msg(new exploration_msgs::ExplorationRobotMessage);
    msg->header.stamp = ros::Time::now();    
    msg->robot_id = id; 
    msg->action = (uint8_t)action;
    
    msg->goal.x = goal.x();
    msg->goal.y = goal.y();
    msg->goal.z = goal.z();
    msg->path = path; 
    msg->path_cost = path_cost; 
    
    p_expl_planner_manager->setRobotMessage(msg, true/*isMsgDirect*/);

    exploration_messages_pub.publish(msg);
}

void sendExplorationMessage(const int id, const int action, const Eigen::Vector3d& position)
{
    boost::recursive_mutex::scoped_lock locker(exploration_messages_pub_mutex); 

    exploration_msgs::ExplorationRobotMessage::Ptr msg(new exploration_msgs::ExplorationRobotMessage);
    msg->header.stamp = ros::Time::now();    
    msg->robot_id = id; 
    msg->action = (uint8_t)action;
    
    msg->position.x = position.x();
    msg->position.y = position.y();
    msg->position.z = position.z();

    p_expl_planner_manager->setRobotMessage(msg, true/*isMsgDirect*/);
    
    exploration_messages_pub.publish(msg);
}

void sendExplorationMessage(const int id, const int action)
{
    boost::recursive_mutex::scoped_lock locker(exploration_messages_pub_mutex); 
    
    exploration_msgs::ExplorationRobotMessage::Ptr msg(new exploration_msgs::ExplorationRobotMessage);
    msg->header.stamp = ros::Time::now();
    msg->robot_id = id;     
    msg->action = (uint8_t)action;
    
    p_expl_planner_manager->setRobotMessage(msg, true/*isMsgDirect*/);
    
    exploration_messages_pub.publish(msg);
}

void mapSyncTimerCallback(const ros::TimerEvent& timer_msg)
{
    geometry_msgs::PoseArray mapSyncMessage; 
    p_expl_planner_manager->getMapSyncMessage(mapSyncMessage);
    map_sync_pub.publish(mapSyncMessage);
}

void mapSyncRecCallback(const geometry_msgs::PoseArray::ConstPtr msg)
{
    boost::recursive_mutex::scoped_lock locker(map_sync_messages_pub_mutex); 

    int fromRobotId = atoi(msg->header.frame_id.c_str());
    if(fromRobotId < 0 || fromRobotId > (kMaxNumberOfRobots-1) )
    {
        ROS_ERROR_STREAM("mapSyncRecCallback() - receiving message from unexpected robot " <<  fromRobotId);
        return; 
    }

    if(fromRobotId == robot_id)  return; 
    std::cout << "mapSyncRecCallback() - robot id: " <<  robot_id << " received message from robot " <<  fromRobotId << std::endl;    

    std::vector<sensor_msgs::PointCloud2::Ptr> cloudsToSend;
    p_expl_planner_manager->mapMessageOverlapCheck(*msg, cloudsToSend);
    if(!cloudsToSend.empty())
    {
        for(size_t ii=0, iiEnd=cloudsToSend.size(); ii < iiEnd; ii++)
        {
            missing_point_cloud_pub[fromRobotId].publish(cloudsToSend[ii]);
        }
        ROS_INFO_STREAM("mapSyncRecCallback() - sent " <<  cloudsToSend.size() << " pointclouds to robot " <<  fromRobotId);
    }
    else
    {
        ROS_INFO_STREAM("mapSyncRecCallback() - robot " <<  fromRobotId << " has map in sync!");
    }
}

void mySigintHandler(int signum)
{
    std::cout << "mySigintHandler()" << std::endl;
    //boost::recursive_mutex::scoped_lock destroy_locker(destroy_mutex); 

    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    
    ros::init(argc, argv, "exploration_manager_node");

    //override default sigint handler 
    signal(SIGINT, mySigintHandler);

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    /// < get parameters
    
    str_robot_name   = getParam<std::string>(nh_private, "robot_name", "ugv1");   /// < multi-robot
    std::string str_robot_prefix = "ugv"; 
    robot_id = atoi(str_robot_name.substr(3,str_robot_name.size()).c_str()) - 1;  
    
    std::string simulator_name  = getParam<std::string>(nh_private, "simulator", "");   /// < multi-robot    
    
    ROS_INFO_STREAM("==========================================================");    
    ROS_INFO_STREAM("exploration manager node of robot " << robot_id << " alive");
    ROS_INFO_STREAM("==========================================================");     
    
    number_of_robots = getParam<int>(nh_private, "number_of_robots", 2);   /// < multi-robot    
    
    robot_frame_id = getParam<std::string>(nh_private, "robot_frame_name", "base_link");
    std::string int_marker_server_name = getParam<std::string>(nh_private, "int_marker_server_name", "expl_marker_controller");
    std::string int_marker_name = getParam<std::string>(nh_private, "int_marker_name", "expl_marker");
    
    bool b_use_marker_controller = getParam<bool>(nh_private, "use_marker_controller", true);

    int cost_function_type = getParam<int>(nh_private, "cost_function_type", (int)BaseCostFunction::kSimpleCost);
    double lambda_trav                  = getParam<double>(nh_private, "lambda_trav", 1.);
    double lambda_utility_2d            = getParam<double>(nh_private, "lambda_aux_utility", 1.);
    std::string utility_2d_service_name = getParam<std::string>(nh_private, "utility_2d_service_name", "");
    
    //std::string path_planning_service_name = getParam<std::string>(nh_private, "path_planning_service_name", "/path_planning_service"); 
    
    std::string goal_topic = getParam<std::string>(nh_private, "goal_topic", "/goal_topic");
    std::string goal_abort_topic = getParam<std::string>(nh_private, "goal_abort_topic", "/goal_abort_topic");
    
    std::string path_plan_status_topic = getParam<std::string>(nh_private, "path_plan_status_topic", "path_planning_status"); // "vrep/ugv%d/path_planning_status"
    
    std::string queue_task_feedback_topic = getParam<std::string>(nh_private, "queue_task_feedback_topic", "/planner/tasks/feedback");
          
    std::string path_planning_service_name = getParam<std::string>(nh_private, "path_planning_service_name", "path_planning_service");
    
    //std::string other_robot_dynamic_point_cloud_base_topic = getParam<std::string>(nh_private, "other_robot_dynamic_point_cloud_base_topic", "dynamic_point_cloud");
    std::string other_robot_dynamic_point_cloud_base_topic = getParam<std::string>(nh_private, "other_robot_dynamic_point_cloud_base_topic", "filtered_pointcloud");

    std::cout << "got parameters" << std::endl;

    /// ========================================================================
    
    
    //volumetric_mapping::OctomapManager* test  = new volumetric_mapping::OctomapManager(nh, nh_private);
    
    /// < get a first robot pose in order to initialize the marker on robot starting position
    Transform transform("map", robot_frame_id); 
    tf::StampedTransform robot_pose;
    try
    {
	    robot_pose = transform.get();
    }
    catch(TransformException e )
    {
	    ROS_WARN("%s",e.what());
    }

    p_expl_planner_manager.reset( new explplanner::ExplorationPlannerManager(nh,nh_private));
    p_expl_planner_manager->setFramesRobot("map", robot_frame_id);
    p_expl_planner_manager->setMyRobotId(robot_id);
    
    p_expl_planner_manager->setCostFunctionType(cost_function_type,lambda_trav,lambda_utility_2d);
    
    std::string my_missing_scan_topic_suffix = "/missing_scans";

    /// < Publishers 

    expl_plan_stat_pub = nh.advertise<exploration_msgs::ExplorationPlanningStatus>("/expl_planning_status", 1);
    
    goal_abort_pub = nh.advertise<std_msgs::Bool>(goal_abort_topic, 1); /// < N.B..: this topic also arrives to the trajectory control and stops it
    //trajectory_control_abort_pub = n.advertise<std_msgs::Bool>("/trajectory_control_abort_topic", 1);
    
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    
    robot_rotation_pub = nh.advertise<nav_msgs::Path>("/robot_rotation", 1); /// < sent to trajectory control!
    
    priority_actions_pub = nh.advertise<exploration_msgs::ExplorationPriorityActions>("/exploration_priority_actions", 10);   
    
    exploration_messages_pub = nh.advertise<exploration_msgs::ExplorationRobotMessage>("/exploration_messages", 40);

    dynamic_cloud_router_pub = nh.advertise<sensor_msgs::PointCloud2>("/volumetric_mapping/pointcloud2", 20); // toward this robot dense/traversability octomap     

    
#if USE_MAP_SYNC         
    map_sync_pub = nh.advertise<geometry_msgs::PoseArray>("/map_sync_messages", 1);

    for(size_t id=0; id < kMaxNumberOfRobots; id++)
    {
        if(id != robot_id)
        {
            std::stringstream missing_scan_topic_name; 
            missing_scan_topic_name << "/" << str_robot_prefix << id+1 << "/" << my_missing_scan_topic_suffix;            
            missing_point_cloud_pub[id] = nh.advertise<sensor_msgs::PointCloud2>(missing_scan_topic_name.str(),1);
        }
    }
#endif 

    /// < Subscribers
    
    ros::Subscriber sub_trav = nh.subscribe("/trav/traversability", 1, traversabilityCloudCallback);
    ros::Subscriber sub_wall = nh.subscribe("/clustered_pcl/wall", 1, wallCloudCallback);

    //ros::Subscriber sub_goal = n.subscribe("/goal_topic", 1, goalSelectionCallback);
    ros::Subscriber sub_expl_pause = nh.subscribe("/expl_pause_topic", 1, explPauseCallback);

    // feedback between nodes: "tool", "planner" and "control"
    ros::Subscriber queue_task_feedback_sub = nh.subscribe(queue_task_feedback_topic, 10, feedbackCallback);
    
    ros::Subscriber path_plan_status_sub = nh.subscribe(path_plan_status_topic, 20, pathPlanningFeedbackCallback);  
    
    ros::Subscriber xy_BB_sub = nh.subscribe("/exploration_fence", 1, xyBoundinBoxCallback); 
    
    ros::Subscriber priority_point_sub = nh.subscribe("/exploration_priority_point", 20, priorityPointCallback); 
    
    ros::Subscriber priority_action_sub = nh.subscribe("/exploration_priority_actions", 20, priorityActionsCallback);    
           
    ros::Subscriber exploration_messages_sub      = nh.subscribe("/exploration_messages", 100, explorationMessagesCallback); 
    ros::Subscriber core_exploration_messages_sub = nh.subscribe("/core/exploration_messages", 100, explorationMessagesCallback); 

#if USE_MAP_SYNC 
    ros::Subscriber map_sync_sub = nh.subscribe("/map_sync_messages", 20, mapSyncRecCallback);

    std::stringstream my_missing_scan_topic_name; 
    my_missing_scan_topic_name << "/" << str_robot_prefix << robot_id+1 << "/" << my_missing_scan_topic_suffix;
    ros::Subscriber my_missing_scans_sub = nh.subscribe(my_missing_scan_topic_name.str(), 20, missingPointCloudCallback); 
#endif 

    for(size_t id=0; id < kMaxNumberOfRobots; id++)
    {
        /// < N.B.: the individual robot scan is integrated by the ExplorationPlanner
        /// <       topic: "/expl_planner/exploration_pointcloud" => callback: ExplorationPlanner::insertPointcloudWithTf()

        // integrate teammate scans 
        if(id != robot_id)
        {
            std::stringstream topic_name;
            if(!simulator_name.empty())
            {
                //topic_name << "/"<< simulator_name;
                topic_name << simulator_name;
            }
            topic_name << "/" << str_robot_prefix << id+1 << "/" << other_robot_dynamic_point_cloud_base_topic;  
            std::cout << "topic name in: " << topic_name.str() << std::endl;
            
            other_dynamic_point_cloud_sub[id] = nh.subscribe(topic_name.str(), 20, otherUgvDynamicPointCloudCallback); /// < multi-robot            
        }
    }    
    
    /// < Services   
    
    //ros::ServiceServer pathPlannerService = n.advertiseService(path_planning_service_name, pathPlanningServiceCallback);
    
    path_planner_service_client = nh.serviceClient<trajectory_control_msgs::PathPlanning>(path_planning_service_name);
    
    
    /// < Timers 
    
    expl_task_timer = nh.createTimer(ros::Duration(kExplorationTaskCallbackPeriod), doExplorationPlanning); // the timer will automatically fire at startup

#if USE_MAP_SYNC   
    map_sync_timer = nh.createTimer(ros::Duration(kMapSyncCallbackPeriod), mapSyncTimerCallback); // the timer will automatically fire at startup
#endif 

    /// < Start
                
    if (b_use_marker_controller)
    {
        p_marker_controller.reset(new ExplorationMarkerController(robot_pose.getOrigin(), nh, str_robot_name, robot_frame_id, "map", "/expl_pause_topic", int_marker_server_name, int_marker_name));
        
        p_marker_controller->setMarkerColor(Colors::Yellow(), "Pause - Select Restart");        
    } 
    
#if 0    
    ros::spin();
#else
    ros::MultiThreadedSpinner spinner(2); // Use 2-4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
#endif 
    
    //if(p_marker_controller) p_marker_controller->reset();

    return 0;
}
