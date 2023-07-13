/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DMR. 
* This file is a VERY modified version of the corresponding file in patrolling_sim 
* http://wiki.ros.org/patrolling_sim see license below.
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

/**
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Luca Iocchi (2014) 
 *********************************************************************/
#ifndef PATROLLING_AGENT_H
#define PATROLLING_AGENT_H


#include "message_types.h"

#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <boost/thread/recursive_mutex.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#if USE_STAMPED_MSG_INT16_MULTIARRAY
#include <stamped_msgs/Int16MultiArray.h>
typedef stamped_msgs::Int16MultiArray Int16MultiArrayMsg; 
#else
#include <std_msgs/Int16MultiArray.h>
typedef std_msgs::Int16MultiArray Int16MultiArrayMsg; 
#endif

#include <trajectory_control_msgs/TrajectoryControlAction.h>
#include <trajectory_control_msgs/PlanningFeedback.h>
#include <trajectory_control_msgs/PlanningTask.h>
#include <trajectory_control_msgs/PlanningStatus.h>
#include <trajectory_control_msgs/TrajectoryControlActionResult.h>

#include <std_msgs/Bool.h>
#include <nifti_robot_driver_msgs/Tracks.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_control_msgs/message_enums.h> // for SegmentStatus and TaskType

#include <patrolling_build_graph_msgs/BuildGraphEvent.h>
#include <patrolling_build_graph_msgs/Graph.h>
#include <patrolling_build_graph_msgs/PriorityPoint.h>

#include <path_planner/KdTreeFLANN.h>

#include "graph.h"
#include "graph_viz.h"


#define NUM_MAX_ROBOTS 32

#define INTERFERENCE_LEVEL_LOW 0
#define INTERFERENCE_LEVEL_MEDIUM 1
#define INTERFERENCE_LEVEL_HIGH 2
#define INTERFERENCE_LEVEL_CRITICAL 3 


typedef unsigned int uint;
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//typedef actionlib::SimpleActionClient<trajectory_control_msgs::TrajectoryControlAction> PlannerClient;


class PatrollingMarkerController;


///	\class TeamModel
///	\author Luigi Freda 
///	\brief A base class for storing team model information.
///	\note 
/// 	\todo 
///	\date
///	\warning
class TeamModel
{
public:
    static const double kExpirationTimeTeamModel; // [s]
    
public:
    
    TeamModel()
    {
        for(size_t id=0;id<NUM_MAX_ROBOTS;id++) init(id);
    }
    
    void init(const size_t& id)
    {
        id_ugv[id] = id; 
        reset(id);        
    }    
    
    void reset(const size_t& id)
    {
        id_current_selected_vertex[id] = -1;
        nav_cost_to_go[id] = -1;        
    }
    
    void resetWithTime(const size_t& id, const ros::Time& time)
    {
        reset(id);
        timestamp[id] = time;
    }    
    
    bool isValid(const size_t& id) const
    {
        return (id_current_selected_vertex[id] != -1) && (nav_cost_to_go[id] != -1);
    }
        
    int id_ugv[NUM_MAX_ROBOTS];
    int id_current_selected_vertex[NUM_MAX_ROBOTS]; // -1 invalid 
    int nav_cost_to_go[NUM_MAX_ROBOTS]; // -1 planned, >0 selected
    ros::Time timestamp[NUM_MAX_ROBOTS]; 
    // N.B.: the planned path is directly used/received by the path planner at metric strategy level 
    
    mutable boost::recursive_mutex mutex;    

};

///	\class TeamDeployment
///	\author Luigi Freda 
///	\brief A base class for representing robot team deployment over th graph: which node a robot is deployed according to last received messages.
///	\note 
/// 	\todo 
///	\date
///	\warning
class TeamDeployment
{
public: 
    
    static const double kExpirationTimeTeamDeployment; // [s]  
    
public:    
        
    TeamDeployment()
    {
        for(size_t id=0;id<NUM_MAX_ROBOTS;id++) init(id);
    }   
    
    void init(const size_t& id)
    {
        id_ugv[id] = id; 
        reset(id);
    } 
    
    void set(int id, int vertex, const ros::Time& time) 
    {
        covered_vertex[id] = vertex;
        covered_timestamp[id] = time;
    }
    
    int where(int id, const ros::Time& time) const
    {
        int res = -1; 
        bool bValid = (covered_vertex[id] > -1) && ( (covered_timestamp[id]-time).toSec() < kExpirationTimeTeamDeployment );
        if(bValid)  res = covered_vertex[id];
        return res;
    }
    
    void reset(int id) 
    {
        covered_vertex[id] = -1;
    }
    
    int id_ugv[NUM_MAX_ROBOTS];    
    int covered_vertex[NUM_MAX_ROBOTS];
    ros::Time covered_timestamp[NUM_MAX_ROBOTS];
    
    mutable boost::recursive_mutex mutex;        
    
};


enum InterferenceState
{
    NO_OP        = -2,
    IDLE         = -1,
    LOW_LEVEL    = 0,
    MEDIUM_LEVEL = 1,
    HIGH_LEVEL   = 2,
    CRITICAL     = 3,
    DEAD         = 4
};


/// \class PatrolAgent
/// \author Luigi Freda 
/// \brief A base class for implementing a patrolling agent strategy. 
/// \note The first version of this class was written by David Portugal and Luca Iocchi in the package patrolling_sim.               \n
///       Luigi Freda and Mario Gianni started modifying it in 2016 for interfacing it with the UGV path planner. After, Luigi Freda \n
///       completely revised and changed it starting from October 2016 up to present time, in order to design and implement the      \n
///       3D patrolling strategy currently presented in the paper "3D Multi-Robot Patrolling with a Two-Level Coordination Strategy".\n
/// \todo 
/// \date
/// \warning Real robots or distributed nodes (on different computers) need to synch their system clocks! If you do not want to do    \n
///          that, then, you have to disable stamped messages in the file "message_types.h". Disabling stamped messages could possibly\n
///          bring to a non-perfect idleness synchronization. \n
class PatrolAgent
{
public:
    
    enum CriticalConflictLevel { kCriticalConflict0=0, kCriticalConflict1, kCriticalConflict2, kCriticalConflict3, kCriticalConflict4, kCriticalConflictNum };    
    
protected:
    
    static const int kPatrolAgentLoopRate = 30; //[Hz] loop rate for the main loop (contained in the run() method)
    static const double kSleepTimeAfterSendingNewGoal; // [seconds] to wait after sending a new goal 
    static const double kSleepTimeAfterSendingAbort; // [seconds] to wait after sending an abort
    static const int kCheckVisitedVertexSubrate = 15; // with the main 30Hz this is equivalent to 2Hz
    static const int kCheckInterferenceSubrate = 15;  // with the main 30Hz this is equivalent to 2Hz
    static const int kCheckTaskResetSubrate = 30; // with the main 30Hz this is equivalent to 1Hz        
    static const int kSelectBroadcastSubrate = 15; // with the main 30Hz this is equivalent to 2Hz 
    static const int kIdlenessSynchBroadcastSubrate = 150; // with the main 30Hz this is equivalent to 0.2Hz
        
    static const float kScalePathNavCostFromFloatToInt; 
    static const double kScaleIdlenessFromFloatToInt;
    static const double kScaleIdlenessFromFloatToIntInv;
    
    static const float kIdlenessPriorityMin; 
    
    static const double kTimeIntervalForCriticalPathPlanningFailure;  // [s]
    static const double kTimeIntervalForCriticalNodeConflict; // [s]
    
    static const double kExpirationTimeTeamModel; // [s]
    
    static const double kTimeoutForReachingGoal; // [s]
    
    static const double kTimeoutForLastPathPlanningMsg; // [s]
    
    static const int kMaxNumAttemptsForRandomNodeSelection = 50;
    
    static const double kInterferenceDistance;
    static const double kInterferenceDistance2;    
    
    static const double kVisitedNodeDistance; // [m] if robot position to vertex position is smaller than this the vertex is considered visited
    static const double kVisitedNodeDistanceSquared;
    
    static const double kMaxDistPointPriority;    
    static const double kMaxDistPointPrioritySquared;    
    
    typedef pcl::PointXYZL PointNode;
    typedef pcl::PointCloud<PointNode> PointCloudNodes;
    typedef pp::KdTreeFLANN<PointNode> KdTreeNodes;
    

public:

    PatrolAgent();

    virtual void init(int argc, char** argv); 
    void get_ready();

    void readParams(); // read ROS parameters


    virtual void run();

    void getRobotPose(int robotid, float &x, float &y, float &theta);
    void getRobotPose3D(int robotid, float &x, float &y, float &z, float &theta);


 public: /// < patrolling main events

    void onGoalNotComplete(); // what to do when a goal has NOT been reached (aborted)
    
    virtual void onGoalComplete(); // what to do when a goal has been reached
    virtual void processEvents(); // processes algorithm-specific events

    
    int compute_next_switching_vertex();
    
    virtual int compute_random_vertex();
    
    void waitForTaskReallocation();
    
    void update_idleness(); // update local idleness
    void check_tasks_reset();     
    bool check_interference(int robot_id);
    void do_interference_management();
    void do_task_reallocation();
    
    bool check_vertex_conflict(const int robot_id, const int next_vertex, const int nav_cost); 
    
    void build_kdtree_nodes(); 
    int check_vertex_interception(const int current_vertex, const int next_vertex, std::vector<int>& list_already_intercepted, bool& new_interception);
    
    int find_closest_node(); 
    
public: /// < pure virtual methods 
    
    // must be implemented by sub-classes
    virtual int compute_next_vertex() = 0;

public: /// <  robot-robot communication
    
    void broadcast_initialize_node();
    void broadcast_goal_reached();
    void broadcast_goal_intention(int next_vertex); 
    void broadcast_goal_selected(int next_vertex, int nav_cost);
    
    void broadcast_nav_cost(int vertex1, int vertex2, int nav_cost);    
    
    void broadcast_goal_abort(int next_vertex); 
    void broadcast_goal_intercepted(int next_vertex);     
    void broadcast_vertex_covered(int vertex);    
    void broadcast_intercepted_vertexes(const std::vector<int>& list); 
      
    void broadcast_positions();
    void broadcast_interference();
    
    void broadcast_idleness();
    
    void sendPathPlannerGoalAndBroadcastIntention(int next_vertex);
    void sendPathPlannerGoalAbortAndBroadcastAbort();
    
    void receive_positions();
    virtual void send_results(); // when goal is completed
    virtual void receive_results(); // asynchronous call
    void do_send_message(Int16MultiArrayMsg &msg);
    
public: /// < callbacks 

    void positionsCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void resultsCallback(const Int16MultiArrayMsg::ConstPtr& msg);
    void results3dCallback(const Int16MultiArrayMsg::ConstPtr& msg); /// < automatically called by resultsCallback()
    void patrollingNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void pathPlanningFeedbackCallback(const trajectory_control_msgs::PlanningStatus::ConstPtr& msg);
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void trajectoryTrackingResultCallback(const trajectory_control_msgs::TrajectoryControlActionResult::ConstPtr& msg);
 
    void buildGraphEventCallback(const patrolling_build_graph_msgs::BuildGraphEvent::ConstPtr&);
    void graphCallback(const patrolling_build_graph_msgs::Graph::ConstPtr& msg);
    void pausePatrollingCallback(const std_msgs::Bool& msg);
 
    //void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalActiveCallback();
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
    
    void priorityPointCallback(const patrolling_build_graph_msgs::PriorityPoint::ConstPtr&);    
    
    boost::recursive_mutex results_callback_mutex;  // to protect double link to callback 
    boost::recursive_mutex positions_callback_mutex; // to protect double link to callback 

protected: /// < 3D GUI
    
    void buildPatrollingNodesAsMarkers();
    void buildPatrollingEdgesAsMarkers(); 
    
    void publishPatrollingNodesAsMarkers();
    void publishPatrollingEdgesAsMarkers();
    
    void updateDynNavCost(const int vertex1, const int vertex2, const int nav_cost, const ros::Time& timestamp);
    
protected:

    volatile bool b_pause_;
    bool b_interactive_;

    TeamModel tasks_;
    
    TeamDeployment team_deployment_;
    
    bool b_task_reallocation_;
    int vertex_id_other_robot_global_;
    int id_sender_global_;

    InterferenceState interference_state_;

    double check_interference_timeout_;

    int TEAMSIZE_;
    int ID_ROBOT_; // my ID 

    double xPos_[NUM_MAX_ROBOTS]; // position table 
    double yPos_[NUM_MAX_ROBOTS]; 
    double zPos_[NUM_MAX_ROBOTS]; 

    tf::TransformListener *p_listener_;

    std::string str_graph_file_name_;
    std::string str_map_name_;
    std::string traj_act_client_name_;

    int current_vertex_; // current vertex
    int next_vertex_;
    int covered_vertex_; // currently covered vertex 
        
    bool b_debug_;
    
    bool b_resend_goal_; // Send the same goal again (if goal failed...)
    bool b_random_vertex_; // To check if a path has been computed
    
    bool b_interference_;
    double last_interference_time_;
    int intereference_count_;
    
    bool b_goal_complete_;
    bool b_waiting_init_msg_;   // waiting for init msg? 
    bool b_have_to_init_;   // have to init structures and time variables?     
    bool b_end_simulation_;
    bool b_first_node_;   // is it the first target node? 
    
    bool b_node_conflict_; // is there a node conflict? 
    int  last_conflict_vertex_; // 
    int  last_conflict_robot_; // 
    
    volatile bool b_next_vertex_intercepted_by_teammate_; 
    std::vector<int> list_intercepted_vertexes_; // visited vertex along the way 
    
    bool b_path_planning_failure_; 
    bool b_critical_path_planning_failure_; 
    double time_last_path_planning_success_;
    double time_last_path_planning_msg_;
    bool b_timeout_path_planning_reply_;
    
    bool b_timeout_for_reaching_goal_;
    double time_last_reached_goal_;
    
    bool b_critical_node_conflict_; 
    bool b_critical_node_conflict_forced_;     
    double time_last_no_node_conflict_; 
    int critical_conflict_level_;
   
    Vertex *graph_;        /// < the graph 
    uint graph_dimension_; /// < graph size
    boost::recursive_mutex graph_mutex_;    
    
    double* vec_instantaneous_idleness_; // local idleness
    boost::recursive_mutex idleness_mutex_;
    
    double* vec_global_instantaneous_idleness_; // local idleness
    double last_update_global_idl_;    
    boost::recursive_mutex global_idleness_mutex_;    
    
    double* vec_last_visit_time_;
    boost::recursive_mutex last_visit_mutex_;
    
    std::vector<int> vec_results_; // results exchanged among robots
    //bool b_goal_aborted_;
    double goal_reached_waiting_time_;
    //int aborted_count_;
    int resend_goal_count_;
    
    double communication_delay_; 
    double last_communication_delay_time_;
    
    double lost_message_rate_;
    
    std::string initial_positions_;
    
    //MoveBaseClient *ac; // action client for reaching target goals
    //PlannerClient *act_client_;

    std::string robot_frame_;
    
    double time_zero_;

protected: /// < ROS
    
    ros::NodeHandle node_;
    
    std::string odom_frame_topic_; // "vrep/ugv%d/odom"
    std::string goal_topic_; // "vrep/ugv%d/goal_topic"
    std::string planning_goal_abort_topic_; // vrep/ugv%d/goal_abort_topic
    std::string nodes_topic_; // "vrep/ugv%d/patrolling_nodes_updated_markers"
    std::string path_plan_stat_topic_; // "vrep/ugv%d/path_planning_status"
    std::string patrolling_pause_topic_; 

    ros::Subscriber odom_sub_;
    
    ros::Subscriber positions_sub_;
    ros::Subscriber core_positions_sub_;
    ros::Publisher  positions_pub_;
    
    ros::Subscriber results_sub_;
    ros::Subscriber core_results_sub_;
    ros::Publisher  results_pub_;
    
    //ros::Publisher cmd_vel_pub;
    ros::Publisher goal_pub_;
    ros::Subscriber traj_res_sub_;
    //ros::Subscriber results_interf_sub_;
    //ros::Publisher results_interf_pub_;
    //ros::Publisher tracks_vel_cmd_pub;
    ros::Subscriber patrolling_pause_sub_;
    ros::Subscriber path_plan_stat_sub_;

    trajectory_control_msgs::TrajectoryControlActionResult traj_res_msg_;
    geometry_msgs::PoseStamped goal_msg_;

    ros::Publisher planning_goal_abort_pub_;
    //double distance_threshold_;

    ros::Subscriber nodes_topic_sub_;
    ros::Publisher nodes_topic_pub_;
    visualization_msgs::MarkerArray marker_nodes_array_;
    boost::recursive_mutex marker_nodes_array_mutex_;
    
    std::string edges_topic_name_;
    ros::Publisher edges_topic_pub_;
    visualization_msgs::Marker marker_edges_list_;
    boost::recursive_mutex marker_edges_list_mutex_;
    
    std::string build_graph_event_topic_;
    ros::Subscriber build_graph_event_sub_;
    std::string graph_topic_;
    ros::Subscriber graph_sub_;
    std::string priority_point_topic_;
    ros::Subscriber priority_point_sub_;        
    
protected: 
    
    trajectory_control_msgs::PlanningStatus planner_status_;
    boost::recursive_mutex planner_status_mutex_;
    bool path_planner_status_message_updated_; 

    patrolling_build_graph_msgs::BuildGraphEvent build_graph_event_;
    
protected: 

    PointCloudNodes plc_nodes_;
    KdTreeNodes     kdtree_nodes_; 
    
protected:
    
    boost::shared_ptr<PatrollingMarkerController> p_marker_controller;
    bool b_use_marker_controller_;
    
};


#endif