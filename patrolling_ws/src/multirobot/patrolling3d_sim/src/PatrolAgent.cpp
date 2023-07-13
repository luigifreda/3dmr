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
 * Author: David Portugal (2011-2014), and Luca Iocchi (2014)
 *********************************************************************/

#include <sstream>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include "PatrolAgent.h"
#include "PatrollingMarkerController.h"


using namespace std;

#define UPDATE_PATROLLING_MAKER 1

//template<typename T>
//T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
//{
//	T v;
//	if (n.getParam(name, v))
//	{
//		ROS_INFO_STREAM("[GRAPH_VIZ] Found parameter: " << name << ", value: " << v);
//		return v;
//	}
//	else
//		ROS_WARN_STREAM("[GRAPH_VIZ] Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
//	return defaultValue;
//}




const double TeamModel::kExpirationTimeTeamModel = 5; // [s]

const double TeamDeployment::kExpirationTimeTeamDeployment = 1; // [s]  
    
// 

const double PatrolAgent::kSleepTimeAfterSendingNewGoal = 0.2; // [seconds] to wait after sending a new goal
const double PatrolAgent::kSleepTimeAfterSendingAbort   = 1; // [seconds] to wait after sending an abort

const float PatrolAgent::kScalePathNavCostFromFloatToInt = patrolling_build_graph_msgs::Graph::kNavCostConversionFloatToUint;//10.; // scaling factor for path navigation cost 
const double PatrolAgent::kScaleIdlenessFromFloatToInt = 10.;
const double PatrolAgent::kScaleIdlenessFromFloatToIntInv = 1./PatrolAgent::kScaleIdlenessFromFloatToInt;

const float PatrolAgent::kIdlenessPriorityMin = 1e-6;

const double PatrolAgent::kTimeIntervalForCriticalPathPlanningFailure = 5; 
const double PatrolAgent::kTimeIntervalForCriticalNodeConflict        = 5; 

const double PatrolAgent::kExpirationTimeTeamModel = TeamModel::kExpirationTimeTeamModel; // [s]

const double PatrolAgent::kTimeoutForReachingGoal = 60; // [s] 

const double PatrolAgent::kTimeoutForLastPathPlanningMsg = 60; // [s]

const double PatrolAgent::kInterferenceDistance = 1.2;
const double PatrolAgent::kInterferenceDistance2 = PatrolAgent::kInterferenceDistance * PatrolAgent::kInterferenceDistance;

const double PatrolAgent::kVisitedNodeDistance  = 1.; // [m] if robot position to vertex position is smaller than this the vertex is considered visited
const double PatrolAgent::kVisitedNodeDistanceSquared = PatrolAgent::kVisitedNodeDistance*PatrolAgent::kVisitedNodeDistance;

const double PatrolAgent::kMaxDistPointPriority = 0.1;  
const double PatrolAgent::kMaxDistPointPrioritySquared = PatrolAgent::kMaxDistPointPriority * PatrolAgent::kMaxDistPointPriority; 

PatrolAgent::PatrolAgent():node_("~")
{
    ID_ROBOT_ = -1;
    
    p_listener_       = NULL;
    current_vertex_   = -1;
    next_vertex_      = -1;
    covered_vertex_   = -1;
    last_conflict_vertex_  = -1; 
    last_conflict_robot_ = -1;
    
    b_waiting_init_msg_   = true;
    b_have_to_init_   = true;
    b_end_simulation_ = false;
    b_first_node_     = true;
    b_node_conflict_  = false;    
    
    b_path_planning_failure_          = false; 
    b_critical_path_planning_failure_ = false;
    
    path_planner_status_message_updated_ = false;
    
    b_timeout_for_reaching_goal_ = false;
    
    b_critical_node_conflict_ = false;
    b_critical_node_conflict_forced_ = false;
    critical_conflict_level_ = 0;
    
    b_next_vertex_intercepted_by_teammate_ = false; 
    
    
    /// < starting in pause mode
    b_pause_ = true;
    
    
    //ac = NULL;
    
    graph_ = 0; 
}

void PatrolAgent::init(int argc, char** argv)
{
    //ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
 
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   

    p_listener_ = new tf::TransformListener();
    build_graph_event_.event = patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING;

    node_.getParam("build_graph_event_topic", build_graph_event_topic_);
    build_graph_event_sub_ = node_.subscribe<patrolling_build_graph_msgs::BuildGraphEvent>(build_graph_event_topic_, 10, boost::bind(&PatrolAgent::buildGraphEventCallback, this, _1));

    node_.getParam("graph_topic", graph_topic_);
    graph_sub_ = node_.subscribe<patrolling_build_graph_msgs::Graph>(graph_topic_, 10, boost::bind(&PatrolAgent::graphCallback, this, _1));
    
    nodes_topic_sub_ = node_.subscribe<visualization_msgs::MarkerArray>("/patrolling_nodes_markers", 1, boost::bind(&PatrolAgent::patrollingNodesCallback, this, _1));

    node_.getParam("nodes_topic", nodes_topic_); // "vrep/ugv%d/patrolling_nodes_updated_markers"
    nodes_topic_pub_ = node_.advertise<visualization_msgs::MarkerArray>(nodes_topic_, 10);
        
    edges_topic_name_ = getParam<std::string>(node_, "edges_topic_name", "/edges_markers");
    edges_topic_pub_ = node_.advertise<visualization_msgs::Marker>(edges_topic_name_, 10);
    
    priority_point_topic_ = getParam<std::string>(node_, "priority_point_topic_", "/priority_point");
    priority_point_sub_ = node_.subscribe(priority_point_topic_, 10, &PatrolAgent::priorityPointCallback, this);    

    std::cout << " Number of input arguments arguments " << argc << std::endl;
    std::cout << argv[0] << std::endl; // =/.../patrolling3d_sim/bin/GBS
    std::cout << argv[1] << std::endl; // WORLD_MAP
    std::cout << argv[2] << std::endl; //ID_ROBOT
    std::cout << argv[3] << std::endl; // INTERACTIVE FLAG


    srand(time(NULL));

    //More than One robot (ID between 0 and 99)
    if (atoi(argv[2]) > NUM_MAX_ROBOTS || atoi(argv[3])<-1)
    {
        ROS_INFO("The Robot's ID must be an integer number between 1 an 99"); //max 100 robots 
        return;
    }
    else
    {
        ID_ROBOT_ = atoi(argv[2]);
        printf("ID_ROBOT = %d\n", ID_ROBOT_); //-1 in the case there is only 1 robot.
    }

    if (argc < 4)
    {
        b_interactive_ = false; // default behavior use predefined nodes
    }
    else
    {        
        b_interactive_ = false;
        std::string in = argv[3];
        std::transform(in.begin(), in.end(), in.begin(), ::tolower);
        if (in.find("true") != std::string::npos) b_interactive_ = true;
        
        std::cout << "Interactive: " << b_interactive_ << std::endl; 
    }

    node_.getParam("robot_frame", robot_frame_); // "vrep/ugv%d/odom"

    
    std::stringstream robot_name;
    robot_name << "ugv" << ID_ROBOT_+1;
    b_use_marker_controller_ = getParam<bool>(node_, "use_marker_controller", true);    
    std::string int_marker_server_name = getParam<std::string>(node_, "int_marker_server_name", "patrolling_marker_controller");
    std::string int_marker_name = getParam<std::string>(node_, "int_marker_name", "patrolling_marker"); 
    
    
    node_.getParam("patrolling_pause_topic", patrolling_pause_topic_); // "vrep/ugv%d/path_planning_status"
            
    float x, y, z, th;
    getRobotPose3D(ID_ROBOT_, x, y, z, th);    
    if (b_use_marker_controller_)
    {     
        p_marker_controller.reset(new PatrollingMarkerController(tf::Vector3(x,y,z), node_, robot_name.str(), robot_frame_, "map", patrolling_pause_topic_, int_marker_server_name, int_marker_name));
    }    
    
    
    
    str_map_name_ = string(argv[1]);
    //graph_file = "maps/"+mapname+"/"+mapname+".graph";
    str_graph_file_name_ = str_map_name_;
        
    if (!b_interactive_)
    {
        printf("Loading graph %s \n", str_graph_file_name_.c_str());
        //Check Graph Dimension:
        graph_dimension_ = GetGraphDimension(str_graph_file_name_.c_str());
        printf("Graph dimension DONE %d ---------------------------- \n", graph_dimension_);
        
        if(graph_dimension_ > 0)
        {
            boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
                
            //Create Structure to save the Graph Info;
            graph_ = new Vertex[graph_dimension_];

            //Get the Graph info from the Graph File
            GetGraphInfo3D(graph_, graph_dimension_, str_graph_file_name_.c_str());
            printf(" Graph info DONE ---------------------------- \n");
            build_graph_event_.event = patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT;
        }
        else
        {
            b_interactive_ = true; 
            ROS_WARN_STREAM("the graph file is empty or it does not exist: waiting in interactive mode");
        }

    }
    
    /// ========================================================================
   
    /// < wait for the input graph 
    if(b_interactive_) /// <  N.B.: this cannot be put in else with the previous block!!!
    {
        
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Yellow(), "Waiting Graph");
        
        ros::Rate rate(5);
        while (
                (build_graph_event_.event != patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT) &&
                (build_graph_event_.event != patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED) 
              )
        {
            rate.sleep();
            ros::spinOnce();
            //std::cout << "Waiting patrolling build graph node to finish to build the graph" << std::endl;
        }

        if(build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT)
        {
            boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
                
            printf("Loading graph %s \n", str_graph_file_name_.c_str());
            //Check Graph Dimension:
            graph_dimension_ = GetGraphDimension(str_graph_file_name_.c_str());
            printf("Graph dimension DONE %d ---------------------------- \n", graph_dimension_);
            //Create Structure to save the Graph Info;
            graph_ = new Vertex[graph_dimension_];

            //Get the Graph info from the Graph File
            GetGraphInfo3D(graph_, graph_dimension_, str_graph_file_name_.c_str());
            printf(" Graph info DONE ---------------------------- \n");
            
            if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Green(), "Graph Received");
        }
        
        if(build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED)
        {
            ROS_INFO_STREAM("Graph received");
            
            if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Green(), "Graph Received");
        }
    }
    
    /// ========================================================================
    
    /// check number of edges in the graph 
    uint nedges = GetNumberEdges(graph_, graph_dimension_);
    printf("graph %s with %d nodes and %d edges\n", str_map_name_.c_str(), graph_dimension_, nedges);

    /// print graph 
    for (int i = 0; i < graph_dimension_; i++)
    {
        printf("ID= %u\n", graph_[i].id);
        printf("X= %f, Y= %f, Z= %f \n", graph_[i].x, graph_[i].y, graph_[i].z);
        printf("#Neigh= %u\n", graph_[i].num_neigh);

        for (int j = 0; j < graph_[i].num_neigh; j++)
        {
            printf("\tID = %u, DIR = %s, COST = %u\n", graph_[i].id_neigh[j], graph_[i].dir[j], graph_[i].cost[j]);
        }

        printf("\n");
    }

    b_interference_  = false;
    b_resend_goal_   = false;
    b_random_vertex_ = false;
    b_goal_complete_ = true;
    
    last_interference_time_ = 0;
    intereference_count_    = 0;
    //b_goal_aborted_ = false;
    //aborted_count_     = 0;
    resend_goal_count_ = 0;
    communication_delay_ = 0.0;
    lost_message_rate_   = 0.0;
    b_task_reallocation_ = false;
    
    vertex_id_other_robot_global_ = -1;
    id_sender_global_             = -1;
    
    planner_status_.success = true;
    
    interference_state_ = NO_OP;
    
    //b_the_other_robot_is_alive_ = true;
    

    //from_build_graph_node.event = patrolling_build_graph_msgs::BuildGraphEvent::STOP_PATROLLING;

    /* Define Starting Vertex/Position (Launch File Parameters) */

    //    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
    //    ros::NodeHandle nh;
    //    ros::NodeHandle n_("~");

    /* Set up listener for global coordinates of robots */
    // listener = new tf::TransformListener();

#if 0    
    // wait a random time (minimize possible conflicts with other robots starting at the same time...)
    double r = 3.0 * ((rand() % 1000) / 1000.0);
    ros::Duration wait(r); // seconds
    wait.sleep();
#endif    

    int value = ID_ROBOT_;
    if (value == -1)
    {
        ROS_ERROR_STREAM("ID_ROBOT: " << ID_ROBOT_);
        value = 0;
    }

    /// < init kdtree
    build_kdtree_nodes();        

    /// < find closest node to starting position 
    current_vertex_ = find_closest_node(); 

    std::cout << "found closest node: " << current_vertex_ << std::endl;

    std::cout << "Initializing both instantaneous_idleness and last_visit array" << std::endl;
        
    vec_instantaneous_idleness_ = new double[graph_dimension_];
    vec_global_instantaneous_idleness_ = new double[graph_dimension_];   
    vec_last_visit_time_ = new double[graph_dimension_];

    for (size_t i = 0; i < graph_dimension_; i++)
    {
        vec_instantaneous_idleness_[i] = 0.0;
        vec_last_visit_time_[i] = 0.0;
        
        vec_global_instantaneous_idleness_[i] = std::numeric_limits<double>::max();        

    }

    
    /// ========================================================================
    bool done = false;
    while (
            ( (build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT) ||
              (build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED) )
            && !done
          )
    {

        b_debug_ = getParam<bool>(node_, "debug", true);
        //distance_threshold_ = getParam<double>(node_, "robot_target_distance_threshold", 0.3);

        // publisher for sending position info 
        positions_pub_ = node_.advertise<nav_msgs::Odometry>("/positions", 1); //only concerned about the most recent

        // subcriber for getting position info from other robots 
        positions_sub_      = node_.subscribe<nav_msgs::Odometry>("/positions", 10, boost::bind(&PatrolAgent::positionsCallback, this, _1));
        core_positions_sub_ = node_.subscribe<nav_msgs::Odometry>("/core/positions", 10, boost::bind(&PatrolAgent::positionsCallback, this, _1));


        node_.getParam("traj_act_client_name", traj_act_client_name_);

        //std::string tmp = traj_act_client_name.substr(0, traj_act_client_name.size() - 1);
        //int k = traj_act_client_name[traj_act_client_name.size() - 1] - '0';
        //k++;
        std::stringstream ss;
        //ss << tmp << k;
        ss << traj_act_client_name_;
        std::cout << "traj_act_client_name param " << ss.str() << std::endl;
        traj_res_sub_ = node_.subscribe<trajectory_control_msgs::TrajectoryControlActionResult>(ss.str() + "/result", 10, boost::bind(&PatrolAgent::trajectoryTrackingResultCallback, this, _1));


        //check_interference_timeout = getParam<double>(n_, "check_interference_timeout", 5);
        node_.getParam("check_interference_timeout", check_interference_timeout_);

        // HERE is where topic names are defined
        //        char string1[40]; // "vrep/ugv%d/odom"
        //        //char string2[40];
        //        char move_string[40]; // "vrep/ugv%d/goal_topic"
        //        //char move_string2[40];
        //        char planning_goal_abort_topic[40]; // vrep/ugv%d/goal_abort_topic
        //        char nodes_topic_name[256]; // "vrep/ugv%d/patrolling_nodes_updated_markers"
        //        char path_plan_stat_topic[256]; // "vrep/ugv%d/path_planning_status"


        // HERE is where topic names are defined
        node_.getParam("odom_frame_topic", odom_frame_topic_); // "vrep/ugv%d/odom"        
        node_.getParam("goal_topic", goal_topic_); // "vrep/ugv%d/goal_topic"
        node_.getParam("planning_goal_abort_topic", planning_goal_abort_topic_); // vrep/ugv%d/goal_abort_topic

        node_.getParam("path_plan_stat_topic", path_plan_stat_topic_); // "vrep/ugv%d/path_planning_status"
        //node_.getParam("patrolling_pause_topic", patrolling_pause_topic_); // "vrep/ugv%d/path_planning_status"


        patrolling_pause_sub_ = node_.subscribe(patrolling_pause_topic_, 20, &PatrolAgent::pausePatrollingCallback, this);

        if (ID_ROBOT_ == -1)
        {
            //            strcpy(string1, "odom"); //string = "odom"
            //            //strcpy (string2,"cmd_vel"); //string = "cmd_vel"
            //            strcpy(move_string, "move_base");
            TEAMSIZE_ = 1;
        }
        else
        {
            //            //sprintf(string1,"vrep/robot_%d/odom",ID_ROBOT);
            //            //sprintf(string2,"vrep/robot_%d/cmd_vel",ID_ROBOT);
            //            //sprintf(move_string,"vrep/robot_%d/goal_topic",ID_ROBOT);
            //            sprintf(string1, "vrep/ugv%d/odom", ID_ROBOT + 1);
            //            //sprintf(string2,"vrep/ugv%d/cmd_vel",ID_ROBOT+1);
            //            sprintf(move_string, "vrep/ugv%d/goal_topic", ID_ROBOT + 1);
            //            //sprintf(move_string2,"vrep/ugv%d/tracks_vel_cmd",ID_ROBOT+1);
            //            sprintf(planning_goal_abort_topic, "vrep/ugv%d/goal_abort_topic", ID_ROBOT + 1);
            //            sprintf(nodes_topic_name, "vrep/ugv%d/patrolling_nodes_updated_markers", ID_ROBOT + 1);
            //            sprintf(path_plan_stat_topic, "vrep/ugv%d/path_planning_status", ID_ROBOT + 1);
            TEAMSIZE_ = ID_ROBOT_ + 1; /// < N.B.: recall that the ID_ROBOT of the first robot is "0"
        }

        goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);

        // act_client_ = new PlannerClient(ss.str(),true);

        //Subscrever para obter dados de "odom" do robot corrente
        odom_sub_ = node_.subscribe<nav_msgs::Odometry>(odom_frame_topic_, 1, boost::bind(&PatrolAgent::odomCallback, this, _1)); //size of the buffer = 1 (?)

        planning_goal_abort_pub_ = node_.advertise<std_msgs::Bool>(planning_goal_abort_topic_, 10);

        path_plan_stat_sub_ = node_.subscribe(path_plan_stat_topic_, 20, &PatrolAgent::pathPlanningFeedbackCallback, this);

        ros::spinOnce();

        //Publicar dados para "results"
        results_pub_ = node_.advertise<Int16MultiArrayMsg>("/results", 100);
        
        // results_sub = n_.subscribe("results", 10, resultsCB); //Subscrever "results" vindo dos robots
        results_sub_      = node_.subscribe<Int16MultiArrayMsg>("/results", 100, boost::bind(&PatrolAgent::resultsCallback, this, _1)); //Subscrever "results" vindo dos robots
        core_results_sub_ = node_.subscribe<Int16MultiArrayMsg>("/core/results", 100, boost::bind(&PatrolAgent::resultsCallback, this, _1)); //Subscrever "results" vindo dos robots

        // last time comm delay has been applied
        last_communication_delay_time_ = ros::Time::now().toSec();

        readParams();
        ROS_INFO("END INIT OK");
        
        done = true;
    }
    
    
    buildPatrollingEdgesAsMarkers();
    //build_kdtree_nodes();
            
    if(ID_ROBOT_ == -1)
    {
        ROS_ERROR_STREAM("robot id has not been correctly assigned"); 
        quick_exit(-1);
    }

}

void PatrolAgent::buildGraphEventCallback(const patrolling_build_graph_msgs::BuildGraphEvent::ConstPtr& msg)
{
    if (build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING)
    {
        build_graph_event_ = *msg;
    }
}

void PatrolAgent::graphCallback(const patrolling_build_graph_msgs::Graph::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    if (build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING)
    {
        GetGraphFromMsg(graph_, graph_dimension_, msg);
        build_graph_event_.event = patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED;
        ROS_INFO_STREAM("PatrolAgent::graphCallback() - graph received");
    }
}


void PatrolAgent::get_ready()
{
    //    char move_string[40];

    /* Define Goal */
    //    if(ID_ROBOT==-1){ 
    //        strcpy (move_string,"move_base"); //string = "move_base
    //    }else{
    //sprintf(move_string,"robot_%d/move_base",ID_ROBOT);
    //	sprintf(move_string,"vrep/robot_%d/goal_topic",ID_ROBOT);
    //    }

    //    ros::Publisher goal_pub = 
    //act_client_ = new PlannerClient(traj_act_client_name,true);
    //ac = new MoveBaseClient(move_string, true); 


    //wait for the action server to come up
    //    while(!act_client_->waitForServer(ros::Duration(5.0))){
    //        ROS_INFO("Waiting for the Planner action server to come up");
    //    } 
    //    ROS_INFO("Connected with Planner action server");

    ros::Rate loop_rate(1); //1 second

    /// < wait until all nodes are ready... 
    while (b_waiting_init_msg_)
    {
        // say hello to the team and patrolling monitor
        broadcast_initialize_node(); 
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void PatrolAgent::readParams()
{
    if (!ros::param::get("/goal_reached_wait", goal_reached_waiting_time_))
    {
        goal_reached_waiting_time_ = 0.0; //0.0;
        ROS_WARN_STREAM("Cannot read parameter /goal_reached_wait. Using default value: " << goal_reached_waiting_time_);
        //ros::param::set("/goal_reached_wait", goal_reached_wait);
    }

    if (!ros::param::get("/communication_delay", communication_delay_))
    {
        communication_delay_ = 0.0; //0.0;
        ROS_WARN_STREAM("Cannot read parameter /communication_delay. Using default value: " << communication_delay_);
        //ros::param::set("/communication_delay", communication_delay);
    }

    if (!ros::param::get("/lost_message_rate", lost_message_rate_))
    {
        lost_message_rate_ = 0.0;
        ROS_WARN_STREAM("Cannot read parameter /lost_message_rate. Using default value: " << lost_message_rate_);
        //ros::param::set("/lost_message_rate", lost_message_rate);
    }

}

void PatrolAgent::run()
{
    long unsigned int time_index = 0; 
    
    /// < get ready
    get_ready();
    
    time_zero_ = ros::Time::now().toSec();
    last_update_global_idl_ = time_zero_;
    
    time_last_path_planning_success_ = time_zero_;
    time_last_no_node_conflict_      = time_zero_; 
    time_last_reached_goal_          = time_zero_; 
            
    // Asynch spinner (non-blocking)
    ros::AsyncSpinner spinner(2); // Use n threads
    spinner.start();
    // ros::waitForShutdown();
    
    /// < now we have initialized the important data structures and time variables 
    
    b_have_to_init_ = false;

    /// < start the algorithm 

    if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Yellow(), "Ready");
    
    ros::Rate loop_rate(kPatrolAgentLoopRate); //0.033 seconds or 30Hz

    while (ros::ok())
    {

        switch (build_graph_event_.event)
        {

        case patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT:
        case patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED:

            /// < update patrolling graph status (in any case, even if paused)
            update_idleness();
            
            if( time_index % kPatrolAgentLoopRate == 0 )
            {
                ROS_INFO_STREAM("PatrolAgent::run() - goal_complete: " << (int)b_goal_complete_ 
                        << ", interference: " << (int)b_interference_ 
                        << ", node conflict: "<< (int)b_node_conflict_
                        << ", planner_status: " << (int)planner_status_.success 
                        //<< ", resend goal: "<< (int)b_resend_goal_
                        //<< ", timeout goal: " << (int)b_timeout_for_reaching_goal_
                        ); 
                ROS_INFO_STREAM("PatrolAgent::run() - next vertex: " << next_vertex_ << ", cost: " << planner_status_.path_cost);
            }
            

            if (!b_pause_)
            {                
                //if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Yellow(), "Patrolling");
                
                /// < check if this robot covered/intercepted a vertex 
                int covered_vertex = -1;                
                if( time_index % kCheckVisitedVertexSubrate == 0 )
                {
                    bool b_new_interception = false; 
                    // check if 1) we are currently over a vertex (distance <= kVisitedNodeDistance, b_new_interception = false)
                    //          2) we are visiting a vertex along the way to the goal (must be different from current_vertex_ and next_vertex_)  (b_new_interception = true)
                    covered_vertex = check_vertex_interception(current_vertex_, next_vertex_, list_intercepted_vertexes_,b_new_interception);
                    if(covered_vertex > -1)  
                    {
                        broadcast_vertex_covered(covered_vertex);
                        if(b_new_interception) broadcast_goal_intercepted(covered_vertex);
                    }
                    //broadcast_visited_vertexes(list_intercepted_vertexes_);
                    
                    covered_vertex_ = covered_vertex; // update corresponding class field
                }
              
                { // < start path planner status lock ==========================
                boost::recursive_mutex::scoped_lock tasks_locker(planner_status_mutex_);

                const double time_now = ros::Time::now().toSec() - time_zero_;
                              
                
                /// < check node conflict  
                b_node_conflict_ = check_vertex_conflict(ID_ROBOT_,next_vertex_,planner_status_.path_cost);
                if(!b_node_conflict_)
                {
                    time_last_no_node_conflict_ = time_now; 
                }
                
                
                /// < check critical node conflict 
                b_critical_node_conflict_ = b_node_conflict_ &&  (time_now - time_last_no_node_conflict_ > kTimeIntervalForCriticalNodeConflict);
                if(b_critical_node_conflict_forced_)
                {
                    b_critical_node_conflict_ = true;
                    b_node_conflict_ = true;
                    b_critical_node_conflict_forced_ = false;
                }
#if UPDATE_PATROLLING_MAKER  
                if( b_critical_node_conflict_ )
                {
                    if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Red(), "Critical Node Conflict");  
                }
#endif                      
                
                /// < check interference (not used in the planning)
                if( time_index % kCheckInterferenceSubrate == 0 )
                {
                    b_interference_ = check_interference(ID_ROBOT_); /// this was managed inside trajResultCallBack()
                }
                
                
                /// < broadcast new received nav_cost and path planning status 
                if(path_planner_status_message_updated_)
                {
                    
                    // broadcast when new info comes
                    if(planner_status_.success)
                    {                         
                        if(covered_vertex > -1) 
                        {
                            // broadcast nav cost for updating the graph (from a covered vertex to next_vertex_)
                            broadcast_nav_cost(covered_vertex, next_vertex_, planner_status_.path_cost);                        
                        }
                        
                        // broadcast normal select message 
                        broadcast_goal_selected(next_vertex_,planner_status_.path_cost);     
                        
#if UPDATE_PATROLLING_MAKER  
                        std::stringstream message;
                        message << "PP Success (N: " << next_vertex_ << ") ";
                        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Green(), message.str());                    
#endif                          
                    }
                    else
                    {
                        broadcast_goal_abort(next_vertex_);
                        
#if UPDATE_PATROLLING_MAKER  
                        std::stringstream message;
                        message << "PP Failure (N: " << next_vertex_ << ") ";                        
                        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Red(), message.str());                    
#endif                   
                        
                    }
                    path_planner_status_message_updated_ = false; 
                }
                else
                {
                    // broadcast continuously selected message (according to sub-rate)
                    if( ( time_index % kSelectBroadcastSubrate == 0 ) && (planner_status_.success) )
                    {
                        broadcast_goal_selected(next_vertex_,planner_status_.path_cost);
                    }                    
                }
                  
                
                /// < check critical path planning failure
                b_critical_path_planning_failure_ = b_path_planning_failure_ && // (!planner_status_.success) && 
                        ( time_now - time_last_path_planning_success_ > kTimeIntervalForCriticalPathPlanningFailure );                
#if UPDATE_PATROLLING_MAKER  
                if( b_critical_path_planning_failure_ )
                {
                    if(p_marker_controller) 
                    {
                        p_marker_controller->setMarkerColor(Colors::Red(), "Critical PP Failure");               
                    }
                }
#endif                      
                                
                
                /// < check if the path planner has not been sending any message for a long time
                /// < if activated we exit from here once a new path planning message is received 
                b_timeout_path_planning_reply_ = ( time_now - time_last_path_planning_msg_ > kTimeoutForLastPathPlanningMsg );
                if(b_timeout_path_planning_reply_) 
                {
                    ROS_WARN_STREAM("PatrolAgent::run() - time out for path planner reply");
                    // trigger a path planning failure and reset the variable 
                    b_path_planning_failure_       = true; // < N.B.: here we force a path planning failure 
                    b_timeout_path_planning_reply_ = false;                     
#if UPDATE_PATROLLING_MAKER  
                    if(p_marker_controller) 
                    {
                        p_marker_controller->setMarkerColor(Colors::Red(), "Timeout PP Reply");                    
                    }
#endif                                            
                }
                
                
# if DISABLE_COOPERATION_AND_COORDINATION
                b_node_conflict_ = false; // disable topological coordination 
#endif
                
                
                }// < end path planner status lock =============================
                
                
                // broadcast continuously idleness message (according to sub-rate)
                if( ( time_index % kIdlenessSynchBroadcastSubrate == 0 ) && ( time_index > 0) )
                {
                    broadcast_idleness();
                }      
                
                // check tasks reset (according to sub-rate)
                if( ( time_index % kCheckTaskResetSubrate == 0 ) && ( time_index > 0) )
                {
                    check_tasks_reset();
                }                     
                
                
                if (b_goal_complete_)
                {
#if 0                    
                    /// < reset node conflict since we've arrived in a new vertex
                    b_node_conflict_          = false; 
                    b_critical_node_conflict_ = false; 
                    b_critical_node_conflict_forced_ = false;
#endif
                                        
                    ROS_INFO_STREAM("PatrolAgent::run() - on goal complete");
                    
#if UPDATE_PATROLLING_MAKER                    
                    std::stringstream message;
                    message << "Goal Complete (N: " << next_vertex_ << ")" << std::endl; 
                    if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Green(), message.str());                    
#endif                    
                    
                    onGoalComplete(); // can be redefined
                    resend_goal_count_ = 0;
                    //intereference_count=0;
                    time_last_reached_goal_  = ros::Time::now().toSec() - time_zero_; 
                }
                else
                { /// < goal not complete (active)
                    
                    if (b_interference_)
                    {
                        broadcast_interference(); 
                        
                        ROS_INFO_STREAM("PatrolAgent::run() - interference");
#if 0                        
                        if (b_task_reallocation_)
                        {
                            /// < task reallocation 
                            //do_task_reallocation();
                        }
                        else
                        {
                            /// < normal management 

                        }
#endif 
                    }

#if 0                    
                    if (b_resend_goal_) // at present time never used  
                    {
                        //onGoalNotComplete(); /// < disabled for security
                        b_resend_goal_ = false; //para nao voltar a entrar (envia goal so uma vez)
                        resend_goal_count_ = 0;
                    }
#endif

                    //if( (!planner_status_.success) || b_node_conflict_) // || b_timeout_for_reaching_goal_)
                    if( b_path_planning_failure_ || b_node_conflict_ || b_next_vertex_intercepted_by_teammate_) // || b_timeout_for_reaching_goal_)
                    {
                        
                        // update your reference node id (in case the robot stopped its travel)
                        current_vertex_ = find_closest_node();
                        
#if UPDATE_PATROLLING_MAKER                             
                        std::stringstream message; 
                        if(b_path_planning_failure_) message << "PP Failure (N: " << next_vertex_ << ") " << std::endl;
                        if(b_node_conflict_) message << "Conflict (N: " << next_vertex_ << ") (R: " << (last_conflict_robot_+1) << ")" << std::endl;
                        if(b_next_vertex_intercepted_by_teammate_) message << "Goal Intercepted (N: " << next_vertex_ << ")" << std::endl;                       
                        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Red(), message.str());                            
#endif                        
                        
                        sendPathPlannerGoalAbortAndBroadcastAbort();
                        
                        //ros::Duration(kSleepTimeAfterSendingAbort).sleep();
                        
                        ROS_INFO_STREAM("ROBOT_ID " << ID_ROBOT_ 
                                << " path planner success: " <<  (int)planner_status_.success 
                                << ", node conflict: "<< (int)b_node_conflict_ 
                                //<< ", timeout goal: " << (int)b_timeout_for_reaching_goal_
                                << ", next vertex intercepted: " << (int) b_next_vertex_intercepted_by_teammate_ 
                                << ", => selecting another target node\n");
                        onGoalNotComplete();
                        
                        ros::Duration(kSleepTimeAfterSendingNewGoal).sleep();
                    }
                    

                    // --------------------------------------------------------
                    
                    processEvents();

                    if (b_end_simulation_)
                    {
                        ROS_INFO_STREAM("PatrolAgent::run() - end simulation");
                        
                        return;
                    }

                } // if (goal_complete)

            } // if(!b_pause_) 

            break;

        case patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING:
            std::cout << "Waiting for patrolling_build_graph_node.... the user is inserting points of interest to patrol" << std::endl;
            break;

        case patrolling_build_graph_msgs::BuildGraphEvent::STOP_PATROLLING:
            std::cout << "This option has not been implemented. In principle patrolling task never end" << std::endl;
            build_graph_event_.event = patrolling_build_graph_msgs::BuildGraphEvent::STOP_PATROLLING;
            break;

        default:
            std::cout << "Unknown event from patrolling_build_graph_node" << std::endl;

        }

        loop_rate.sleep();
        time_index++;
        
        ///ros::spinOnce();

    } // while ros.ok    
}

void PatrolAgent::onGoalComplete()
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " onGoalComplete - begin" << std::endl;
    if (next_vertex_ > -1)  // if next_vertex_ is valid
    {
        //Update Idleness Table:
        //update_idleness();
        //printf("Current vertex %d\n",current_vertex);
        //printf("Next vertex %d\n",next_vertex);
        current_vertex_ = next_vertex_;
    }

    //devolver proximo vertex tendo em conta apenas as idlenesses;
    /// < compute next vertex 
    if(!b_first_node_)
    {
        next_vertex_ = compute_next_vertex();
    }
    else
    {
        // first time selection 
        next_vertex_ = current_vertex_; /// < the closest node found at the beginning 
    }
    printf("Want to move robot to vertex %d (%f,%f,%f)\n", next_vertex_, graph_[next_vertex_].x, graph_[next_vertex_].y, graph_[next_vertex_].z);

    
    /// < broadcast goal reached
    if(!b_first_node_)
    {
        broadcast_goal_reached(); // Send TARGET to monitor
        send_results(); // Algorithm specific function
    }

    //Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f,%f)\n", next_vertex_, graph_[next_vertex_].x, graph_[next_vertex_].y,graph_[next_vertex_].z);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendPathPlannerGoalAndBroadcastIntention(next_vertex_); // send to path planner

    b_goal_complete_ = false;
    b_first_node_    = false;
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " onGoalComplete - end" << std::endl;
}


void PatrolAgent::onGoalNotComplete()
{
    //int prev_vertex = next_vertex_;
    
    ROS_INFO("Goal not complete - From vertex %d to vertex %d\n", current_vertex_, next_vertex_);

    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex_ = compute_next_vertex();

    if( next_vertex_ != -1 )
    {
        ROS_INFO("Goal not complete - sending NEW goal - Vertex %d (%f,%f)\n", next_vertex_, graph_[next_vertex_].x, graph_[next_vertex_].y);
        sendPathPlannerGoalAndBroadcastIntention(next_vertex_); // send to path planner
    }
    else
    {
        ROS_WARN("Goal not complete - selected invalid goal");
    }
    
    b_goal_complete_ = false;
}

int PatrolAgent::compute_random_vertex()
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    int prev_vertex = next_vertex_;

    ROS_INFO("RandomChoice - From vertex %d to vertex %d\n", current_vertex_, next_vertex_);

    // Look for a random adjacent vertex different from the previous one
    int random_cnt = 0; 
    while (next_vertex_ == prev_vertex && random_cnt++ < 10)
    { 
        int num_neighs = graph_[current_vertex_].num_neigh;
        int i = rand() % num_neighs;
        next_vertex_ = graph_[current_vertex_].id_neigh[i];
        ROS_INFO("RandomChoice::Choosing another random vertex %d\n", next_vertex_);
    }

    // Look for any random vertex different from the previous one
    while (next_vertex_ == prev_vertex && next_vertex_ == current_vertex_)
    {
        int i = rand() % graph_dimension_;
        next_vertex_ = i;
        ROS_INFO("RandomChoice::Choosing another random vertex %d\n", next_vertex_);
    }

    //Send the goal to the robot (Global Map)
    //ROS_INFO("Re-Sending NEW goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    //sendGoal(next_vertex); // send to move_base

    //goal_complete = false;
    return next_vertex_;
}

void PatrolAgent::processEvents()
{

}

void PatrolAgent::update_idleness()
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
    
    boost::recursive_mutex::scoped_lock idleness_locker(idleness_mutex_);    
    
    //boost::recursive_mutex::scoped_lock global_idleness_locker(global_idleness_mutex_);    
            
    double now = ros::Time::now().toSec() - time_zero_;
 
#if 0 
    std::cout << ".............................................................\n";    
    std::cout << "............   update_idleness()   ..........................\n";    
    std::cout << "Current VERTEX ID: " << current_vertex_ << std::endl;
    std::cout << "Next VERTEX ID: " << next_vertex_ << std::endl;
#endif
    
    for (size_t i = 0; i < graph_dimension_; i++)
    {

        if ((int) i == next_vertex_ && b_goal_complete_)
        {
            vec_last_visit_time_[i] = now;
#if 0
            std::cout << "(Next_vertex,last_visit)=(" << next_vertex_ << "," << vec_last_visit_time_[i] - time_zero_ << ")" << std::endl;
#endif
        }
#if 0      
        else
        {
            std::cout << "(Vertex,last_visit)=(" << i << "," << vec_last_visit_time_[i] - time_zero_ << ")" << std::endl;
        }
#endif        

        vec_instantaneous_idleness_[i] = graph_[i].priority*(now - vec_last_visit_time_[i]);
      
#if 0
        std::cout << "(Vertex,instantaneous_idleness)=(" << i << "," << vec_instantaneous_idleness_[i] << ")" << std::endl;
        std::cout << ".............................................................\n";
#endif
    } 
    
    buildPatrollingNodesAsMarkers();
    
    publishPatrollingNodesAsMarkers();
    
    publishPatrollingEdgesAsMarkers(); 
}

void PatrolAgent::check_tasks_reset()
{
    boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
    ros::Time time_now = ros::Time::now(); 
    
    for (int i = 0; i < TEAMSIZE_; i++)
    {
        if( tasks_.isValid(i) )
        {
            const double delta = (time_now - tasks_.timestamp[i]).toSec();  
            if( delta > kExpirationTimeTeamModel )
            {
                std::cout << "PatrolAgent::check_tasks_reset() - resetting task for robot: " << i << std::endl; 
                tasks_.resetWithTime(i,time_now);                
            }
        }
    }      
}

void PatrolAgent::broadcast_initialize_node()
{ //ID,msg_type,1

    std::cout << ".......................................................\n";    
    ROS_INFO("Initialize Node: Robot %d", ID_ROBOT_);

    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(INITIALIZE_MSG_TYPE);
    msg.data.push_back(SUB_MSG_ROBOT_INIT); // Robot initialized

    int count = 0;

    //ATENÇÃO ao PUBLICADOR!
    ros::Rate loop_rate(0.5); //meio segundo

    while (count < 3)
    { //send activation msg 3times
        ROS_INFO("PatrolAgent::initialize_node");
#if 0        
        results_pub_.publish(msg);
        //        std::cout << "Publiquei msg: ";
        //        for(int i = 0; i < msg.data.size(); i++){
        //            std::cout << msg.data[i] << " ";
        //        }
        std::cout << std::endl;
        ros::spinOnce();
#endif        
        
        do_send_message(msg);
        
        loop_rate.sleep();
        count++;
    }
    std::cout << ".......................................................\n";    
}

/// < this function is only used by the DTAGreedy_Agent.cpp

void PatrolAgent::getRobotPose(int robotid, float &x, float &y, float &theta)
{

    if (p_listener_ == NULL)
    {
        ROS_ERROR("TF listener null");
        return;
    }

    //std::stringstream ss; ss << "robot_" << robotid;
    std::stringstream ss;
    ss << "ugv" << (robotid + 1);
    std::string robotname = ss.str();
    std::string sframe = "map"; //global map frame is "map"

    std::string dframe = robotname + "/base_link";
    //std::string dframe = robot_frame_;

    tf::StampedTransform transform;

    try
    {
        p_listener_->waitForTransform(sframe, dframe, ros::Time(0), ros::Duration(3));
        p_listener_->lookupTransform(sframe, dframe, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot transform from %s to %s\n", sframe.c_str(), dframe.c_str());
        ROS_ERROR("%s", ex.what());
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    theta = tf::getYaw(transform.getRotation());
    //printf("Robot %d pose : %.1f %.1f \n",robotid,x,y);
}

void PatrolAgent::getRobotPose3D(int robotid, float &x, float &y, float &z, float &theta)
{

    if (p_listener_ == NULL)
    {
        ROS_ERROR("TF listener null");
        return;
    }

    //std::stringstream ss; ss << "robot_" << robotid;
    std::stringstream ss;
    ss << "ugv" << (robotid + 1); 
    std::string robotname = ss.str();
    std::string sframe = "map"; //global map frame is "map"

    std::string dframe = robotname + "/base_link";
    //std::string dframe = robot_frame_;

    //std::cout << "ROBOT ID " << ID_ROBOT << " getRobotPose3D robot_frame: " << dframe << std::endl;
    tf::StampedTransform transform;

    try
    {
        p_listener_->waitForTransform(sframe, dframe, ros::Time(0), ros::Duration(3));
        p_listener_->lookupTransform(sframe, dframe, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot transform from %s to %s\n", sframe.c_str(), dframe.c_str());
        ROS_ERROR("%s", ex.what());
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    z = transform.getOrigin().z();
    theta = tf::getYaw(transform.getRotation());
    //printf("Robot %d pose : %.1f %.1f \n",robotid,x,y);
}

void PatrolAgent::pathPlanningFeedbackCallback(const trajectory_control_msgs::PlanningStatus::ConstPtr& msg)
{
    std::cout << "..............................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " pathPlanningFeedbackCallback() - begin \n";
        
    boost::recursive_mutex::scoped_lock tasks_locker(planner_status_mutex_);
    
    // copy the incoming message 
    planner_status_ = *msg;
    
    time_last_path_planning_msg_ = ros::Time::now().toSec() - time_zero_; 
    
    if(planner_status_.path_cost > -1)  
    {
        planner_status_.path_cost *= kScalePathNavCostFromFloatToInt;
    }
    
#if 0    
    std::cout << "ROBOT_ID: " << ID_ROBOT << " pathPlanningFeedbackCallback..................\n";
    std::cout << "ROBOT_ID: " << ID_ROBOT << " planner_status_ " << (bool)planner_status_.data << "\n";
#endif    
    
    {
    boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
    
    if(planner_status_.success)
    {
        //if(planner_status_.status == trajectory_control_msgs::PlanningStatus::kFirstSuccess)

        tasks_.id_current_selected_vertex[ID_ROBOT_] = next_vertex_; 
        tasks_.nav_cost_to_go[ID_ROBOT_] = planner_status_.path_cost;
        tasks_.timestamp[ID_ROBOT_] = planner_status_.header.stamp;
        
        time_last_path_planning_success_ = ros::Time::now().toSec() - time_zero_; 
        
        b_path_planning_failure_ = false; 
        
        if(planner_status_.status == trajectory_control_msgs::PlanningStatus::kArrived)
        {
            std::cout << "ROBOT_ID " << ID_ROBOT_ << " pathPlanningFeedbackCallback() - goal reached \n";
            b_goal_complete_ = true;
            
            // check the reached goal is correct 
            const int arrived_vertex = find_closest_node();
            if( arrived_vertex != next_vertex_ )
            {
                ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " pathPlanningFeedbackCallback() - wrong goal reached");                
                current_vertex_ = arrived_vertex;
                b_goal_complete_ = false; // avoid broadcast of wrong messages 
                b_path_planning_failure_ = false; // force the selection of a new vertex 
#if UPDATE_PATROLLING_MAKER  
                if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Red(), "Wrong Goal!");                  
#endif                   
            }
            
        }
    }
    else
    {
        // reset target node 
        tasks_.id_current_selected_vertex[ID_ROBOT_] = -1;
        tasks_.nav_cost_to_go[ID_ROBOT_] = -1;
        tasks_.timestamp[ID_ROBOT_] = planner_status_.header.stamp;
        
        b_path_planning_failure_ = true; 
    }
    
    path_planner_status_message_updated_ = true; 
    
    } // end boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
   
    std::cout << "ROBOT_ID: " << ID_ROBOT_ << " planner_status " << (bool)planner_status_.success << " path_cost " << planner_status_.path_cost <<"\n";
    
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " pathPlanningFeedbackCallback() - end \n";
    std::cout << "..............................................................\n";
}

void PatrolAgent::patrollingNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock nodes_marker_array_locker(marker_nodes_array_mutex_);
    marker_nodes_array_.markers.clear();
    marker_nodes_array_ = *msg;
    ROS_INFO_STREAM("patrollingNodesCallback() - received patrolling nodes");

}

void PatrolAgent::trajectoryTrackingResultCallback(const trajectory_control_msgs::TrajectoryControlActionResult::ConstPtr& msg)
{
    std::cout << "..............................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " trajectoryTrackingResultCallback - begin \n";
    traj_res_msg_ = *msg;

    {
    boost::recursive_mutex::scoped_lock tasks_locker(planner_status_mutex_);
    
    if (planner_status_.success)
    {
        if(traj_res_msg_.result.done) //|| (planner_status_.status == trajectory_control_msgs::PlanningStatus::kArrived) )  // if trajectory completed
        { 
            /// < current goal is running then wait 

            std::cout << "ROBOT_ID " << ID_ROBOT_ << " Goal reached ... WAITING " << goal_reached_waiting_time_ << " sec\n";
            ros::Duration delay(goal_reached_waiting_time_); // wait after goal is reached
            delay.sleep();
            std::cout << "ROBOT_ID " << ID_ROBOT_ << " Goal reached ... DONE\n";
            b_goal_complete_ = true;
        }
    }
    else
    {
        /// < path planning failed 
        //b_random_vertex_       = true; 
        //planner_status_.success = true;
        std::cout << "ROBOT_ID " << ID_ROBOT_ << " Path not found - random_vertex = true - selecting another target node\n";
    }

    }
    
    /// < send current position to the other robot 
    broadcast_positions();
    
    /// < check interference 
    //b_interference_ = check_interference(ID_ROBOT_);
    
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " trajectoryTrackingResultCallback - end \n";
    std::cout << "..............................................................\n";

}

void PatrolAgent::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ //colocar propria posicao na tabela

    //printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
    int idx = ID_ROBOT_;

    if (ID_ROBOT_ <= -1)
    {
        idx = 0;
    }

    float x, y, z, th;
    getRobotPose3D(idx, x, y, z, th);

    xPos_[idx] = x; // msg->pose.pose.position.x;
    yPos_[idx] = y; // msg->pose.pose.position.y;
    zPos_[idx] = z; // msg->pose.pose.position.y;
    //std::cout << "Robot position x " << xPos[idx] << " y " <<  yPos[idx] << std::endl;

    //printf("Posicao colocada em Pos[%d]\n",idx);
}

void PatrolAgent::sendPathPlannerGoalAndBroadcastIntention(int next_vertex)
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    if(next_vertex == -1) return;
    
    //std::cout << "sendGoal - begin" << std::endl;
    //b_goal_aborted_ = false;

    /// < send goal to path planner 
    double target_x = graph_[next_vertex].x;
    double target_y = graph_[next_vertex].y;
    double target_z = graph_[next_vertex].z;
    
    goal_msg_.header.frame_id = "map";
    goal_msg_.header.stamp = ros::Time::now();
    goal_msg_.pose.position.x = target_x; // vertex_web[current_vertex].x;
    goal_msg_.pose.position.y = target_y; // vertex_web[current_vertex].y;
    goal_msg_.pose.position.z = target_z;

    //    if (interactive) { // TO BE FIXED
    //        double target_z = vertex_web[next_vertex].z;
    //        goal.pose.position.z = target_z;
    //    }

    geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);
    goal_msg_.pose.orientation = angle_quat;
    goal_pub_.publish(goal_msg_);

    
    /// < broadcast intention to other robots
    broadcast_goal_intention(next_vertex); 
    
    
    /// < reset relevant variables 
    
    { // start lock path planner status
    boost::recursive_mutex::scoped_lock tasks_locker(planner_status_mutex_);
    
    // reset timeout checking
    if(b_timeout_for_reaching_goal_) b_timeout_for_reaching_goal_ = false;
    
    // reset path planning failure checking once we send a new goal  
    b_path_planning_failure_ = false;
    
    // reset the current path planning cost 
    planner_status_.path_cost = -1;
        
    } // end lock path planner status
        
    b_next_vertex_intercepted_by_teammate_ = false;   
    list_intercepted_vertexes_.clear(); 
     
}

void PatrolAgent::sendPathPlannerGoalAbortAndBroadcastAbort()
{

    boost::recursive_mutex::scoped_lock tasks_locker(planner_status_mutex_);    

    std_msgs::Bool msg;
    msg.data = true;
    planning_goal_abort_pub_.publish(msg);
    
    broadcast_goal_abort(next_vertex_);

    //b_goal_aborted_ = true;
}

void PatrolAgent::goalActiveCallback()
{ //enquanto o robot esta a andar para o goal...
    b_goal_complete_ = false;
    //      ROS_INFO("Goal is active.");
}

void PatrolAgent::broadcast_goal_reached()
{
    /// < [ID_ROBOT, msg_type, vertex] 

    std::cout << ".......................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_reached()\n";
    
    if(current_vertex_ == -1) 
    {
        ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_reached() with current_vertex -1");
        return;
    }
    
    if( (current_vertex_ < graph_dimension_) && (current_vertex_ > -1) )
    {                                 
        boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
        vec_last_visit_time_[current_vertex_] = ros::Time::now().toSec() - time_zero_; 
    }    
    
    // update team model immediately 
    {
    boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);        
    tasks_.resetWithTime(ID_ROBOT_, ros::Time::now());    
    }    

    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(TARGET_REACHED_MSG_TYPE);
    msg.data.push_back(current_vertex_);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?
    //ROS_INFO("PatrolAgent::send_goal_reached");

    std::cout << "ROBOT_ID " << ID_ROBOT_ << " TARGET_REACHED_MSG_TYPE current_vertex " << current_vertex_ << "\n";
    std::cout << ".......................................................\n";
    
    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);    
}

// aka broadcast_goal_planned()
void PatrolAgent::broadcast_goal_intention(int next_vertex)
{
    /// < [ID_ROBOT, msg_type, vertex] 

    std::cout << ".......................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_intention()\n";
    
    if(next_vertex == -1) 
    {
        ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_intention() with next_vertex -1");
        return;
    }

    // udpate team model immediately 
    {
    boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);        
    tasks_.id_current_selected_vertex[ID_ROBOT_] = next_vertex;
    tasks_.nav_cost_to_go[ID_ROBOT_]             = -1; // still an intention
    tasks_.timestamp[ID_ROBOT_]                  = ros::Time::now();
    }    

    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(TARGET_PLANNED_MSG_TYPE);
    msg.data.push_back(next_vertex);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?
    //ROS_INFO("PatrolAgent::send_goal_reached");

    std::cout << "ROBOT_ID " << ID_ROBOT_ << " CURRENT_TARGET_MSG_TYPE next_vertex " << next_vertex << "\n";
    std::cout << ".......................................................\n";
    
    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);
}

void PatrolAgent::broadcast_goal_selected(int next_vertex, int nav_cost)
{
    /// < [ID_ROBOT, msg_type, vertex, nav_cost] 

    std::cout << ".......................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_selected()\n";
    
    if(next_vertex == -1) 
    {
        ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_selected() with next_vertex -1");
        return;
    }
    
    // udpate team model immediately 
    {
    boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);        
    tasks_.id_current_selected_vertex[ID_ROBOT_] = next_vertex;
    tasks_.nav_cost_to_go[ID_ROBOT_]             = nav_cost; 
    tasks_.timestamp[ID_ROBOT_]                  = ros::Time::now();
    }      
    
    Int16MultiArrayMsg msg; // -1,msg_type,100,0,0
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(TARGET_SELECTED_MSG_TYPE);
    msg.data.push_back(next_vertex); 
    msg.data.push_back(nav_cost); 
    
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " TARGET_SELECTED_MSG_TYPE next_vertex " << next_vertex << ", cost: " << nav_cost <<  "\n";
    std::cout << ".......................................................\n";
    
    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);
}

void PatrolAgent::broadcast_nav_cost(int vertex1, int vertex2, int nav_cost)
{
    /// < [ID_ROBOT, msg_type, vertex1, vertex2, nav_cost] 

    std::cout << ".......................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " broadcast_nav_cost()\n";
    
    if( !((vertex1 > -1) && (vertex1 < graph_dimension_ )) || !((vertex2 > -1) && (vertex2 < graph_dimension_ )) )
    {
        ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " broadcast_nav_cost() with invalid vertexes ");
        return;
    }
    
    // udpate graph immediately 
    updateDynNavCost(vertex1,vertex2,nav_cost,ros::Time::now());  
    
    
    Int16MultiArrayMsg msg; // -1,msg_type,100,0,0
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(NAV_COST_MSG_TYPE);
    msg.data.push_back(vertex1); 
    msg.data.push_back(vertex2);     
    msg.data.push_back(nav_cost); 
    
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " NAV_COST_MSG_TYPE vertex1: " << vertex1 << ", vertex2: " << vertex1 << ", cost: " << nav_cost <<  "\n";
    std::cout << ".......................................................\n";
    
    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);
}


void PatrolAgent::broadcast_goal_abort(int next_vertex)
{
    /// < [ID_ROBOT, msg_type, vertex] 

    std::cout << ".......................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_abort()\n";
    
    if(next_vertex == -1) 
    {
        ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_abort() with next_vertex -1");
        return;
    }
    
    // update team model immediately 
    {
    boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);        
    tasks_.resetWithTime(ID_ROBOT_, ros::Time::now());    
    }
    
    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(TARGET_ABORTED_MSG_TYPE);
    msg.data.push_back(next_vertex);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?
    //ROS_INFO("PatrolAgent::send_goal_reached");

    std::cout << "ROBOT_ID " << ID_ROBOT_ << " TARGET_ABORTED_MSG_TYPE next_vertex " << next_vertex << "\n";
    std::cout << ".......................................................\n";
    
    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);
}


void PatrolAgent::broadcast_goal_intercepted(int vertex)
{
    /// < [ID_ROBOT, msg_type, vertex] 

    std::cout << ".......................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_visited()\n";
    
    if(vertex == -1) 
    {
        ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_visited() with next_vertex -1");
        return;
    }
    
    if ( (vertex < graph_dimension_) && (vertex > -1) ) 
    {
        boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
        vec_last_visit_time_[vertex] = ros::Time::now().toSec() - time_zero_;
    }

    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(TARGET_INTERCEPTED_MSG_TYPE);
    msg.data.push_back(vertex);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?
    //ROS_INFO("PatrolAgent::send_goal_reached");

    std::cout << "ROBOT_ID " << ID_ROBOT_ << " TARGET_INTERCEPTED_MSG_TYPE next_vertex " << vertex << "\n";
    std::cout << ".......................................................\n";
    
    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);
}


void PatrolAgent::broadcast_vertex_covered(int vertex)
{
    /// < [ID_ROBOT, msg_type, vertex] 

    std::cout << ".......................................................\n";
    std::cout << "ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_covered()\n";
    
    if(vertex == -1) 
    {
        ROS_ERROR_STREAM("ROBOT_ID " << ID_ROBOT_ << " broadcast_goal_covered() with next_vertex -1");
        return;
    }
    
    if ( (vertex < graph_dimension_) && (vertex > -1) ) 
    {
        {
        boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
        vec_last_visit_time_[vertex] = ros::Time::now().toSec() - time_zero_;
        }
        
        {
        boost::recursive_mutex::scoped_lock locker(team_deployment_.mutex);
        team_deployment_.set(ID_ROBOT_,vertex,ros::Time::now());
        }        
    }        
    

    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(VERTEX_COVERED_MSG_TYPE);
    msg.data.push_back(vertex);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?
    //ROS_INFO("PatrolAgent::send_goal_reached");

    std::cout << "ROBOT_ID " << ID_ROBOT_ << " TARGET_COVERED_TYPE next_vertex " << vertex << "\n";
    std::cout << ".......................................................\n";
    
    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);
}

void PatrolAgent::broadcast_intercepted_vertexes(const std::vector<int>& list)
{
    for(size_t i=0;i<list.size();i++) broadcast_goal_intercepted(list[i]);
}
    

bool PatrolAgent::check_interference(int robot_id)
{ 
    //verificar se os robots estao proximos
    //ROS_INFO("check_interference - start");
    int i;
    double dist_quad;

    //secure_dist = false;
    
    //static double first_time = ros::Time::now().toSec();
    //std::cout << "PatrolAgent::check_interference() - current time: " << ros::Time::now().toSec() - first_time << std::endl; 

    /// < check if a reset of the interference counter is needed 
    if( (ros::Time::now().toSec() - last_interference_time_) < check_interference_timeout_)
    { // previous 10 seconds

        //ROS_INFO("check_interference - end -> ros::Time::now().toSec()-last_interference<10 FALSE");
        std::cout << "ROBOT_ID " << robot_id << " ros::Time::now().toSec() - last_interference < 10 -> TRUE - reset interference count\n";
        intereference_count_ = 0;
        return false; // false if within 10 seconds from the last one
    }

    float x, y, z, th;
    getRobotPose3D(robot_id, x, y, z, th);
    xPos_[robot_id] = x;
    yPos_[robot_id] = y;
    zPos_[robot_id] = z;

    for (i = 0; i < TEAMSIZE_; i++)
    { 
        /// < Get the priority w.r.t. robot with lower ID
        if (i != robot_id)
        {

            float x, y, z, th;
            getRobotPose3D(i, x, y, z, th); 
            xPos_[i] = x;
            yPos_[i] = y;
            zPos_[i] = z;

            dist_quad = pow((xPos_[i] - xPos_[robot_id]), 2) + pow((yPos_[i] - yPos_[robot_id]), 2) + pow((zPos_[i] - zPos_[robot_id]), 2);

            std::cout << "......................................................\n";
            std::cout << "ROBOT_ID " << robot_id << " check_interference........\n";
            std::cout << "ROBOT_ID " << robot_id << " actual distance^2 between ROBOT_ID " << robot_id << " and robot_id " << i << " -> " << dist_quad << std::endl;
            std::cout << "......................................................\n";

            if (dist_quad <= kInterferenceDistance2)
            {
                last_interference_time_ = ros::Time::now().toSec();

                intereference_count_++;
                ROS_INFO("check_interference - end Number of interferences %d", intereference_count_);
                return true; /// < EXIT POINT 
            }
        }
    }

    intereference_count_ = 0;
    return false;

}



bool PatrolAgent::check_vertex_conflict(const int robot_id, const int next_vertex, const int nav_cost)
{        
    last_conflict_vertex_ = -1;
    last_conflict_robot_ = -1;        
        
    // if node is invalid return no interference 
    if( (robot_id < 0) || (next_vertex < 0) || ( nav_cost < 0) ) 
    {        
        return false; // the target node is still an intention or not selected 
    }
    
    bool is_conflict = false; 
        
    int i_other_robot        = -1;
    int vertex_id_other_robot = -1;
    
    boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
    for (int i = 0; i < TEAMSIZE_; i++)
    {
        if (tasks_.id_ugv[i] != robot_id)
        {
            if( 
                (tasks_.nav_cost_to_go[i] != -1) // the target node of robot i is not an intention 
                && (next_vertex == tasks_.id_current_selected_vertex[i])  // we both selected the same node
                && (nav_cost >= tasks_.nav_cost_to_go[i])  // my navigation cost is higher than the one of robot i
              )
            {
  
                last_conflict_vertex_ = next_vertex;
                last_conflict_robot_  = tasks_.id_ugv[i];
                
                is_conflict           = true;
                i_other_robot        = i; 
                vertex_id_other_robot = tasks_.id_current_selected_vertex[i];
                
                if(nav_cost == tasks_.nav_cost_to_go[i])    
                {
                    is_conflict = robot_id > i; // in case we have same navigation cost I have the priority if my ID is smaller
                }
                
                if(is_conflict) break;

            }
        }
    }

    if (is_conflict)
    {
        ROS_INFO_STREAM("node conflict: robot " << robot_id << " is moving towards node " << vertex_id_other_robot << " selected by robot " << last_conflict_robot_);
        ROS_INFO_STREAM("-------------  robot " << robot_id << " cost :" << nav_cost << " other robot cost:  " << tasks_.nav_cost_to_go[i_other_robot]);
    }
    
    
    return is_conflict; 
}



void PatrolAgent::waitForTaskReallocation()
{
    float progress = 0.0;
    while (progress < 1.0)
    {
        int barWidth = 70;

        std::cout << "[";
        int pos = barWidth * progress;
        for (int i = 0; i < barWidth; ++i)
        {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();

        progress += 0.1; // for demonstration only
    }
    std::cout << std::endl;

}


// ROBOT-ROBOT COMMUNICATION

void PatrolAgent::broadcast_positions()
{    
    std::cout << "..............................................................\n";
    std::cout << "ROBOT ID " << ID_ROBOT_ << " broadcast_positions()\n";
    
//    //ROS_INFO("Send position - start");
    //Publish Position to common node:
    nav_msgs::Odometry msg;

    int idx = ID_ROBOT_;

    if (ID_ROBOT_ <= -1)
    {
        msg.header.frame_id = "map"; //identificador do robot q publicou
        idx = 0;
    }
    else
    {
//        char string[20];
//        //sprintf(string,"robot_%d/map",ID_ROBOT);
//        //sprintf(string,"ugv%d/map",ID_ROBOT);
//        //sprintf(string,"map",ID_ROBOT);
//        //printf("send_positions_ line 754 %s ",string);
//        msg.header.frame_id = "map";
        
        /// < send a string: "robot_XX/map"
        std::stringstream ss;
        ss << "robot_" << ID_ROBOT_ << "/map";
        msg.header.frame_id = ss.str();
    }

    float x, y, z, th;
    getRobotPose3D(idx, x, y, z, th);
    //    msg.pose.pose.position.x = xPos[idx]; //send odometry.x
    //    msg.pose.pose.position.y = yPos[idx]; //send odometry.y
    msg.pose.pose.position.x = x; //send odometry.x
    msg.pose.pose.position.y = y; //send odometry.y
    msg.pose.pose.position.z = z; //send odometry.y

    //    std::cout << "..............................................................\n";
    //    std::cout << "ROBOT_ID " << ID_ROBOT << " send_positions ....................\n";
    //    std::cout << "ROBOT_ID " << ID_ROBOT << " current position: ["<< x << ", " << y << ", " << z << "]\n";
    //    std::cout << "..............................................................\n";
    
    std::cout << "..............................................................\n";
    
    positions_pub_.publish(msg);
    //ROS_INFO("Send position - end");
    ros::spinOnce();
}

void PatrolAgent::receive_positions()
{

}

/// < at present time this seems not useful: the positions are always rechecked by the method check_interference()
void PatrolAgent::positionsCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
    boost::recursive_mutex::scoped_lock locker(positions_callback_mutex);

    //     printf("Construir tabela de posicoes (receber posicoes), ID_ROBOT = %d\n",ID_ROBOT);    
    //ROS_INFO("positionsCB - start");
    char id[20]; //identificador do robot q enviou a msg d posição...
    strcpy(id, msg->header.frame_id.c_str());
    //int stamp = msg->header.seq;
    //     printf("robot q mandou msg = %s\n", id);

    // Build Positions Table
    if (ID_ROBOT_>-1)
    {
        /// < verify id "XX" of robot: (string: "robot_XX/map")

        char str_idx[4];
        uint i;

        for (i = 6; i < 10; i++)
        {
            if (id[i] == '/')
            {
                str_idx[i - 6] = '\0';
                break;
            }
            else
            {
                str_idx[i - 6] = id[i];
            }
        }

        int idx = atoi(str_idx);
        //printf("id robot q mandou msg = %d\n",idx);

        if (idx >= TEAMSIZE_ && TEAMSIZE_ <= NUM_MAX_ROBOTS)
        {
            //update teamsize:
            TEAMSIZE_ = idx + 1;
        }

        if (ID_ROBOT_ != idx)
        { //Ignore own positions   
            xPos_[idx] = msg->pose.pose.position.x;
            yPos_[idx] = msg->pose.pose.position.y;
            zPos_[idx] = msg->pose.pose.position.z;
        }
        //printf ("Position Table:\n frame.id = %s\n id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx, xPos_[idx], idx, yPos_[idx] );       
    }

    receive_positions();
    //ROS_INFO("positionsCB - end");
}

void PatrolAgent::send_results()
{

}

// simulates blocking send operation with delay in communication

void PatrolAgent::do_send_message(Int16MultiArrayMsg &msg)
{
#if USE_STAMPED_MSG_INT16_MULTIARRAY 
    msg.header.stamp = ros::Time::now();
#endif    
    
    if (communication_delay_ > 0.001)
    {
        //double current_time = ros::Time::now().toSec();
        //if (current_time-last_communication_delay_time>1.0) { 
        //ROS_INFO("Communication delay %.1f",communication_delay);
        ros::Duration delay(communication_delay_); // seconds
        delay.sleep();
        //last_communication_delay_time = current_time;
        //}
    }
    ROS_INFO_STREAM("PatrolAgent::do_send_message - timestamp: " << msg.header.stamp << std::endl;);
    
    results_pub_.publish(msg);
    ros::spinOnce();
}

void PatrolAgent::receive_results()
{

}

void PatrolAgent::broadcast_interference()
{
    //interference: [ID,msg_type]

    //printf("Send Interference: Robot %d\n", ID_ROBOT);

    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(INTERFERENCE_MSG_TYPE);

    std::cout << "..............................................................\n";
    std::cout << "ROBOT ID " << ID_ROBOT_ << " broadcast_interference()\n";

    std::cout << ".............................................................\n";

    //results_pub_.publish(msg);
    //ros::spinOnce();
    do_send_message(msg);
}


void PatrolAgent::broadcast_idleness() 
{
    //result= [ID,msg_type,idleness[1..dimension]]
    const int msg_type = IDLENESS_SYNC_MSG_TYPE;

    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(msg_type);
    
    std::cout << "..............................................................\n";
    std::cout << "ROBOT ID " << ID_ROBOT_ << " broadcast_idleness()\n";    
    
    {
    boost::recursive_mutex::scoped_lock idleness_locker(idleness_mutex_);       
    
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        // convert in 1/10 of secs (integer value) Max value 3276.8 second (> 50 minutes) !!!
        int ms = (int) (vec_instantaneous_idleness_[i]*kScaleIdlenessFromFloatToInt);
        if (ms > 32767)  // check if bigger than max short
        { 
            // Int16 is used to send messages
            ROS_WARN("Wrong conversion when sending idleness value in messages!!!");
            printf("*** idleness value = %.1f -> int16 value = %d\n", vec_instantaneous_idleness_[i], ms);
            ms = 32000;
        }
        std::cout << " Vertex ID: " << i << ", idleness: " << vec_instantaneous_idleness_[i] << std::endl;        
        msg.data.push_back(ms);
    }
    
    }// end block mutex 

    std::cout << ".............................................................\n";
    
    do_send_message(msg);   
}

void PatrolAgent::resultsCallback(const Int16MultiArrayMsg::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker(results_callback_mutex);
    
    std::vector<signed short>::const_iterator it = msg->data.begin();

    vec_results_.clear();

    for (size_t k = 0; k < msg->data.size(); k++)
    {
        vec_results_.push_back(*it);
        it++;
    }
    

    int id_sender = vec_results_[0];
    int msg_type  = vec_results_[1];
    

    /// < check if we have to expand the team size
    if (id_sender >= TEAMSIZE_ && TEAMSIZE_ <= NUM_MAX_ROBOTS)
    {
        //update teamsize:
        TEAMSIZE_ = id_sender + 1;
    }

    //printf(" MESSAGE FROM %d TYPE %d ...\n",id_sender, msg_type);

    
    switch(msg_type) 
    {
    case INITIALIZE_MSG_TYPE:
        // messages coming from the monitor
        //if (id_sender == -1 && msg_type == INITIALIZE_MSG_TYPE)
        if (id_sender == -1)
        {
            if (b_waiting_init_msg_ == true && vec_results_[2] == SUB_MSG_START)
            { //"-1,msg_type,100,seq_flag" (BEGINNING)
                ROS_INFO("Let's Patrol!\n");
                double r = 1.0 * ((rand() % 1000) / 1000.0);

                ros::Duration wait(r); // seconds

                printf("Wait %.1f seconds (init pos:%s)\n", r, initial_positions_.c_str());

                wait.sleep();
                b_waiting_init_msg_ = false;
            }

            if (b_waiting_init_msg_ == false && vec_results_[2] == SUB_MSG_END)
            { //"-1,msg_type,999" (END)
                ROS_INFO("The simulation is over. Let's leave");
                b_end_simulation_ = true;
            }
        }
        break;

    default:
        ;
    }
    
    if (!b_waiting_init_msg_)
    {
#if 0
        // communication delay
        if ((communication_delay_ > 0.001) && (id_sender != ID_ROBOT_))
        {
            double current_time = ros::Time::now().toSec();
            if (current_time - last_communication_delay_time_ > 1.0)
            {
                ROS_INFO("Communication delay %.1f", communication_delay_);
                ros::Duration delay(communication_delay_); // seconds
                delay.sleep();
                last_communication_delay_time_ = current_time;
            }
        }
        bool lost_message = false;
        if ((lost_message_rate_ > 0.0001)&& (id_sender != ID_ROBOT_))
        {
            double r = (rand() % 1000) / 1000.0;
            lost_message = r < lost_message_rate_;
        }
        if (lost_message)
        {
            ROS_INFO("Lost message");
            return;
        }
        else
#endif            
        {
            if(!b_have_to_init_) results3dCallback(msg);           
            receive_results();
        }
    }
             
    ros::spinOnce();

}

void PatrolAgent::results3dCallback(const Int16MultiArrayMsg::ConstPtr& msg)
{
    ros::Time time_now = ros::Time::now();
    
#if USE_STAMPED_MSG_INT16_MULTIARRAY
    ros::Time time_msg = msg->header.stamp;    /// < N.B.: real robots or distributed nodes (on different computers) need to synch their system clocks!     
#else
    ros::Time time_msg = time_now;   
#endif    
     
    std::cout << std::endl; 
    std::cout << "..............................................................\n";
    std::cout << "...................  New MSG Arrived .........................\n";


    std::vector<signed short>::const_iterator it = msg->data.begin();

    const int id_sender = vec_results_[0];
    const int msg_type  = vec_results_[1];  

    std::cout << "ROBOT ID " << ID_ROBOT_ << ", SENDER ID = " << id_sender << "\n";
    std::cout << "timestamp: " << time_msg << std::endl;
#if USE_STAMPED_MSG_INT16_MULTIARRAY
    std::cout<< "delay: " <<  (time_now - time_msg).toSec() << std::endl;        
#endif        

    std::cout << "\n";
    
    switch( msg_type )
    {
    
    case INITIALIZE_MSG_TYPE:  // initialization: [ID_ROBOT, msg_type, sub_msg_type]
    {
        if (vec_results_[2] == SUB_MSG_START)
        {
            std::cout << " Message of start patrolling!" << std::endl;
        }
        else if (vec_results_[2] == SUB_MSG_END)
        {
            std::cout << " Message of the end of the simulation" << std::endl;
        }
        else if (vec_results_[2] == SUB_MSG_ROBOT_INIT)
        {
            std::cout << " Message of inizialization of ROBOT " << id_sender << std::endl;
        }      
        
        break;
    }
    
    case INTERFERENCE_MSG_TYPE: // interference is occurring:  [ID_ROBOT, msg_type]
    {
        std::cout << " Message of INTERFERENCE_MSG_TYPE from ROBOT " << id_sender << std::endl;
        
        break;
    }    
    
    case TARGET_REACHED_MSG_TYPE:  // reached goal:  [ID_ROBOT, msg_type, vertex] 
    {    
        std::cout << " Message of TARGET_REACHED_MSG_TYPE\n";
        
        const int goal = vec_results_[2];  
        
        if(id_sender != ID_ROBOT_) // already done for this robot (skip it, a delay could cause troubles)
        {
            
#if DISABLE_COOPERATION_AND_COORDINATION     
                /// < NOP (do not update shared idleness)
#else            
            if( (goal < graph_dimension_) && (goal > -1) )
            {                                 
                ROS_INFO("Robot %d reached Goal %d.\n", id_sender, goal);
                
                boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
                vec_last_visit_time_[goal] = time_msg.toSec() - time_zero_; 

            }
#endif            
     
            boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
            for (int i = 0; i < TEAMSIZE_; i++)
            {
                if (tasks_.id_ugv[i] == id_sender)
                {
                    tasks_.resetWithTime(i,time_msg);
                    
                    std::cout << " ROBOT ID [updated]: " << tasks_.id_ugv[i];
                    std::cout << " , Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << " , Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
                else
                {
                    std::cout << " ROBOT ID: " << tasks_.id_ugv[i];
                    std::cout << ", Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << ", Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
            }  
        
        }       
        
        break;
    }

    case TARGET_PLANNED_MSG_TYPE:  // planned (intention) goal but still to check if I can go there: [ID_ROBOT, msg_type, vertex]
    {
        std::cout << " Message of CURRENT_TARGET_MSG_TYPE\n";
       
        if(id_sender != ID_ROBOT_) // already done for this robot (skip it, a delay could cause troubles)
        {
                   
            boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
            for (int i = 0; i < TEAMSIZE_; i++)
            {
                if (tasks_.id_ugv[i] == id_sender)
                {
                    tasks_.id_current_selected_vertex[i] = vec_results_[2];
                    tasks_.nav_cost_to_go[i]             = -1; // still an intention
                    tasks_.timestamp[i]                  = time_msg;
                    
                    std::cout << " ROBOT ID [updated]: " << tasks_.id_ugv[i];
                    std::cout << " , Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << " , Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
                else
                {
                    std::cout << " ROBOT ID: " << tasks_.id_ugv[i];
                    std::cout << ", Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << ", Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
            }    
        }
        
        break;
    }
    
    case TARGET_SELECTED_MSG_TYPE:           // selected and verified a goal: [ID_ROBOT, msg_type, vertex, nav_cost]      
    {
        std::cout << " Message of TARGET_SELECTED_MSG_TYPE\n";
     
        if(id_sender != ID_ROBOT_) // already done for this robot (skip it, a delay could cause troubles)
        {
                            
            boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
            for (int i = 0; i < TEAMSIZE_; i++)
            {
                if (tasks_.id_ugv[i] == id_sender)
                {
                    tasks_.id_current_selected_vertex[i] = vec_results_[2];
                    tasks_.nav_cost_to_go[i]             = vec_results_[3];
                    tasks_.timestamp[i]                  = time_msg;
                    
                    std::cout << " ROBOT ID [updated]: " << tasks_.id_ugv[i];
                    std::cout << " , Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << " , Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
                else
                {
                    std::cout << " ROBOT ID: " << tasks_.id_ugv[i];
                    std::cout << ", Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << ", Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
            }
            
        }
        break;
    }
    
    case TARGET_ABORTED_MSG_TYPE:  // aborted goal: [ID_ROBOT, msg_type, vertex] 
    {
        std::cout << " Message of TARGET_ABORTED_MSG_TYPE\n";
        
        if(id_sender != ID_ROBOT_) // already done for this robot (skip it, a delay could cause troubles)
        {
                         
            boost::recursive_mutex::scoped_lock tasks_locker(tasks_.mutex);
            for (int i = 0; i < TEAMSIZE_; i++)
            {
                if (tasks_.id_ugv[i] == id_sender)
                {
                    tasks_.resetWithTime(i,time_msg);
                    
                    std::cout << " ROBOT ID [updated]: " << tasks_.id_ugv[i];
                    std::cout << " , Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << " , Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
                else
                {
                    std::cout << " ROBOT ID: " << tasks_.id_ugv[i];
                    std::cout << ", Next Vertex ID: " << tasks_.id_current_selected_vertex[i];
                    std::cout << ", Next Vertex cost: " << tasks_.nav_cost_to_go[i] << std::endl;
                }
            }   
            
        }
        break;
    }
    
    case TARGET_INTERCEPTED_MSG_TYPE: // a new vertex has been intercepted/visited along the way to the goal: [ID_ROBOT, msg_type, vertex] 
    {
        std::cout << " Message of TARGET_VISITED_MSG_TYPE\n";
        const int visited_node = vec_results_[2];
        
#if DISABLE_COOPERATION_AND_COORDINATION     
                /// < NOP  (do not update shared idleness)
#else
        if(visited_node == next_vertex_)
        {
            b_next_vertex_intercepted_by_teammate_ = true; 
        }
        
        if ( (visited_node < graph_dimension_) && (visited_node > -1) ) // a node can be visited/intercepted by any robot 
        {
            boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);

            ROS_INFO("Robot %d visited Goal %d.\n", id_sender, visited_node);
            vec_last_visit_time_[visited_node] = time_msg.toSec() - time_zero_;
        }
#endif
       
        break;
    }
    
    case VERTEX_COVERED_MSG_TYPE: // a vertex is currently covered: [ID_ROBOT, msg_type, vertex]   
    {
        std::cout << " Message of TARGET_COVERED_TYPE\n";
        const int covered_node = vec_results_[2];
        
#if DISABLE_COOPERATION_AND_COORDINATION     
                /// < NOP  (do not update shared idleness)
#else
        if( (covered_node == next_vertex_) && (ID_ROBOT_ != id_sender) )  // N.B.: if ID_ROBOT_ == id_sender then I'm covering a node: this is not an interception by a teammate!
        {
            b_next_vertex_intercepted_by_teammate_ = true; 
        }
        
        if ( (covered_node < graph_dimension_) && (covered_node > -1) )
        {
            if(ID_ROBOT_ != id_sender) // for this robot already done in broadcasting 
            {
                boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);

                ROS_INFO("Robot %d covered vertex %d.\n", id_sender, covered_node);
                vec_last_visit_time_[covered_node] = time_msg.toSec() - time_zero_;
            }
            
            if(ID_ROBOT_ != id_sender) // for this robot already done in broadcasting 
            {
                boost::recursive_mutex::scoped_lock locker(team_deployment_.mutex);
                team_deployment_.set(id_sender,covered_node,time_msg);
            }
                        
        }
        
#endif
      
        break;
    }
    
    case IDLENESS_SYNC_MSG_TYPE: // idleness synchronization message:  [ID_ROBOT, msg_type, idleness[1],...,idleness[dimension]] 
    {
#if DISABLE_COOPERATION_AND_COORDINATION     
                /// < NOP 
#else             
        //if(id_sender != ID_ROBOT_) 
        {
                         
            std::cout << " Message of IDLENESS_SYNC_MSG_TYPE\n";

            std::vector<int>::const_iterator it = vec_results_.begin();
            int id_sender = *it; it++;
            int msg_type = *it; it++;        

            const double time_now_min_time_zero = (time_now.toSec() - time_zero_);

            boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);          
            boost::recursive_mutex::scoped_lock idleness_locker(idleness_mutex_);        
            boost::recursive_mutex::scoped_lock global_idleness_locker(global_idleness_mutex_);
            boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);            

            for(size_t i=0; i<graph_dimension_; i++) 
            {
                int ms = *it; it++; // received value
                const double receivedValue = ((double)ms) * kScaleIdlenessFromFloatToIntInv; // convert back to seconds  

                const double teammateEstimatedIdleness = receivedValue + (time_now - time_msg).toSec();
                const double myEstimatedGlobalIdleness = vec_global_instantaneous_idleness_[i] + graph_[i].priority*(time_now.toSec() - last_update_global_idl_);           
                vec_global_instantaneous_idleness_[i] = std::min(myEstimatedGlobalIdleness, teammateEstimatedIdleness);

                // update local idleness        
                vec_instantaneous_idleness_[i] = graph_[i].priority*(time_now_min_time_zero - vec_last_visit_time_[i]);            

                std::cout << " Vertex ID: " << i << ", idleness  local: " << vec_instantaneous_idleness_[i] << ", global: " << vec_global_instantaneous_idleness_[i]  
                          << "  (received: " << teammateEstimatedIdleness << ", global: " << myEstimatedGlobalIdleness << ") ";     

                // compare local with global and correct local if needed 
                if( vec_instantaneous_idleness_[i] > (vec_global_instantaneous_idleness_[i] + kScaleIdlenessFromFloatToIntInv) ) //  kScaleIdlenessFromFloatToIntInv is the resolution of received-converted idleness values  
                {
                    vec_instantaneous_idleness_[i] = vec_global_instantaneous_idleness_[i];

                    std::cout << " - correcting visit time from: " << vec_last_visit_time_[i];                
                    vec_last_visit_time_[i] = time_now_min_time_zero - vec_instantaneous_idleness_[i]/std::max( graph_[i].priority, kIdlenessPriorityMin ); // N.B.: this can be problematic if priority is changed wildly!
                    std::cout << " to: " << vec_last_visit_time_[i];

                }             

                std::cout << std::endl;
            }
            last_update_global_idl_ = time_now.toSec(); 
            
        }
#endif        
        break;
    }
    
    case NAV_COST_MSG_TYPE: // selected and verified a goal when over a graph node: [ID_ROBOT, msg_type, vertex1, vertex2, nav_cost]
    {
        std::cout << " Message of NAV_COST_MSG_TYPE\n";
        int vertex1 = vec_results_[2];
        int vertex2 = vec_results_[3];        
        float nav_cost = vec_results_[4];
        
#if DISABLE_COOPERATION_AND_COORDINATION     
                /// < NOP  (do not update shared idleness)
#else

        if(id_sender != ID_ROBOT_) 
        {                 
            updateDynNavCost(vertex1,vertex2,nav_cost,time_msg);  
        }          
                
#endif
       
        break;
    }    

    } /// < end switch    


    std::cout << "..............................................................\n";
    std::cout << "..............................................................\n\n";    

}


void PatrolAgent::pausePatrollingCallback(const std_msgs::Bool& msg)
{
    ROS_INFO_STREAM("PatrolAgent::pauseCallback() pause: " << (int)msg.data);
    if (msg.data) 
    {
        if(!b_pause_) 
        {
            sendPathPlannerGoalAbortAndBroadcastAbort();
            ros::Duration(1).sleep();
            sendPathPlannerGoalAbortAndBroadcastAbort();
        }
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Yellow(), "Pause");        
    }
    else
    {
        // this is for triggering the choice of a new node 
        if(b_pause_) b_goal_complete_ = true; 
        
        current_vertex_ = find_closest_node();
        b_first_node_   = true; /// in order not to compute a new vertex where to go
        
        if(p_marker_controller) p_marker_controller->setMarkerColor(Colors::Green(), "Patrolling");
    }
    b_pause_ = msg.data;           
}


void PatrolAgent::buildPatrollingNodesAsMarkers()
{
    boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
    boost::recursive_mutex::scoped_lock nodes_marker_array_locker(marker_nodes_array_mutex_);
    
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();

    for (int i = 0; i < marker_nodes_array_.markers.size(); i++)
    {
        marker_nodes_array_.markers[i].header = header;
        for (int j = 0; j < graph_dimension_; j++)
        {
            if (marker_nodes_array_.markers[i].type == visualization_msgs::Marker::TEXT_VIEW_FACING)
            {
                if (marker_nodes_array_.markers[i].id == j)
                {

                    marker_nodes_array_.markers[i].color.a = 1.0; // Don't forget to set the alpha!
                    marker_nodes_array_.markers[i].color.r = 1.0;
                    marker_nodes_array_.markers[i].color.g = 1.0;
                    marker_nodes_array_.markers[i].color.b = 1.0;
                    
                    if(j == next_vertex_)
                    {
                        marker_nodes_array_.markers[i].color.r = .0;
                        marker_nodes_array_.markers[i].color.g = .0;
                        marker_nodes_array_.markers[i].color.b = 1.0;
                    }
                    
                    std::string text = marker_nodes_array_.markers[i].text;

                    std::stringstream sss;
                    sss.precision(2);
                    sss.setf(std::ios::fixed, std::ios::floatfield);
        
                    sss << "ID=" << j << "\n";
//                  sss << "X= " << nodes_marker_array_updated_.markers[i].pose.position.x << " Y= " << nodes_marker_array_updated_.markers[i].pose.position.y << " Z= " << nodes_marker_array_updated_.markers[i].pose.position.z << "\n";

//                    std::istringstream iss(text);
//                    std::string line;
//                    while (std::getline(iss, line))
//                    {
//                        int n_neighborhood;
//                        if (sscanf(line.c_str(), "#Neigh= %d", &n_neighborhood) == 1)
//                        {
//                            sss << "#Neigh= " << n_neighborhood << "\n";
//                        }
//
//                        int neighborhood_id;
//                        char dir[3];
//                        int cost;
//                        if (sscanf(line.c_str(), "%*s = %d %*s = %s %*s = %d", &neighborhood_id, dir, &cost) == 3)
//                        {
//                            sss << "\tID = " << neighborhood_id << " DIR = " << dir << " COST = " << cost << "\n";
//                        }
//                    }

                    double tt = header.stamp.toSec() - time_zero_;
                    //sss << "Current Time: " << tt << " sec." << "\n";
                    sss << "Idleness: " << vec_instantaneous_idleness_[j] << "\n";
//                    if (vec_instantaneous_idleness_[j] - time_zero_ <= 0)
//                    {
//                        sss << "Current Idleness: " << vec_instantaneous_idleness_[j] << "\n";
//                    }
//                    else
//                    {
//                        sss << "Current Idleness: " << vec_instantaneous_idleness_[j] - time_zero_ << "\n";
//                    }
                    
                    //sss << "Last Visit: " << vec_last_visit_time_[j] << "\n";
//                    if (vec_last_visit_time_[j] - time_zero_ <= 0)
//                    {
//                        sss << "Last Visit: " << vec_last_visit_time_[j] << "\n";
//                    }
//                    else
//                    {
//                        sss << "Last Visit: " << vec_last_visit_time_[j] - time_zero_ << "\n";
//                    }

                    marker_nodes_array_.markers[i].text = sss.str();

                }
            }

        }
    }


    //    for(int i = 0; i < msg->markers.size(); i++){
    //        if(msg->markers[i].type == visualization_msgs::Marker::TEXT_VIEW_FACING)
    //        {
    //            // TO DO!!
    //            std::string text = msg->markers[i].text;
    //            
    ////            std::stringstream sss;
    ////            sss << "ID=" << vertex_web[i].id << "\n"; 
    ////            sss << "X= " << vertex_web[i].x <<" Y= " << vertex_web[i].y  << "\n";
    ////            sss << "#Neigh= " << vertex_web[i].num_neigh << "\n";
    ////            for (int j = 0; j < vertex_web[i].num_neigh; j++) {
    ////                sss << "\tID = " << vertex_web[i].id_neigh[j] << " DIR = " << vertex_web[i].dir[j] << " COST = " << vertex_web[i].cost[j] << "\n";
    ////            }         
    ////            double tt =  header.stamp.toSec() - time_zero.toSec();
    ////            sss << "Current Time: " << tt << " sec."<< "\n"; 
    ////            sss << "Num of visits: " << vertex_web[i].number_of_visits << "\n";
    ////            sss << "Current Idleness: " << vertex_web[i].current_idleness << "\n";
    ////            sss << "Last Visit: " << vertex_web[i].last_visit << "\n";
    ////            marker_t.text = sss.str();
    //        }
    //    }    
}

void PatrolAgent::buildPatrollingEdgesAsMarkers()
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    boost::recursive_mutex::scoped_lock nodes_marker_array_locker(marker_edges_list_mutex_);  

    marker_edges_list_.points.clear();

    marker_edges_list_.header.frame_id = "map";;
    marker_edges_list_.header.stamp = ros::Time::now();
    
    marker_edges_list_.action = visualization_msgs::Marker::ADD;
    marker_edges_list_.pose.orientation.w = 1.0;
    marker_edges_list_.ns = "edges";
    marker_edges_list_.id = 1;
    marker_edges_list_.type = visualization_msgs::Marker::LINE_LIST;
    marker_edges_list_.scale.x = 0.05;
    marker_edges_list_.color.r = 0.8;
    marker_edges_list_.color.g = 0.8;
    marker_edges_list_.color.b =  .0;
    marker_edges_list_.color.a = 0.5;
    
    geometry_msgs::Point pointA;
    geometry_msgs::Point pointB;
    
    std::vector< std::vector<int> > mat_adj = std::vector< std::vector<int> >(graph_dimension_, std::vector<int>(graph_dimension_,0));
    
    for (int id = 0; id < graph_dimension_; id++)
    {
        pointA.x = graph_[id].x;
        pointA.y = graph_[id].y;
        pointA.z = graph_[id].z;
        
        int num_neighbours = graph_[id].num_neigh;

        for (int j = 0; j < num_neighbours; j++)
        {
            int id_neighbour = graph_[id].id_neigh[j];
            pointB.x = graph_[id_neighbour].x;
            pointB.y = graph_[id_neighbour].y;
            pointB.z = graph_[id_neighbour].z;
            
            if(mat_adj[id][id_neighbour] == 0)
            {
                marker_edges_list_.points.push_back(pointA);
                marker_edges_list_.points.push_back(pointB);
                mat_adj[id][id_neighbour] = 1;
                mat_adj[id_neighbour][id] = 1;
            }
        }
        
    }
    
    edges_topic_pub_.publish(marker_edges_list_);
}


void PatrolAgent::publishPatrollingNodesAsMarkers()
{
    boost::recursive_mutex::scoped_lock locker(marker_nodes_array_mutex_);
    nodes_topic_pub_.publish(marker_nodes_array_);
}

void PatrolAgent::publishPatrollingEdgesAsMarkers()
{
     boost::recursive_mutex::scoped_lock locker(marker_edges_list_mutex_);
    edges_topic_pub_.publish(marker_edges_list_);
}


void PatrolAgent::build_kdtree_nodes()
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    PointNode point; 
    plc_nodes_.clear(); 
    for(size_t i=0; i<graph_dimension_;i++)
    {
        point.x     = graph_[i].x;
        point.y     = graph_[i].y;
        point.z     = graph_[i].z;
        point.label = graph_[i].id;
        plc_nodes_.push_back(point);
    }
    kdtree_nodes_.setInputCloud(plc_nodes_.makeShared());
}
   
// check if 1) we are currently over a vertex (distance should be <= kVisitedNodeDistance)(then we have to return b_new_interception = false, close_node !=-1)
//          2) we are visiting a vertex along the way to the goal (must be different from current_vertex_ and next_vertex_)(then we have to return b_new_interception = true, close_node !=-1)
int PatrolAgent::check_vertex_interception(const int current_vertex, const int next_vertex, std::vector<int>& list_already_intercepted, bool& b_new_interception)
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    int close_node = -1; 
    
    b_new_interception = false; // reset input var 
    
    std::vector<int>   pointIdxRadiusSearch(1,0);
    std::vector<float> pointRadiusSquaredDistance(1);

    PointNode robot_position;

    robot_position.x = xPos_[ID_ROBOT_];
    robot_position.y = yPos_[ID_ROBOT_];
    robot_position.z = zPos_[ID_ROBOT_];
        
    std::vector<int>::iterator it; 
    
    int n_found = kdtree_nodes_.radiusSearch(robot_position,kVisitedNodeDistance,pointIdxRadiusSearch,pointRadiusSquaredDistance);
    for(size_t i=0;i<n_found;i++)
    {
        close_node = plc_nodes_[pointIdxRadiusSearch[i]].label; // this is a node closer than kVisitedNodeDistance
        
        std::cout << "check_vertex_interception() - found close vertex: " << close_node << std::endl; 
        
        std::cout << "current intercepted vertices: " << std::endl; 
        for(size_t jj=0;jj<list_already_intercepted.size();jj++)
        {
            std::cout << " " << list_already_intercepted[i];
        }
        cout << std::endl; 
        
        // check if this a new interception
        it = std::find (list_already_intercepted.begin(), list_already_intercepted.end(), close_node);
        bool node_is_in_the_list = (it != list_already_intercepted.end());
        if( !node_is_in_the_list  && (close_node != current_vertex) && (close_node != next_vertex) )
        {
            list_already_intercepted.push_back(close_node);
            //res = close_node;
            b_new_interception = true; 
            break; 
        }
        
    }   
    return close_node; 
}


int PatrolAgent::find_closest_node()
{    
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    /// < find closest node to starting position 
    int closest_node = 0;
    
    double initial_x, initial_y, initial_z;
    float i_x, i_y, i_z, i_theta;
    getRobotPose3D(ID_ROBOT_, i_x, i_y, i_z, i_theta);

#if 0    
    
    initial_x = (double) i_x;
    initial_y = (double) i_y;
    initial_z = (double) i_z;    
    //printf("initial robot position: x = %f, y = %f\n", initial_x, initial_y,initial_z);
    closest_node = IdentifyVertex3D(graph_, graph_dimension_, initial_x, initial_y, initial_z);
    
#else
    
    PointNode point;    
    point.x = (double) i_x;
    point.y = (double) i_y;
    point.z = (double) i_z;  
    
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1,std::numeric_limits<float>::max());
    int n_found = kdtree_nodes_.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);    
    if(n_found > 0)
    {
        closest_node = plc_nodes_[pointIdxNKNSearch[0]].label;
        ROS_INFO_STREAM("PatrolAgent::find_closest_node() - closest node: " << closest_node << ", closest point distance: " << pointNKNSquaredDistance[0]);         
    }
    else
    {
        ROS_ERROR_STREAM("could not find close node with kdtree!");
        // use standard function as a backup
        closest_node = IdentifyVertex3D(graph_, graph_dimension_, initial_x, initial_y, initial_z);        
    }    
    
#endif    
    
    printf("PatrolAgent::find_closest_node() - initial vertex = %d\n", closest_node);
        
    return closest_node;
}


void PatrolAgent::priorityPointCallback(const patrolling_build_graph_msgs::PriorityPoint::ConstPtr& msg)
{
    std::cout << "PatrolAgent::priorityPointCallback()" << std::endl; 
    
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    if(
       !(build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT) &&
       !(build_graph_event_.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED)
    )
    {
        ROS_WARN_STREAM("PatrolAgent::priorityPointCallback() - graph is empty ");
        return;
    }
        
    PointNode point;    
    point.x = msg->position.x; 
    point.y = msg->position.y;
    point.z = msg->position.z;
    
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1,std::numeric_limits<float>::max());
    int n_found = kdtree_nodes_.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);    
    if( (n_found > 0) && (pointNKNSquaredDistance[0] < PatrolAgent::kMaxDistPointPrioritySquared) )
    {
        int close_node_id = plc_nodes_[pointIdxNKNSearch[0]].label;
        graph_[close_node_id].priority = msg->priority;  
        ROS_INFO_STREAM("set priority on node: " << msg->id << ", closest point distance: " << pointNKNSquaredDistance[0]);         
    }
    else
    {
        ROS_WARN_STREAM("received priority point too far from graph, dist: " << pointNKNSquaredDistance[0]);
    }
}

    
void PatrolAgent::updateDynNavCost(const int vertex1, const int vertex2, const int nav_cost, const ros::Time& timestamp)
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   

    Vertex& v1 = graph_[vertex1];
    VertexNavData& v1NavData = v1.nav_data[vertex2];
    v1NavData.cost = nav_cost;    
    v1NavData.timestamp = timestamp;        
    
    Vertex& v2 = graph_[vertex2];
    VertexNavData& v2NavData = v2.nav_data[vertex1];
    v2NavData.cost = nav_cost;    
    v2NavData.timestamp = timestamp;    

    PrintDynamicGraph(graph_,graph_dimension_,time_zero_);      
}
