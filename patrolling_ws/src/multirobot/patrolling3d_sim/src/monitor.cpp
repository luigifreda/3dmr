/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3dpatrolling. 
* This file is a modified version of the corresponding file in patrolling_sim 
* http://wiki.ros.org/patrolling_sim see license below.
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

#include "message_types.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <float.h>
#include <fstream>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#if USE_STAMPED_MSG_INT16_MULTIARRAY
#include <stamped_msgs/Int16MultiArray.h>
typedef stamped_msgs::Int16MultiArray Int16MultiArrayMsg; 
#else
#include <std_msgs/Int16MultiArray.h>
typedef std_msgs::Int16MultiArray Int16MultiArrayMsg; 
#endif

#include <std_msgs/String.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <patrolling_build_graph_msgs/BuildGraphEvent.h>
#include <patrolling_build_graph_msgs/Graph.h>
#include <patrolling_build_graph_msgs/PriorityPoint.h>

#include <boost/thread/recursive_mutex.hpp>

using namespace std;

#include "MovingAverage.h"

#include "graph.h"
#include "graph_viz.h"

#define NUM_MAX_ROBOTS 32
#define MAX_COMPLETE_PATROL 100
#define MAX_EXPERIMENT_TIME 86400  // seconds
#define DEAD_ROBOT_TIME 3000.0 // (seconds) time from last goal reached after which a robot is considered dead
#define TIMEOUT_WRITE_RESULTS 60.0 // (seconds) timeout for writing results to file
#define FOREVER true

// For hystograms
#define RESOLUTION 1.0 // seconds
#define MAXIDLENESS 500.0 // seconds

#define LOG_MONITOR 0
#define SAVE_HYSTOGRAMS 0

#define SAVE_DATA 1


static const int kMonitorLoopRate = 10; 
static const double kMaxDistPointPriority = 0.2;

using std::string;
using std::cout;
using std::endl;

typedef unsigned int uint;

ros::Subscriber results_sub;
ros::Subscriber core_results_sub;
ros::Publisher results_pub; //, screenshot_pub;

ros::ServiceClient client;
ros::Publisher map_pub;

std::string build_graph_event_topic;
ros::Subscriber build_graph_event_sub;
std::string graph_topic;
ros::Subscriber graph_sub;
patrolling_build_graph_msgs::BuildGraphEvent from_build_graph_node;
bool b_interactive;

std::string priority_point_topic_;
ros::Subscriber priority_point_sub_;        

//Initialization:
bool b_initialize = true; // Initialization flag
uint cnt = 0; // Count number of robots connected
uint teamsize = 0;
bool init_robots[NUM_MAX_ROBOTS];
double time_last_goal_reached[NUM_MAX_ROBOTS];

// mutex for accessing last_goal_reached vector
pthread_mutex_t lock_last_goal_reached;

//State Variables:
bool b_goal_reached = false;

int current_goal;
double time_zero, last_report_time;
time_t real_time_zero;
double goal_reached_wait, comm_delay, lost_message_rate;
string algorithm, algparams, nav_mod, initial_positions;


#define MAX_DIMENSION 200


MovingAverage node_idleness_moving_avg[MAX_DIMENSION]; 
MovingAverage graph_idleness_moving_avg;

/* ESTRUTURAS DE DADOS A CALCULAR */
double relative_time_last_visit [MAX_DIMENSION], current_idleness [MAX_DIMENSION], avg_idleness [MAX_DIMENSION], stddev_idleness [MAX_DIMENSION];
double total_0 [MAX_DIMENSION], total_1 [MAX_DIMENSION], total_2[MAX_DIMENSION];
int number_of_visits [MAX_DIMENSION];


boost::recursive_mutex statistics_mutex; /// < for booking the statistics 

Vertex *graph_ = 0;       /// < the graph
uint graph_dimension_ = 0; /// < graph size
boost::recursive_mutex graph_mutex_;    

double worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, previous_avg_graph_idl = DBL_MAX;
// global measures
double min_idleness = 0, max_idleness = 0.0;
double min_idleness_moving = DBL_MAX, max_idleness_moving = 0.0;
double gavg, gstddev;
double gT0 = 0.0, gT1 = 0.0, gT2 = 0.0;

uint interference_cnt = 0;
uint complete_patrol = 0;
uint patrol_cnt = 1;

std::string nodes_topic_name = "/patrolling_nodes_markers";
std::string global_frame = "map";
std::string robot_frame = "base_link";
double safety_radius = 1.0;
std::string boundaries_topic_name = "/patrolling_robot_boundaries_markers";

GraphViz* graph_viz = 0;
boost::recursive_mutex graph_viz_mutex;    

bool b_first_draw_ = true; 

#if SAVE_HYSTOGRAMS
#define hn ((int)(MAXIDLENESS/RESOLUTION)+1)
int hsum;
int hv[hn];
#endif

// Idleness file
FILE *idlfile;

// log file
FILE *logfile = NULL;

void dolog(const char *str)
{
    if (logfile)
    {
        fprintf(logfile, "%s\n", str);
        fflush(logfile);
    }
}

void update_stats(int id_robot, int goal);
void update_idleness_goal(int goal);

double get_time_last_goal_reached(int k)
{
    pthread_mutex_lock(&lock_last_goal_reached);
    double r = time_last_goal_reached[k];
    pthread_mutex_unlock(&lock_last_goal_reached);
    return r;
}

void set_time_last_goal_reached(int k, double current_absolute_time)
{
    pthread_mutex_lock(&lock_last_goal_reached);
    time_last_goal_reached[k] = current_absolute_time;
    pthread_mutex_unlock(&lock_last_goal_reached);
}

void resultsCallback(const Int16MultiArrayMsg::ConstPtr& msg)
{
    ROS_INFO("resultsCB - begin");
    
    boost::recursive_mutex::scoped_lock locker(statistics_mutex);

    std::vector<signed short>::const_iterator it = msg->data.begin();

    std::vector<int> vresults;

    vresults.clear();

    for (size_t k = 0; k < msg->data.size(); k++)
    {
        vresults.push_back(*it);
        it++;
    }

    int id_robot = vresults[0]; // robot sending the message
    int msg_type = vresults[1]; // message type
    

    if( (id_robot >= (int)teamsize) && (teamsize <= NUM_MAX_ROBOTS) )
    {
        teamsize = id_robot + 1;
        std::cout << "Updating teamsize: " << teamsize << std::endl; 
    }

    std::cout << "Robot " << id_robot << " sent msg of type " << msg_type << std::endl;
    if (msg_type == INITIALIZE_MSG_TYPE)
        std::cout << "INITIALIZE_MSG_TYPE" << std::endl;
    if (msg_type == TARGET_REACHED_MSG_TYPE)
        std::cout << "TARGET_REACHED_MSG_TYPE" << std::endl;
    if (msg_type == INTERFERENCE_MSG_TYPE)
        std::cout << "INTERFERENCE_MSG_TYPE" << std::endl;
    if (msg_type == TARGET_INTERCEPTED_MSG_TYPE)
        std::cout << "TARGET_INTERCEPTED_MSG_TYPE" << std::endl;

    switch (msg_type)
    {
    case INITIALIZE_MSG_TYPE:
    {
        //if (initialize && vresults[2] == SUB_MSG_ROBOT_INIT)
        if (vresults[2] == SUB_MSG_ROBOT_INIT)
        {
            if (init_robots[id_robot] == false)
            { //receive init msg: "ID,msg_type,1"
                printf("Robot [ID = %d] is Active!\n", id_robot);
                init_robots[id_robot] = true;
                cnt++;
            }
            
            if( (cnt == teamsize) && (b_initialize) ) 
            {
                b_initialize = false;

                //Clock Reset:
                time_zero        = ros::Time::now().toSec(); 
                last_report_time = time_zero;

                time(&real_time_zero);
                printf("Time zero = %.1f (sim) = %lu (real) \n", time_zero, (long) real_time_zero);

                printf("****************************************************\n");
                printf("All Robots GO!\n");
                printf("****************************************************\n");

//                Int16MultiArrayMsg msg; // -1,msg_type,100,0,0
//                msg.data.clear();
//                msg.data.push_back(-1);
//                msg.data.push_back(INITIALIZE_MSG_TYPE);
//                msg.data.push_back(SUB_MSG_START); // Go !!!
//                ROS_INFO("Monitor::resultsCB");
//                results_pub.publish(msg);
//                ros::spinOnce();
            }
            
            if (cnt == teamsize) 
            {
                Int16MultiArrayMsg msg; // -1,msg_type,100,0,0
#if USE_STAMPED_MSG_INT16_MULTIARRAY
                msg.header.stamp = ros::Time::now();
#endif                    
                msg.data.clear();
                msg.data.push_back(-1);
                msg.data.push_back(INITIALIZE_MSG_TYPE);
                msg.data.push_back(SUB_MSG_START); // Go !!!
                ROS_INFO("Monitor::resultsCB");
                printf("sending start to Robots\n");
                results_pub.publish(msg);
                ros::spinOnce(); 
            }
            
        }
        break;
    }

    case TARGET_REACHED_MSG_TYPE:
    {
        //goal sent by a robot during the experiment [ID,msg_type,vertex,intention,0]
        if (b_initialize == false)
        {
            current_goal = vresults[2];
            ROS_INFO("Robot %d reached Goal %d.\n", id_robot, current_goal);
            fflush(stdout);
            if( (current_goal > -1) && (current_goal < graph_dimension_) )
            {
                b_goal_reached = true;
                update_stats(id_robot, current_goal);
            }
            ros::spinOnce();
        }
        break;
    }

    case TARGET_INTERCEPTED_MSG_TYPE:
    {
        
//#if DISABLE_COORDINATION_AND_COORDINATION     
//                /// < NOP 
//#else        
        //goal sent by a robot during the experiment [ID,msg_type,vertex,intention,0]
        if (b_initialize == false)
        {
            current_goal = vresults[2];
            ROS_INFO("Robot %d intercepted vertex %d.\n", id_robot, current_goal);
            fflush(stdout);
            if( (current_goal > -1) && (current_goal < graph_dimension_) )
            {
                //goal_reached = true;
                update_stats(id_robot, current_goal);
            }
            ros::spinOnce();
        }
//#endif
        
        break;
    }
    
    
    case VERTEX_COVERED_MSG_TYPE:
    {
        
//#if DISABLE_COORDINATION_AND_COORDINATION     
//                /// < NOP 
//#else        
        //goal sent by a robot during the experiment [ID,msg_type,vertex,intention,0]
        if (b_initialize == false)
        {
            current_goal = vresults[2];
            ROS_INFO("Robot %d covered vertex %d.\n", id_robot, current_goal);
            fflush(stdout);
            if( (current_goal > -1) && (current_goal < graph_dimension_) )
            {
                //goal_reached = true;
                update_idleness_goal(current_goal);
            }
            ros::spinOnce();
        }
//#endif
        
        break;
    }
        
    case INTERFERENCE_MSG_TYPE:
    {
        //interference: [ID,msg_type]
        if (b_initialize == false)
        {
            ROS_INFO("Robot %d sent interference.\n", id_robot);
            interference_cnt++;
            ros::spinOnce();
        }
        break;
    }
    }

    //    g_v->setNodesNumVisits(number_of_visits);
    //    g_v->setNodesCurrentIdleness(current_idleness);
    //    g_v->setNodesLastVisit(last_visit);  
    //    g_v->buildPatrollingNodesAsMarkers();
    //    g_v->publishPatrollingNodesAsMarkers();
    //    g_v->getRobotPoses();
    //    g_v->checkCollision();
    //    g_v->publishRobotBoudariesAsMarkers();

    ROS_INFO("resultsCB - end");
}

void finish_simulation()
{ //-1,msg_type,999,0,0
    ROS_INFO("Sending stop signal to patrol agents.");
    Int16MultiArrayMsg msg;
#if USE_STAMPED_MSG_INT16_MULTIARRAY
    msg.header.stamp = ros::Time::now();
#endif      
    msg.data.clear();
    msg.data.push_back(-1);
    msg.data.push_back(INITIALIZE_MSG_TYPE);
    msg.data.push_back(SUB_MSG_END); // end of the simulation
    ROS_INFO("Monitor::finish_simulation");
    results_pub.publish(msg);
    ros::spinOnce();

    //  ROS_INFO("Taking a screenshot of the simulator...");
    //  std_msgs::String ss;
    //  ss.data = "screenshot";
    //  screenshot_pub.publish(ss);
    //  ros::spinOnce();  
}

// return the median value in a vector of size "dimension" floats pointed to by a
//double Median(double *a, uint dimension)
//{
//    uint table_size = dimension / 2;
//    if (dimension % 2 != 0)
//    { //odd
//        table_size++;
//    }
//    if (table_size == 0)
//    {
//        table_size = 1;
//    }
//
//    double left[table_size], right[table_size], median, *p;
//    unsigned char nLeft, nRight;
//
//    // pick first value as median candidate
//    p = a;
//    median = *p++;
//    nLeft = nRight = 1;
//
//    for (;;)
//    {
//        // get next value
//        double val = *p++;
//
//        // if value is smaller than median, append to left heap
//        if (val < median)
//        {
//            // move biggest value to the heap top
//            unsigned char child = nLeft++, parent = (child - 1) / 2;
//            while (parent && val > left[parent])
//            {
//                left[child] = left[parent];
//                child = parent;
//                parent = (parent - 1) / 2;
//            }
//            left[child] = val;
//
//            // if left heap is full
//            if (nLeft == table_size)
//            {
//                // for each remaining value
//                for (unsigned char nVal = dimension - (p - a); nVal; --nVal)
//                {
//                    // get next value
//                    val = *p++;
//
//                    // if value is to be inserted in the left heap
//                    if (val < median)
//                    {
//                        child = left[2] > left[1] ? 2 : 1;
//                        if (val >= left[child])
//                            median = val;
//                        else
//                        {
//                            median = left[child];
//                            parent = child;
//                            child = parent * 2 + 1;
//                            while (child < table_size)
//                            {
//                                if (child < table_size - 1 && left[child + 1] > left[child])
//                                    ++child;
//                                if (val >= left[child])
//                                    break;
//                                left[parent] = left[child];
//                                parent = child;
//                                child = parent * 2 + 1;
//                            }
//                            left[parent] = val;
//                        }
//                    }
//                }
//                return median;
//            }
//        }
//            // else append to right heap
//        else
//        {
//            // move smallest value to the heap top
//            unsigned char child = nRight++, parent = (child - 1) / 2;
//            while (parent && val < right[parent])
//            {
//                right[child] = right[parent];
//                child = parent;
//                parent = (parent - 1) / 2;
//            }
//            right[child] = val;
//
//            // if right heap is full
//            if (nRight == 14)
//            {
//                // for each remaining value
//                for (unsigned char nVal = dimension - (p - a); nVal; --nVal)
//                {
//                    // get next value
//                    val = *p++;
//
//                    // if value is to be inserted in the right heap
//                    if (val > median)
//                    {
//                        child = right[2] < right[1] ? 2 : 1;
//                        if (val <= right[child])
//                            median = val;
//                        else
//                        {
//                            median = right[child];
//                            parent = child;
//                            child = parent * 2 + 1;
//                            while (child < table_size)
//                            {
//                                if (child < 13 && right[child + 1] < right[child])
//                                    ++child;
//                                if (val <= right[child])
//                                    break;
//                                right[parent] = right[child];
//                                parent = child;
//                                child = parent * 2 + 1;
//                            }
//                            right[parent] = val;
//                        }
//                    }
//                }
//                return median;
//            }
//        }
//    }
//}

template <typename T>
inline T computeMedian(const T array[], int size)
{
    // copy input 
    std::vector<T> vec(array, array + size);
    std::sort(vec.begin(), vec.end());

    int middle = size / 2;
    T median;
    if (size % 2 == 0)
        median = static_cast<T> (vec[middle - 1] + vec[middle]) / 2.;
    else
        median = static_cast<T> (vec[middle]);

    return median;
}

// patrol cycle is computed as the minimum number of visits in the graph 
uint calculate_patrol_cycle(int *nr_visits, uint dimension)
{
    ROS_INFO("    calculate_patrol_cycle - begin");
    uint result = INT_MAX;
    uint imin = 0;
    // compute minimum number of visits in the graph 
    for (uint i = 0; i < dimension; i++)
    {
        if ((uint) nr_visits[i] < result)
        {
            result = nr_visits[i];
            imin = i;
        }
    }
    //printf("  --- complete patrol: visits of %d : %d\n",imin,result);
    ROS_INFO("    calculate_patrol_cycle - end");
    return result;
}

void scenario_name(char* name, const char* graph_file, const char* teamsize_str)
{
    uint i, start_char = 0, end_char = strlen(graph_file) - 1;

    for (i = 0; i < strlen(graph_file); i++)
    {
        if (graph_file[i] == '/' && i < strlen(graph_file) - 1)
        {
            start_char = i + 1;
        }

        if (graph_file[i] == '.' && i > 0)
        {
            end_char = i - 1;
            break;
        }
    }

    for (i = start_char; i <= end_char; i++)
    {
        name [i - start_char] = graph_file [i];
        if (i == end_char)
        {
            name[i - start_char + 1] = '\0';
        }
    }

    strcat(name, "_");
    strcat(name, teamsize_str);
}

//write_results to file

void write_results(double *avg_idleness, double *stddev_idleness, int *number_of_visits, uint complete_patrol, uint dimension,
                   double worst_avg_idleness, double avg_graph_idl, double median_graph_idl, double stddev_graph_idl,
                   double min_idleness, double gavg, double gstddev, double max_idleness,
                   uint interference_cnt, uint tot_visits, float avg_visits,
                   const char* graph_file, const char* teamsize_str,
                   double duration, double real_duration, double comm_delay,
                   string filename)
{

    ROS_INFO("write_results - begin");

    FILE *file;

    printf("writing to file %s\n", filename.c_str());
    // printf("graph file %s\n",graph_file);

    file = fopen(filename.c_str(), "a");

    //fprintf(file,"%i\n%i\n%i\n\n",num_nos,largura(),altura());
    fprintf(file, "\nComplete Patrol Cycles:\t%u\n\n", complete_patrol);
    fprintf(file, "Vertex\tAvg Idl\tStdDev Idl\t#Visits\n");
    for (uint i = 0; i < dimension; i++)
    {
        fprintf(file, "%u\t%.1f\t%.1f\t%d\n", i, avg_idleness[i], stddev_idleness[i], number_of_visits[i]);
    }

    fprintf(file, "\nNode idleness\n");
    fprintf(file, "   worst_avg_idleness (graph) = %.1f\n", worst_avg_idleness);
    fprintf(file, "   avg_idleness (graph) = %.1f\n", avg_graph_idl);
    fprintf(file, "   median_idleness (graph) = %.1f\n", median_graph_idl);
    fprintf(file, "   stddev_idleness (graph) = %.1f\n", stddev_graph_idl);

    fprintf(file, "\nGlobal idleness\n");
    fprintf(file, "   min = %.1f\n", min_idleness);
    fprintf(file, "   avg = %.1f\n", gavg);
    fprintf(file, "   stddev = %.1f\n", gstddev);
    fprintf(file, "   max = %.1f\n", max_idleness);

    fprintf(file, "\nInterferences\t%u\nInterference rate\t%.2f\nVisits\t%u\nAvg visits per node\t%.1f\nTime Elapsed\t%.1f\nReal Time Elapsed\t%.1f\nComm delay: %.2f\n",
            interference_cnt, (float) interference_cnt / duration * 60, tot_visits, avg_visits, duration, real_duration, comm_delay);

    fprintf(file, "----------------------------------------------------------------------------------------------------------------------------------------------------------------\n\n\n");


    fclose(file); /*done!*/

    ROS_INFO("write_results - end");

}

bool check_dead_robots()
{

    //ROS_INFO("  check_dead_robots - begin");

    //double current_time = ros::Time::now().toSec() - time_zero;
    double current_relative_time = ros::Time::now().toSec() - time_zero;
    //    double current_time_tonsec = ros::Time::now().toNSec();
    //    double current_time_tonsec_div = current_time_tonsec/1e9;
    //    double current_diff = current_time - time_zero;
    //printf("current_time = %.2ff \n", current_time);
    //    printf("current_time_tonsec = %.f \n",current_time_tonsec);
    //    printf("current_time_tonsec_div = %.f \n",current_time_tonsec_div);
    //    printf("current_diff = %.f \n",current_diff);


    bool r = false;
    for (size_t i = 0; i < teamsize; i++)
    {
        //printf("Robot %lu: current time %.2ff \n", i, current_time);
        
        /// < get the timestamp of the last msg "goal reached"
        double last_time_goal_reached = get_time_last_goal_reached(i);
        //printf("Robot %lu: last goal reached %.1f \n", i, l);
        double elapsed_time = 0.0;
        if (last_time_goal_reached == 0.0)
        { // It may be FIXED
            elapsed_time = (current_relative_time - last_time_goal_reached);
            //return -1;
        }
        else
        {
            last_time_goal_reached = last_time_goal_reached - time_zero;
            elapsed_time = (current_relative_time - last_time_goal_reached);
        }
        //double delta = (current_time - l);
        //printf("DEBUG dead robot: %lu   %.1f - %.1f = %.1f\n", i, current_time, l, delta);
        if (elapsed_time > DEAD_ROBOT_TIME * 0.75)
        {
            printf("Robot %lu: dead robot - delta = %.1f / %.1f \n", i, elapsed_time, DEAD_ROBOT_TIME);
            //system("play -q beep.wav");
        }
        if (elapsed_time > DEAD_ROBOT_TIME)
        {
            printf("Dead robot %lu. Time from last goal reached = %.1f\n", i, elapsed_time);
            r = true;
            break;
        }
    }

    //ROS_INFO("  check_dead_robots - end");

    return r;
}


// update stats after robot 'id_robot' visits node 'goal'
void update_stats(int id_robot, int goal)
{
    boost::recursive_mutex::scoped_lock locker(statistics_mutex);
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);     
        
    if( !( (goal > -1) && (goal < graph_dimension_) ) )
    {
        ROS_ERROR_STREAM("update_stats() - invalid goal: " << goal);
        return;
    }
    ROS_INFO("  update_stats - begin");


    //   printf("last_visit [%d] = %.1f\n", goal, last_visit [goal]);
    double current_absolute_time         = ros::Time::now().toSec();
    double new_relative_time_last_visit  = current_absolute_time - time_zero;
    
    printf("Robot %d reached/visited goal %d (current time: %.2f, alg: %s, nav: %s)\n", id_robot, goal, current_absolute_time, algorithm.c_str(), nav_mod.c_str());

    number_of_visits [goal]++;

    set_time_last_goal_reached(id_robot, current_absolute_time);

    printf("   nr_of_visits = %d -", number_of_visits [goal]);

    if (number_of_visits [goal] == 0)
    {
        avg_idleness [goal] = 0.0;
        stddev_idleness[goal] = 0.0;
        total_0 [goal] = 0.0;
        total_1 [goal] = 0.0;
        total_2 [goal] = 0.0;
    }
    else
    { // if (number_of_visits [goal] > 0) {

        std::cout << "Goal ID: " << goal << std::endl;
        std::cout << "Last visit: " << relative_time_last_visit [goal] << std::endl;
        current_idleness [goal] = graph_[goal].priority * (new_relative_time_last_visit - relative_time_last_visit [goal]);

        if (current_idleness [goal] > max_idleness)
            max_idleness = current_idleness [goal];
        if (current_idleness [goal] < min_idleness || min_idleness < 0.1)
            min_idleness = current_idleness [goal];

        // global stats
        gT0++;
        gT1 += current_idleness[goal];
        gT2 += current_idleness[goal] * current_idleness[goal];

        // node stats
        total_0 [goal] += 1.0;
        total_1 [goal] += current_idleness [goal];
        total_2 [goal] += current_idleness [goal] * current_idleness [goal];
        avg_idleness [goal] = total_1[goal] / total_0[goal];
        stddev_idleness[goal] = 1.0 / total_0[goal] * sqrt(total_0[goal] * total_2[goal] - total_1[goal] * total_1[goal]);

        printf(" idl current = %.2f, ", current_idleness[goal]);
        printf(" avg = %.1f, stddev = %.1f,", avg_idleness [goal], stddev_idleness[goal]);
        printf(" max = %.1f - interf = %d\n", max_idleness, interference_cnt);

        // save data in idleness file
        fprintf(idlfile, "%.1f;%d;%d;%.1f;%d\n", current_absolute_time, id_robot, goal, current_idleness[goal], interference_cnt);
        fflush(idlfile);

#if SAVE_HYSTOGRAMS
        // compute values for hystograms
        int b = (int) (current_idleness[goal] / RESOLUTION);
        if (b < hn)
        {
            hv[b]++;
            hsum++;
        }
#endif

    }

    complete_patrol = calculate_patrol_cycle(number_of_visits, graph_dimension_);
    printf("   complete patrol cycles = %d\n", complete_patrol);

    // Compute node with highest current idleness
    size_t hnode;
    double hidl = 0;
    double cidl_[graph_dimension_];
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        double cidl = new_relative_time_last_visit - relative_time_last_visit [i];
        cidl_[i] = cidl;
        if (cidl > hidl)
        {
            hidl = cidl;
            hnode = i;
        }
    }
    printf("   highest current idleness: node %lu idl %.1f\n\n", hnode, hidl);

    relative_time_last_visit [goal] = new_relative_time_last_visit;
    std::cout << "Last visit updated: " << relative_time_last_visit [goal] << std::endl;
    b_goal_reached = false;

    {
    boost::recursive_mutex::scoped_lock graph_viz_locker(graph_viz_mutex);        
    graph_viz->setNodesNumVisits(number_of_visits);
    graph_viz->setNodesCurrentIdleness(cidl_);
    graph_viz->setNodesLastVisit(relative_time_last_visit);
    graph_viz->buildPatrollingNodesAsMarkers(current_absolute_time, time_zero);
    graph_viz->publishPatrollingNodesAsMarkers();
    graph_viz->buildPatrollingEdgesAsMarkers(); 
    graph_viz->publishPatrollingEdgesAsMarkers();
    graph_viz->setNumRobots(teamsize);
    }

    ROS_INFO("  update_stats - end");

}

// update stats
void update_idleness(double current_absolute_time)
{
    static bool first_time = true; 
        
    boost::recursive_mutex::scoped_lock locker(statistics_mutex);
    
//    if(graph_ == 0)
//    {
//        ROS_WARN_STREAM("update_idleness - graph is empty! cannot use priority");
//        return;
//    }    
        
    double new_relative_time_last_visit = current_absolute_time - time_zero;

    //if(first_time && !b_initialize)
    if(first_time)
    {
        for (size_t i = 0; i < graph_dimension_; i++)
        {
            relative_time_last_visit[i] = new_relative_time_last_visit;
        }
     
        first_time = false;
        return; 
    }
        
    double current_avg_graph_idleness = 0; 
    
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        current_idleness[i] = graph_[i].priority * (new_relative_time_last_visit - relative_time_last_visit[i]);
        
        std::cout << " update_idleness() : idleness [" << i << "] " << current_idleness[i] << std::endl; 
                 
        if (current_idleness [i] > max_idleness_moving)
        {
            max_idleness_moving = current_idleness [i];
        }
        
        if (current_idleness [i] < min_idleness_moving )
        {
            min_idleness_moving = current_idleness [i];
        }
       
        current_avg_graph_idleness += node_idleness_moving_avg[i].GetAverage(current_idleness[i]); 
        
        std::cout << " update_idleness() : min " << min_idleness_moving << std::endl; 
        std::cout << " update_idleness() : max " << max_idleness_moving << std::endl;  
    }
    
    current_avg_graph_idleness = current_avg_graph_idleness/std::max(graph_dimension_,(uint)1);
    
    graph_idleness_moving_avg.GetAverage(current_avg_graph_idleness);
    
    std::cout << " update_idleness() : min_idleness_moving " << min_idleness_moving << std::endl; 
    std::cout << " update_idleness() : max_idleness_moving " << max_idleness_moving << std::endl;  
}

// update stats after robot 'id_robot' visits node 'goal'
void update_idleness_goal(int vertex)
{
    boost::recursive_mutex::scoped_lock locker(statistics_mutex);
    
    double current_absolute_time    = ros::Time::now().toSec();
    double new_relative_time_last_visit = current_absolute_time - time_zero;

    relative_time_last_visit [vertex] = new_relative_time_last_visit;
}

void drawGraph(double current_absolute_time)
{
    boost::recursive_mutex::scoped_lock locker(statistics_mutex);
     
    boost::recursive_mutex::scoped_lock graph_viz_locker(graph_viz_mutex);
    if(graph_viz)
    {
        //double current_absolute_time = ros::Time::now().toSec();
        graph_viz->setNumRobots(teamsize);
        graph_viz->setNodesNumVisits(number_of_visits);
        graph_viz->setNodesCurrentIdleness(current_idleness);
        graph_viz->setNodesLastVisit(relative_time_last_visit);
        graph_viz->buildPatrollingNodesAsMarkers(current_absolute_time, time_zero);
        graph_viz->publishPatrollingNodesAsMarkers();
        graph_viz->buildPatrollingEdgesAsMarkers(); 
        graph_viz->publishPatrollingEdgesAsMarkers();
    }
}

void buildGraphEventCallback(const patrolling_build_graph_msgs::BuildGraphEvent::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker(statistics_mutex);
        
    if (from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING)
    {
        from_build_graph_node = *msg;
    }
}

void graphCallback(const patrolling_build_graph_msgs::Graph::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker(statistics_mutex);
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_); 
    
    if (from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING)
    {
        GetGraphFromMsg(graph_, graph_dimension_, msg);
        from_build_graph_node.event = patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED;
        ROS_INFO_STREAM("graphCallback() - graph received");
    }
}

void priorityPointCallback(const patrolling_build_graph_msgs::PriorityPoint::ConstPtr& msg)
{
    std::cout << "priorityPointCallback()" << std::endl;     
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    if(
       !(from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT) &&
       !(from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED)
    )
    {
        ROS_WARN_STREAM("priorityPointCallback() - graph is empty ");
        return;
    }
        
    if(graph_ == 0)
    {
        ROS_WARN_STREAM("priorityPointCallback - graph is empty! cannot insert priority point");
        return;
    }
    
    double px = msg->position.x; 
    double py = msg->position.y; 
    double pz = msg->position.z;     
            
    double dist_min = std::numeric_limits<double>::max();
    int index_min = -1; 
    for(size_t ii=0; ii < graph_dimension_; ii++)
    {
        double dist = sqrt( pow(px-graph_[ii].x,2) + pow(py-graph_[ii].y,2) + pow(pz-graph_[ii].z,2) );
        if(dist < dist_min)
        {
            dist_min  = dist;
            index_min = ii;
        }
    }
    
    if( (dist_min < kMaxDistPointPriority) &&  (index_min > -1) )
    {
        graph_[index_min].priority = msg->priority; 
        ROS_INFO_STREAM("set priority on node: " << msg->id << ", closest point distance: " << dist_min);     
        
        {
        boost::recursive_mutex::scoped_lock graph_viz_locker(graph_viz_mutex); 
        if(graph_viz != 0)
        {
            graph_viz->setNodePriority(index_min, msg->priority);
        }
        }
    }
    else
    {
        if(index_min > -1)
        {
            ROS_WARN_STREAM("received priority point too far from graph: " << dist_min);
        }
    }
}

void initVisitsAndIdleness()
{
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        number_of_visits[i] = -1; // first visit should not be counted for avg
        current_idleness[i] = 0.0;
        relative_time_last_visit[i] = 0.0;
        avg_idleness[i] = 0.0;
        stddev_idleness[i] = 0.0;
        
    }
}


int main(int argc, char** argv)
{ //pass TEAMSIZE GRAPH ALGORITHM

    ros::init(argc, argv, "monitor");
    ros::NodeHandle nh;

    ros::NodeHandle n_("~");
    from_build_graph_node.event = patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING;

    n_.getParam("build_graph_event_topic", build_graph_event_topic);
    build_graph_event_sub = nh.subscribe<patrolling_build_graph_msgs::BuildGraphEvent>(build_graph_event_topic, 10, buildGraphEventCallback);

    n_.getParam("graph_topic", graph_topic);
    graph_sub = nh.subscribe<patrolling_build_graph_msgs::Graph>(graph_topic, 10, graphCallback);

    priority_point_topic_ = getParam<std::string>(n_, "priority_point_topic_", "/priority_point");
    priority_point_sub_   = nh.subscribe(priority_point_topic_, 10, priorityPointCallback);    
    
    time_zero = ros::Time::now().toSec();
    /*
    argc=3
    argv[0]=/.../patrolling3d_sim/bin/monitor
    argv[1]=grid
    argv[2]=ALGORITHM = {MSP,Cyc,CC,CR,HCR}
    argv[3]=TEAMSIZE
     */

    //ex: "rosrun patrolling3d_sim monitor maps/example/example.graph MSP 2"

    //   uint teamsize;
    char teamsize_str[3];
    teamsize = atoi(argv[3]);

    if (teamsize >= NUM_MAX_ROBOTS || teamsize < 1)
    {
        ROS_INFO("The Team size must be an integer number between 1 and %d", NUM_MAX_ROBOTS);
        return 0;
    }
    else
    {
        strcpy(teamsize_str, argv[3]);
        std::cout << "Team size: " << teamsize << std::endl; 
        //     printf("teamsize: %s\n", teamsize_str);
        //     printf("teamsize: %u\n", teamsize);
    }


    algorithm = string(argv[2]);
    printf("Algorithm: %s\n", algorithm.c_str());

    if (argc < 4)
    {
        b_interactive = false; // default behavior use predefined nodes
    }
    else
    {
        b_interactive = false;
        std::string in = argv[4];
        std::transform(in.begin(), in.end(), in.begin(), ::tolower);
        if (in.find("true") != std::string::npos) b_interactive = true;
        
        std::cout << "Interactive: " << b_interactive << std::endl; 
    }

    string mapname;
    string graph_file;

    mapname = string(argv[1]);
    //string graph_file = "maps/"+mapname+"/"+mapname+".graph";
    graph_file = mapname;
            
    
    /// < if not interactive read the graph from file 
    if (!b_interactive)
    {
        printf("Graph: %s\n", graph_file.c_str());

        //Check Graph Dimension:
        graph_dimension_ = GetGraphDimension(graph_file.c_str());
        if (graph_dimension_ > MAX_DIMENSION)
        {
            cout << "ERROR!!! dimension > MAX_DIMENSION (static value) !!!" << endl;
            abort();
        }
        printf("Dimension: %u\n", (uint) graph_dimension_);
        
        if(graph_dimension_>0)
        {
            boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_); 
            
            //Create Structure to save the Graph Info;
            graph_ = new Vertex[graph_dimension_];

            printf(" Loading Graph \n");            
            //Get the Graph info from the Graph File
            GetGraphInfo3D(graph_, graph_dimension_, graph_file.c_str());
            printf(" Graph built \n");
            
            from_build_graph_node.event = patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT;
        }
        else
        {
            b_interactive = true;
            ROS_WARN_STREAM("the graph file is empty or it does not exist: waiting in interactive mode");
        }

    }    
    
    /// < interactive mode: wait for the graph  
    if(b_interactive)
    {
        ros::Rate rate(5);  
        while (
                (from_build_graph_node.event != patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT) && 
                (from_build_graph_node.event != patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED)
              )
        {
            rate.sleep();
            ros::spinOnce();
            //std::cout << "Waiting patrolling build graph node to finish to build the graph" << std::endl;
        }

                
        if(from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT)
        {
            printf("Graph: %s\n", graph_file.c_str());

            //Check Graph Dimension:
            graph_dimension_ = GetGraphDimension(graph_file.c_str());
            if (graph_dimension_ > MAX_DIMENSION)
            {
                cout << "ERROR!!! dimension > MAX_DIMENSION (static value) !!!" << endl;
                abort();
            }
            printf("Dimension: %u\n", (uint) graph_dimension_);
        }
        
        if(from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED)
        {
            ROS_INFO_STREAM("Graph received");
        }
    }

    char hostname[80];

    int r = gethostname(hostname, 80);
    if (r < 0)
        strcpy(hostname, "default");

    printf("Host name: %s\n", hostname);


//    for (size_t i = 0; i < graph_dimension; i++)
//    {
//        number_of_visits[i] = -1; // first visit should not be cnted for avg
//        current_idleness[i] = 0.0;
//        relative_time_last_visit[i] = 0.0;
//    }
    
    initVisitsAndIdleness();

    for (size_t i = 0; i < NUM_MAX_ROBOTS; i++)
    {
        init_robots[i] = false;
        time_last_goal_reached[i] = 0.0;
    }

    bool dead = false; // check if there is a dead robot

    bool simrun, simabort; // check if simulation is running and if it has been aborted by the user

    // Scenario name (to be used in file and directory names)
    char sname[80];
    scenario_name(sname, graph_file.c_str(), teamsize_str);


    // Create directory results if does not exist
    string path1 = "results";

    string path2, path3, path4_intermed, path4;

    path2          = path1 + "/" + string(sname);
    path3          = path2 + "/" + algorithm;
    path4_intermed = path3 + "/" + hostname;

#if  DISABLE_COOPERATION_AND_COORDINATION
    path4 = path4_intermed + "/" + "no_cc"; // no cooperation-coordination
#else
    path4 = path4_intermed + "/" + "cc";    // with cooperation-coordination 
#endif
    
    printf("Path experimental results: %s\n", path4.c_str());
        
    struct stat st;

    if (stat(path1.c_str(), &st) != 0)
        mkdir(path1.c_str(), 0777);
    if (stat(path2.c_str(), &st) != 0)
        mkdir(path2.c_str(), 0777);
    if (stat(path3.c_str(), &st) != 0)
        mkdir(path3.c_str(), 0777);
    if (stat(path4_intermed.c_str(), &st) != 0)
        mkdir(path4_intermed.c_str(), 0777);
    if (stat(path4.c_str(), &st) != 0)
        mkdir(path4.c_str(), 0777);



    // Local time (real clock time)
    time_t rawtime;
    struct tm * timeinfo;
    char strnow[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(strnow, "%d%02d%02d_%02d%02d%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    printf("Date-time of the experiment: %s\n", strnow);

    // File to log all the idlenesses of an experimental scenario

    string idlfilename, resultsfilename, resultstimecsvfilename, expname, moving_avg_time_results_csv;
    
#if  DISABLE_COOPERATION_AND_COORDINATION
    expname = path4 + "/" + "no_cc_" + string(strnow);
#else
    expname = path4 + "/" + "cc_" + string(strnow);
#endif
    
    idlfilename = expname + "_idleness.csv";
    resultsfilename = expname + "_results.txt";
    resultstimecsvfilename = expname + "_timeresults.csv";
    moving_avg_time_results_csv = expname + "_timeresults_moving_avg.csv";

    FILE *fexplist;
    fexplist = fopen("experiments.txt", "a");
    fprintf(fexplist, "%s\n", expname.c_str());
    fclose(fexplist);

    idlfile = fopen(idlfilename.c_str(), "a");
    fprintf(idlfile, "Time;Robot;Node;Idleness;Interferences\n"); // header

    FILE *resultstimecsvfile;
    resultstimecsvfile = fopen(resultstimecsvfilename.c_str(), "w");
    
    FILE* moving_avg_time_results_file;
    moving_avg_time_results_file = fopen(moving_avg_time_results_csv.c_str(), "w");

    fprintf(resultstimecsvfile, "Time;Idleness min;Idleness avg;Idleness stddev;Idleness max;Interferences\n"); // header
    fprintf(moving_avg_time_results_file, "Time;Idleness min;Idleness avg;Idleness stddev;Idleness max;Interferences\n"); // header

#if LOG_MONITOR
    char logfilename[80];
    sprintf(logfilename, "monitor_%s.log", strnow);
    logfile = fopen(logfilename, "w");
#endif

    ROS_INFO("Monitor node starting");
    dolog(expname.c_str());

#if SAVE_HYSTOGRAMS    
    // Vectors for hystograms
    for (int k = 0; k < hn; k++) hv[k] = 0;
    hsum = 0;
#endif

    //Wait for all robots to connect! (Exchange msgs)
    //    ros::init(argc, argv, "monitor");
    //    ros::NodeHandle nh;

    double duration = 0.0, real_duration = 0.0;
    double current_absolute_time = ros::Time::now().toSec();
    ros::Rate loop_rate(kMonitorLoopRate); //0.033 seconds or 30Hz

    
    //Subscrever "results" vindo dos robots
    results_sub      = nh.subscribe("/results", 100, resultsCallback);
    core_results_sub = nh.subscribe("/core/results", 100, resultsCallback);

    //Publicar dados para "results"
    results_pub      = nh.advertise<Int16MultiArrayMsg>("/results", 100);
        
    bool done = false;
    while (
            ( (from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT) ||
              (from_build_graph_node.event == patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED) )
            && !done
          )
    {


        //screenshot_pub = nh.advertise<std_msgs::String>("/stageGUIRequest", 100);

        //    client = nh.serviceClient<octomap_msgs::GetOctomap>("octomap_full");
        //    map_pub = nh.advertise<octomap_msgs::Octomap>("octomap_full", 100);


        nh.setParam("/simulation_runnning", "true");
        nh.setParam("/simulation_abort", "false");


        // read parameters
        if (!ros::param::get("/goal_reached_wait", goal_reached_wait))
        {
            goal_reached_wait = 0.0;
            ROS_WARN_STREAM("Cannot read parameter /goal_reached_wait. Using default value: " << goal_reached_wait);
        }

        if (!ros::param::get("/communication_delay", comm_delay))
        {
            comm_delay = 0.0;
            ROS_WARN_STREAM("Cannot read parameter /communication_delay. Using default value: " << comm_delay);
        }

        if (!ros::param::get("/lost_message_rate", lost_message_rate))
        {
            lost_message_rate = 0.0;
            ROS_WARN_STREAM("Cannot read parameter /lost_message_rate. Using default value: " << lost_message_rate);
        }
        
        
        if (!ros::param::get("/algorithm_params", algparams))
        {
            algparams = "no parameters specified";
            ROS_WARN_STREAM("Cannot read parameter /algorithm_params. Using default value: " << algparams);
        }

        //    if (!ros::param::get("/initial_positions", initial_positions)) {
        //        initial_positions = "default";
        //        ROS_WARN("Cannot read parameter /initial_positions. Using default value '%s'!", initial_positions.c_str());
        //    }

        //    if (!ros::param::get("/navigation_module", nav_mod)) {
        //        ROS_WARN("Cannot read parameter /navigation_module. Using default value 'ros'!");
        //        nav_mod = "ros";
        //    }


        // mutex for accessing last_goal_reached vector
        pthread_mutex_init(&lock_last_goal_reached, NULL);

        switch(from_build_graph_node.event)
        {
        case patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT:
            {
            std::cout<< "building graph viz from file " << std::endl; 
            boost::recursive_mutex::scoped_lock graph_viz_locker(graph_viz_mutex);
            graph_viz = new GraphViz(graph_file, nodes_topic_name, (int) teamsize, global_frame, robot_frame, safety_radius, boundaries_topic_name);
            }
            break;
         
        case patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED:
            {
            std::cout<< "building graph viz from msg " << std::endl;
            boost::recursive_mutex::scoped_lock graph_viz_locker(graph_viz_mutex);  
            boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);             
            graph_viz = new GraphViz(graph_, graph_dimension_, nodes_topic_name, (int) teamsize, global_frame, robot_frame, safety_radius, boundaries_topic_name);
            }
            break;
            
        default:
            ROS_ERROR_STREAM("unexpected case");
            quick_exit(-1);
        }
        
//        g_v->setNodesNumVisits(number_of_visits);
//        g_v->setNodesCurrentIdleness(current_idleness);
//        g_v->setNodesLastVisit(last_visit);
//        g_v->buildPatrollingNodesAsMarkers(current_time, time_zero);
//        g_v->publishPatrollingNodesAsMarkers();
//        g_v->buildPatrollingEdgesAsMarkers(); 
//        g_v->publishPatrollingEdgesAsMarkers();
        
        drawGraph(current_absolute_time); 

        ROS_INFO("ROS OK? %d", ros::ok());
        done = true;
    }

    
    /// < main loop ============================================================
    int time_index=1; 
    while (ros::ok())
    {

        if( time_index++ % kMonitorLoopRate == 0 )
        {
            double time_now = ros::Time::now().toSec();
            if(!b_initialize)
            {
                update_idleness(current_absolute_time);
            }
            drawGraph(time_now);
        }
                    
        dolog("main loop - begin");

        switch (from_build_graph_node.event)
        {

        case patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT:
        case patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_RECEIVED:

            if (!b_initialize)
            { //check if msg is goal or interference -> compute necessary results.

                // check time
                double report_time = ros::Time::now().toSec();

                // printf("### report time=%.1f  last_report_time=%.1f diff = %.1f\n",report_time, last_report_time, report_time - last_report_time);

                // write results every TIMEOUT_WRITE_RESULTS seconds anyway
                bool timeout_write_results = (report_time - last_report_time > TIMEOUT_WRITE_RESULTS);

                if ((patrol_cnt == complete_patrol) || timeout_write_results) 
                {
                    boost::recursive_mutex::scoped_lock locker(statistics_mutex);
                    
                    std::cout << "writing results " << std::endl; 
                    
                    //ROS_INFO("main loop - write results begin");

//                    if (complete_patrol == 1)
//                    {
//                        ros::param::get("/algorithm_params", algparams);
//                        if (!ros::param::get("/goal_reached_wait", goal_reached_wait))
//                            goal_reached_wait = 0.0;
//                    }

                    // write results every time a patrolling cycle is finished.
                    // or after some time
                    previous_avg_graph_idl = avg_graph_idl; //save previous avg idleness graph value

                    printf("******************************************\n");
                    printf("Patrol completed [%d]. Write to File!\n", complete_patrol);
#if SAVE_DATA
                    worst_avg_idleness = 0.0;
                    avg_graph_idl = 0.0;
                    stddev_graph_idl = 0.0;
                    avg_stddev_graph_idl = 0.0;

                    // Compute avg and stddev
                    double T0 = 0.0, T1 = 0.0, T2 = 0.0, S1 = 0.0;
                    for (size_t i = 0; i < graph_dimension_; i++)
                    {
                        T0++;
                        T1 += avg_idleness[i];
                        T2 += avg_idleness[i] * avg_idleness[i];
                        S1 += stddev_idleness[i];
                        if (avg_idleness[i] > worst_avg_idleness)
                        {
                            worst_avg_idleness = avg_idleness[i];
                        }
                    }

                    avg_graph_idl = T1 / T0;
                    stddev_graph_idl = 1.0 / T0 * sqrt(T0 * T2 - T1 * T1);
                    avg_stddev_graph_idl = S1 / T0;
                    // global stats
                    gavg = gT1 / std::max(gT0,1.);
                    gstddev = 1.0 / std::max(gT0,1.) * sqrt(gT0 * gT2 - gT1 * gT1);
                    
//                    gavg_moving_avg    = 
//                    gstddev_moving_avg = 

                    int tot_visits = 0;
                    for (size_t i = 0; i < graph_dimension_; i++)
                    {
                        tot_visits += number_of_visits[i];
                    }
                    tot_visits = std::max(tot_visits,0);
                    float avg_visits = (float) tot_visits / graph_dimension_;
                    
                    printf("tot_visits %d, graph_dimension %d\n",tot_visits,graph_dimension_);

                    duration = report_time - time_zero;
                    time_t real_now;
                    time(&real_now);
                    real_duration = (double) real_now - (double) real_time_zero;

                    printf("Node idleness\n");
                    printf("   worst_avg_idleness (graph) = %.2f\n", worst_avg_idleness);
                    printf("   avg_idleness (graph) = %.2f\n", avg_graph_idl);
                    /// < median_graph_idl = Median(avg_idleness, graph_dimension);  <<-- this generates a crash   
                    median_graph_idl = computeMedian<double>(avg_idleness, graph_dimension_);
                    printf("   median_idleness (graph) = %.2f\n", median_graph_idl);
                    printf("   stddev_idleness (graph) = %.2f\n", stddev_graph_idl);

                    printf("Global idleness\n");
                    printf("   min = %.1f\n", min_idleness);
                    printf("   avg = %.1f\n", gavg);
                    printf("   stddev = %.1f\n", gstddev);
                    printf("   max = %.1f\n", max_idleness);

                    printf("\nInterferences\t%u\nInterference rate\t%.2f\nVisits\t%u\nAvg visits per node\t%.1f\nTime Elapsed\t%.1f\nReal Time Elapsed\t%.1f\n",
                           interference_cnt, (float) interference_cnt / duration * 60, tot_visits, avg_visits, duration, real_duration);
#endif                     
                    if (timeout_write_results)
                        last_report_time = report_time;
                    else
                        patrol_cnt++;

#if SAVE_DATA
                    double tolerance = 0.025 * avg_graph_idl; //2.5% tolerance
                    printf("diff avg_idleness = %.1f\n", fabs(previous_avg_graph_idl - avg_graph_idl));
                    printf("tolerance = %.1f\n", tolerance);

                    // write results to file
                    if (!timeout_write_results)
                    {
                        write_results(avg_idleness, stddev_idleness, number_of_visits, complete_patrol, graph_dimension_,
                                      worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl,
                                      min_idleness, gavg, gstddev, max_idleness,
                                      interference_cnt, tot_visits, avg_visits,
                                      graph_file.c_str(), teamsize_str, duration, real_duration, comm_delay,
                                      resultsfilename);
                    }
                    else
                    {
                        /*
                        write_results (avg_idleness, stddev_idleness, number_of_visits, complete_patrol, dimension, 
                               worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl,
                               min_idleness, gavg, gstddev, max_idleness,
                               interference_cnt, tot_visits, avg_visits,
                               graph_file.c_str(), teamsize_str, duration, real_duration, comm_delay,
                               resultstimefilename);
                         */

                        fprintf(resultstimecsvfile, "%.1f;%.1f;%.1f;%.1f;%.1f;%d\n",duration, min_idleness, gavg, gstddev, max_idleness, interference_cnt);
                        fflush(resultstimecsvfile);
                        
                        fprintf(moving_avg_time_results_file, "%.1f;%.1f;%.1f;%.1f;%.1f;%d\n",duration, min_idleness_moving, graph_idleness_moving_avg.GetCurrentVal(), graph_idleness_moving_avg.GetSigma(), max_idleness_moving, interference_cnt);
                        fflush(moving_avg_time_results_file);
                        max_idleness_moving = 0; 
                        min_idleness_moving = DBL_MAX; 
                        
                    }
#endif
                    //                ROS_INFO("main loop - write results begin");

                    
                } // if ((patrol_cnt == complete_patrol) || timeout_write_results)


                //ROS_INFO("    check - begin");

                // Check if simulation must be terminated

                //            g_v->setNodesNumVisits(number_of_visits);
                //            g_v->setNodesCurrentIdleness(current_idleness);
                //            g_v->setNodesLastVisit(last_visit);
                //            g_v->buildPatrollingNodesAsMarkers();
                //            g_v->publishPatrollingNodesAsMarkers();
                {   
                boost::recursive_mutex::scoped_lock graph_viz_locker(graph_viz_mutex);
                graph_viz->getRobotPoses();
                graph_viz->checkCollision();
                graph_viz->publishRobotBoudariesAsMarkers();
                graph_viz->setNumRobots(teamsize);
                }

                dead = check_dead_robots();

                simrun = true;
                simabort = false;
                std::string psimrun, psimabort;
                bool bsimabort;
                if (nh.getParam("/simulation_runnning", psimrun))
                    if (psimrun == "false")
                        simrun = false;
                if (nh.getParam("/simulation_abort", psimabort))
                    if (psimabort == "true")
                        simabort = true;
                if (nh.getParam("/simulation_abort", bsimabort))
                    simabort = bsimabort;

                if ((dead) || (!simrun) || (simabort))
                {
                    printf("Simulation is Over\n");
                    nh.setParam("/simulation_runnning", false);
                    finish_simulation();
                    ros::spinOnce();
                    break;
                }
                
                
                //ROS_INFO("    check - end");

            } // if ! initialize

            break;

        case patrolling_build_graph_msgs::BuildGraphEvent::START_PATROLLING:

            std::cout << "Waiting for patrolling_build_graph_node.... the user is inserting points of interest to patrol" << std::endl;

            break;

        case patrolling_build_graph_msgs::BuildGraphEvent::STOP_PATROLLING:

            std::cout << "This option has not been implemented. In principle patrolling task never end" << std::endl;

            from_build_graph_node.event = patrolling_build_graph_msgs::BuildGraphEvent::STOP_PATROLLING;

            break;

        default:

            std::cout << "Unknown event from patrolling_build_graph_node" << std::endl;
        }


        //        octomap_msgs::GetOctomap map;
        //        if (client.call(map)) {
        //            ROS_INFO("Map received");
        //            //std::shared_ptr<octomap::OcTree> octree_;
        //            //octomap::OcTree* octree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(map.response.map));        
        //                    
        //                    
        //            
        //            //octomap_msgs::Octomap octomap;
        //            //octomap_msgs::fullMapToMsg(map,octomap);	
        //            map_pub.publish(map.response.map);
        //        } else {
        //            ROS_ERROR("Failed to call service octomap_full");
        //            return 1;
        //        }

        //        if (!initialize) { //check if msg is goal or interference -> compute necessary results.
        //
        //            // check time
        //            double report_time = ros::Time::now().toSec();
        //
        //            // printf("### report time=%.1f  last_report_time=%.1f diff = %.1f\n",report_time, last_report_time, report_time - last_report_time);
        //
        //            // write results every TIMEOUT_WRITE_RESULTS seconds anyway
        //            bool timeout_write_results = (report_time - last_report_time > TIMEOUT_WRITE_RESULTS);
        //
        //            //if ((patrol_cnt == complete_patrol) || timeout_write_results) {
        //            if (patrol_cnt == complete_patrol) {    
        //
        ////                ROS_INFO("main loop - write results begin");
        //
        //                if (complete_patrol == 1) {
        //                    ros::param::get("/algorithm_params", algparams);
        //                    if (!ros::param::get("/goal_reached_wait", goal_reached_wait))
        //                        goal_reached_wait = 0.0;
        //                }
        //
        //                // write results every time a patrolling cycle is finished.
        //                // or after some time
        //                previous_avg_graph_idl = avg_graph_idl; //save previous avg idleness graph value
        //
        //                printf("******************************************\n");
        //                printf("Patrol completed [%d]. Write to File!\n", complete_patrol);
        //#if 0
        //                worst_avg_idleness = 0.0;
        //                avg_graph_idl = 0.0;
        //                stddev_graph_idl = 0.0;
        //                avg_stddev_graph_idl = 0.0;
        //
        //                // Compute avg and stddev
        //                double T0 = 0.0, T1 = 0.0, T2 = 0.0, S1 = 0.0;
        //                for (size_t i = 0; i < dimension; i++) {
        //                    T0++;
        //                    T1 += avg_idleness[i];
        //                    T2 += avg_idleness[i] * avg_idleness[i];
        //                    S1 += stddev_idleness[i];
        //                    if (avg_idleness[i] > worst_avg_idleness) {
        //                        worst_avg_idleness = avg_idleness[i];
        //                    }
        //                }
        //
        //                avg_graph_idl = T1 / T0;
        //                stddev_graph_idl = 1.0 / T0 * sqrt(T0 * T2 - T1 * T1);
        //                avg_stddev_graph_idl = S1 / T0;
        //                // global stats
        //                gavg = gT1 / gT0;
        //                gstddev = 1.0 / gT0 * sqrt(gT0 * gT2 - gT1 * gT1);
        //
        //                uint i, tot_visits = 0;
        //                for (size_t i = 0; i < dimension; i++) {
        //                    tot_visits += number_of_visits[i];
        //                }
        //                float avg_visits = (float) tot_visits / dimension;
        //
        //                duration = report_time - time_zero;
        //                time_t real_now;
        //                time(&real_now);
        //                real_duration = (double) real_now - (double) real_time_zero;
        //
        //                printf("Node idleness\n");
        //                printf("   worst_avg_idleness (graph) = %.2f\n", worst_avg_idleness);
        //                printf("   avg_idleness (graph) = %.2f\n", avg_graph_idl);
        //                median_graph_idl = Median(avg_idleness, dimension);
        //                printf("   median_idleness (graph) = %.2f\n", median_graph_idl);
        //                printf("   stddev_idleness (graph) = %.2f\n", stddev_graph_idl);
        //
        //                printf("Global idleness\n");
        //                printf("   min = %.1f\n", min_idleness);
        //                printf("   avg = %.1f\n", gavg);
        //                printf("   stddev = %.1f\n", gstddev);
        //                printf("   max = %.1f\n", max_idleness);
        //
        //                printf("\nInterferences\t%u\nInterference rate\t%.2f\nVisits\t%u\nAvg visits per node\t%.1f\nTime Elapsed\t%.1f\nReal Time Elapsed\t%.1f\n",
        //                        interference_cnt, (float) interference_cnt / duration * 60, tot_visits, avg_visits, duration, real_duration);
        //
        //                if (timeout_write_results)
        //                    last_report_time = report_time;
        //                else
        //                    patrol_cnt++;
        //
        //
        //                double tolerance = 0.025 * avg_graph_idl; //2.5% tolerance
        //                printf("diff avg_idleness = %.1f\n", fabs(previous_avg_graph_idl - avg_graph_idl));
        //                printf("tolerance = %.1f\n", tolerance);
        //
        //                // write results to file
        //                if (!timeout_write_results)
        //                    write_results(avg_idleness, stddev_idleness, number_of_visits, complete_patrol, dimension,
        //                        worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl,
        //                        min_idleness, gavg, gstddev, max_idleness,
        //                        interference_cnt, tot_visits, avg_visits,
        //                        graph_file.c_str(), teamsize_str, duration, real_duration, comm_delay,
        //                        resultsfilename);
        //                else {
        //                    /*
        //                    write_results (avg_idleness, stddev_idleness, number_of_visits, complete_patrol, dimension, 
        //                           worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl,
        //                           min_idleness, gavg, gstddev, max_idleness,
        //                           interference_cnt, tot_visits, avg_visits,
        //                           graph_file.c_str(), teamsize_str, duration, real_duration, comm_delay,
        //                           resultstimefilename);
        //                     */
        //
        //                    fprintf(resultstimecsvfile, "%.1f;%.1f;%.1f;%.1f;%.1f;%d\n",
        //                            duration, min_idleness, gavg, gstddev, max_idleness, interference_cnt);
        //                    fflush(resultstimecsvfile);
        //
        //                }
        //#endif
        ////                ROS_INFO("main loop - write results begin");
        //
        //            } // if ((patrol_cnt == complete_patrol) || timeout_write_results)
        //
        //
        //            //ROS_INFO("    check - begin");
        //
        //            // Check if simulation must be terminated
        //
        ////            g_v->setNodesNumVisits(number_of_visits);
        ////            g_v->setNodesCurrentIdleness(current_idleness);
        ////            g_v->setNodesLastVisit(last_visit);
        ////            g_v->buildPatrollingNodesAsMarkers();
        ////            g_v->publishPatrollingNodesAsMarkers();
        //            g_v->getRobotPoses();
        //            g_v->checkCollision();
        //            g_v->publishRobotBoudariesAsMarkers();
        //    
        //            dead = check_dead_robots();
        //
        //            simrun = true;
        //            simabort = false;
        //            std::string psimrun, psimabort;
        //            bool bsimabort;
        //            if (nh.getParam("/simulation_runnning", psimrun))
        //                if (psimrun == "false")
        //                    simrun = false;
        //            if (nh.getParam("/simulation_abort", psimabort))
        //                if (psimabort == "true")
        //                    simabort = true;
        //            if (nh.getParam("/simulation_abort", bsimabort))
        //                simabort = bsimabort;
        //
        //            if ((dead) || (!simrun) || (simabort)) {
        //                printf("Simulation is Over\n");
        //                nh.setParam("/simulation_runnning", false);
        //                finish_simulation();
        //                ros::spinOnce();
        //                break;
        //            }
        //
        //            //ROS_INFO("    check - end");
        //
        //        } // if ! initialize  

        current_absolute_time = ros::Time::now().toSec();
        ros::spinOnce();
        loop_rate.sleep();

        dolog("main loop - end");

    } // while ros ok

    ros::shutdown();

    fclose(idlfile);
    fclose(resultstimecsvfile);




    duration = current_absolute_time - time_zero;
    time_t real_now;
    time(&real_now);
    real_duration = (double) real_now - (double) real_time_zero;

    uint tot_visits = 0;
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        tot_visits += number_of_visits[i];
    }
    float avg_visits = (float) tot_visits / graph_dimension_;


    // Write info file with overall results 
    string infofilename;
    infofilename = expname + "_info.csv";

    FILE *infofile;
    infofile = fopen(infofilename.c_str(), "w");
    fprintf(infofile, "%s;%s;%s;%.1f;%.2f;%s;%s;%s;%s;%s;%.1f;%.1f;%d;%s;%.1f;%.1f;%.1f;%.1f;%.2f;%d;%.1f;%d\n",
            mapname.c_str(), teamsize_str, initial_positions.c_str(), goal_reached_wait, comm_delay, nav_mod.c_str(),
            algorithm.c_str(), algparams.c_str(), hostname,
            strnow, duration, real_duration, interference_cnt, (dead ? "FAIL" : (simabort ? "ABORT" : "TIMEOUT")),
            min_idleness, gavg, gstddev, max_idleness, (float) interference_cnt / duration * 60,
            tot_visits, avg_visits, complete_patrol
            );

    fclose(infofile);
    cout << "Info file " << infofilename << " saved." << endl;


#if SAVE_HYSTOGRAMS
    // Hystogram files
    string hfilename, chfilename;
    hfilename = expname + ".hist";
    chfilename = expname + ".chist";

    cout << "Histogram output files: " << hfilename << endl;
    std::ofstream of1;
    of1.open(hfilename.c_str());
    std::ofstream of2;
    of2.open(chfilename.c_str());
    double c = 0;
    for (int k = 0; k < hn; k++)
    {
        of1 << k * RESOLUTION << " " << (double) hv[k] / hsum << endl;
        c += (double) hv[k] / hsum;
        of2 << k * RESOLUTION << " " << c << endl;
    }
    of1.close();
    of2.close();
#endif

    printf("Monitor closed.\n");

    dolog("Monitor closed");

#if 0  
    sleep(5);
    char cmd[80];
    sprintf(cmd, "mv ~/.ros/stage-000003.png %s/%s_stage.png", path4.c_str(), strnow);
    system(cmd);
    printf("%s\n", cmd);
    printf("Screenshot image copied.\n");
    sleep(3);

    dolog("Snapshots done");
#endif
}

