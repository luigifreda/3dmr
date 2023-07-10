/*********************************************************************
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
 * Author: David Portugal (2011-2014), and Luca Iocchi (2014-2015) 
 * and Mario Gianni (2016), and Luigi Freda (2016-Present) 
 *********************************************************************/

#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "PatrolAgent.h"
#include "graph.h"
#include "algorithms.h"


using namespace std;

class CBLS_Agent : public PatrolAgent
{
private:

    int NUMBER_OF_ROBOTS;
    int *tab_intention;
    uint *node_count;
    bool arrived;
    uint vertex_arrived;
    int robot_arrived;
    bool intention;
    uint vertex_intention;
    int robot_intention;
    int number_of_edges;
    reinforcement_learning RL;
    long long int decision_number;
    double now;
    double *avg_idleness; // local idleness
    double *cur_avg_idleness; // local idleness
    double *real_histogram;
    double *histogram;
    uint *source;
    uint *destination;
    uint hist_dimension;

public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    virtual void processEvents();
    virtual void send_results();
    virtual void receive_results();
    virtual void onGoalComplete();
};

void CBLS_Agent::init(int argc, char** argv)
{

    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX

    PatrolAgent::init(argc, argv);

    NUMBER_OF_ROBOTS = atoi(argv[3]);
    uint number_of_edges = GetNumberEdges(graph_, graph_dimension_);

    //INITIALIZE tab_intention:
    int i;

    tab_intention = new int[NUMBER_OF_ROBOTS];
    for (i = 0; i < NUMBER_OF_ROBOTS; i++)
    {
        tab_intention[i] = -1;
    }

    //INITIALIZE node_count:
    node_count = new uint[graph_dimension_];
    for (i = 0; i < graph_dimension_; i++)
    {
        node_count[i] = 0;
    }

    hist_dimension = 2 * number_of_edges; //directed edges, i.e.: arcs

    source = new uint [hist_dimension];
    destination = new uint [hist_dimension];

    create_source_and_dest_tables(graph_, source, destination, graph_dimension_);

    real_histogram = new double[hist_dimension];
    histogram = new double[hist_dimension]; // between 0 and 1

    for (i = 0; i < hist_dimension; i++)
    {
        real_histogram[i] = 1.0;
        histogram[i] = 1.0 / (double) hist_dimension;
    }

    //INITIALIZE tables:
    double time_zero = ros::Time::now().toSec();

    avg_idleness = new double[graph_dimension_]; //closed avg
    cur_avg_idleness = new double[graph_dimension_]; //avg + inst

    for (i = 0; i < graph_dimension_; i++)
    {
        vec_instantaneous_idleness_[i] = 0.0;
        avg_idleness[i] = 0.0;
        cur_avg_idleness[i] = 0.0;
        vec_last_visit_time_[i] = time_zero;

        if (i == current_vertex_)
        {
            vec_last_visit_time_[i] = time_zero + 0.1; //Avoids getting back immediately at the initial vertex
            node_count[i] = 1;
        }
    }

    decision_number = 0;

}

void CBLS_Agent::onGoalComplete()
{

    if ((next_vertex_>-1) && (current_vertex_ != next_vertex_))
    {

        /** PUNISH & REWARD -- BEFORE **/
        //Update Idleness Table:
        now = ros::Time::now().toSec();

        for (int i = 0; i < graph_dimension_; i++)
        {
            if (i == next_vertex_)
            {
                vec_last_visit_time_[i] = now;
                node_count[i]++;
                avg_idleness[i] = (avg_idleness[i] * (double) (node_count [i] - 1) + vec_instantaneous_idleness_ [i]) / ((double) node_count [i]);
            }
            vec_instantaneous_idleness_[i] = now - vec_last_visit_time_[i]; //ou seja: Zero para o next_vertex			
            //ROS_INFO("inst_idleness[%d] = %f", i, instantaneous_idleness[i]);

            //Update Curr Avg Idleness Table:
            cur_avg_idleness [i] = (avg_idleness [i] * (double) (node_count [i]) + vec_instantaneous_idleness_ [i]) / ((double) node_count [i] + 1);
        }

        /** PUNISH & REWARD -- NOW **/
        update_likelihood_new(RL, node_count, vec_instantaneous_idleness_, graph_dimension_, real_histogram, source, destination, hist_dimension, graph_, ID_ROBOT_);
        normalize_histogram(real_histogram, histogram, hist_dimension);

        /*if(decision_number%50 == 0){ //write histogram each 50 iterations 
          write_histogram_to_file (vertex_web, real_histogram, histogram, source, destination, hist_dimension,decision_number,ID_ROBOT);
        }*/

        decision_number++;
        //ROS_INFO("decision_number = %lld", decision_number);		

        current_vertex_ = next_vertex_;
    }

    /** *************CALL LEARNING FUNCTION *****************/
    next_vertex_ = compute_next_vertex();

    if (next_vertex_ == -1)
    {
        ROS_ERROR("ABORT (learning_algorithm: next_vertex = -1)");
        exit(-1);
    }
    /** *****************************************************/


    /** David Portugal: 23 Dec. 2015 -- changed this behavior. 
     *  The robot no longer stays in the same place if all neighbor vertices are occupied**/

    //if (current_vertex==next_vertex){
    //  goal_complete = true; //do not try to go there!
    //  //Stay in the same place to avoid interference

    //}else{

    //send_info(current_vertex, next_vertex);
    broadcast_goal_reached(); // Send TARGET to monitor
    send_results(); // Algorithm specific function

    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex_, graph_[next_vertex_].x, graph_[next_vertex_].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendPathPlannerGoalAndBroadcastIntention(next_vertex_); // send to move_base

    b_goal_complete_ = false;
    //}	

}

// Executed at any cycle when goal is not reached

void CBLS_Agent::processEvents()
{

    if (arrived && NUMBER_OF_ROBOTS > 1)
    { //a different robot arrived at a vertex: update idleness table and keep track of last vertices positions of other robots.

        //ROS_INFO("Robot %d reached Goal %d.\n", robot_arrived, vertex_arrived);    

        //Update Idleness Table:
        now = ros::Time::now().toSec();

        for (int i = 0; i < graph_dimension_; i++)
        {
            if (i == vertex_arrived)
            {
                //actualizar last_visit[dimension]
                vec_last_visit_time_[vertex_arrived] = now;
                node_count[vertex_arrived]++;
                avg_idleness[i] = (avg_idleness[i] * (double) (node_count [i] - 1) + vec_instantaneous_idleness_ [i]) / ((double) node_count [i]);
            }
            //actualizar instantaneous_idleness[dimension]
            vec_instantaneous_idleness_[i] = now - vec_last_visit_time_[i];
            cur_avg_idleness [i] = (avg_idleness [i] * (double) (node_count [i]) + vec_instantaneous_idleness_ [i]) / ((double) node_count [i] + 1);
            //ROS_INFO("idleness[%d] = %f", i, instantaneous_idleness[i]);
        }

        arrived = false;
    }

    if (intention && NUMBER_OF_ROBOTS > 1)
    {
        tab_intention[robot_intention] = vertex_intention;
        //printf("tab_intention[ID=%d]=%d\n",robot_intention,tab_intention[robot_intention]);
        intention = false;
    }
    // ros::spinOnce();   

}

int CBLS_Agent::compute_next_vertex()
{
    return learning_algorithm(current_vertex_, graph_, vec_instantaneous_idleness_, cur_avg_idleness, tab_intention, histogram, source, destination, hist_dimension, TEAMSIZE_, ID_ROBOT_, node_count, RL);
}

void CBLS_Agent::send_results()
{
    // [ID,msg_type,vertex,intention]
    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(CBLS_MSG_TYPE);
    msg.data.push_back(current_vertex_);
    msg.data.push_back(next_vertex_);
    do_send_message(msg);
}

void CBLS_Agent::receive_results()
{

    std::vector<int>::const_iterator it = vec_results_.begin();
    int id_sender = *it;
    it++;
    int msg_type = *it;
    it++;

    if ((id_sender == ID_ROBOT_) || (msg_type != CBLS_MSG_TYPE))
        return;

    robot_arrived = vec_results_[0];
    vertex_arrived = vec_results_[2];
    arrived = true;
    robot_intention = vec_results_[0];
    vertex_intention = vec_results_[3];
    intention = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
    
    CBLS_Agent agent;
    agent.init(argc, argv);
    agent.run();

    return 0;
}


