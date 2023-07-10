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
 * Author: David Portugal (2011-2014), and Luca Iocchi (2014)
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

class GBS_Agent : public PatrolAgent
{
private:

    double G1, G2;
    double edge_min;
    int NUMBER_OF_ROBOTS;
    bool arrived;
    uint vertex_arrived;
    int robot_arrived;

public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    virtual void send_results();
    virtual void receive_results();
    virtual void processEvents();
};

void GBS_Agent::init(int argc, char** argv)
{

    PatrolAgent::init(argc, argv);

    NUMBER_OF_ROBOTS = atoi(argv[3]);
    arrived = false;

    /** Define G1 and G2 **/
    G1 = 0.1;

    //default:
    G2 = 100.0;
    edge_min = 1.0;

    if (str_graph_file_name_ == "maps/grid/grid.graph")
    {
        if (NUMBER_OF_ROBOTS == 1)
        {
            G2 = 20.54;
        }
        if (NUMBER_OF_ROBOTS == 2)
        {
            G2 = 17.70;
        }
        if (NUMBER_OF_ROBOTS == 4)
        {
            G2 = 11.15;
        }
        if (NUMBER_OF_ROBOTS == 6)
        {
            G2 = 10.71;
        }
        if (NUMBER_OF_ROBOTS == 8)
        {
            G2 = 10.29;
        }
        if (NUMBER_OF_ROBOTS == 12)
        {
            G2 = 9.13;
        }

    }
    else if (str_graph_file_name_ == "maps/example/example.graph")
    {
        if (NUMBER_OF_ROBOTS == 1)
        {
            G2 = 220.0;
        }
        if (NUMBER_OF_ROBOTS == 2)
        {
            G2 = 180.5;
        }
        if (NUMBER_OF_ROBOTS == 4)
        {
            G2 = 159.3;
        }
        if (NUMBER_OF_ROBOTS == 6)
        {
            G2 = 137.15;
        }
        if (NUMBER_OF_ROBOTS == 8 || NUMBER_OF_ROBOTS == 12)
        {
            G2 = 126.1;
        }
        edge_min = 20.0;

    }
    else if (str_graph_file_name_ == "maps/cumberland/cumberland.graph")
    {
        if (NUMBER_OF_ROBOTS == 1)
        {
            G2 = 152.0;
        }
        if (NUMBER_OF_ROBOTS == 2)
        {
            G2 = 100.4;
        }
        if (NUMBER_OF_ROBOTS == 4)
        {
            G2 = 80.74;
        }
        if (NUMBER_OF_ROBOTS == 6)
        {
            G2 = 77.0;
        }
        if (NUMBER_OF_ROBOTS == 8 || NUMBER_OF_ROBOTS == 12)
        {
            G2 = 63.5;
        }
        edge_min = 50.0;

    }

    printf("G1 = %f, G2 = %f\n", G1, G2);
}

// Executed at any cycle when goal is not reached

void GBS_Agent::processEvents()
{

    if (arrived && NUMBER_OF_ROBOTS > 1)
    { //a different robot arrived at a vertex: update idleness table and keep track of last vertices positions of other robots.

        //Update Idleness Table:
        double now = ros::Time::now().toSec();

        for (int i = 0; i < graph_dimension_; i++)
        {
            if (i == vertex_arrived)
            {
                //actualizar last_visit[dimension]
                vec_last_visit_time_[vertex_arrived] = now;
                //ROS_INFO("Just updated idleness of vertex %d", i);		
            }
            //actualizar instantaneous_idleness[dimension]
            vec_instantaneous_idleness_[i] = now - vec_last_visit_time_[i];
        }

        arrived = false;
    }

    ros::spinOnce();
}

int GBS_Agent::compute_next_vertex()
{
    return greedy_bayesian_strategy(current_vertex_, graph_, vec_instantaneous_idleness_, G1, G2, edge_min);
}

void GBS_Agent::send_results()
{
    // [ID,msg_type,vertex]
    Int16MultiArrayMsg msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT_);
    msg.data.push_back(GBS_MSG_TYPE);
    msg.data.push_back(current_vertex_);
    do_send_message(msg);
}

void GBS_Agent::receive_results()
{

    std::vector<int>::const_iterator it = vec_results_.begin();
    int id_sender = *it;
    it++;
    int msg_type = *it;
    it++;

    if ((id_sender == ID_ROBOT_) || (msg_type != GBS_MSG_TYPE))
        return;

    robot_arrived = vec_results_[0];
    vertex_arrived = vec_results_[2];
    arrived = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
    
    GBS_Agent agent;
    agent.init(argc, argv);
    agent.run();

    return 0;
}
