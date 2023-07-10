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
#include "algorithms.h"

class Cyclic_Agent : public PatrolAgent
{
private:
    int *path;
    int path_elements;
    int i_vertex;
public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    //virtual void send_results();
    //virtual void receive_results();    
};

void Cyclic_Agent::init(int argc, char** argv)
{

    PatrolAgent::init(argc, argv);

    //robot's cyclic path:
    path = new int[8 * graph_dimension_];

    //get cyclic path:
    path_elements = cyclic(graph_dimension_, graph_, path);

    //Shift the cyclic path to start at the current vertex:
    shift_cyclic_path(current_vertex_, path, path_elements);

    printf("\nFinal Path: ");
    for (int i = 0; i < path_elements; i++)
    {
        if (i == path_elements - 1)
        {
            printf("%i\n", path[i]);
        }
        else
        {
            printf("%i, ", path[i]);
        }
    }
    printf("Number of elements = %i\n", path_elements);
    i_vertex = 0;
    // if (path_elements>1) { i_vertex=1; next_vertex = path[i_vertex]; }

}

int Cyclic_Agent::compute_next_vertex()
{
    i_vertex++;
    if (i_vertex >= path_elements)
    {
        i_vertex = 1;
    }
    return path[i_vertex];
}

#if 0
// FIXME DONE

void Cyclic_Agent::send_results()
{
    ros::spinOnce();
}

// FIXME DONE

void Cyclic_Agent::receive_results()
{
    ros::spinOnce();
}
#endif

int main(int argc, char** argv)
{

    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
    
    Cyclic_Agent agent;
    agent.init(argc, argv);
    agent.run();

    return 0;
}
