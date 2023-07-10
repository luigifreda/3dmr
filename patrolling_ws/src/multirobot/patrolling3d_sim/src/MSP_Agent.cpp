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
 *********************************************************************/

#include <string>
#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "PatrolAgent.h"
#include "algorithms.h"

using namespace std;

class MSP_Agent : public PatrolAgent
{
private:
    int i_vertex;
    uint route_dimension;
    uint *route;

public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    //virtual void send_results();
    //virtual void receive_results();    
};

void MSP_Agent::init(int argc, char** argv)
{

    string msp_file = string(argv[4]);

    PatrolAgent::init(argc, argv);

    //Check Route Dimension:
    route_dimension = get_MSP_dimension(msp_file.c_str());

    //Create Structure to save the Route Info;
    route = new uint[route_dimension];

    //Get the Route info from the MSP Route File
    get_MSP_route(route, route_dimension, msp_file.c_str());

    printf("\nFinal Path: ");
    for (size_t i = 0; i < route_dimension; i++)
    {
        if (i == route_dimension - 1)
            printf("%i\n", route[i]);
        else
            printf("%i, ", route[i]);
    }
    printf("Number of elements = %i\n", route_dimension);

    i_vertex = 0;
    //    if(route_dimension>1){ i=1; next_vertex = route[i]; }  

}

int MSP_Agent::compute_next_vertex()
{
    i_vertex++;
    if (i_vertex >= route_dimension)
        i_vertex = 1;
    return route[i_vertex];
}

#if 0
// FIXME DONE

void MSP_Agent::send_results()
{
    ros::spinOnce();
}

// FIXME DONE

void MSP_Agent::receive_results()
{
    ros::spinOnce();
}
#endif

int main(int argc, char** argv)
{
    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
    
    MSP_Agent agent;
    agent.init(argc, argv);
    agent.run();

    return 0;
}




