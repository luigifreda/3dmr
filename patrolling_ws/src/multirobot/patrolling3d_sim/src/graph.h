/**
 * This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DPATROLLING. 
 * This file is a modified version of the corresponding file in patrolling_sim 
 * http://wiki.ros.org/patrolling_sim see license below.
 *
 * Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> (La Sapienza University)
 * For more information see <https://gitlab.com/luigifreda/3dpatrolling>
 *
 * 3DPATROLLING is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * 3DPATROLLING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
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
 * and Luigi Freda (2016-Present) Mario Gianni (2016)
 *********************************************************************/

#ifndef PATROLLING_GRAPH_H
#define PATROLLING_GRAPH_H

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <map>
#include <limits>

#include <ros/ros.h>
#include <patrolling_build_graph_msgs/Graph.h>

//File Line of the First Vertex ID to read (Protection) - fscanf() ignores blank lines
#define FIRST_VID 5

typedef unsigned int uint;

///	\class VertexNavData
///	\author Luigi Freda 
///	\brief A base class for representing navigation cost-to-go and its last update timestamp 
///	\note 
/// 	\todo 
///	\date
///	\warning
struct VertexNavData {
    
    VertexNavData():cost(-1.){}
    VertexNavData( float costIn, const ros::Time& timeIn):cost(costIn),timestamp(timeIn){}
    
    float cost;
    ros::Time timestamp; 
    
};

///	\class TeamDeployment
///	\author Luigi Freda 
///	\brief A base class for representing a graph vertex.
///	\note The first version of this class was written by David Portugal and Luca Iocchi in the package patrolling_sim.                        \n
///           Luigi Freda completely revised and changed it starting from October 2016 up to present time, in order to design and implement the   \n
///           3D patrolling strategy currently presented in the paper "3D Multi-Robot Patrolling with a Two-Level Coordination Strategy".         \n
///           New fields have been added to manage dynamic graph information. 
/// 	\todo 
///	\date
///	\warning
class Vertex {

public: 
    
    typedef std::map<uint, VertexNavData> MapNavData;  // for each node:  { key: node_id, value: { cost-to-go, timestamp} }
    
    static const double kExpirationTimeValidDynCost; // [s]   
    
public: 
    
    Vertex();

    float getDynCostToGo(int idTo, const ros::Time& time) const;
    
public: // main data 
    
    uint id, num_neigh;
    float x, y, z; //pass these attributes in meters
    float priority;
    uint id_neigh[8];
    uint cost[8];
    float cost_m[8]; // used in get_hist_sort(),get_edge_cost_between()
    bool visited[8];
    char dir [8][3]; //table of 8 strings with 3 chars max ("N","NE","E","SE","S","SW","W","NW")   

    MapNavData nav_data; // for dynamic graph modeling: navigation costs to other vertices 
                         // key: id of another vertex 
                         // value: { cost-to-go, timestamp }    

};


extern uint WIDTH_PX;
extern uint HEIGHT_PX;
extern float RESOLUTION;
extern float WIDTH_M;
extern float HEIGHT_M;

uint GetGraphDimension(const char* graph_file);

void GetGraphInfo3D(Vertex *vertex_web, uint dimension, const char* graph_file);

void GetGraphInfo(Vertex *vertex_web, uint dimension, const char* graph_file);

uint IdentifyVertex(Vertex *vertex_web, uint size, double x, double y);

uint IdentifyVertex3D(Vertex *vertex_web, uint size, double x, double y, double z);

uint GetNumberEdges(Vertex *vertex_web, uint dimension);

uint GetGraphFromMsg(Vertex* &vertex_web, uint& dimension, const patrolling_build_graph_msgs::Graph::ConstPtr& msg);

void PrintDynamicGraph(const Vertex* vertex_web, size_t size, double time_zero);

//integer to array (itoa for linux c)
char* itoa(int value, char* str, int radix);


#endif
