/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DPATROLLING. 
* This file is a modified version of the corresponding file in patrolling_sim 
* http://wiki.ros.org/patrolling_sim see license below.
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
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
 * Author: David Portugal (2011-2014), and Luca Iocchi (2014) and
 * and Luigi Freda (2016-present) with Mario Gianni (2016)
 *********************************************************************/

#include <ros/ros.h>

#include "graph.h"

uint WIDTH_PX;
uint HEIGHT_PX;
float RESOLUTION;
float WIDTH_M;
float HEIGHT_M;


const double Vertex::kExpirationTimeValidDynCost = 5; // [s]  

Vertex::Vertex() 
{
    id = 0;
    num_neigh = 0;
    x = y = z = 0;
    priority = 1.;
    for(size_t ii=0; ii<8; ii++) 
    {            
        cost_m[ii]=-1.f;

    }
}

float Vertex::getDynCostToGo(int idTo, const ros::Time& time) const
{
    float res = -1; 
    MapNavData::const_iterator it = nav_data.find(idTo);
    if( it != nav_data.end())
    {
        const ros::Time& timestamp = it->second.timestamp;
        if ( ( timestamp - time).toSec() < kExpirationTimeValidDynCost )
        {
            res = it->second.cost;
        }
    }
    return res;
}   
    
uint GetGraphDimension(const char* graph_file)
{

    FILE *file;
    file = fopen(graph_file, "r");
    uint dimension = 0;

    if (file == NULL)
    {
        ROS_INFO("Can not open filename %s", graph_file);
        ROS_BREAK();
    }
    else
    {
        ROS_INFO("Graph File Opened. Reading Dimensions.\n");
        int r;
        r = fscanf(file, "%u", &dimension);

        //Initialize other dimension variables:
        r = fscanf(file, "%u", &WIDTH_PX);
        r = fscanf(file, "%u", &HEIGHT_PX);
        r = fscanf(file, "%f", &RESOLUTION);
        WIDTH_M = (float) WIDTH_PX * RESOLUTION;
        HEIGHT_M = (float) HEIGHT_PX * RESOLUTION;
    }
    fclose(file);
    return dimension;
}

void GetGraphInfo(Vertex *vertex_web, uint dimension, const char* graph_file)
{

    FILE *file;
    file = fopen(graph_file, "r");

    if (file == NULL)
    {
        ROS_INFO("Can not open filename %s", graph_file);
        ROS_BREAK();
    }
    else
    {
        ROS_INFO("Graph File Opened. Getting Graph Info.\n");

        uint i, j;
        float temp;
        int r;

        //Start Reading the File from FIRST_VID On:
        for (i = 0; i < FIRST_VID - 1; i++)
        {
            r = fscanf(file, "%f", &temp);
        }

        for (i = 0; i < dimension; i++)
        {

            r = fscanf(file, "%u", &vertex_web[i].id);
            //        printf ("Node id: %u \n",vertex_web[i].id);

            r = fscanf(file, "%f", &vertex_web[i].x);
            vertex_web[i].x *= RESOLUTION; //convert to m	
            //        printf ("Node id: %u x: %f \n",vertex_web[i].id,vertex_web[i].x);

            r = fscanf(file, "%f", &vertex_web[i].y);
            vertex_web[i].y *= RESOLUTION; //convert to m
            //        printf ("Node id: %u y: %f \n",vertex_web[i].id,vertex_web[i].y);

            r = fscanf(file, "%u", &vertex_web[i].num_neigh);
            //        printf ("Node id: %u num of neighborhood: %u \n",vertex_web[i].id,vertex_web[i].num_neigh);

            for (j = 0; j < vertex_web[i].num_neigh; j++)
            {
                r = fscanf(file, "%u", &vertex_web[i].id_neigh[j]);
                r = fscanf(file, "%s", &vertex_web[i].dir[j]);
                r = fscanf(file, "%u", &vertex_web[i].cost[j]); //can eventually be converted to meters also...
                //printf("\tNode id: %u Neigh Node id = %u, DIR = %s, COST = %u\n",vertex_web[i].id, vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
            }

        }

    }
#if 0 
    for (int i = 0; i < dimension; i++)
    {
        printf("ID= %u\n", vertex_web[i].id);
        printf("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf("#Neigh= %u\n", vertex_web[i].num_neigh);

        for (int j = 0; j < vertex_web[i].num_neigh; j++)
        {
            printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }

        printf("\n");
    }
#endif 
    //printf ("[v=10], x = %f (meters)\n",vertex_web[10].x); 

    fclose(file);

}

void GetGraphInfo3D(Vertex *vertex_web, uint dimension, const char* graph_file)
{

    FILE *file;
    file = fopen(graph_file, "r");

    if (file == NULL)
    {
        ROS_INFO("Can not open filename %s", graph_file);
        ROS_BREAK();
    }
    else
    {
        ROS_INFO("Graph File Opened. Getting Graph Info.\n");
        //ROS_INFO_STREAM("dimension: " << dimension);

        uint i, j;
        float temp;
        int r;

        //Start Reading the File from FIRST_VID On:
        for (i = 0; i < FIRST_VID - 1; i++)
        {
            r = fscanf(file, "%f", &temp);
        }

        for (i = 0; i < dimension; i++)
        {

            r = fscanf(file, "%u", &vertex_web[i].id);
            //printf ("Node id: %u \n",vertex_web[i].id);

            r = fscanf(file, "%f", &vertex_web[i].x);
            vertex_web[i].x *= RESOLUTION; //convert to m	
            //printf ("Node id: %u x: %f \n",vertex_web[i].id,vertex_web[i].x);

            r = fscanf(file, "%f", &vertex_web[i].y);
            vertex_web[i].y *= RESOLUTION; //convert to m
            //printf ("Node id: %u y: %f \n",vertex_web[i].id,vertex_web[i].y);

            r = fscanf(file, "%f", &vertex_web[i].z);
            vertex_web[i].z *= RESOLUTION; //convert to m
            //printf ("Node id: %u z: %f \n",vertex_web[i].id,vertex_web[i].z);

            r = fscanf(file, "%u", &vertex_web[i].num_neigh);
            //printf ("Node id: %u num of neighborhood: %u \n",vertex_web[i].id,vertex_web[i].num_neigh);

            for (j = 0; j < vertex_web[i].num_neigh; j++)
            {
                r = fscanf(file, "%u", &vertex_web[i].id_neigh[j]);
                r = fscanf(file, "%s", &vertex_web[i].dir[j]);
                r = fscanf(file, "%u", &vertex_web[i].cost[j]); //could be possibly converted to meters...
                //printf("\tNode id: %u Neigh Node id = %u, DIR = %s, COST = %u\n",vertex_web[i].id, vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
            }
            
            //getchar(); 

        }

    }
#if 0 
    for (int i = 0; i < dimension; i++)
    {
        printf("ID= %u\n", vertex_web[i].id);
        printf("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf("#Neigh= %u\n", vertex_web[i].num_neigh);

        for (int j = 0; j < vertex_web[i].num_neigh; j++)
        {
            printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }

        printf("\n");
    }
#endif 
    //printf ("[v=10], x = %f (meters)\n",vertex_web[10].x); 

    fclose(file);

}

uint IdentifyVertex(Vertex *vertex_web, uint size, double x, double y)
{

    uint i, v = 0;
    double dif_x, dif_y, result = INFINITY;

    for (i = 0; i < size; i++)
    {
        dif_x = vertex_web[i].x - x;
        dif_y = vertex_web[i].y - y;

        //     printf("[%u] result = %f, (dif_x+dif_y) = %f\n",i,result, fabs(dif_x) + fabs(dif_y));
        if (result > fabs(dif_x) + fabs(dif_y))
        { //Identify the Vertex closer to the initial coordinates x & y
            result = fabs(dif_x) + fabs(dif_y);
            v = i;
        }
    }
    return v;
}

uint IdentifyVertex3D(Vertex *vertex_web, uint size, double x, double y, double z)
{

    uint i, v = 0;
    double dif_x, dif_y, dif_z;
    double min_dist3d = std::numeric_limits<double>::max();
    double dist3d = 0; 

    for (i = 0; i < size; i++)
    {
        dif_x = vertex_web[i].x - x;
        dif_y = vertex_web[i].y - y;
        dif_z = vertex_web[i].z - z;
        
        //dist3d = sqrt(dif_x*dif_x + dif_y*dif_y + dif_z*dif_z); 
        dist3d = (dif_x*dif_x + dif_y*dif_y + dif_z*dif_z); // SQUARED

        if (min_dist3d > dist3d)
        { 
            //Identify the Vertex closer to the initial coordinates x & y
            min_dist3d = dist3d;
            v = i;
        }
    }
    return v;
}

uint GetNumberEdges(Vertex *vertex_web, uint dimension)
{

    //std::cout << "GetNumberEdges() - dimension: " << dimension << std::endl; 
    
    uint result = 0;

    for (uint i = 0; i < dimension; i++)
    {
        //std::cout << "GetNumberEdges() - node: " << i  << " num_neigh : " << vertex_web[i].num_neigh << "[" << std::endl; 
                    
        for (uint j = 0; j < vertex_web[i].num_neigh; j++)
        {
            //std::cout << "neighbour id: " << vertex_web[i].id_neigh[j];
            //std::cout << " dir: " << std::string(vertex_web[i].dir[j]);
            //std::cout << " cost: " << vertex_web[i].cost[j] << std::endl;
            if (vertex_web[i].id < vertex_web[i].id_neigh[j])
            {
                result++;
            }
        }
       
        //std::cout << "]" << std::endl;
    }

    return result;

}

//integer to array (itoa for linux c)

char* itoa(int value, char* str, int radix)
{
    static char dig[] =
            "0123456789"
            "abcdefghijklmnopqrstuvwxyz";
    int n = 0, neg = 0;
    unsigned int v;
    char* p, *q;
    char c;

    if (radix == 10 && value < 0)
    {
        value = -value;
        neg = 1;
    }
    v = value;
    do
    {
        str[n++] = dig[v % radix];
        v /= radix;
    }
    while (v);
    if (neg)
        str[n++] = '-';
    str[n] = '\0';

    for (p = str, q = p + (n - 1); p < q; ++p, --q)
        c = *p, *p = *q, *q = c;
    return str;
}

uint GetGraphFromMsg(Vertex* &vertex_web, uint& dimension, const patrolling_build_graph_msgs::Graph::ConstPtr& msg)
{
    std::cout << "GetGraphFromMsg()" << std::endl; 
    
    dimension = 0;
            
    WIDTH_PX  = 25;  // whatever! we don't use it here 
    HEIGHT_PX = 25; // whatever! we don't use it here 
    RESOLUTION = 1;
    WIDTH_M  = (float) WIDTH_PX * RESOLUTION;
    HEIGHT_M = (float) HEIGHT_PX * RESOLUTION;

    if (vertex_web) delete vertex_web;
    vertex_web = 0;
    
    //    # general information
//    uint32   num_nodes  # graph number of nodes
//
//    # lists (i-th item of each list contains information concerning the i-th node)
//    uint32[] node_id    # node id
//    geometry_msgs/Point[] node_position  # node position
//    uint32[] num_neighbours  # number of neighbours
//
//    # matrices of size num_nodes x num_nodes in row-major order 
//    bool[]  adjacency_matrix
//    string[]  direction_matrix
//    float32[] cost_matrix
    
    dimension  = msg->num_nodes; 
    int num_nodes_ = dimension;
         
//    std::cout << "......................................................." << std::endl;
//    std::cout << "msg adiacency Matrix = [ " << "\n";
//    for (int h = 0; h < num_nodes_; h++)
//    {
//        for (int k = 0; k < num_nodes_; k++)
//        {
//            int index = h*num_nodes_ + k;
//            std::cout << (int)msg->adjacency_matrix[index] << " ";
//        }
//        std::cout << "]" << "\n";
//    }
//    std::cout << "......................................................." << std::endl;
//    std::cout << "msg direction Matrix = [ " << "\n";
//    for (int h = 0; h < num_nodes_; h++)
//    {
//        for (int k = 0; k < num_nodes_; k++)
//        {
//            int index = h*num_nodes_ + k;
//            std::cout << msg->direction_matrix[index] << " ";
//        }
//        std::cout << "]" << "\n";
//    }
//    std::cout << "......................................................." << std::endl;
//    std::cout << "msg cost Matrix = [ " << "\n";
//    for (int h = 0; h < num_nodes_; h++)
//    {
//        for (int k = 0; k < num_nodes_; k++)
//        {
//            int index = h*num_nodes_ + k;
//            std::cout << msg->cost_matrix[index] << " ";
//        }
//        std::cout << "]" << "\n";
//    }
//    std::cout << "......................................................." << std::endl;
    
    
    
    if(dimension == 0) 
    {
        ROS_WARN_STREAM("GetGraphFromMsg() - received zero size graph");
        return 0;
    }
    
    vertex_web = new Vertex[dimension];

    for (int i = 0; i < dimension; i++)
    {
        vertex_web[i].id = msg->node_id[i];
        std::cout << "vertex id: " <<  msg->node_id[i] << std::endl;         
        
        vertex_web[i].priority = msg->node_priority[i];
        std::cout << "priority: " <<  msg->node_priority[i] << std::endl;         
        
        vertex_web[i].x = msg->node_position[i].x;
        vertex_web[i].y = msg->node_position[i].y;
        vertex_web[i].z = msg->node_position[i].z;
        
        vertex_web[i].num_neigh = std::max(std::min((int)msg->num_neighbours[i],(int)8),0);
        
        //std::cout << "id: " << i << " num_neighbours: " << vertex_web[i].num_neigh << std::endl;
        
        int index_insertion = 0; 
        
        for (int j = 0; j < dimension; j++)
        {
            // row-major order
            int index = i*dimension + j;
            if(msg->adjacency_matrix[index] == 1)
            {
                vertex_web[i].id_neigh[index_insertion] = msg->node_id[j];
                //std::cout << "neighbour id: " << vertex_web[i].id_neigh[index_insertion];
                
                memset(&(vertex_web[i].dir[index_insertion][0]),0,3);
                msg->direction_matrix[index].copy(&(vertex_web[i].dir[index_insertion][0]),msg->direction_matrix[index].size());
                //std::cout << " dir: " << std::string(vertex_web[i].dir[index_insertion]);
                
                vertex_web[i].cost[index_insertion]   = (uint)std::min( (double)rint(msg->cost_matrix[index]), (double)std::numeric_limits<uint>::max() );
                vertex_web[i].cost_m[index_insertion] = msg->cost_matrix[index];
                
                vertex_web[i].nav_data[msg->node_id[j]].cost = vertex_web[i].cost_m[index_insertion]; 
                vertex_web[i].nav_data[msg->node_id[j]].timestamp = ros::Time::now(); 
                //std::cout << " cost: " << vertex_web[i].cost[index_insertion] << std::endl;
                
                index_insertion++;
            }
            
            if (index_insertion == vertex_web[i].num_neigh) break; 
        }
    }
    
    return dimension; 
}


void PrintDynamicGraph(const Vertex* vertex_web, size_t size, double time_zero)
{
    std::cout << "PrintDynamicGraph() - size: " << size << std::endl; 

    for (size_t i = 0; i < size; i++)
    {
        const Vertex& vi = vertex_web[i];
        std::cout << "vertex: " << i << ", id: "  << vi.id << std::endl;
        std::map<uint, VertexNavData>::const_iterator it=vi.nav_data.begin(), itEnd=vi.nav_data.end();
        for( ; it!=itEnd; it++)
        {
            const uint idTo = it->first;
            const VertexNavData& viNavData = it->second;
            std::cout << "\t" << "(id,cost,update) = (" << idTo << ", " << viNavData.cost << ", " <<  (viNavData.timestamp.toSec()-time_zero) << ")" << std::endl;              
        }
    
    }
}