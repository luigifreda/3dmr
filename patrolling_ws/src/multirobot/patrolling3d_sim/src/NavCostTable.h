/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> 
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

#ifndef NAV_COST_TABLE_H
#define NAV_COST_TABLE_H

#include <sstream>
#include <set>  
#include <vector>

#include "PatrolAgent.h"

///	\class NodeInfo
///	\author Luigi Freda 
///	\brief A class for representing a navigation cost table (available information from Team Model, Team Deployment and Dynamic Graph) 
///	\note 
/// 	\todo 
///	\date
///	\warning
class NavCostTable
{
public: 
    typedef std::map<int, float> RobotNavCostMap; // for each robot =>  { key: node id, value: nav cost-to-go }
    
public: 
    
    NavCostTable(size_t teamSizeIn):table(teamSizeIn),teamSize(teamSizeIn){}
    
    // set navigation cost 
    void set(const int robot_id, const int node_id, const float nav_cost) 
    {
        if( (robot_id < 0) || (node_id < 0) || ( nav_cost < 0) )
        {
            ROS_ERROR_STREAM("NavCostTable::set() - invalid robot_id: " << robot_id << ", node_id: " << node_id << ", nav_cost: " << nav_cost);            
            return;
        }            
        
        table[robot_id][node_id] = nav_cost; 
    }
    
    // get navigation cost 
    float get(const int robot_id, const int node_id)
    {
        float res = -1;
        if( (robot_id < 0) || (node_id < 0) )       
        {
            ROS_ERROR_STREAM("NavCostTable::get() - invalid robot_id: " << robot_id << ", node_id: " << node_id );            
            return res;
        }                       
        
        const RobotNavCostMap& robotCosts = table[robot_id];
        RobotNavCostMap::const_iterator it = robotCosts.find(node_id);
        if(it!=robotCosts.end())
        {
            res = it->second;
        }
        return res;
    }
    
    // check if robot_id has the best navigation cost-to-go for node_id
    bool is_best(const int robot_id, const int node_id, const float nav_cost)
    {
        bool res = true; 
        if( (robot_id < 0) || (node_id < 0) || (nav_cost < 0) ) 
        {
            ROS_ERROR_STREAM("NavCostTable::is_best() - invalid robot_id: " << robot_id << ", node_id: " << node_id << ", nav_cost: " << nav_cost);            
            return res;
        }
        
        for(size_t ri=0; ri<teamSize; ri++)
        {
            if( ri != robot_id )
            {
                const float cost_ri = get(ri, node_id);
                if( ( cost_ri >= 0) && ( cost_ri < nav_cost ) )
                {
                    res = false;
                    break;
                }
            }            
        }  
        std::cout << "NavCostTable::is_best() robot_id: " << robot_id << ", node_id: " << node_id << ", nav_cost: " << nav_cost << ", res: " << res << std::endl;
        return res; 
    }
    
    void update(const TeamModel& teamModel, const TeamDeployment& teamDeployment, const Vertex* graph, const size_t graphSize, boost::recursive_mutex& graph_mutex)
    {        
        // get available information from dynamic graph and update
        {          
        boost::recursive_mutex::scoped_lock locker(graph_mutex);   
        
        const ros::Time time_now = ros::Time::now();
        
        for(size_t ri=0; ri<teamSize; ri++)
        {
            // check if robot ri is over a vertex and get the id of that vertex 
            // N.B.: we can use the navigation cost computed from node1 to node2 only if the robot is over one of these two evaluation nodes 
            const int node_id = teamDeployment.where(ri, time_now);
            
            if( ( node_id > -1 ) && ( node_id < graphSize) )
            {
                // robot ri is over the vertex with id node_id    
                set(ri,node_id,0);
                
                Vertex::MapNavData::const_iterator it = graph[node_id].nav_data.begin(), itEnd = graph[node_id].nav_data.end();
                for( ; it != itEnd; it++)
                {
                    const uint& idTo = it->first;
                    const ros::Time& timestamp = it->second.timestamp;
                    if ( (timestamp - time_now).toSec() < Vertex::kExpirationTimeValidDynCost )
                    {
                        const float cost = it->second.cost;
                        if( cost > 0 )
                        {
                            set(ri,idTo,cost);
                        }                        
                    }
                }                

            }
        }   
        
        } // end mutex block  
        
        
        // get available information from team model and update 
        {
        boost::recursive_mutex::scoped_lock locker(teamModel.mutex);            
        
        const ros::Time time_now = ros::Time::now();
        
        for(size_t ri=0; ri<teamSize; ri++)
        {
            if(teamModel.isValid(ri))
            {  
                const ros::Time& timestamp = teamModel.timestamp[ri];
                if ( (timestamp - time_now).toSec() < Vertex::kExpirationTimeValidDynCost )  // use a more conservative expiration time than TeamModel::kExpirationTimeTeamModel
                {
                    const int ri_goal = teamModel.id_current_selected_vertex[ri];
                    const float ri_cost = (float)teamModel.nav_cost_to_go[ri];
                    set(ri,ri_goal,ri_cost);                    
                }
            }         
        }
        
        } // end mutex block  
        
    }
       
    void print()
    {
        std::cout << "NavCostTable::print() - teamSize: " << teamSize << std::endl;
        
        for(size_t ri=0; ri<teamSize; ri++)
        {
            std::cout << "robot: " << ri << std::endl;            
            const RobotNavCostMap& robotCosts = table[ri];
            RobotNavCostMap::const_iterator it = robotCosts.begin(), itEnd = robotCosts.end();
            for(; it!=itEnd; it++)
            {
                std::cout << "\t";
                const int node_id = it->first;
                const float nav_cost = it->second;
                std::cout << "\t" << "(id,cost) = (" << node_id << ", " << nav_cost << ")" << std::endl;    
            }    
        }           
    }    
    
public:     
    
    std::vector<RobotNavCostMap> table;
    size_t teamSize;
    
};

#endif