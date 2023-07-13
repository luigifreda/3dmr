/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DMR. 
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

#include <sstream>
#include <set>  
#include <vector>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "PatrolAgent.h"
#include "NavCostTable.h"


///	\class NodeInfo
///	\author Luigi Freda 
///	\brief A class for implementing a node information container (for sorting info)
///	\note 
/// 	\todo 
///	\date
///	\warning
class NodeInfo
{
public:
    NodeInfo(int node_id_in = -1, double idleness_in = 0., double cost_to_go_in = -1) : node_id(node_id_in), idleness(idleness_in), cost_to_go(cost_to_go_in){}

    friend std::ostream &operator<<(std::ostream &out, const NodeInfo &n)
    {
        out << "(node id: " << n.node_id << ", idleness: " << n.idleness<<")";
        return out;
    }

    int node_id;
    double idleness;
    int cost_to_go;
};

///	\class CompareNodeInfoById
///	\author Luigi Freda 
///	\brief A class for sorting node information containers by id
///	\note 
/// 	\todo 
///	\date
///	\warning
class CompareNodeInfoById
{
public:
    bool operator()(const NodeInfo& lhs, const NodeInfo& rhs) const
    {
        return lhs.node_id > rhs.node_id;
    }
};

///	\class CompareNodeInfoById
///	\author Luigi Freda 
///	\brief A class for sorting node information containers by idleness
///	\note 
/// 	\todo 
///	\date
///	\warning
class CompareNodeInfoByIdleness
{
public:
    bool operator()(const NodeInfo& lhs, const NodeInfo& rhs) const
    {
        return lhs.idleness > rhs.idleness;
    }
};

typedef std::set<NodeInfo, CompareNodeInfoById> SetNodeInfo; // set of nodes ordered by id


///	\class Conscientious_Reactive_Agent
///	\author Luigi Freda 
///	\brief A class for implementing a "SMART" conscientious-reactive patrolling strategy
///	\note This implementation started as a simple conscientious-reactive strategy then it evolved to a smarter strategy over the time.
/// 	\todo 
///	\date
///	\warning
class Conscientious_Reactive_Agent : public PatrolAgent
{
    static const float kIldenessEqualThreshold; // [s]
    static const float kMaxElapsedTimeForDynamicGraphMessage; // [s]
    
public:    
    
    virtual int compute_next_vertex();
    
protected:
    
    //virtual void send_results();
    //virtual void receive_results();    
        
    int compute_best_node_in_set(SetNodeInfo& set_nodes,std::vector<NodeInfo>& vec_ordered_nodes);
    int compute_random_node_in_set(SetNodeInfo& set_nodes);
    
    void compute_neighborhood(int current_vertex, SetNodeInfo& set_nodes);
    // patch = neigborhood + node     
    void compute_patch(int current_vertex, SetNodeInfo& set_nodes); 
    void compute_neighborhood_depth2(int current_vertex, SetNodeInfo& set_nodes);

    int compute_best_neighbour_vertex(int current_vertex, SetNodeInfo& set_nodes, std::vector<NodeInfo>& vec_ordered_nodes);
    int compute_best_neighbour_vertex_depth2(int current_vertex, int prev_vertex, SetNodeInfo& set_nodes, std::vector<NodeInfo>& vec_ordered_nodes);
    int compute_best_neighbour_vertex_depth2(int current_vertex, std::vector<int> vec_avoid_vertexes, SetNodeInfo& set_nodes, std::vector<NodeInfo>& vec_ordered_nodes);    
    
    int compute_random_vertex_depth2(int current_vertex, int prev_vertex);
    
    int compute_random_vertex_strategy(int prev_planned_vertex); 
    
};


const float Conscientious_Reactive_Agent::kIldenessEqualThreshold = 5; // [s]
const float Conscientious_Reactive_Agent::kMaxElapsedTimeForDynamicGraphMessage = 5; // [s]


int Conscientious_Reactive_Agent::compute_best_node_in_set(SetNodeInfo& set_nodes, std::vector<NodeInfo>& vec_ordered_nodes)
{
    int next_vertex = -1;
    
    // build a new vector of nodes sorted by idleness
    vec_ordered_nodes = std::vector<NodeInfo>(set_nodes.begin(),set_nodes.end());
    std::sort(vec_ordered_nodes.begin(),vec_ordered_nodes.end(),CompareNodeInfoByIdleness());
    
    /// print the ordered vector 
    std::vector<NodeInfo>::iterator it, itEnd;
    std::cout << "set  : ";
    for (it = vec_ordered_nodes.begin(), itEnd = vec_ordered_nodes.end(); it != itEnd; ++it)
    {
        std::cout << ' ' << *it << ' ';
    }
    std::cout << std::endl;

    /// < take the node with the highest idleness 
    if (!vec_ordered_nodes.empty())
    {
        it = vec_ordered_nodes.begin();
        next_vertex = (*it).node_id;
    }    
    std::cout << "best node  : " << next_vertex << std::endl;
    
    return next_vertex;
}

int Conscientious_Reactive_Agent::compute_random_node_in_set(SetNodeInfo& set_nodes)
{
    int next_vertex = -1;

    // take random node 
    if (!set_nodes.empty())
    {        
        int r = rand() % set_nodes.size();
        SetNodeInfo::iterator it = set_nodes.begin();
        std::advance(it,r);
        next_vertex = (*it).node_id;
    }

    return next_vertex;       
}

// compute neighborhood of current vertex 
void Conscientious_Reactive_Agent::compute_neighborhood(int current_vertex, SetNodeInfo& set_nodes)
{    
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);
    boost::recursive_mutex::scoped_lock idleness_locker(idleness_mutex_);       
    
    set_nodes.clear();

    uint num_neighs = graph_[current_vertex].num_neigh;
    uint neighbour_id[num_neighs]; //neighbors table
    int neighbour_cost_to_go[num_neighs];

    if (num_neighs > 0)
    {
        /// < build a set of nodes 
        {
            boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
            for (size_t i = 0; i < num_neighs; i++)
            {
                neighbour_id[i]         = graph_[current_vertex].id_neigh[i];
                neighbour_cost_to_go[i] = graph_[current_vertex].cost[i];
                set_nodes.insert(NodeInfo(neighbour_id[i], vec_instantaneous_idleness_[ neighbour_id[i] ], neighbour_cost_to_go[i]));
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("Conscientious_Reactive_Agent::compute_neighbourhood() - vertex "<< current_vertex << " with no neighbours");
    }
}

// compute a patch of a vertex, i.e. the vertex + its neighborhood 
void Conscientious_Reactive_Agent::compute_patch(int current_vertex, SetNodeInfo& set_nodes)
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    boost::recursive_mutex::scoped_lock idleness_locker(idleness_mutex_);       
    
    set_nodes.clear();

    uint num_neighs = graph_[current_vertex].num_neigh;
    uint neighbour_id[num_neighs]; //neighbors table
    int neighbour_cost_to_go[num_neighs];

    if (num_neighs > 0)
    {
        /// < build a set of nodes
        {
            boost::recursive_mutex::scoped_lock visit_locker(last_visit_mutex_);
            for (size_t i = 0; i < num_neighs; i++)
            {
                neighbour_id[i]         = graph_[current_vertex].id_neigh[i];
                neighbour_cost_to_go[i] = graph_[current_vertex].cost[i];
                set_nodes.insert(NodeInfo(neighbour_id[i], vec_instantaneous_idleness_[ neighbour_id[i] ], neighbour_cost_to_go[i]));
            }
            // add current vertex with its info 
            set_nodes.insert(NodeInfo(current_vertex, vec_instantaneous_idleness_[ current_vertex ], neighbour_cost_to_go[current_vertex]));
        }
    }
    else
    {
        ROS_ERROR_STREAM("Conscientious_Reactive_Agent::compute_neighbourhood() - vertex "<< current_vertex << " with no neighbours");
    }
}

// compute neighborhood with depth 2 
void Conscientious_Reactive_Agent::compute_neighborhood_depth2(int current_vertex, SetNodeInfo& set_nodes)
{
    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_);   
    
    set_nodes.clear();

    uint num_neighs = graph_[current_vertex].num_neigh;
    uint neighbour_id[num_neighs]; //neighbors table

    if (num_neighs > 0)
    {
        for (size_t i = 0; i < num_neighs; i++)
        {
            neighbour_id[i] = graph_[current_vertex].id_neigh[i];
            
            // compute the neighborhood of each neighbor             
            SetNodeInfo set_neighbour_nodes;            
            compute_neighborhood(neighbour_id[i], set_neighbour_nodes);
            
            // compute the union between set_nodes and current set_neighbour_nodes
            set_nodes.insert(set_neighbour_nodes.begin(),set_neighbour_nodes.end());
        }
    }
}

// compute the best vertex in the neighborhood 
int Conscientious_Reactive_Agent::compute_best_neighbour_vertex(int current_vertex, SetNodeInfo& set_nodes, std::vector<NodeInfo>& vec_ordered_nodes)
{
    int next_vertex = -1;
    
    compute_neighborhood(current_vertex, set_nodes);

    next_vertex = compute_best_node_in_set(set_nodes,vec_ordered_nodes);

    return next_vertex;
}

// compute best vertex at depth 2 by eliminating prev_vertex if present 
int Conscientious_Reactive_Agent::compute_best_neighbour_vertex_depth2(int current_vertex, int prev_vertex, SetNodeInfo& set_nodes, std::vector<NodeInfo>& vec_ordered_nodes)
{
    int next_vertex = -1;
    
    // compute neighborhood at depth 2
    compute_neighborhood_depth2(current_vertex, set_nodes);
    
    // eliminate prev_vertex if present 
    SetNodeInfo::iterator it;
    it = set_nodes.find( prev_vertex );
    if(it != set_nodes.end()) set_nodes.erase(it);
    
    next_vertex = compute_best_node_in_set(set_nodes,vec_ordered_nodes);

    return next_vertex;    
}

// compute best vertex at depth 2 by eliminating a list of vertexes if present 
int Conscientious_Reactive_Agent::compute_best_neighbour_vertex_depth2(int current_vertex, std::vector<int> vec_avoid_vertexes, SetNodeInfo& set_nodes, std::vector<NodeInfo>& vec_ordered_nodes)
{
    int next_vertex = -1;
    
    // compute neighborhood at depth 2
    compute_neighborhood_depth2(current_vertex, set_nodes);
    
    // eliminate list of vertices to avoid if present 
    SetNodeInfo::iterator it;
    for(size_t jj=0, jjEnd=vec_avoid_vertexes.size();jj<jjEnd;jj++)
    {
        it = set_nodes.find( vec_avoid_vertexes[jj] );
        if(it != set_nodes.end()) set_nodes.erase(it);        
    }
    
    next_vertex = compute_best_node_in_set(set_nodes,vec_ordered_nodes);

    return next_vertex;        
}


// compute random vertex at depth 2 by eliminating prev_vertex if present 
int Conscientious_Reactive_Agent::compute_random_vertex_depth2(int current_vertex, int prev_vertex)
{
    int next_vertex = -1;
    
    SetNodeInfo set_nodes;
    
    // compute neighborhood at depth 2
    compute_neighborhood_depth2(current_vertex, set_nodes);
    
    // eliminate prev_vertex if present 
    SetNodeInfo::iterator it;
    it = set_nodes.find( prev_vertex );
    if(it != set_nodes.end()) set_nodes.erase(it);
    
    next_vertex = compute_random_node_in_set(set_nodes);

    return next_vertex;    
}

int Conscientious_Reactive_Agent::compute_random_vertex_strategy(int prev_planned_vertex)
{
    ROS_INFO("Conscientious_Reactive_Agent::compute_random_vertex_strategy() - level: %d\n", critical_conflict_level_);
    
    SetNodeInfo set_nodes;

    int next_planned_vertex = prev_planned_vertex; // init variables for the following search loop
    int random_count = 0;  

    switch( critical_conflict_level_) 
    {

    case kCriticalConflict1:
    {
        // select a random vertex in the current neighborhood up to a maximum number of trials  
        while ((next_planned_vertex == prev_planned_vertex) && (random_count++ < kMaxNumAttemptsForRandomNodeSelection))
        {
            int num_neighs = graph_[current_vertex_].num_neigh;
            int i = rand() % num_neighs;
            next_planned_vertex = graph_[current_vertex_].id_neigh[i];
        }   
        ROS_INFO("Conscientious_Reactive_Agent::compute_next_vertex() - selected local random vertex: %d\n", next_planned_vertex);            

    }
    break;

    case kCriticalConflict2:
    {
        // look for a random vertex at depth 2 (different from prev planned vertex)
        next_planned_vertex = compute_random_vertex_depth2(current_vertex_, prev_planned_vertex);
        ROS_INFO("Conscientious_Reactive_Agent::compute_next_vertex() - selected local random vertex: %d at depth 2 with, prev vertex: %d \n", next_planned_vertex, prev_planned_vertex);            
    }
    break;        

    case kCriticalConflict3:
    {
        // look for a random vertex different from the previous one all over the graph  and select the best vertex in the random vertex patch 
        while ((next_planned_vertex == prev_planned_vertex) && (random_count++ < kMaxNumAttemptsForRandomNodeSelection))
        {
            // select a random vertex 
            int rand_vertex = rand() % graph_dimension_;

            // select the best node in the selected random vertex patch (i.e. random vertex + its neighborhood ) 
            compute_patch(rand_vertex, set_nodes);

            std::vector<NodeInfo> vec_ordered_nodes;
            next_planned_vertex = compute_best_node_in_set(set_nodes,vec_ordered_nodes);                   

            ROS_INFO("Conscientious_Reactive_Agent::compute_next_vertex() - selected local best of random vertex: %d, prev vertex: %d\n", next_planned_vertex, prev_planned_vertex);
            if (next_planned_vertex == -1) next_planned_vertex = prev_planned_vertex;
        }            
    }
    break; 

    case kCriticalConflict4:
    default:
    {        
        // let's take a pure random node all over the graph 
        next_planned_vertex = rand() % graph_dimension_;         
        ROS_INFO("Conscientious_Reactive_Agent::compute_next_vertex() - selected pure random vertex: %d, prev vertex: %d\n", next_planned_vertex, prev_planned_vertex);                  

    }

    }  // end switch 

    if (next_planned_vertex == -1) next_planned_vertex = prev_planned_vertex;

#if 0       
    // as a backup, in the unlikely case we failed in the above previous selection, let's take a pure random node 
    if(next_planned_vertex == prev_planned_vertex)
    {
        int rand_vertex = rand() % graph_dimension_;
        ROS_INFO("Conscientious_Reactive_Agent::compute_next_vertex() - backup - pure random vertex: %d, prev vertex: %d\n", rand_vertex, prev_planned_vertex);  
        next_planned_vertex = rand_vertex;
    }
#endif   

    return next_planned_vertex;
}

int Conscientious_Reactive_Agent::compute_next_vertex()
{     
    std::cout << "============================================================" << std::endl;
    std::cout << "Conscientious_Reactive_Agent::compute_next_vertex() - start " << std::endl;
    std::cout << "============================================================" << std::endl;

    boost::recursive_mutex::scoped_lock graph_locker(graph_mutex_); 
        
    std::cout << "current vertex: " << current_vertex_ << ", previous planned vertex: " << next_vertex_ << std::endl;

    const int prev_planned_vertex = next_vertex_;
    int next_planned_vertex = prev_planned_vertex; // init for critical cases

    if (b_critical_path_planning_failure_ || b_critical_node_conflict_) /// < select a random node and take its neighbour node with highest idleness
    {
        /// < CRITICAL CONDITION 
        /// => use random-based strategy 
        
        critical_conflict_level_ = std::min(critical_conflict_level_+1 ,(int)(kCriticalConflictNum-1)); // starting from kCriticalConflict0 this is increased every time we enter here  
        
        std::cout << "Conscientious_Reactive_Agent::compute_next_vertex() - critical planning failure: " << (int)b_critical_path_planning_failure_ 
                << ", critical node conflict: " << (int)b_critical_node_conflict_ 
                << ", critical level: " << critical_conflict_level_ << std::endl;        
                
        int random_count = 0;
        while ((next_planned_vertex == prev_planned_vertex) && (random_count++ < kMaxNumAttemptsForRandomNodeSelection))
        {        
            next_planned_vertex = compute_random_vertex_strategy(prev_planned_vertex);      
            if(next_planned_vertex == prev_planned_vertex) critical_conflict_level_ = std::min(critical_conflict_level_+1 ,(int)(kCriticalConflictNum-1));
            std::cout << "critical level: " << critical_conflict_level_ << std::endl; 
        }
        
    }
    else
    {
        /// < NORMAL SELECTION 
        /// => select best neighbor 
        
        critical_conflict_level_ = 0; // reset critical conflict level 
        
        SetNodeInfo set_nodes;
        std::vector<NodeInfo> vec_ordered_nodes;
        next_planned_vertex = compute_best_neighbour_vertex(current_vertex_, set_nodes, vec_ordered_nodes);

        /// < (1) if there is a conflict then do not insist in selecting the same conflicting node
        /// < (2) if the vertex has been intercepted by a teammate then do not insist in selecting the same vertex
        /// < (3) if the vertex is covered then do not insist in selecting the same vertex 
        if ( 
                ( b_node_conflict_ && (next_planned_vertex == prev_planned_vertex) ) ||                         // (1)
                ( b_next_vertex_intercepted_by_teammate_ && (next_planned_vertex == prev_planned_vertex) ) ||   // (2)
                ( (covered_vertex_ > -1) && (next_planned_vertex == covered_vertex_) ) ||                                                                         // (3) 
                (next_planned_vertex == -1)
           )
        {
     
            std::cout << "managing node conflict on " << prev_planned_vertex <<", covered vertex: "<< covered_vertex_ << std::endl;
#if 0            
            
            /// < take the second best if it exists 
            if (vec_ordered_nodes.size() > 1)
            {

                next_planned_vertex = vec_ordered_nodes[1].node_id;
                std::cout << "Conscientious_Reactive_Agent::compute_next_vertex() - selected second best node: " << next_planned_vertex << std::endl; 
            }
            else
            {
                /// < compute best vertex at depth 2 by eliminating prev_vertex if present 
                next_planned_vertex = compute_best_neighbour_vertex_depth2(current_vertex_, prev_planned_vertex, set_nodes, vec_ordered_nodes);
                std::cout << "Conscientious_Reactive_Agent::compute_next_vertex() - selected best node at depth 2: " << next_planned_vertex << std::endl; 
            }
            
#else          
                        
            /// < compute best vertex at depth 2 by eliminating prev_vertex and covered_vertex_ if present  => get an order list of vertex info at depth 2           
            std::vector<NodeInfo> vec_ordered_nodes2;
            
            std::vector<int> vec_avoid_vertexes; // list of vertices to be avoided 
            vec_avoid_vertexes.push_back(prev_planned_vertex);
            if(covered_vertex_ > -1) vec_avoid_vertexes.push_back(covered_vertex_);
            
            const int next_planned_vertex2 = compute_best_neighbour_vertex_depth2(current_vertex_, vec_avoid_vertexes, set_nodes, vec_ordered_nodes2);
            size_t index_ordered_nodes2 = 0; // index to be used with vec_ordered_nodes2 (the ordered vector of nodes at depth 2)
            
            if (vec_ordered_nodes.size() > 1)
            {
                /// < take the second best if it exists 
                next_planned_vertex = vec_ordered_nodes[1].node_id; // here we use the vec_ordered_nodes, ordered vector of nodes at depth 1
                index_ordered_nodes2 = 0; 
            }   
            else
            {
                /// < take best vertex at depth 2 
                next_planned_vertex = next_planned_vertex2;
                index_ordered_nodes2 = 1;
            }
                
            // check if next_planned_vertex has best navigation cost according to available information 
            if( vec_ordered_nodes2.size() > 0 )
            {
                const size_t list_size = vec_ordered_nodes2.size();
                
                // build a navigation cost table on the basis of available information: team model, team deployment and dynamic graph  
                NavCostTable navCostTable(TEAMSIZE_); // N.B.: this can be limited to a small group of close robots 
                navCostTable.update(tasks_, team_deployment_, graph_, graph_dimension_, graph_mutex_);
                navCostTable.print();

                // N.B.1: the cost-to-go is originally received in "float" and then converted into "int" by using a scale factor (see for instance PatrolAgent::pathPlanningFeedbackCallback() )
                // N.B.2: if nav_cost == -1 we do not have any info concerning our choice                 
                float nav_cost = navCostTable.get(ID_ROBOT_, next_planned_vertex); 
                
                std::cout << "current selection - next_planned_vertex: " << next_planned_vertex << ",  navigation cost: " << nav_cost << std::endl; 
                
                // check if our choice is the best by using available information in the built navigation cost table (robot belief)
                // and take the best node (best = bigger idleness and smaller navigation cost)
                while( ( nav_cost > 0 ) && !navCostTable.is_best(ID_ROBOT_, next_planned_vertex, (int)nav_cost) && ( index_ordered_nodes2 < list_size ) )
                {
                    // N.B.: vec_ordered_nodes2 is ordered (decreasing idleness)
                    next_planned_vertex = vec_ordered_nodes2[index_ordered_nodes2].node_id;
                    nav_cost = navCostTable.get(ID_ROBOT_, next_planned_vertex);
                    
                    std::cout << "current selection - next_planned_vertex: " << next_planned_vertex << ",  navigation cost: " << nav_cost << std::endl; 
                    
                    index_ordered_nodes2++;
                }                
            }
            
#endif            
            
            /// < (1) if there is a node conflict and we are still insisting on the same node then we have a critical conflict => force it!
            /// < (2) if we are still insting toward a covered vertex then we force a critical conflict 
            if( 
                    ( next_planned_vertex == prev_planned_vertex ) ||                          // (1)
                    ( (covered_vertex_ > -1) && (next_planned_vertex == covered_vertex_) ) ||  // (2)
                    ( next_planned_vertex == -1) 
              )
            {
                b_critical_node_conflict_forced_ = true; 
                next_planned_vertex = -1;
                std::cout << "forcing critical node conflict" << std::endl; 
            }
        }
        else
        {
            std::cout << "Conscientious_Reactive_Agent::compute_next_vertex() - selected best node: " << next_planned_vertex << std::endl;              
        }
        
    }
        
    std::cout << "Conscientious_Reactive_Agent::compute_next_vertex() - current: " << current_vertex_ << ", previous: " << prev_planned_vertex << ", planned: " << next_planned_vertex << std::endl;
    std::cout << "Conscientious_Reactive_Agent::compute_next_vertex() - end " << std::endl;     

    return next_planned_vertex;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX

    Conscientious_Reactive_Agent agent;
    agent.init(argc, argv);
    agent.run();

    return 0;
}

