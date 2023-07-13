
/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>
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

#include "NodeSet.h"

#include <limits>

static double squared(double x){ return x*x; }

namespace explplanner
{
    
template<typename StateVec>       
NodeSet<StateVec>::NodeSet():informationGain(-1),utility(std::numeric_limits<double>::max()),navigationCost(-1),label(-1)
{}

template<typename StateVec>
NodeSet<StateVec>::NodeSet(Node<StateVec>* nodeIn, int labelIn):informationGain(-1),utility(std::numeric_limits<double>::max()),navigationCost(-1),label(labelIn)
{
    this->insert(nodeIn);    
}

template<typename StateVec>
NodeSet<StateVec>::NodeSet(std::set<Node<StateVec>*>& nodesIn, int labelIn):informationGain(-1),utility(std::numeric_limits<double>::max()),navigationCost(-1),label(labelIn)
{
    this->insert(nodesIn);
}

template<typename StateVec>
NodeSet<StateVec>::~NodeSet()
{
    nodes.clear();
}

template<typename StateVec>
void NodeSet<StateVec>::clear()
{
    informationGain = -1; // invalid 
    utility = std::numeric_limits<double>::max();
    navigationCost = -1; // invalid   
    label = -1; 
    centroid = Eigen::Vector3d(0.,0.,0.);
    nodes.clear();
}

template<typename StateVec>
void NodeSet<StateVec>::insert(Node<StateVec>* node)
{
    if(node == NULL) return; 
    node->label_ = this->label;                
    this->nodes.insert(node);
}

template<typename StateVec>
void NodeSet<StateVec>::insert(std::set<Node<StateVec>*>& otherNodes)
{
    for(auto it=otherNodes.begin(), itEnd = otherNodes.end(); it != itEnd; it++)
    {
        Node<StateVec>* node = *it; 
        if(node == NULL) continue;         
        node->label_ = this->label;
        this->nodes.insert(node);
    }
}

template<typename StateVec>
void NodeSet<StateVec>::updateData()
{
    size_t counter = 0; 

    centroid << 0.,0.,0.;
    informationGain = 0; 
    utility = std::numeric_limits<double>::max();    

    for(auto it=nodes.begin(), itEnd = nodes.end(); it != itEnd; it++)
    {
        const Node<StateVec>* node = *it;      
        
        centroid[0]+=node->state_[0]; 
        centroid[1]+=node->state_[1];
        centroid[2]+=node->state_[2];
        counter++;

        informationGain += node->local_gain_;
    }        
    if(counter>0)   centroid = centroid/counter; 

    findClosestNodeToCentroid();
}

template<typename StateVec>
void NodeSet<StateVec>::findClosestNodeToCentroid()
{
    centroidNode = NULL;
    
    double distMin2 = std::numeric_limits<double>::max();
    for(auto it=nodes.begin(), itEnd = nodes.end(); it != itEnd; it++)
    {
        Node<StateVec>* node = *it; 
        
        double dist2 = squared(centroid[0] - node->state_[0]) + squared(centroid[1] - node->state_[1]) + squared(centroid[2] - node->state_[2]);
        if( dist2 < distMin2 )  
        {
            distMin2 = dist2;
            centroidNode = node;
        }
    }     

    if(centroidNode == NULL)
    {
        std::cout << "NodeSet<StateVec>::findClosestNodeToCentroid() - ERROR" << std::endl;
    }
}

template class NodeSet<Eigen::Vector4d>;
    
}