/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
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

#ifndef NODE_SET_H_
#define NODE_SET_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <set>
#include <map>

#include "Tree.h"

namespace explplanner
{
    
template<typename StateVec>    
class NodeSet 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public:

    NodeSet();
    NodeSet(Node<StateVec>* nodeIn, int labelIn);
    NodeSet(std::set<Node<StateVec>*>& nodesIn, int labelIn);
    
    ~NodeSet();

    void clear();
    
    void insert(Node<StateVec>* node);
    
    void insert(std::set<Node<StateVec>*>& otherNodes);
    
    void updateData();
    
public: 
    
    double informationGain; // total information gain of the set 
    double utility;         // total utility = exp(-params_.degressiveCoeff_ * centroidNode->distance_) * informationGain
    double navigationCost;  // navigation cost to reach the centroid 
    int label;              // label of the node set 
    
    Eigen::Vector3d centroid;     
    Node<StateVec>* centroidNode; // closest node to centroid 
            
    std::set<Node<StateVec>*> nodes;  
    
private: 
    
    void findClosestNodeToCentroid();
    
};

}

#endif
