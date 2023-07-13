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

#ifndef EXPLORATION_NAVIGATION_TREE_H_
#define EXPLORATION_NAVIGATION_TREE_H_

#include <ros/ros.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <kdtree/kdtree.h>

#include <set>
#include <map>

#include <unordered_map>

#include "Tree.h"
#include "PriorityQueue.h"

namespace explplanner
{
        
///\class NavigationTree 
///\brief 
///\author Luigi Freda  
class NavigationTree
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const double kColorGainScale; 
    static const double kTreeZVisualizationOffset;
    
    static const std::string kRvizNamespace;     
       
public:
    typedef Eigen::Vector4d StateVec;
    
    ///\class PriorityNodeQueue 
    ///\brief Class for implementing a priority queue with nodes
    ///\author Luigi Freda    
    class PriorityNodeQueue: public PriorityQueue<Node<StateVec>*, double> 
    {
    public: 
        typedef PriorityQueue::PriorityElementT PriorityNode;

    public:

        PriorityNodeQueue(){}  
        
        Node<StateVec>* TopNode()
        {
            PriorityNode& priority_elem = Top(); 
            return priority_elem.data;
        }        
    };    
    
public:    

    NavigationTree();
    ~NavigationTree();
    
    void setParams(const ExplParams& params);
    void setRobotId(int id){ robot_id_ = id; }     
  
    void setInputKdtree(kdtree* kdTree);
    void setRoot(double x, double y, double z);
    
    void initialize();
    void clear();
    
    Node<StateVec>* addNode(double x, double y, double z, Node<StateVec> * parent); 
    Node<StateVec>* addNode(StateVec& newState, Node<StateVec> * parent);
    
    void expand();
    
public:
    
    void publishNode(Node<StateVec>* node);
    void publishTree();
    void publishNodeRecursive(Node<StateVec>* node);

public:
    
    // get nearest from kdTree_
    Node<StateVec>* getNearest(const double x, const double y, const double z);
    void getNearestRange(const double x, const double y, const double z, const double range, std::vector<Node<StateVec>*>& neighbors);     
    
    // get nearest from extKdTree_
    Node<StateVec>* getExtNearest(const double x, const double y, const double z);
    void getExtNearestRange(const double x, const double y, const double z, const double range, std::vector<Node<StateVec>*>& neighbors);  
    
    // get nearest from input tree 
    Node<StateVec>* getTreeNearest(kdtree* tree, const double x, const double y, const double z);
    void getTreeNearestRange(kdtree* tree, const double x, const double y, const double z, const double range, std::vector<Node<StateVec>*>& neighbors);   
    
    // get neighbors from the tree data structure which the node belongs to
    void getNeighbors(Node<StateVec>* node, std::vector<Node<StateVec>*>& neighbors);
    
    double distanceSquared(Node<StateVec>* node1, Node<StateVec>* node2);
       
protected:    
    
    ExplParams params_;
    int robot_id_;     
  
    int g_ID_; // graphics ID for visual markers    
    
    kdtree* extKdTree_; // input kdTree_ (set from an external object), this acts as a set of nodes to reorganize in the form of a navigation tree
    
    kdtree* kdTree_; // internal kdTree_    
    Node<StateVec>* rootNode_;
    StateVec root_;    
    
    int counter_; // number of added nodes 
    
    visualization_msgs::MarkerArray markerArray_;    

protected:

    std::unordered_map<int, bool> visitedNodes_;
    PriorityNodeQueue priorityQueue_;             
};

}

#endif
