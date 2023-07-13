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

#ifndef EXPLORATION_TREE_H_
#define EXPLORATION_TREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>

#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "Tree.h"

namespace explplanner
{

///\class ExplorationTree 
///\brief 
///\author Luigi Freda  
class ExplorationTree : public TreeBase<Eigen::Vector4d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const double kColorGainScale; 
    static const double kTreeZVisualizationOffset;
    static const double kTreeTextZVisualizationOffset;    
    static const double kTreeTextMessageHeight;  
    static const std::string kRvizNamespace; 
    
    enum VizIds { kVizSelectedNode=0, kVizSelectedNodeDisc, kVizCurrentNode}; 
    
public:
    typedef Eigen::Vector4d StateVec;

    ExplorationTree();
    ExplorationTree(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager);
    ~ExplorationTree();

    void initialize();
    
    void setRoot(double x, double y, double z);
    bool addNode(Node<StateVec>* node);      
    
    void removeCurrentNode();
    
    void backTrack();
    std::vector<geometry_msgs::Pose> getPathBackToPrevious();    
        
    void clear();
    
    int getNumNodes() { return counter_; }    
    
public:
    
    void publishNode(Node<StateVec> * node);
    void publishTree();
    void publishNodeRecursive(Node<StateVec> * node);
    
    void publishCurrentNode(Node<StateVec> * node);    
    void publishSelectedNode(Node<StateVec> * node);

protected:
    
    Node<StateVec>* currentNode_;
    
    int g_ID_; // graphics ID for visual markers    
    int iterationCount_; 
    
private:     
  
    visualization_msgs::MarkerArray treeMarkerArray_;
};

}

#endif
