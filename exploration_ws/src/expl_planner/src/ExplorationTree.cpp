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

#include <cstdlib>
#include <limits>
#include <algorithm>

#include "ExplorationTree.h"
#include "Tree.hpp"


#define SHOW_TEXT_MARKERS 0

namespace explplanner
{


const double ExplorationTree::kColorGainScale = 1.; 
const double ExplorationTree::kTreeZVisualizationOffset = 0.15;
const double ExplorationTree::kTreeTextZVisualizationOffset = 2 * ExplorationTree::kTreeZVisualizationOffset;
const double ExplorationTree::kTreeTextMessageHeight = 0.14; 

const std::string ExplorationTree::kRvizNamespace = "expl_tree";

ExplorationTree::ExplorationTree()
: TreeBase<StateVec>::TreeBase()
{
    iterationCount_ = 0;
}

ExplorationTree::ExplorationTree(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager)
{
    p_octomap_manager_ = p_manager;
    iterationCount_ = 0;    
}

ExplorationTree::~ExplorationTree()
{
    delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively deletes all its children    
}

void ExplorationTree::setRoot(double x, double y, double z)
{
    root_[0] = x;
    root_[1] = y;
    root_[2] = z;
    root_[3] = 0;    
        
    rootNode_->state_ = root_;
    exact_root_ = root_;    
    
    //publishNode(rootNode_);
    
    std::cout << "ExplorationTree::setRoot() - root:  " <<  root_ << std::endl;     
}

bool ExplorationTree::addNode(Node<StateVec>* node)
{
    std::cout << "ExplorationTree::addNode() - start " << std::endl; 
    
    //boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);
    
    StateVec newState;
    newState = node->state_;

    Node<StateVec>* newParent = currentNode_;

    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
    
    newState[3] = atan2(direction[1],direction[0]);

    // Create new node and insert into tree
    Node<StateVec>* newNode = new Node<StateVec>;
    newNode->id_ = ++counter_;    
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newNode->local_gain_ = node->local_gain_;
    newNode->gain_ = node->gain_;

    newParent->children_.push_back(newNode);

    currentNode_ = newNode; 
    
    return true;      
}

void ExplorationTree::removeCurrentNode()
{
    if( currentNode_ != rootNode_)
    {
        Node<StateVec>* node = currentNode_;   
        
        currentNode_ = currentNode_->parent_;
        std::remove(currentNode_->children_.begin(),currentNode_->children_.end(),node);
             
        delete node;
        // counter_--;  // this cannot be decreased since it is used for managing node ids 
    }    
    
    publishTree();
}

void ExplorationTree::backTrack()
{
    if( currentNode_ != rootNode_)
    {
        currentNode_ = currentNode_->parent_;   
    }
}

std::vector<geometry_msgs::Pose> ExplorationTree::getPathBackToPrevious()
{
    std::vector<geometry_msgs::Pose> ret;
    if(currentNode_ == rootNode_)
    {
        return ret;
    }
    Node<StateVec>* backtrackNode = currentNode_->parent_;
    
    geometry_msgs::Pose start, end; 
    
    start.position.x = currentNode_->state_[0];
    start.position.y = currentNode_->state_[1];
    start.position.z = currentNode_->state_[2];            
    ret.push_back(start);
    
    end.position.x = backtrackNode->state_[0];
    end.position.y = backtrackNode->state_[1];
    end.position.z = backtrackNode->state_[2];      
    ret.push_back(end);
    return ret;
   
}

void ExplorationTree::initialize()
{
    std::cout << "ExplorationTree::initialize() - start " << std::endl; 
    
    g_ID_ = 0;

    rootNode_ = new Node<StateVec>;
    rootNode_->id_ = 0;    
    rootNode_->distance_ = 0.0;
    rootNode_->gain_ = params_.zero_gain_ * 0.0;
    rootNode_->parent_ = NULL;

    std::cout << "ExplorationTree::initialize() - root node created " << std::endl;
    
    if (params_.exact_root_)
    {
        if (iterationCount_ <= 1)
        {
            exact_root_ = root_;
        }
        rootNode_->state_ = exact_root_; // after the first iteration this is set to the current robot state
    }
    else
    {
        rootNode_->state_ = root_;
    }
    
    currentNode_ = rootNode_;

    std::cout << "ExplorationTree::initialize() - end " << std::endl;     
}

void ExplorationTree::clear()
{
    std::cout << "ExplorationTree::clear()" << std::endl;   
    delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively delete all its children    
    rootNode_ = NULL;
    currentNode_ = NULL;

    counter_ = 0;
}


void ExplorationTree::publishNode(Node<StateVec> * node)
{
    ros::Time timestamp = ros::Time::now();
    
    // < node part 
    visualization_msgs::Marker p;
    p.header.stamp = timestamp;
    p.header.seq = g_ID_;
    p.header.frame_id = params_.navigationFrame_;
    p.id = g_ID_;
    g_ID_++;
    p.ns = kRvizNamespace;
    p.type = visualization_msgs::Marker::SPHERE;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + kTreeZVisualizationOffset;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, node->state_[3]);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = 0.3;
    p.scale.y = 0.3;
    p.scale.z = 0.1;
    //    const float gain = std::min( std::max(node->gain_ / kColorGainScale, 20.0) , 255.0);    
    //    const float gain_color = gain/255.0;
    //    p.color.r = gain_color;
    //    p.color.g = gain_color;
    p.color.r = 1.;
    p.color.g = 1.;
    p.color.b = 0.0;
    p.color.a = 1.0;
    //p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;    
    
    //params_.explorationPathPub_.publish(p);
    treeMarkerArray_.markers.push_back(p);
    
#if SHOW_TEXT_MARKERS
    
    visualization_msgs::Marker tm;
    tm.header.stamp = p.header.stamp;
    tm.header.seq = g_ID_;
    tm.header.frame_id = params_.navigationFrame_;
    tm.id = g_ID_;
    g_ID_++;
    tm.ns = kRvizNamespace;
    tm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tm.action = visualization_msgs::Marker::ADD;
    tm.pose.position.x = node->state_[0];
    tm.pose.position.y = node->state_[1];
    tm.pose.position.z = node->state_[2] + kTreeTextZVisualizationOffset;
    tm.scale.z = kTreeTextMessageHeight;
    std::stringstream sss;
    sss.precision(2);
    sss.setf(std::ios::fixed, std::ios::floatfield);
    sss << node->gain_; 
    tm.text = sss.str();
    tm.color.r = 1.;
    tm.color.g = 1.;
    tm.color.b = 1.;
    tm.color.a = 1.;
    //tm.lifetime = ros::Duration(10.0);
    tm.frame_locked = false;    
    
    //params_.explorationPathPub_.publish(tm);
    treeMarkerArray_.markers.push_back(tm);    
        
#endif     
        
    if (!node->parent_)
        return;

    // < arrow part 
    visualization_msgs::Marker arrowMarker;
    arrowMarker.header.stamp = p.header.stamp;
    arrowMarker.header.seq = g_ID_;
    arrowMarker.header.frame_id = params_.navigationFrame_; 
    arrowMarker.id = g_ID_;
    g_ID_++;
    arrowMarker.ns = kRvizNamespace;
    arrowMarker.type = visualization_msgs::Marker::ARROW;
    arrowMarker.action = visualization_msgs::Marker::ADD;
    arrowMarker.points.resize(2);
    arrowMarker.points[0].x = node->parent_->state_[0];
    arrowMarker.points[0].y = node->parent_->state_[1];
    arrowMarker.points[0].z = node->parent_->state_[2] + kTreeZVisualizationOffset;
    arrowMarker.points[1].x = node->state_[0];
    arrowMarker.points[1].y = node->state_[1];
    arrowMarker.points[1].z = node->state_[2] + kTreeZVisualizationOffset;    
    arrowMarker.scale.x = 0.03;
    arrowMarker.scale.y = 0.03;
    arrowMarker.scale.z = 0.03;
    arrowMarker.color.r = 0.0;
    arrowMarker.color.g = 1.0;
    arrowMarker.color.b = 0.0;
    arrowMarker.color.a = 1.0;
    //p.lifetime = ros::Duration(10.0);
    //arrowMarker.frame_locked = false;
    
    //params_.explorationPathPub_.publish(arrowMarker);
    treeMarkerArray_.markers.push_back(arrowMarker);    
    
}

void ExplorationTree::publishCurrentNode(Node<StateVec> * node)
{
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = kVizCurrentNode;
    p.header.frame_id = params_.navigationFrame_;
    p.id = kVizCurrentNode;
    p.ns = kRvizNamespace;
    p.type = visualization_msgs::Marker::CYLINDER;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + kTreeZVisualizationOffset;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    p.scale.x = 0.5;
    p.scale.y = 0.5;
    p.scale.z = 0.01;
    p.color.r = 0.;
    p.color.g = 1.;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;    

    params_.explorationNodesPub_.publish(p);
}

void ExplorationTree::publishSelectedNode(Node<StateVec> * node)
{
    std::cout << "ExplorationTree::publishSelectedNode()" << std::endl; 
    
    {
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = kVizSelectedNode;
    p.header.frame_id = params_.navigationFrame_;
    p.id = kVizSelectedNode;
    p.ns = kRvizNamespace;
    p.type = visualization_msgs::Marker::SPHERE;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + kTreeZVisualizationOffset;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    p.scale.x = 0.3;
    p.scale.y = 0.3;
    p.scale.z = 0.1;
    p.color.r = 1.;
    p.color.g = 1.;
    p.color.b = 0.0;
    p.color.a = 1.0;    
    p.lifetime = ros::Duration(5.0);
    p.frame_locked = false;    

    params_.explorationNodesPub_.publish(p);  
    }
    
    {
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = kVizSelectedNodeDisc;
    p.header.frame_id = params_.navigationFrame_;
    p.id = kVizSelectedNodeDisc;
    p.ns = kRvizNamespace;
    p.type = visualization_msgs::Marker::CYLINDER;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + kTreeZVisualizationOffset;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    p.scale.x = 0.5;
    p.scale.y = 0.5;
    p.scale.z = 0.01;
    p.color.r = 0.;
    p.color.g = 1.;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;    

    params_.explorationNodesPub_.publish(p); 
    }
    
}

void ExplorationTree::publishTree()
{
    ros::Time timestamp = ros::Time::now();
    
    // delete all markers 
    treeMarkerArray_.markers.clear();
    
    // now republish all the node markers by using a recursive function 
    g_ID_ = 0;
    publishNodeRecursive(rootNode_);
    
    params_.explorationPathPub_.publish(treeMarkerArray_);
}

void ExplorationTree::publishNodeRecursive(Node<StateVec> * node)
{
    if( node == NULL ) return; 
    
    publishNode(node);
    
    for(size_t ii=0, iiEnd=node->children_.size(); ii<iiEnd; ii++)
    {
        publishNodeRecursive(node->children_[ii]);
    }
}

}

