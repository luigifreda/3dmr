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

#include "NavigationTree.h"
#include "ColorUtils.h"

namespace explplanner
{
   
const double NavigationTree::kColorGainScale = 2.; 
const double NavigationTree::kTreeZVisualizationOffset = 0.2;
const std::string NavigationTree::kRvizNamespace = "expl_nav_tree";

NavigationTree::NavigationTree()
{
    extKdTree_ = NULL;
    kdTree_ = NULL;
    
    rootNode_ = NULL;
    
    counter_ = 0;
    
    kdTree_ = kd_create(3);
}    
    
NavigationTree::~NavigationTree()
{
    if(rootNode_) delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively delete all its children
    if(kdTree_) kd_free(kdTree_);    
}

void NavigationTree::setParams(const ExplParams& params) 
{ 
    params_ = params;   
}

void NavigationTree::initialize()
{
    std::cout << "FrontierTree::initialize() - start " << std::endl;    
    
    g_ID_ = 0;
    
    if(kdTree_) kd_free(kdTree_);
    kdTree_ = kd_create(3);    

    rootNode_ = new Node<StateVec>;
    rootNode_->id_ = 0;
    rootNode_->state_ = root_;     
    rootNode_->distance_ = 0.0;
    rootNode_->parent_ = NULL; 

    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);  
    
    if(extKdTree_ == NULL)
    {
        ROS_ERROR_STREAM("NavigationTree::setRoot() - you have to first set the input kdtree");
        quick_exit(-1);
    }
    
    Node<StateVec>* extNearestNode = getExtNearest(root_.x(), root_.y(), root_.z());    
    Node<StateVec>* newNode = addNode(extNearestNode->state_, rootNode_); // newNode becomes a clone of extNearestNode    
    newNode->local_gain_ = extNearestNode->local_gain_;
    newNode->is_frontier_ = extNearestNode->is_frontier_;    
    newNode->label_ = extNearestNode->label_;  
    //newNode->neighbors_radius_ = extNearestNode->neighbors_radius_;
    
    newNode->data_ = extNearestNode;
    
    priorityQueue_.Push(newNode->id_,newNode,0);
    visitedNodes_[extNearestNode->id_] = true;    

    std::cout << "ExplorationFrontierTree::initialize() - end " << std::endl;     
}

void NavigationTree::clear()
{
    if(rootNode_) delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively deletes all its children
    rootNode_ = NULL;

    counter_ = 0;     
    
    if(kdTree_) kd_free(kdTree_);    
    kdTree_ = 0;  
    
    extKdTree_ = 0;
    
    visitedNodes_.clear();
    
    priorityQueue_.Clear();
}

void NavigationTree::setRoot(double x, double y, double z)    
{                
    root_[0] = x;
    root_[1] = y;
    root_[2] = z;
    root_[3] = 0;    
}

void NavigationTree::setInputKdtree(kdtree* kdTree)
{
    extKdTree_ = kdTree;
}

Node<NavigationTree::StateVec>* NavigationTree::addNode(double x, double y, double z, Node<NavigationTree::StateVec>* parent)
{
    StateVec newState;
    newState[0] = x;
    newState[1] = y;    
    newState[2] = z; 
    
    return addNode(newState, parent);
}

Node<NavigationTree::StateVec>* NavigationTree::addNode(NavigationTree::StateVec& newState, Node<NavigationTree::StateVec>* parent)
{    
    // Create new node and insert into tree
    Node<StateVec>* newNode = new Node<StateVec>();
    newNode->id_ = ++counter_;
    newNode->state_ = newState;
    newNode->parent_ = parent;

    if(parent)
    {
        Eigen::Vector3d origin(parent->state_[0], parent->state_[1], parent->state_[2]);
        Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);

        newState[3] = atan2(direction[1],direction[0]);
        
        newNode->distance_ = parent->distance_ + direction.norm(); // update the distance from the root 
                        
        //std::cout << " distance: " << direction.norm() << std::endl;         

        parent->children_.push_back(newNode);    
    }
     
    kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode); // internal kdtree  
    
    return newNode;     
}

Node<NavigationTree::StateVec>* NavigationTree::getNearest(const double x, const double y, const double z)
{
    return getTreeNearest(kdTree_,x,y,z);
}
    
void NavigationTree::getNearestRange(const double x, const double y, const double z, const double range, std::vector<Node<StateVec>*>& neighbors)
{
    return getTreeNearestRange(kdTree_,x,y,z,range,neighbors);    
}

Node<NavigationTree::StateVec>* NavigationTree::getExtNearest(const double x, const double y, const double z)
{
    return getTreeNearest(extKdTree_,x,y,z);
}
    
void NavigationTree::getExtNearestRange(const double x, const double y, const double z, const double range, std::vector<Node<StateVec>*>& neighbors)
{
    return getTreeNearestRange(extKdTree_,x,y,z,range,neighbors);    
}
    
Node<NavigationTree::StateVec>* NavigationTree::getTreeNearest(kdtree* tree, const double x, const double y, const double z)
{
    Node<StateVec>* res = NULL;
    
    // Find nearest neighbor
    kdres * nearest = kd_nearest3(tree, x, y, z);
    if (kd_res_size(nearest) <= 0)
    {
        std::cout << "NavigationTree::getTreeNearestRange() - WARNING - did not find nearest neighbour" << std::endl;
        kd_res_free(nearest);
        return res;
    }
 
    res = (Node<StateVec> *) kd_res_item_data(nearest);
    kd_res_free(nearest);
    return res;    
}


void NavigationTree::getTreeNearestRange(kdtree* tree, const double x, const double y, const double z, const double range, std::vector<Node<NavigationTree::StateVec>*>& neighbors)
{
    neighbors.clear();
    
    std::vector< std::pair<double,Node<StateVec>*> > list; 
    
    // Find nearest neighbors within range 
    kdres * nnResult = kd_nearest_range3(tree, x, y, z, range);
    if (kd_res_size(nnResult) <= 0)
    {
        std::cout << "NavigationTree::getTreeNearestRange() - WARNING - did not find nearest neighbor" << std::endl;
        kd_res_free(nnResult);
    }
    else
    {
        while( !kd_res_end( nnResult ) ) 
        {
            Node<StateVec>* neighbor = (Node<StateVec> *) kd_res_item_data(nnResult);   
            
            Eigen::Vector3d direction(neighbor->state_[0] - x, neighbor->state_[1] - y, neighbor->state_[2] - z);
            double distance2 = direction.squaredNorm();
            
            list.push_back(std::make_pair(distance2,neighbor));
                                
            // go to the next entry 
            kd_res_next( nnResult );
        }

    }   
    kd_res_free(nnResult);         
    
    std::sort(list.begin(),list.end());
    for(auto it=list.begin(),itEnd=list.end();it<itEnd;it++)
    {
        neighbors.push_back(it->second);    
    }
}        
        
void NavigationTree::getNeighbors(Node<StateVec>* node, std::vector<Node<StateVec>*>& neighbors)
{
    neighbors.clear();
    
    std::vector< std::pair<double,Node<StateVec>*> > list; 
    
    if(node->parent_) 
    {
        const double distance2 = distanceSquared(node, node->parent_);
        list.push_back(std::make_pair(distance2, node->parent_));
    }
    
    for(size_t ii=0,iiEnd=node->children_.size(); ii<iiEnd; ii++)
    {
        const double distance2 = distanceSquared(node, node->children_[ii]);        
        list.push_back(std::make_pair(distance2, node->children_[ii]));
    }
    
    std::sort(list.begin(),list.end());
    for(auto it=list.begin(),itEnd=list.end();it<itEnd;it++)
    {
        neighbors.push_back(it->second);    
    }    
}

double NavigationTree::distanceSquared(Node<StateVec>* node1, Node<StateVec>* node2)
{
    Eigen::Vector3d direction(node1->state_[0] - node2->state_[0], node1->state_[1] - node2->state_[1], node1->state_[2] - node2->state_[2]);
    return direction.squaredNorm();    
}

void NavigationTree::expand()
{
    std::cout << "NavigationTree::expand() - start " << std::endl;
        
    int count = 0; 
    while(!priorityQueue_.IsEmpty())
    {        
        // get top elem
        Node<StateVec>* currentNode = priorityQueue_.TopNode();   
        priorityQueue_.Pop();
        count++;
        
        // find external neighbors of current node in external kdtree
        std::vector<Node<NavigationTree::StateVec>*> extNeighbors;
        
        //const double neighbors_radius = extensionDistance_;
        //const double neighbors_radius = currentNode->neighbors_radius_*1.05;
        //getExtNearestRange(currentNode->state_.x(), currentNode->state_.y(), currentNode->state_.z(), neighbors_radius, extNeighbors);
        
        Node<StateVec>* extCurrentNode = static_cast<Node<StateVec>*>(currentNode->data_);
        if(extCurrentNode) getNeighbors(extCurrentNode, extNeighbors);
     
        //std::cout << "NavigationTree::expand() - iteration: " << count << ", #neighbors: " << extNeighbors.size() <<  std::endl;
        
        // add all the unvisited external neighbors to this nav tree and mark them as visited 
        for(size_t ii=0, iiEnd=extNeighbors.size(); ii<iiEnd; ii++)        
        {
            const auto foundElem = visitedNodes_.find(extNeighbors[ii]->id_);
            if(foundElem == visitedNodes_.end())
            {
                // manage the unvisited node 
                Node<StateVec>* newNode = addNode(extNeighbors[ii]->state_, currentNode); // newNode is a clone of extNeighbors[ii]
                                
                newNode->local_gain_ = extNeighbors[ii]->local_gain_;
                newNode->is_frontier_ = extNeighbors[ii]->is_frontier_;                
                newNode->label_ = extNeighbors[ii]->label_;         
                //newNode->neighbors_radius_ = extNeighbors[ii]->neighbors_radius_;   
                
                newNode->data_ = extNeighbors[ii];

                visitedNodes_[extNeighbors[ii]->id_] = true;
                priorityQueue_.Push(extNeighbors[ii]->id_, newNode, -newNode->distance_); // highest priority goes to the node with shortest distance from root (we invert the sign)                
            }
        }
    }
    
    std::cout << "NavigationTree::expand() - expanded #nodes: " << count << std::endl;    
}

void NavigationTree::publishNode(Node<StateVec> * node)
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
    float gain_color = 0.1;
    if(node->is_frontier_)
    {
        const float gain = std::min( std::max( (node->local_gain_ - params_.zero_gain_)*kColorGainScale, 100.0 ) , 255.0);        
        gain_color = gain/255.0;
    }
    p.color.r = gain_color;
    p.color.g = gain_color;
    p.color.b = 0.0;
    p.color.a = 1.0;
    //p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;    
    
    //params_.explorationFrontierPathPub_.publish(p);
    markerArray_.markers.push_back(p);
    
    if (node->parent_)
    {
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

        Color color = Colors::GetColor(robot_id_);

        arrowMarker.color.r = color.r;
        arrowMarker.color.g = color.g;
        arrowMarker.color.b = color.b;    
        arrowMarker.color.a = 1.0;

        //p.lifetime = ros::Duration(10.0);
        //arrowMarker.frame_locked = false;

        //params_.explorationFrontierPathPub_.publish(arrowMarker);
        markerArray_.markers.push_back(arrowMarker);
    }
}



void NavigationTree::publishTree()
{
    ros::Time timestamp = ros::Time::now();
    
    // delete all markers     
    markerArray_.markers.clear();
    
    // now republish all the node markers by using a recursive function 
    g_ID_ = 0;
    publishNodeRecursive(rootNode_);
    
    params_.explorationNavTreePub_.publish(markerArray_); 
  
}

void NavigationTree::publishNodeRecursive(Node<StateVec> * node)
{
    if( node == NULL ) return; 
    
    publishNode(node);
    
    for(size_t ii=0, iiEnd=node->children_.size(); ii<iiEnd; ii++)
    {
        publishNodeRecursive(node->children_[ii]);
    }   
}

    
}