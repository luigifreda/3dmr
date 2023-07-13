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

#include "FrontierTree.h"
#include "Tree.hpp"
#include "ColorUtils.h"


#define VERBOSE 0
#define SHOW_FRONTIER_CLUSTERS 1


static double squared(double x){ return x*x; }


namespace explplanner
{

const double FrontierTree::kColorGainScale = 2.; 

const double FrontierTree::kTreeZVisualizationOffset = 0.2;
const double FrontierTree::kTreeTextZVisualizationOffset = 2 * FrontierTree::kTreeZVisualizationOffset;
const double FrontierTree::kTreeTextMessageHeight = 0.14; 

const std::string FrontierTree::kRvizNamespace = "expl_frontier_tree";

FrontierTree::FrontierTree()
: TreeBase<StateVec>::TreeBase()
{
    iterationCount_ = 0;
    numClusters_ = 0;
    
    kdTree_ = kd_create(3);
}

FrontierTree::FrontierTree(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager)
{
    p_octomap_manager_ = p_manager;
    iterationCount_ = 0;
    
    kdTree_ = kd_create(3);    
        
}

FrontierTree::~FrontierTree()
{
    delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively delete all its children
    if(kdTree_) kd_free(kdTree_);    
}

void FrontierTree::setRoot(double x, double y, double z)
{
    root_[0] = x;
    root_[1] = y;
    root_[2] = z;
    root_[3] = 0;    
        
    rootNode_->state_ = root_;
    exact_root_ = root_;    
    
    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);    
        
    std::cout << "FrontierTree::setRoot() - root:  " <<  root_ << std::endl;     
}

bool FrontierTree::addNode(Node<StateVec>* node)
{
    //std::cout << "FrontierTree::addNode() - start " << std::endl; 
    
    StateVec newState;
    newState = node->state_;
    
    // Find nearest neighbor
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0)
    {
        std::cout << "FrontierTree::addNode() - WARNING - did not find nearest neighbour" << std::endl;
        kd_res_free(nearest);
        return false;
    }

    //Node<StateVec> * newParent = currentNode_;    
    Node<StateVec> * newParent = (Node<StateVec> *) kd_res_item_data(nearest);
    kd_res_free(nearest);

    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
    
    double directionNorm = direction.norm();
    
    // if the new point is too close then do not add a new one but update the found nearest neighbor; 
    // WARNING: this won't update the corresponding Cartesian coordinates stored in the kd-tree (but that's still a reasonable approximation for our purposes here) 
    if (directionNorm < params_.extensionRange_)
    {
        if(newParent->local_gain_ < node->local_gain_)
        {
            //newParent->state_ = node->state_;            
            newParent->local_gain_ = node->local_gain_;
            newParent->is_frontier_ = (node->local_gain_ > params_.zero_gain_);               
        }
        if(newParent->is_frontier_) frontierNodes_.insert(newParent);
        //newParent->neighbors_radius_ = std::max(newParent->neighbors_radius_, directionNorm);               
        return false; 
    }
    
    newState[3] = atan2(direction[1],direction[0]);

    // Create new node and insert into tree
    Node<StateVec> * newNode = new Node<StateVec>;
    newNode->id_ = ++counter_;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newNode->local_gain_ = node->local_gain_;
    //newNode->gain_ = node->gain_; // not actually used here!
    newNode->is_frontier_ = (node->local_gain_ > params_.zero_gain_); 
    
    //newNode->neighbors_radius_ = std::max(newNode->neighbors_radius_, directionNorm);  
    //newParent->neighbors_radius_ = std::max(newParent->neighbors_radius_, directionNorm);    
    
    newParent->children_.push_back(newNode);
    
    if(newNode->is_frontier_) frontierNodes_.insert(newNode);
    
    kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);    
            
    currentNode_ = newNode; 
    
    return true;     
}

void FrontierTree::updateFrontierNodes()
{        
    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);
    
    for (auto it = frontierNodes_.begin(); it != frontierNodes_.end(); ) 
    {
        Node<StateVec>* node = *it;
        node->local_gain_ = gain(node->state_); // update local gain 
        if ( node->local_gain_ < params_.zero_gain_ ) 
        {
            node->is_frontier_ = false; 
            node->label_ = -1;
            it = frontierNodes_.erase(it++);
        }
        else 
        {
            ++it;
        }
    }        
}
    
void FrontierTree::addFrontierNodes(const std::vector<Node<StateVec>* >& frontierNodes)
{
    for(size_t ii=0, iiEnd=frontierNodes.size(); ii<iiEnd; ii++)
    {
        this->addNode(frontierNodes[ii]);
    }
}

void FrontierTree::initialize()
{
    std::cout << "FrontierTree::initialize() - start " << std::endl; 
    
    g_ID_ = 0;
    
    if(kdTree_) kd_free(kdTree_);
    // Initialize kd-tree with root node and prepare log file
    kdTree_ = kd_create(3);    
    
    std::cout << "FrontierTree::initialize() - log file created" << std::endl;   

    rootNode_ = new Node<StateVec>;
    rootNode_->id_ = counter_ = 0;
    rootNode_->distance_ = 0.0;
    //rootNode_->gain_ = params_.zero_gain_ * 0.0;
    rootNode_->parent_ = NULL;

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
    
    // we cannot insert here the root node cause its position may be not set 
    //kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);    

    std::cout << "FrontierTree::initialize() - end " << std::endl;     
}

void FrontierTree::clear()
{
    delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively deletes all its children
    rootNode_ = NULL;
    currentNode_ = NULL;
    bestBacktrackingNode_ = NULL;

    counter_ = 0;
    
    if(kdTree_) kd_free(kdTree_);    
    kdTree_ = 0;  
    
}


void FrontierTree::publishNode(Node<StateVec> * node)
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
    frontiersMarkerArray_.markers.push_back(p);
    
#if SHOW_FRONTIER_CLUSTERS     
    if(node->label_ != -1)    
    {
        visualization_msgs::Marker p;
        p.header.stamp = ros::Time::now();
        p.header.seq = g_ID_;
        p.header.frame_id = params_.navigationFrame_;
        p.id = g_ID_;
        g_ID_++;    
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
        p.scale.x = 0.4;
        p.scale.y = 0.4;
        p.scale.z = 0.01;

        Color color = Colors::GetColor(node->label_);
        p.color.r = color.r;
        p.color.g = color.g;
        p.color.b = color.b;
        p.color.a = 1.0;

        p.lifetime = ros::Duration(10.0);
        p.frame_locked = false;    

        clustersMarkerArray_.markers.push_back(p); 
    }
#endif     
    
    if(node->parent_)
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
        frontiersMarkerArray_.markers.push_back(arrowMarker);
    }
    
}



void FrontierTree::publishTree()
{
    ros::Time timestamp = ros::Time::now();
    
    // delete all markers     
    frontiersMarkerArray_.markers.clear();
#if SHOW_FRONTIER_CLUSTERS 
    clustersMarkerArray_.markers.clear();
#endif     
    
    // now republish all the node markers by using a recursive function 
    g_ID_ = 0;
    publishNodeRecursive(rootNode_);
    
    params_.explorationFrontierTreePub_.publish(frontiersMarkerArray_); 
#if SHOW_FRONTIER_CLUSTERS     
    params_.explorationFrontierClustersPub_.publish(clustersMarkerArray_);     
#endif     
}

void FrontierTree::publishNodeRecursive(Node<StateVec> * node)
{
    if( node == NULL ) return; 
    
    publishNode(node);
    
    for(size_t ii=0, iiEnd=node->children_.size(); ii<iiEnd; ii++)
    {
        publishNodeRecursive(node->children_[ii]);
    }   
}

void FrontierTree::clusterFrontiers()
{
    // reset clustering data 
    numClusters_ = 0;
    clusters_.clear();    
    
    // reset labels of frontier nodes 
    for( auto it=frontierNodes_.begin(), itEnd=frontierNodes_.end(); it != itEnd; it++)
    {
        Node<StateVec>* node = *it; 
        node->label_ = -1;
    }    
    
    int frontierNodeCount = 0; 
    int currentLabel = 0;
    const double maxDist = params_.frontierClusteringRadius_;
    const double maxDist2 = squared(maxDist);    
    
    // let's cluster the frontier nodes 
    for( auto it=frontierNodes_.begin(), itEnd=frontierNodes_.end(); it != itEnd; it++)
    {
        frontierNodeCount++;
        
        Node<StateVec>* currentFrontierNode = *it;  
        std::set<Node<StateVec>*> neighbors;
        std::set<int> labelsOfClustersToMerge;
        
#if VERBOSE        
        std::cout << "* frontier node " << frontierNodeCount <<", id: " << currentFrontierNode->id_ <<  std::endl;        
#endif         
        
        // find neighbors 
#if 0        
        auto itNext = it; 
        itNext++;
        for( ; itNext != itEnd; itNext++)
        {
            Node<StateVec>* otherNode = *itNext;               
            double distance2 = squared(currentFrontierNode->state_[0] - otherNode->state_[0]) + squared(currentFrontierNode->state_[1] - otherNode->state_[1]) + squared(currentFrontierNode->state_[2] - otherNode->state_[2]);
            if( distance2 < maxDist2 )
            {
                neighbors.insert(otherNode);
                if(otherNode->label_ != -1)
                {
                    labelsOfClustersToMerge.insert(otherNode->label_);
                }
            }
        }
#else
        // Find nearest neighbors within range 
        kdres * nnResult = kd_nearest_range3(kdTree_, currentFrontierNode->state_[0], currentFrontierNode->state_[1], currentFrontierNode->state_[2], maxDist);
        if (kd_res_size(nnResult) <= 0)
        {
            std::cout << "FrontierTree::clusterFrontiers() - WARNING - did not find nearest neighbor" << std::endl;
            kd_res_free(nnResult);
        }
        else
        {
            while( !kd_res_end( nnResult ) ) 
            {
                Node<StateVec>* otherNode = (Node<StateVec> *) kd_res_item_data(nnResult);   
                
                // here, neighbors can be non-frontier nodes; we are clustering just frontier nodes 
                if(otherNode->is_frontier_)
                {
                    neighbors.insert(otherNode);
                    if(otherNode->label_ != -1)
                    {
                        labelsOfClustersToMerge.insert(otherNode->label_);
                    }
                }
                // go to the next entry 
                kd_res_next( nnResult );
            }
            
        }   
        kd_res_free(nnResult);           
#endif         
        
#if VERBOSE            
        std::cout << " #neighbors: " << neighbors.size() << std::endl;
#endif        
                
        // if the current node has a label then let's put it inside labelsOfClustersToMerge
        if(currentFrontierNode->label_ != -1)
        {
            labelsOfClustersToMerge.insert(currentFrontierNode->label_);
        }        
        
        if(labelsOfClustersToMerge.empty())
        {      
            // the current node and its found neighbors do not belong to a cluster             
            currentFrontierNode->label_ = ++currentLabel;
            numClusters_++;               
            
#if VERBOSE              
            std::cout << " no label to merge => creating new cluster: " << currentFrontierNode->label_  << std::endl;      
#endif             
 
            // create a new cluster with the node and its neighbors 
            neighbors.insert(currentFrontierNode);
            clusters_[currentFrontierNode->label_] = NodeSet<StateVec>(neighbors,currentFrontierNode->label_);                                          
        }
        else
        {            
            std::vector<int> vecLabels(labelsOfClustersToMerge.begin(),labelsOfClustersToMerge.end());

            // get the oldest cluster and merge all the other clusters into it
            int destLabel = vecLabels[0];
            
            neighbors.insert(currentFrontierNode);            
            
            // insert the current node and its found neighbors into the oldest cluster 
            clusters_[destLabel].insert(neighbors); 

            size_t numClustersToMerge = vecLabels.size();
            
            // merge clusters and destroy the merged clusters [!starting from second element, in order to avoid vecLabels[0]!]
            for(size_t ii=1,iiEnd=vecLabels.size(); ii<iiEnd; ii++)
            {                
                // merge and delete cluster `ii`
                auto itDel = clusters_.find(vecLabels[ii]);
                if(itDel != clusters_.end())
                {
#if VERBOSE                            
                    std::cout << " merging cluster: " << vecLabels[ii] <<" into cluster: " << vecLabels[0] << std::endl;                    
#endif                     
                    // merge cluster `ii` into cluster `destLabel`
                    clusters_[destLabel].insert(clusters_[vecLabels[ii]].nodes);
                
                    clusters_.erase(vecLabels[ii]);
                    numClusters_--;
                    //std::cout << " deleted cluster: " << vecLabels[ii] << std::endl;                      
                }                            
            }
        }                
    }
#if VERBOSE       
    std::cout << "FrontierTree::clusterFrontiers() - #clusters: " << numClusters_ << ", #frontier-nodes: " <<  frontierNodes_.size() << std::endl;   
#endif     
    
    // update frontier cluster data 
    for(auto it=clusters_.begin(), itEnd=clusters_.end(); it != itEnd; it++)
    {
        it->second.updateData();
    }
}

// get the clusters_ as a vector of pointers (without duplicating the node sets)
void FrontierTree::getClustersVecPtr(std::vector<NodeSet<FrontierTree::StateVec>* >& clusters)
{
    clusters.clear();
    for(auto it=clusters_.begin(), itEnd=clusters_.end(); it != itEnd; it++)
    {
        clusters.push_back(&(it->second));
    }    
    
}

Node<FrontierTree::StateVec>* FrontierTree::planBestFrontierCentroid(std::shared_ptr<NavigationTree>& p_nav_tree)
{
    std::cout << "FrontierTree::planBestFrontierCentroid() - start " << std::endl; 
    
    // get cluster as a vector of ptrs 
    std::vector<NodeSet<StateVec>* > clusters;
    getClustersVecPtr(clusters);    
            
    Node<StateVec>* bestNode = NULL;
    double bestUtility = -1;
    int bestii = -1; 
    
    // compute their utility value of the cluster centroids 
    // and select the maximum utility centroid as best next view node   
    for(size_t ii=0, iiEnd=clusters.size(); ii<iiEnd; ii++)
    {
        Node<StateVec>* centroidNode = clusters[ii]->centroidNode;
        Node<StateVec>* navNode = p_nav_tree->getNearest(centroidNode->state_[0], centroidNode->state_[1], centroidNode->state_[2]);
        clusters[ii]->utility = exp(-params_.degressiveCoeff_ * navNode->distance_) * clusters[ii]->informationGain;
        if( bestUtility < clusters[ii]->utility)
        {
            bestUtility = clusters[ii]->utility; 
            bestNode = navNode;
            bestii = ii;
        }        
        std::cout << "FrontierTree::planBestFrontierCentroid() - cluster " << ii << ", utility: " << clusters[ii]->utility <<  std::endl; 
    }
    
    std::cout << "FrontierTree::planBestFrontierCentroid() - selected cluster " << bestii << ", utility: " << bestUtility <<  std::endl;     
    
    return bestNode;
}

std::vector<geometry_msgs::Pose> FrontierTree::getPathToBestFrontierCentroid(std::shared_ptr<NavigationTree>& p_nav_tree, Eigen::Vector3d& robot_position)
{
    std::vector<geometry_msgs::Pose> ret;
    
    Node<StateVec>* bestNode = planBestFrontierCentroid(p_nav_tree);   
    bestBacktrackingNode_ = bestNode;
    
    if(bestNode == NULL)
    {
        return ret;
    }
    
    geometry_msgs::Pose start, end; 
    
    start.position.x = robot_position[0];
    start.position.y = robot_position[1];
    start.position.z = robot_position[2];            
    ret.push_back(start);
    
    end.position.x = bestNode->state_[0];
    end.position.y = bestNode->state_[1];
    end.position.z = bestNode->state_[2];      
    ret.push_back(end);
    
    return ret;
   
}

void FrontierTree::resetSelectedBackTrackingCluster()
{
    if(bestBacktrackingNode_ != NULL)
    {
        std::cout << "FrontierTree::resetSelectedBackTrackingCluster() - resetting cluster of node " << std::endl;   
        NodeSet<StateVec>& cluster = clusters_[bestBacktrackingNode_->label_];
        std::set<Node<StateVec>*>& nodes = cluster.nodes; 
        for(auto it=nodes.begin(),itEnd=nodes.end(); it != itEnd; it++)
        {
            Node<StateVec>* node = *it; 
            node->is_frontier_ = false; 
            node->label_ = -1;
            frontierNodes_.erase(node);            
        }
        cluster.clear();
    }
}

}
