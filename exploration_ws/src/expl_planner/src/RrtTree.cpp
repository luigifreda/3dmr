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

/*
* Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef RRTTREE_HPP_ 
#define RRTTREE_HPP_

#include <cstdlib>
#include <limits>
#include <multiagent_collision_check/multiagent_collision_checker.h>

#include "RrtTree.h"
#include "Tree.hpp"
#include "GainAngleHistogram.h"
#include "ColorUtils.h"

#define SHOW_DEBUG 0

#define USE_LOCAL_GAIN_AS_ACTUAL_GAIN 0
        // 1: newNode->gain_ = newNode->local_gain_ * exp(-params_.degressiveCoeff_ * newNode->distance_);
        // 0: newNode->gain_ = newParent->gain_ + newNode->local_gain_ * exp(-params_.degressiveCoeff_ * newNode->distance_);

#define USE_SOFT_LINE_COLLISION_CHECKING 1    // N.B.: consider also that collision checking is managed when expanding the tree on the traversability map 
#define USE_SOFT_ENDPOINT_COLLISION_CHECKING 0    // N.B.: we just check at the end

#define SHOW_TEXT_MARKERS 1

namespace explplanner
{

const double RrtTree::kColorGainScale = 2; 

const double RrtTree::kTreeZVisualizationOffset = 0.2;
const double RrtTree::kTreeTextZVisualizationOffset = 2 * RrtTree::kTreeZVisualizationOffset;
const double RrtTree::kTreeTextMessageHeight = 0.14; 

const double RrtTree::kZVisibilityOffset = 0.1;  

const std::string RrtTree::kRvizNamespaceNodes = "search_tree_nodes";
const std::string RrtTree::kRvizNamespaceTexts = "search_tree_texts";   
const std::string RrtTree::kRvizNamespaceEdges = "search_tree_edges";       

inline double deadZone(const double& val, const double& th)
{
#if 1
    if(val>th) return val; 
    return 0; 
#else
    return val; 
#endif 
}

RrtTree::RrtTree()
: TreeBase<StateVec>::TreeBase()
{
    kdTree_ = kd_create(3);
    //kd_data_destructor(kdTree_, nodeDestructor);
    
    iterationCount_ = 0;
    for (int i = 0; i < 4; i++)
    {
        inspectionThrottleTime_.push_back(ros::Time::now().toSec());
    }

    // If logging is required, set up files here
    bool ifLog = false;
    std::string ns = ros::this_node::getName();
    ros::param::get(ns + "/nbvp/log/on", ifLog);
    if (ifLog)
    {
        time_t rawtime;
        struct tm * ptm;
        time(&rawtime);
        ptm = gmtime(&rawtime);
        logFilePath_ = ros::package::getPath("expl_planner") + "/data/"
                + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
                + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
                + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
        int ret = system(("mkdir -p " + logFilePath_).c_str());
        logFilePath_ += "/";
        fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
        filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
    }
}

RrtTree::RrtTree(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager)
{
    //mesh_ = mesh;
    p_octomap_manager_ = p_manager;

    kdTree_ = kd_create(3);
    //kd_data_destructor(kdTree_, nodeDestructor);
    
    iterationCount_ = 0;
    for (int i = 0; i < 4; i++)
    {
        inspectionThrottleTime_.push_back(ros::Time::now().toSec());
    }

    // If logging is required, set up files here
    bool ifLog = false;
    std::string ns = ros::this_node::getName();
    ros::param::get(ns + "/nbvp/log/on", ifLog);
    if (ifLog)
    {
        time_t rawtime;
        struct tm * ptm;
        time(&rawtime);
        ptm = gmtime(&rawtime);
        logFilePath_ = ros::package::getPath("expl_planner") + "/data/"
                + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
                + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
                + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
        int ret = system(("mkdir -p " + logFilePath_).c_str());
        logFilePath_ += "/";
        fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
        filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
    }
}

RrtTree::~RrtTree()
{
    delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively deletes all its children
    if(kdTree_) kd_free(kdTree_);
    
    if (fileResponse_.is_open())
    {
        fileResponse_.close();
    }
    if (fileTree_.is_open())
    {
        fileTree_.close();
    }
    if (filePath_.is_open())
    {
        filePath_.close();
    }  
}

void RrtTree::setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                                 transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);

    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.bLog_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }
    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;

        //    if (mesh_) {
        //      geometry_msgs::Pose poseTransformed;
        //      tf::poseTFToMsg(transform * poseTF, poseTransformed);
        //      mesh_->setPeerPose(poseTransformed, 0);
        //      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
        //      // Publish the mesh marker for visualization in rviz
        //      visualization_msgs::Marker inspected;
        //      inspected.ns = "meshInspected";
        //      inspected.id = 0;
        //      inspected.header.seq = inspected.id;
        //      inspected.header.stamp = pose.header.stamp;
        //      inspected.header.frame_id = params_.navigationFrame_;
        //      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
        //      inspected.lifetime = ros::Duration(10);
        //      inspected.action = visualization_msgs::Marker::ADD;
        //      inspected.pose.position.x = 0.0;
        //      inspected.pose.position.y = 0.0;
        //      inspected.pose.position.z = 0.0;
        //      inspected.pose.orientation.x = 0.0;
        //      inspected.pose.orientation.y = 0.0;
        //      inspected.pose.orientation.z = 0.0;
        //      inspected.pose.orientation.w = 1.0;
        //      inspected.scale.x = 1.0;
        //      inspected.scale.y = 1.0;
        //      inspected.scale.z = 1.0;
        //      visualization_msgs::Marker uninspected = inspected;
        //      uninspected.header.seq++;
        //      uninspected.id++;
        //      uninspected.ns = "meshUninspected";
        //      mesh_->assembleMarkerArray(inspected, uninspected);
        //      if (inspected.points.size() > 0) {
        //        params_.inspectionPath_.publish(inspected);
        //      }
        //      if (uninspected.points.size() > 0) {
        //        params_.inspectionPath_.publish(uninspected);
        //      }
        //    }
    }
}

void RrtTree::setStateFromOdometryMsg(const nav_msgs::Odometry& pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);

    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.bLog_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }
    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;

        //    if (mesh_) {
        //      geometry_msgs::Pose poseTransformed;
        //      tf::poseTFToMsg(transform * poseTF, poseTransformed);
        //      mesh_->setPeerPose(poseTransformed, 0);
        //      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
        //      // Publish the mesh marker for visualization in rviz
        //      visualization_msgs::Marker inspected;
        //      inspected.ns = "meshInspected";
        //      inspected.id = 0;
        //      inspected.header.seq = inspected.id;
        //      inspected.header.stamp = pose.header.stamp;
        //      inspected.header.frame_id = params_.navigationFrame_;
        //      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
        //      inspected.lifetime = ros::Duration(10);
        //      inspected.action = visualization_msgs::Marker::ADD;
        //      inspected.pose.position.x = 0.0;
        //      inspected.pose.position.y = 0.0;
        //      inspected.pose.position.z = 0.0;
        //      inspected.pose.orientation.x = 0.0;
        //      inspected.pose.orientation.y = 0.0;
        //      inspected.pose.orientation.z = 0.0;
        //      inspected.pose.orientation.w = 1.0;
        //      inspected.scale.x = 1.0;
        //      inspected.scale.y = 1.0;
        //      inspected.scale.z = 1.0;
        //      visualization_msgs::Marker uninspected = inspected;
        //      uninspected.header.seq++;
        //      uninspected.id++;
        //      uninspected.ns = "meshUninspected";
        //      mesh_->assembleMarkerArray(inspected, uninspected);
        //      if (inspected.points.size() > 0) {
        //        params_.inspectionPath_.publish(inspected);
        //      }
        //      if (uninspected.points.size() > 0) {
        //        params_.inspectionPath_.publish(uninspected);
        //      }
        //    }
    }
}

void RrtTree::setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose.pose, poseTF);
    geometry_msgs::Pose poseTransformed;
    tf::poseTFToMsg(transform * poseTF, poseTransformed);
    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[n_peer] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[n_peer] += params_.inspection_throttle_;

        //    if (mesh_) {
        //      mesh_->setPeerPose(poseTransformed, n_peer);
        //      mesh_->incorporateViewFromPoseMsg(poseTransformed, n_peer);
        //    }
    }
}

void RrtTree::iterate(int iterations)
{
    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);
    
    // In this function a new configuration is sampled and added to the tree.
    StateVec newState;

    // Sample over a sphere with the radius of the maximum diagonal of the exploration
    // space. Throw away samples outside the sampling region it exiting is not allowed
    // by the corresponding parameter. This method is to not bias the tree towards the
    // center of the exploration space.
    double radius = sqrt(
                         SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
                         + SQ(params_.minZ_ - params_.maxZ_));
    bool solutionFound = false;
    while (!solutionFound)
    {
        for (int i = 0; i < 3; i++)
        {
            newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        }
        if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
            continue;
        // Offset new state by root
        newState += rootNode_->state_;
        if (!params_.softBounds_)
        {
            if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x())
            {
                continue;
            }
            else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y())
            {
                continue;
            }
            else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z())
            {
                continue;
            }
            else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x())
            {
                continue;
            }
            else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y())
            {
                continue;
            }
            else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z())
            {
                continue;
            }
        }
        solutionFound = true;
    }

    // Find nearest neighbour
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0)
    {
        kd_res_free(nearest);
        return;
    }
    Node<StateVec> * newParent = (Node<StateVec> *) kd_res_item_data(nearest);
    kd_res_free(nearest);

    // Check for collision of new connection plus some overshoot distance.
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                              newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_)
    {
        direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];
    if (volumetric_mapping::OctomapManager::CellStatus::kFree
            == p_octomap_manager_->getLineStatusBoundingBox(origin, direction + origin + direction.normalized() * params_.dOvershoot_,
                                                            params_.boundingBox_)
            && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_))
    {
        // Sample the new orientation
        newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        
        // Create new node and insert into tree
        Node<StateVec> * newNode = new Node<StateVec>;
        newNode->state_ = newState;
        newNode->parent_ = newParent;
        newNode->distance_ = newParent->distance_ + direction.norm();
        newParent->children_.push_back(newNode);
        newNode->gain_ = newParent->gain_ + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_);

        kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

        // Display new node
        publishNode(newNode);

        // Update best IG and node if applicable
        if (newNode->gain_ > bestGain_)
        {
            bestGain_ = newNode->gain_;
            bestNode_ = newNode;
        }
        
        if (newNode->local_gain_ > params_.zero_gain_)
        {
            frontierNodes_.push_back(newNode);
        }     
        
        counter_++;
    }
}


void RrtTree::setRoot(double x, double y, double z)
{
    root_[0] = x;
    root_[1] = y;
    root_[2] = z;
    root_[3] = 0;    
    
    //rootNode_->state_ = root_;  // not yet allocated   
    exact_root_ = root_;       
}

bool RrtTree::addNode(double x, double y, double z)
{    
    //std::cout << "RrtTree::addNode() - start " << std::endl; 
    
    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);
        
    //std::cout << "RrtTree::addNode() - trying to add point: (" << x << ", " << y << ", " << z << ")" << std::endl;   
    
    // In this function a new configuration is sampled and added to the tree.
    StateVec newState;
    newState[0] = x;
    newState[1] = y;
    newState[2] = z;

    if( (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) || (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) )
    {
        return false;
    }
    if( (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) || (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) )
    {
        return false;
    }
    if( (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) || (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) )
    {
        return false;
    } 
    
    //const double disc = sqrt(3)*p_octomap_manager_->getResolution();    
    const double& disc = disc_sqrt3_;
    
    // Find nearest neighbor
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0)
    {
        kd_res_free(nearest);
        return false;
    }
    
    Node<StateVec> * newParent = (Node<StateVec> *) kd_res_item_data(nearest);
    kd_res_free(nearest);


    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
    
    // if the new point is too close reject it 
    if (direction.norm() < params_.extensionRange_)
    {
        return false; 
    }
        
//    if (direction.norm() > params_.extensionRange_)
//    {
//        direction = params_.extensionRange_ * direction.normalized();
//    }
//    newState[0] = origin[0] + direction[0];
//    newState[1] = origin[1] + direction[1];
//    newState[2] = origin[2] + direction[2];
//    if (volumetric_mapping::OctomapManager::CellStatus::kFree
//            == p_octomap_manager_->getLineStatusBoundingBox(origin, direction + origin + direction.normalized() * params_.dOvershoot_,
//                                                            params_.boundingBox_)
//            && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_))
#if USE_SOFT_LINE_COLLISION_CHECKING
    if (volumetric_mapping::OctomapManager::CellStatus::kFree
            == p_octomap_manager_->getLineStatusBoundingBox(origin + direction.normalized()*disc, origin + direction, params_.boundingBox_) ) /// < N.B soft geometric visibility check
#else
        
#if USE_SOFT_ENDPOINT_COLLISION_CHECKING        
    volumetric_mapping::OctomapManager::CellStatus node = p_octomap_manager_->getCellProbabilityPoint(origin + direction, NULL);
    if (node != volumetric_mapping::OctomapManager::CellStatus::kOccupied)        
#endif 
        
#endif 
    {
        // Sample the new orientation
        //newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        newState[3] = atan2(direction[1],direction[0]);
        
        // Create new node and insert into tree
        Node<StateVec>* newNode = new Node<StateVec>;
        newNode->state_ = newState;
        newNode->parent_ = newParent;
        newNode->distance_ = newParent->distance_ + direction.norm();
        newParent->children_.push_back(newNode);
        
        newNode->local_gain_ = gain(newNode->state_);
        
#if USE_LOCAL_GAIN_AS_ACTUAL_GAIN
        newNode->gain_ = newNode->local_gain_ * exp(-params_.degressiveCoeff_ * newNode->distance_);
#else
        //newNode->gain_ = newParent->gain_ + newNode->local_gain_ * exp(-params_.degressiveCoeff_ * newNode->distance_);                 
        newNode->gain_ = deadZone(newParent->gain_,params_.zero_gain_) + deadZone(newNode->local_gain_,params_.zero_gain_) * exp(-params_.degressiveCoeff_ * newNode->distance_);    
#endif        
        
        kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

        // Display new node
        publishNode(newNode);

        // Update best IG and node if applicable
        if (newNode->gain_ > bestGain_)
        {
            bestGain_ = newNode->gain_;
            bestNode_ = newNode;
        }
        
        if (newNode->local_gain_ > params_.zero_gain_)
        {
            frontierNodes_.push_back(newNode);
        }
        
        counter_++;
        
#if SHOW_DEBUG        
        std::cout << "RrtTree: added new node with gain: "<< newNode->gain_<< " *** " << std::endl; 
        //std::cout << "                           point: (" << x << ", " << y << ", " << z << ")" << std::endl;          
#endif 
        
        }
#if USE_SOFT_LINE_COLLISION_CHECKING || USE_SOFT_ENDPOINT_COLLISION_CHECKING   
    else
    {
        return false;
    }
#endif
    
    return true; 
}

void RrtTree::initialize()
{
    std::cout << "RrtTree::initialize() - start " << std::endl; 

    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);
    
    std::cout << "RrtTree::initialize() - mutex locked " << std::endl;    
    
    //const double disc = sqrt(3)*p_octomap_manager_->getResolution();
    disc_sqrt3_ = sqrt(3)*p_octomap_manager_->getResolution();
    const double& disc = disc_sqrt3_;
        
    // This function is to initialize the tree, including insertion of remainder of previous best branch.
    g_ID_ = 0;
    
    // Remove last segment from segment list (multi agent only)
    int i;
    for (i = 0; i < agentNames_.size(); i++)
    {
        if (agentNames_[i].compare(params_.navigationFrame_) == 0)
        {
            break;
        }
    }
    if (i < agentNames_.size())
    {
        segments_[i]->clear();
    }
    
    if(kdTree_) kd_free(kdTree_);
    // Initialize kd-tree with root node and prepare log file
    kdTree_ = kd_create(3);
    //kd_data_destructor(kdTree_, nodeDestructor);    

    if (params_.bLog_)
    {
        if (fileTree_.is_open())
        {
            fileTree_.close();
        }
        fileTree_.open((logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt").c_str(), std::ios::out);
    }
    
    std::cout << "RrtTree::initialize() - log file created" << std::endl;   

    rootNode_ = new Node<StateVec>;
    rootNode_->distance_ = 0.0;
    rootNode_->gain_ = params_.zero_gain_ * 0.0;
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
    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
    iterationCount_++;
    
    
    /// < let's initialize the best node as the root in any case 
    bestNode_ = rootNode_;
    selectedNode_ = rootNode_;

    std::cout << "RrtTree::initialize() - reinserting previous best branch " << std::endl; 
        
    // Insert all nodes of the remainder of the previous best branch, checking for collisions and
    // recomputing the gain.
    for (auto iter = bestBranchMemory_.rbegin(), iterEnd = bestBranchMemory_.rend(); iter != iterEnd; ++iter)
    {
        StateVec newState = *iter;
        kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
        if (kd_res_size(nearest) <= 0)
        {
            kd_res_free(nearest);
            continue;
        }
        Node<StateVec> * newParent = (Node<StateVec> *) kd_res_item_data(nearest);
        kd_res_free(nearest);

        // Check for collision
        Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
        Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
        
        //std::cout << "origin: " << origin << std::endl; 
        //std::cout << "direction: " << direction << std::endl;  
        
        if(direction.norm() < std::numeric_limits<double>::min())
            continue; 
                
//        if (direction.norm() > params_.extensionRange_)
//        {
//            direction = params_.extensionRange_ * direction.normalized();
//        }
//        newState[0] = origin[0] + direction[0];
//        newState[1] = origin[1] + direction[1];
//        newState[2] = origin[2] + direction[2];
        
// < N.B: disabled collision checking         
//        if (volumetric_mapping::OctomapManager::CellStatus::kFree
//                == p_octomap_manager_->getLineStatusBoundingBox(
//                                                                origin, direction + origin + direction.normalized() * params_.dOvershoot_,
//                                                                params_.boundingBox_)
//                && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_,
//                                              segments_))
#if USE_SOFT_LINE_COLLISION_CHECKING         
        if (volumetric_mapping::OctomapManager::CellStatus::kFree
            == p_octomap_manager_->getLineStatusBoundingBox(origin + direction.normalized()*disc, origin + direction, params_.boundingBox_) )    /// < N.B soft geometric visibility check
#else
        
#if USE_SOFT_ENDPOINT_COLLISION_CHECKING        
        volumetric_mapping::OctomapManager::CellStatus node = p_octomap_manager_->getCellProbabilityPoint(origin + direction, NULL);
        if (node != volumetric_mapping::OctomapManager::CellStatus::kOccupied)        
#endif 
        
#endif 
        {
            // Create new node and insert into tree
            Node<StateVec> * newNode = new Node<StateVec>;
            newNode->state_ = newState;
            newNode->parent_ = newParent;
            newNode->distance_ = newParent->distance_ + direction.norm();
            newParent->children_.push_back(newNode);
            
            newNode->local_gain_ = gain(newNode->state_);
#if USE_LOCAL_GAIN_AS_ACTUAL_GAIN
            newNode->gain_ = newNode->local_gain_ * exp(-params_.degressiveCoeff_ * newNode->distance_);
#else
            //newNode->gain_ = newParent->gain_ + newNode->local_gain_ * exp(-params_.degressiveCoeff_ * newNode->distance_);               
            newNode->gain_ = deadZone(newParent->gain_,params_.zero_gain_) + deadZone(newNode->local_gain_,params_.zero_gain_) * exp(-params_.degressiveCoeff_ * newNode->distance_);                       
#endif                
            kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

            // Display new node
            publishNode(newNode);

            // Update best IG and node if applicable
            if (newNode->gain_ > bestGain_)
            {
                bestGain_ = newNode->gain_;
                bestNode_ = newNode;
            }
            
            if (newNode->local_gain_ > params_.zero_gain_)
            {
                frontierNodes_.push_back(newNode);
            }
            
            counter_++;
        }
    }

    // Publish visualization of total exploration area
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = g_ID_;
    p.header.frame_id = params_.navigationFrame_;
    p.id = g_ID_;
    g_ID_++;
    p.ns = "search_tree_bounds";
    p.type = visualization_msgs::Marker::CUBE;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
    p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
    p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, 0.0);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = params_.maxX_ - params_.minX_;
    p.scale.y = params_.maxY_ - params_.minY_;
    p.scale.z = params_.maxZ_ - params_.minZ_;
    p.color.r = 200.0 / 255.0;
    p.color.g = 100.0 / 255.0;
    p.color.b = 0.0;
    p.color.a = 0.1;
    p.lifetime = ros::Duration(0.0);
    p.frame_locked = false;
    
    params_.inspectionPathPub_.publish(p);
    //.markers.push_back(p);
    
    std::cout << "RrtTree::initialize() - end " << std::endl;     
}

// This function returns the first edge of the best branch
std::vector<geometry_msgs::Pose> RrtTree::getBestEdge(std::string targetFrame)
{

    std::vector<geometry_msgs::Pose> ret;
    Node<StateVec> * current = bestNode_;
    selectedNode_ = NULL;
    if (current->parent_ != NULL)
    {
        while (current->parent_ != rootNode_ && current->parent_ != NULL)
        {
            current = current->parent_;
        }
        ret = samplePath(current->parent_->state_, current->state_, targetFrame);
        history_.push(current->parent_->state_);
        exact_root_ = current->state_;
        
        selectedNode_ = current;
        
        publishSelectedNode(current);
    }
    return ret;
}

// This function returns the first "numEdges" edges of the best branch
std::vector<geometry_msgs::Pose> RrtTree::getBestEdges(std::string targetFrame, int numEdges)
{
    numEdges = std::max(1,numEdges); // we want it >=1
    
    std::vector<geometry_msgs::Pose> ret; 
    Node<StateVec> * current = bestNode_;
    selectedNode_ = NULL;
    std::deque<Node<StateVec> * > node_path; 
            
    if (current->parent_ != NULL)
    {    
        if(current->parent_ == rootNode_ ) 
        {
            node_path.push_front(current); // push bestNode_ here since we do not enter in the while loop
        }
        else
        {
            while (current->parent_ != rootNode_ && current->parent_ != NULL)
            {
                node_path.push_front(current);                 
                current = current->parent_;        
            }
            node_path.push_front(current); //push the last one with (current->parent_ == rootNode_ )      
        }        
        node_path.push_front(current->parent_);  // in the end this is the rootNode_
//        if(current->parent_ != rootNode_)
//        {
//            std::cout << "current->parent_ != rootNode_!!" << std::endl;
//            quick_exit(-1);
//        }
        
//        std::cout << "********************************************"<< std::endl; 
//        std::cout << "current node: " <<  current->parent_->state_ << std::endl;         
        int dest_node_idx = std::min((int)node_path.size(),numEdges+1)-1;
        Node<StateVec> * destination_node = node_path[dest_node_idx];
//        std::cout << "node_path.size(): " << node_path.size() << std::endl;
//        for(int i=0; i<node_path.size(); i++)
//        {
//            std::cout << "node_path[" << i << "]: " << node_path[i]->state_ << std::endl; 
//        }
//        std::cout << "dest_node_idx: " << dest_node_idx << std::endl; 
        
//        if(destination_node != current)
//        {
//            std::cout << "destination_node != current !!" << std::endl;
//            std::cout << "current: " << current->state_ << std::endl; 
//            quick_exit(-1);
//        }        
        
        for(int i=0; i<dest_node_idx; i++)
        {
            //std::cout << "adding path " << i << " from: " << node_path[i]->state_ << ", to: " << node_path[i+1]->state_ << std::endl; 
            std::vector<geometry_msgs::Pose> edge_poses;
            edge_poses = samplePath(node_path[i]->state_, node_path[i+1]->state_, targetFrame);
            ret.insert(ret.end(),edge_poses.begin(),edge_poses.end()); // concatenate on the end
        }
                
        //ret = samplePath(current->parent_->state_, current->state_, targetFrame);        
        //history_.push(current->parent_->state_);
        //exact_root_ = current->state_;
        
        history_.push(current->parent_->state_); // where we are (this is the old rootNode_)
        exact_root_ = destination_node->state_;  // where we are going (this will be the new rootNode_)
        
        
        selectedNode_ = destination_node;
                
        publishSelectedNode(destination_node);
    }
    return ret;    
}

// This function returns the farther point within a certain range along the best branch
std::vector<geometry_msgs::Pose> RrtTree::getBestEdgesWithinRange(std::string targetFrame, double range)
{
#if SHOW_DEBUG     
    std::cout << "RrtTree::getBestEdgesWithinRange()" << std::endl; 
#endif     
    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);
        
    range = std::max(params_.extensionRange_ , range); // at least range >= params_.extensionRange_ 
    
    std::vector<geometry_msgs::Pose> ret; 
    Node<StateVec> * current = bestNode_;
    selectedNode_ = NULL;
    
    publishBestIgNode(bestNode_);
    
    std::deque<Node<StateVec> * > node_path; 
            
    if (current->parent_ != NULL)
    {    
        if(current->parent_ == rootNode_ ) 
        {
            node_path.push_front(current); // push bestNode_ here since we do not enter in the while loop
        }
        else
        {
            while (current->parent_ != rootNode_ && current->parent_ != NULL)
            {
                node_path.push_front(current);                 
                current = current->parent_;        
            }
            node_path.push_front(current); //push the last one with (current->parent_ == rootNode_ )                 
        }
        node_path.push_front(current->parent_);  // in the end this is the rootNode_

#if SHOW_DEBUG          
        std::cout << "node_path.size(): " << node_path.size() << std::endl;  
        for(int i=0; i<node_path.size(); i++)
        {
            std::cout << "node_path[" << i << "]: " << node_path[i]->state_ << std::endl; 
        }
#endif               
        
        int dest_node_idx = node_path.size()-1; 
        const Eigen::Vector3d rootNodePoint = rootNode_->state_.segment(0,3);
        
        ///for(int i=node_path.size()-1; i>=0; i--)
        for(int i=node_path.size()-1; i>0; i--)
        {
            const Eigen::Vector3d nodePoint = node_path[i]->state_.segment(0,3);
            const double dist = (rootNodePoint - nodePoint).norm();
            //std::cout << "dist[" << i << "]: " << dist << std::endl;             
            //if( 
            //   (dist <= range) && 
            //   (volumetric_mapping::OctomapManager::CellStatus::kOccupied 
            //     != p_octomap_manager_->getVisibility(rootNodePoint, nodePoint, false)) )  < N.B.: problem with this since the map can give occupied voxel on robot position and the robot does not succeed to find any goal 
            if(dist <= range)
            {
                dest_node_idx = i;
                break; 
            }
        }
        
        if(dest_node_idx == 0)
        {
            ROS_WARN_STREAM("RrtTree::getBestEdgesWithinRange() - selecting root node as destination!!!");
        }
        
        Node<StateVec> * destination_node = node_path[dest_node_idx];

        for(int i=0; i<dest_node_idx; i++)
        {
#if SHOW_DEBUG                
            std::cout << "adding path " << i << " from: " << node_path[i]->state_ << ", to: " << node_path[i+1]->state_ << std::endl; 
#endif             
            std::vector<geometry_msgs::Pose> edge_poses;
            edge_poses = samplePath(node_path[i]->state_, node_path[i+1]->state_, targetFrame);
            ret.insert(ret.end(),edge_poses.begin(),edge_poses.end()); // concatenate on the end
        }
                
        //ret = samplePath(current->parent_->state_, current->state_, targetFrame);        
        //history_.push(current->parent_->state_);
        //exact_root_ = current->state_;
        
        history_.push(current->parent_->state_); // where we are (this is the old rootNode_)
        exact_root_ = destination_node->state_;  // where we are going (this will be the new rootNode_)
        
        selectedNode_ = destination_node;
                
        publishSelectedNode(destination_node);
    }
    else
    {
         ROS_WARN_STREAM("RrtTree::getBestEdgesWithinRange() - best node is root"); 
    }
    return ret;        
}
    

std::vector<geometry_msgs::Pose> RrtTree::getPathBackToPrevious(std::string targetFrame)
{
    std::vector<geometry_msgs::Pose> ret;
    if (history_.empty())
    {
        return ret;
    }
    StateVec destination = history_.top();
    ret = samplePath(root_, destination, targetFrame);
    exact_root_ = destination;
    history_.pop();
    return ret;
}

void RrtTree::memorizeBestBranch()
{
    bestBranchMemory_.clear();
    Node<StateVec> * current = bestNode_;
    while (current->parent_ && current->parent_->parent_)
    {
        bestBranchMemory_.push_back(current->state_);
        current = current->parent_;
    }
}

void RrtTree::resetBestBranch()
{
    bestBranchMemory_.clear();    
}

void RrtTree::clear()
{
    delete rootNode_;  // N.B.: deleting the root node we use the destructor ~Node() which recursively deletes all its children
    rootNode_ = NULL;

    counter_ = 0;
    bestGain_ = params_.zero_gain_;
    bestNode_ = NULL;
    selectedNode_ = NULL;

    if(kdTree_) kd_free(kdTree_);    
    kdTree_ = 0;
    
    frontierNodes_.clear();  
}


void RrtTree::publishNode(Node<StateVec> * node)
{
    // < node part 
#if 0    
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = g_ID_;
    p.header.frame_id = params_.navigationFrame_;
    p.id = g_ID_;
    g_ID_++;
    p.ns = kRvizNamespaceNodes;
    p.type = visualization_msgs::Marker::ARROW;
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
//    p.scale.x = std::max(node->gain_ / 20.0, 0.05);
//    p.scale.y = 0.1;
//    p.scale.z = 0.1;
    p.scale.x = std::max(node->gain_ / kColorGainScale, 0.05);
    p.scale.y = 0.1;
    p.scale.z = 0.1;    
    p.color.r = 167.0 / 255.0;
    p.color.g = 167.0 / 255.0;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;
#else
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = g_ID_;
    p.header.frame_id = params_.navigationFrame_;
    p.id = g_ID_;
    g_ID_++;
    p.ns = kRvizNamespaceNodes;
    p.type = visualization_msgs::Marker::ARROW;
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
    p.scale.x = 0.2;
    p.scale.y = 0.2;
    p.scale.z = 0.2;
    //const float gain = std::min( std::max(node->gain_ / kColorGainScale, 20.0) , 255.0);    
    const float gain = std::min( std::max( (node->gain_ - params_.zero_gain_)*kColorGainScale, 50.0 ) , 255.0);        
    const float gain_color = gain/255.0;
    p.color.r = gain_color;
    p.color.g = gain_color;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;    
#endif
    
    params_.inspectionPathPub_.publish(p); 
    
#if SHOW_TEXT_MARKERS
    
    visualization_msgs::Marker tm;
    tm.header.stamp = p.header.stamp;
    tm.header.seq = g_ID_;
    tm.header.frame_id = params_.navigationFrame_;
    tm.id = g_ID_;
    g_ID_++;
    tm.ns = kRvizNamespaceTexts;
    tm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tm.action = visualization_msgs::Marker::ADD;
    tm.pose.position.x = node->state_[0];
    tm.pose.position.y = node->state_[1];
    tm.pose.position.z = node->state_[2] + kTreeTextZVisualizationOffset;
    tm.scale.z = kTreeTextMessageHeight;
    std::stringstream sss;
    sss.precision(1);
    sss.setf(std::ios::fixed, std::ios::floatfield);
    
//#if !LOCAL_GAIN_EQUAL_TO_ACTUAL_GAIN    
//    sss << node->local_gain_ << "/";
//#endif 
    sss << node->gain_; 
    
    tm.text = sss.str();
    tm.color.r = 1.;
    tm.color.g = 1.;
    tm.color.b = 1.;
    tm.color.a = 1.;
    tm.lifetime = ros::Duration(10.0);
    tm.frame_locked = false;    
    
    params_.inspectionPathPub_.publish(tm);
    
#endif     
        
    //std::cout << "publishing node in: " << params_.navigationFrame_ << ", pose: " << p.pose << ", scale: " << p.scale << std::endl;     
    //std::cout << "publishing node in: " << params_.navigationFrame_ << ", gain_color: " << gain_color << std::endl; 

    if (node->parent_)
    {     
        // < arrow part 
        p.id = g_ID_;
        g_ID_++;
        p.ns = kRvizNamespaceEdges;
        p.type = visualization_msgs::Marker::ARROW;
        p.action = visualization_msgs::Marker::ADD;
        p.pose.position.x = node->parent_->state_[0];
        p.pose.position.y = node->parent_->state_[1];
        p.pose.position.z = node->parent_->state_[2] + kTreeZVisualizationOffset;
        Eigen::Quaternion<float> q;
        Eigen::Vector3f init(1.0, 0.0, 0.0);
        Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                            node->state_[1] - node->parent_->state_[1],
                            node->state_[2] - node->parent_->state_[2]);
        q.setFromTwoVectors(init, dir);
        q.normalize();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();
        p.scale.x = dir.norm();
        p.scale.y = 0.03;
        p.scale.z = 0.03;

        Color color = Colors::GetColor(robot_id_);

        /*p.color.r = 100.0 / 255.0;
        p.color.g = 100.0 / 255.0;
        p.color.b = 0.7;*/

        p.color.r = color.r;
        p.color.g = color.g;
        p.color.b = color.b;    

        p.color.a = 1.0;
        p.lifetime = ros::Duration(10.0);
        p.frame_locked = false;

        params_.inspectionPathPub_.publish(p);      
    }
    
    //std::cout << "publishing parent node in: " << params_.navigationFrame_ << ", pose: " << p.pose << ", scale: " << p.scale << std::endl;     

    if (params_.bLog_)
    {
        for (int i = 0; i < node->state_.size(); i++)
        {
            fileTree_ << node->state_[i] << ",";
        }
        fileTree_ << node->gain_ << ",";
        for (int i = 0; i < node->parent_->state_.size(); i++)
        {
            fileTree_ << node->parent_->state_[i] << ",";
        }
        fileTree_ << node->parent_->gain_ << "\n";
    }
}


void RrtTree::publishBestIgNode(Node<StateVec> * node)
{
    if(node == NULL)
    {
        ROS_ERROR_STREAM("RrtTree::publishBestIgNode() - ERROR - node is null");
        return;
    }
    
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = g_ID_;
    p.header.frame_id = params_.navigationFrame_;
    p.id = g_ID_;
    g_ID_++;
    p.ns = kRvizNamespaceNodes;
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
    p.color.r = 1.;
    p.color.g = 1.;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = ros::Duration(10.0);
    p.frame_locked = false;    

    params_.inspectionPathPub_.publish(p);
}

void RrtTree::publishSelectedNode(Node<StateVec> * node)
{
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = g_ID_;
    p.header.frame_id = params_.navigationFrame_;
    p.id = g_ID_;
    g_ID_++;
    p.ns = kRvizNamespaceNodes;
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

    params_.inspectionPathPub_.publish(p);
}


std::vector<geometry_msgs::Pose> RrtTree::samplePath(StateVec start, StateVec end, std::string targetFrame)
{
    std::vector<geometry_msgs::Pose> ret;
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(targetFrame, params_.navigationFrame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return ret;
    }
    Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
    double yaw_direction = end[3] - start[3];
    if (yaw_direction > M_PI)
    {
        yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI)
    {
        yaw_direction += 2.0 * M_PI;
    }
    double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                           params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
    assert(disc > 0.0);
    for (double it = 0.0; it <= 1.0; it += disc)
    {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0], (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = start[3] + yaw_direction * it;
        if (yaw > M_PI)
            yaw -= 2.0 * M_PI;
        if (yaw < -M_PI)
            yaw += 2.0 * M_PI;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        origin = transform * origin;
        quat = transform * quat;
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        ret.push_back(pose);
        if (params_.bLog_)
        {
            filePath_ << poseTF.getOrigin().x() << ",";
            filePath_ << poseTF.getOrigin().y() << ",";
            filePath_ << poseTF.getOrigin().z() << ",";
            filePath_ << tf::getYaw(poseTF.getRotation()) << "\n";
        }
    }
    return ret;
}

}

#endif
