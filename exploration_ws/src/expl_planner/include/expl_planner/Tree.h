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
* 
* Modified by Luigi Freda since 2017
*/

#ifndef TREE_H_
#define TREE_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <octomap_world/octomap_manager.h>

#include <multiagent_collision_check/Segment.h>

#include "ExplorationParams.h"


namespace explplanner {
    

template<typename StateVec>
class Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Node();
     
    // N.B.: recursive destruction of children!
    ~Node();

public:
    StateVec state_;
    Node* parent_;
    std::vector<Node*> children_;

    size_t id_; 

    double gain_; // total gain along the branch 
    double local_gain_; // local gain 
    double distance_; // distance from root 

    //double neighbors_radius_; // max distance to neighbor 

    int label_;
    bool is_frontier_; 

    void* data_; 
};

template<typename NodeTypePtr>
class CompareNodeByDistance
{
public:
    bool operator()(NodeTypePtr lhs, NodeTypePtr rhs) const
    {
        return lhs->distance_ < rhs->distance_;
    }
};

template<typename StateVec>
class TreeBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    
protected:
     
  ExplParams params_;
  int counter_;
  double bestGain_;
  Node<StateVec> * bestNode_;
  Node<StateVec> * rootNode_;
  Node<StateVec> * selectedNode_;
  //mesh::StlMesh * mesh_;
  std::shared_ptr<volumetric_mapping::OctomapManager> p_octomap_manager_; 
  StateVec root_;
  StateVec exact_root_;
  std::vector<std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >*> segments_;
  std::vector<std::string> agentNames_;
  int robot_id_; 
      
public:
     
  TreeBase();
  //TreeBase(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  TreeBase(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager);
  ~TreeBase();
  
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose) {}
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose) {};
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer) {}
  
  void setPeerStateFromPoseMsg1(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg2(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg3(const geometry_msgs::PoseWithCovarianceStamped& pose);
  
  void evade(const multiagent_collision_check::Segment& segmentMsg);
  virtual void iterate(int iterations) {}
  
  virtual void setRoot(double x, double y, double z) {}
  virtual bool addNode(double x, double y, double z) { return false; }
  
  virtual bool addNode(Node<StateVec>* node){ return false; } 
  virtual void removeCurrentNode(){}
  virtual void backTrack() {}  
  
  virtual void initialize() = 0;
  
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame) { return std::vector<geometry_msgs::Pose>(); }
  virtual std::vector<geometry_msgs::Pose> getBestEdges(std::string targetFrame, int numEdges) { return std::vector<geometry_msgs::Pose>(); }
  virtual std::vector<geometry_msgs::Pose> getBestEdgesWithinRange(std::string targetFrame, double range) { return std::vector<geometry_msgs::Pose>(); }
  
  virtual void clear() = 0;
  
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame) { return std::vector<geometry_msgs::Pose>(); }
  virtual void memorizeBestBranch() {}
  virtual void resetBestBranch() {}  
  
  void setParams(const ExplParams& params);
  int getCounter();
  bool gainFound();
  double getBestGain();
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  
  Node<StateVec>* getRootNode();
  Node<StateVec>* getBestNode();
  Node<StateVec>* getSelectedNode();
  
  void setRobotId(int id){ robot_id_ = id; }   
  
  double gain(StateVec& state);  
  
};


}

#endif
