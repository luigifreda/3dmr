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

#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <deque>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>

#include "Tree.h"


#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace explplanner
{

class RrtTree : public TreeBase<Eigen::Vector4d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const double kColorGainScale; 
    static const double kTreeZVisualizationOffset;
    static const double kTreeTextZVisualizationOffset;    
    static const double kTreeTextMessageHeight;   
    
    static const double kZVisibilityOffset; 
    
    static const std::string kRvizNamespaceNodes; 
    static const std::string kRvizNamespaceTexts;   
    static const std::string kRvizNamespaceEdges;       
        
public:
    typedef Eigen::Vector4d StateVec;
  
    RrtTree();
    //RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
    RrtTree(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager);
    ~RrtTree();
    virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
    virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
    virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer);
    virtual void initialize();
    virtual void iterate(int iterations);
    
    virtual void setRoot(double x, double y, double z);
    virtual bool addNode(double x, double y, double z);
    
    virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);
    virtual std::vector<geometry_msgs::Pose> getBestEdges(std::string targetFrame, int numEdges);
    virtual std::vector<geometry_msgs::Pose> getBestEdgesWithinRange(std::string targetFrame, double range);    
  
    virtual void clear();
    virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);
    
    virtual void memorizeBestBranch();
    virtual void resetBestBranch();
    
    void publishNode(Node<StateVec> * node);
    void publishBestIgNode(Node<StateVec> * node);        
    void publishSelectedNode(Node<StateVec> * node);  
    
    std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end, std::string targetFrame);
    
    std::vector<Node<StateVec>* >& getFrontierNodes() { return frontierNodes_; }
    
protected:
    kdtree * kdTree_;
    std::stack<StateVec, std::deque<StateVec, Eigen::aligned_allocator<StateVec> > > history_; 
    std::vector<StateVec, Eigen::aligned_allocator<StateVec> > bestBranchMemory_;
    int g_ID_; // graphics ID for visual markers 
    int iterationCount_;
    
    std::fstream fileTree_;
    std::fstream filePath_;
    std::fstream fileResponse_;
    std::string logFilePath_;
    
    std::vector<double> inspectionThrottleTime_;
    
    double disc_sqrt3_;
    
    std::vector<Node<StateVec>* > frontierNodes_;
};

}

#endif
