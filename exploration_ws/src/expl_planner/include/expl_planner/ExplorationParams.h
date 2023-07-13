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

#ifndef EXPLORATION_PARAMS_H_
#define EXPLORATION_PARAMS_H_

#include <ros/ros.h>

#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Dense>

namespace explplanner {

    
struct ExplParams
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  std::vector<double> camPitch_;
  std::vector<double> camHorizontal_;
  std::vector<double> camVertical_;
  std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > camBoundNormals_;
  double cameraVerticalFov_; // [degs] 
  double robotSensorHeight;  // [m] 

  double igProbabilistic_;
  double igFree_;
  double igOccupied_;
  double igUnmapped_;
  double igArea_;
  double gainRange_;
  double degressiveCoeff_;
  double zero_gain_;

  double v_max_;
  double dyaw_max_;
  double dOvershoot_;
  double extensionRange_;
  int numEdgeSteps_; 
  double explStep_;   
  double explStepBacktracking_;   
  bool exact_root_;
  int initIterations_;
  int cuttoffIterations_;
  double dt_;

  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  bool softBounds_;
  Eigen::Vector3d boundingBox_;

  double meshResolution_;

  ros::Publisher inspectionPathPub_;
  ros::Publisher explorationPathPub_;
  ros::Publisher explorationNodesPub_;  
  ros::Publisher explorationFrontierTreePub_;
  ros::Publisher explorationFrontierClustersPub_;  
  ros::Publisher explorationNavTreePub_;    
  
  std::string navigationFrame_;

  bool bLog_;
  double log_throttle_;
  double pcl_throttle_;
  double inspection_throttle_;
  
  double probHit_; 
  double probMiss_;
  double clampingThresMin_;
  double clampingThresMax_;
  double occupancyThres_;  
  
  bool bDownSampleTraversability_;
  double downsampleTravResolution_; 
  
  double conflictDistance_; 
  
  double frontierClusteringRadius_;
};

}

#endif
