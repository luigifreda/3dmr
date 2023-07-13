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
*/

#ifndef TREE_HPP_
#define TREE_HPP_

#include "Tree.h"
#include "GainAngleHistogram.h"


template<typename StateVec>
explplanner::Node<StateVec>::Node()
{
  parent_ = NULL;
  distance_ = DBL_MAX;
  gain_ = 0.0;
  local_gain_ = 0.0; 
  
  //neighbors_radius_ = 0.;
  
  label_ = -1; // invalid 
  is_frontier_ = false;
  id_ = 0;
  
  data_ = NULL;
}

template<typename StateVec>
explplanner::Node<StateVec>::~Node()
{
  for (typename std::vector<Node<StateVec> *>::iterator it = children_.begin(); it != children_.end(); it++) {
    delete (*it);
    (*it) = NULL;
  }
}



template<typename StateVec>
explplanner::TreeBase<StateVec>::TreeBase()
{
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
  selectedNode_ = NULL;
}

template<typename StateVec>
explplanner::TreeBase<StateVec>::TreeBase(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager)
{
  //mesh_ = mesh;
  p_octomap_manager_ = p_manager;
  
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
  selectedNode_ = NULL;
}

template<typename StateVec>
explplanner::TreeBase<StateVec>::~TreeBase()
{
}

template<typename StateVec>
void explplanner::TreeBase<StateVec>::setPeerStateFromPoseMsg1(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  setPeerStateFromPoseMsg(pose, 1);
}

template<typename StateVec>
void explplanner::TreeBase<StateVec>::setPeerStateFromPoseMsg2(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  setPeerStateFromPoseMsg(pose, 2);
}

template<typename StateVec>
void explplanner::TreeBase<StateVec>::setPeerStateFromPoseMsg3(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  setPeerStateFromPoseMsg(pose, 3);
}

template<typename StateVec>
void explplanner::TreeBase<StateVec>::setParams(const ExplParams& params)
{
  params_ = params; 
}

template<typename StateVec>
int explplanner::TreeBase<StateVec>::getCounter()
{
  return counter_;
}

template<typename StateVec>
bool explplanner::TreeBase<StateVec>::gainFound()
{
  return bestGain_ > params_.zero_gain_;
}

template<typename StateVec>
double explplanner::TreeBase<StateVec>::getBestGain()
{
  return bestGain_;
}

template<typename StateVec>
void explplanner::TreeBase<StateVec>::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);    
  p_octomap_manager_->insertPointcloudWithTf(pointcloud);
}

template<typename StateVec>
void explplanner::TreeBase<StateVec>::evade(const multiagent_collision_check::Segment& segmentMsg) {
  int i;
  for(i = 0; i < agentNames_.size(); i++) {
    if(agentNames_[i].compare(segmentMsg.header.frame_id) == 0) {
      break;
    }
  }
  if (i == agentNames_.size()) {
    agentNames_.push_back(segmentMsg.header.frame_id);
    segments_.push_back(new std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >);
  }
  segments_[i]->clear();
  for(typename std::vector<geometry_msgs::Pose>::const_iterator it = segmentMsg.poses.begin(); it != segmentMsg.poses.end(); it++) {
    segments_[i]->push_back(Eigen::Vector3d(it->position.x, it->position.y, it->position.z));
  }
}

template<typename StateVec>
explplanner::Node<StateVec>* explplanner::TreeBase<StateVec>::getRootNode()
{
    return rootNode_;
}

template<typename StateVec>
explplanner::Node<StateVec>* explplanner::TreeBase<StateVec>::getBestNode()
{
    return bestNode_;
}

template<typename StateVec>
explplanner::Node<StateVec>* explplanner::TreeBase<StateVec>::getSelectedNode()
{
    return selectedNode_;
}


template<typename StateVec>
double explplanner::TreeBase<StateVec>::gain(StateVec& state)
{
    //std::cout << "RrtTree::gain()" << std::endl;    
    boost::recursive_mutex::scoped_lock locker(p_octomap_manager_->interaction_mutex);
        
    // This function computes the gain
    double gain = 0.0;
    const double disc = p_octomap_manager_->getResolution();
    const double disc2 = disc * disc; 
    const double disc3 = disc * disc2; 
    
    const Eigen::Vector3d origin(state[0], state[1], state[2] + disc); // we increase the z in order to avoid possible "false-occupied" ground cell (effective for coarse resolutions)
    Eigen::Vector3d vec;
    
    const double range = params_.gainRange_ - disc; // we remove disc (the resolution in order to avoid counting possible unreachable cells)   
    const double range2 = range*range; 

    static const double occupied_occupancy_scale = params_.clampingThresMax_ - params_.occupancyThres_;
    static const double free_occupancy_scale = params_.occupancyThres_ - params_.clampingThresMin_;

    // TODO: here we can take into account the current robot pitch 
    static const double minAngleWrtVert = (M_PI/180.) * (90. - 0.5*params_.cameraVerticalFov_);
    static const double maxCosVert = cos(minAngleWrtVert);
    
    // we use gainAngleHistogram in order to compute the best orientation with a 360 degree FOV 
    GainAngleHistogram gainAngleHistogram; 

    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
            vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
                vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                    vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                double dirNorm2 = dir.squaredNorm();
                
                // Skip if distance is too large
                //if (dir.transpose().dot(dir) > range2)
                if (dirNorm2 > range2)
                {
                    continue;
                }

                // check divergence with respect to vertical axis 
                const double cos = dir.dot(Eigen::Vector3d::UnitZ());
                if (cos > maxCosVert)
                {
                    continue; 
                }

                /// < N.B: at the present time, we consider a virtual omni-directional sensor, no FOV geometrical constraints;
                /// <      we compute the best sensor orientation by using an object of GainAngleHistogram

                //                bool insideAFieldOfView = false;
                //                // Check that voxel center is inside one of the fields of view.
                //                for (typename std::vector<std::vector < Eigen::Vector3d>>::iterator itCBN = params_
                //                        .camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++)
                //                {
                //                    bool inThisFieldOfView = true;
                //                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
                //                            itSingleCBN != itCBN->end(); itSingleCBN++)
                //                    {
                //                        Eigen::Vector3d normal = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ())
                //                                * (*itSingleCBN);
                //                        double val = dir.dot(normal.normalized());
                //                        if (val < SQRT2 * disc)
                //                        {
                //                            inThisFieldOfView = false;
                //                            break;
                //                        }
                //                    }
                //                    if (inThisFieldOfView)
                //                    {
                //                        insideAFieldOfView = true;
                //                        break;
                //                    }
                //                }
                //                if (!insideAFieldOfView)
                //                {
                //                    continue;
                //                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                double cellGain = 0; 
                volumetric_mapping::OctomapManager::CellStatus node = p_octomap_manager_->getCellProbabilityPoint(vec, &probability);
                if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown)
                {
                    // Rayshooting to evaluate inspectability of cell
                    if (volumetric_mapping::OctomapManager::CellStatus::kOccupied != this->p_octomap_manager_->getVisibility(origin, vec, false))
                    {
                        //gain += params_.igUnmapped_;
                        cellGain = params_.igUnmapped_;
                        // TODO: Add probabilistic gain
                        // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
                    }
                }
                else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied)
                {
                    // Rayshooting to evaluate inspectability of cell
                    if (volumetric_mapping::OctomapManager::CellStatus::kOccupied != this->p_octomap_manager_->getVisibility(origin, vec, false))
                    {
                        //gain += params_.igOccupied_;

                        // probabilistic gain
                        // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
                        //if (probability > params_.clamping_thres_min)
                        if (probability > params_.occupancyThres_ )
                        {
                            //gain += params_.igOccupied_ * (params_.igProbabilistic_ / occupied_occupancy_scale) * (params_.clamping_thres_max - probability);
                            cellGain = params_.igOccupied_ * (params_.igProbabilistic_ / occupied_occupancy_scale) * (params_.clampingThresMax_ - probability); 
                        }

                    }
                }
                else
                {
                    // Rayshooting to evaluate inspectability of cell
                    if (volumetric_mapping::OctomapManager::CellStatus::kOccupied != this->p_octomap_manager_->getVisibility(origin, vec, false))
                    {
                        //gain += params_.igFree_;

                        // probabilistic gain
                        // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
                        // < N.B. probability{cell is free} = 1.0 - probability
                        //gain += params_.igFree_ * (params_.igProbabilistic_ / free_occupancy_scale) * (1.0 - probability - params_.clamping_thres_min );
                        if (probability > params_.clampingThresMin_)
                        {                        
                            //gain += params_.igFree_ * (params_.igProbabilistic_ / free_occupancy_scale) * (probability - params_.clamping_thres_min );
                            cellGain = params_.igFree_ * (params_.igProbabilistic_ / free_occupancy_scale) * (probability - params_.clampingThresMin_ );
                        }
                    }
                }
                
                // if the dirNorm is too small then the cell contributes to all the angles
                if(dirNorm2 > disc2)  
                {
                    gainAngleHistogram.add(atan2(dir[1],dir[0]),cellGain);
                }
                gain += cellGain; 
            }
        }
    }
    
    state[3]= gainAngleHistogram.getBestOrientation();
    
    // Scale with volume
    //gain *= pow(disc, 3.0);
    gain *= disc3;

    // Check the gain added by inspectable surface
    //  if (mesh_) {
    //    tf::Transform transform;
    //    transform.setOrigin(tf::Vector3(state.x(), state.y(), state.z()));
    //    tf::Quaternion quaternion;
    //    quaternion.setEuler(0.0, 0.0, state[3]);
    //    transform.setRotation(quaternion);
    //    gain += params_.igArea_ * mesh_->computeInspectableArea(transform);
    //  }

    return gain;
}

#endif
