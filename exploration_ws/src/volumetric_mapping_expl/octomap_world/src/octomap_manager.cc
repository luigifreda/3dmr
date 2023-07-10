/*
Copyright (c) 2015, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "octomap_world/octomap_manager.h"

#include <glog/logging.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_msg.h>

namespace volumetric_mapping
{

  OctomapManager::OctomapManager(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        world_frame_("world"),
        Q_initialized_(false),
        Q_(Eigen::Matrix4d::Identity()),
        full_image_size_(256, 128),
        map_publish_frequency_(0.0)
  {

    tf_cache_length_ = 30.;
    nh_private_.param("tf_cache_length", tf_cache_length_, tf_cache_length_);
    p_tf_listener_ = boost::make_shared<tf::TransformListener>(ros::Duration(tf_cache_length_));

    time0_ = ros::Time::now();

    setParametersFromROS();
    subscribe();
    advertiseServices();
    advertisePublishers();

    // After creating the manager, if the octomap_file parameter is set,
    // load the octomap at that path and publish it.
    std::string octomap_file;
    if (nh_private_.getParam("octomap_file", octomap_file))
    {
      if (loadOctomapFromFile(octomap_file))
      {
        ROS_INFO_STREAM(
            "Successfully loaded octomap from path: " << octomap_file);
        publishAll();
      }
      else
      {
        ROS_ERROR_STREAM("Could not load octomap from path: " << octomap_file);
      }
    }
  }

  void OctomapManager::setParametersFromROS()
  {
    OctomapParameters params;
    nh_private_.param("tf_frame", world_frame_, world_frame_);

    std::cout << "OctomapManager::setParametersFromROS(): world frame: " << world_frame_ << std::endl;

    nh_private_.param("resolution", params.resolution, params.resolution);
    nh_private_.param("probability_hit", params.probability_hit,
                      params.probability_hit);
    nh_private_.param("probability_miss", params.probability_miss,
                      params.probability_miss);
    nh_private_.param("threshold_min", params.threshold_min,
                      params.threshold_min);
    nh_private_.param("threshold_max", params.threshold_max,
                      params.threshold_max);
    nh_private_.param("threshold_occupancy", params.threshold_occupancy,
                      params.threshold_occupancy);
    nh_private_.param("filter_speckles", params.filter_speckles,
                      params.filter_speckles);
    nh_private_.param("sensor_max_range", params.sensor_max_range,
                      params.sensor_max_range);
    nh_private_.param("visualize_min_z", params.visualize_min_z,
                      params.visualize_min_z);
    nh_private_.param("visualize_max_z", params.visualize_max_z,
                      params.visualize_max_z);
    nh_private_.param("full_image_width", full_image_size_.x(),
                      full_image_size_.x());
    nh_private_.param("full_image_height", full_image_size_.y(),
                      full_image_size_.y());
    nh_private_.param("map_publish_frequency", map_publish_frequency_,
                      map_publish_frequency_);
    nh_private_.param("treat_unknown_as_occupied",
                      params.treat_unknown_as_occupied,
                      params.treat_unknown_as_occupied);
    nh_private_.param("change_detection_enabled", params.change_detection_enabled,
                      params.change_detection_enabled);

    // Try to initialize Q matrix from parameters, if available.
    std::vector<double> Q_vec;
    if (nh_private_.getParam("Q", Q_vec))
    {
      Q_initialized_ = setQFromParams(&Q_vec);
    }

    // Set the parent class parameters.
    setOctomapParameters(params);
  }

  bool OctomapManager::setQFromParams(std::vector<double> *Q_vec)
  {
    if (Q_vec->size() != 16)
    {
      ROS_ERROR_STREAM("Invalid Q matrix size, expected size: 16, actual size: "
                       << Q_vec->size());
      return false;
    }

    // Try to map the vector as coefficients.
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> Q_vec_map(
        Q_vec->data());
    // Copy over to the Q member.
    Q_ = Q_vec_map;

    return true;
  }

  void OctomapManager::subscribe()
  {
    left_info_sub_ = nh_.subscribe("cam0/camera_info", 1,
                                   &OctomapManager::leftCameraInfoCallback, this);
    right_info_sub_ = nh_.subscribe(
        "cam1/camera_info", 1, &OctomapManager::rightCameraInfoCallback, this);
    disparity_sub_ = nh_.subscribe(
        "disparity", 40, &OctomapManager::insertDisparityImageWithTf, this);
    pointcloud_sub_ = nh_.subscribe(
        "pointcloud", 40, &OctomapManager::insertPointcloudWithTf, this);
  }

  void OctomapManager::advertiseServices()
  {
    reset_map_service_ = nh_private_.advertiseService(
        "reset_map", &OctomapManager::resetMapCallback, this);
    publish_all_service_ = nh_private_.advertiseService(
        "publish_all", &OctomapManager::publishAllCallback, this);
    get_map_service_ = nh_private_.advertiseService(
        "get_map", &OctomapManager::getOctomapCallback, this);
    save_octree_service_ = nh_private_.advertiseService(
        "save_map", &OctomapManager::saveOctomapCallback, this);
    load_octree_service_ = nh_private_.advertiseService(
        "load_map", &OctomapManager::loadOctomapCallback, this);
    set_box_occupancy_service_ = nh_private_.advertiseService(
        "set_box_occupancy", &OctomapManager::setBoxOccupancyCallback, this);
    set_display_bounds_service_ = nh_private_.advertiseService(
        "set_display_bounds", &OctomapManager::setDisplayBoundsCallback, this);
  }

  void OctomapManager::advertisePublishers()
  {
    occupied_nodes_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "octomap_occupied", 1, true);
    free_nodes_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "octomap_free", 1, true);

    binary_map_pub_ =
        nh_private_.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);
    full_map_pub_ =
        nh_private_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);

    if (map_publish_frequency_ > 0.0)
      map_publish_timer_ =
          nh_private_.createTimer(ros::Duration(1.0 / map_publish_frequency_),
                                  &OctomapManager::publishAllEvent, this);
  }

  void OctomapManager::publishAll()
  {
    visualization_msgs::MarkerArray occupied_nodes, free_nodes;
    if (occupied_nodes_pub_.getNumSubscribers() > 0 || free_nodes_pub_.getNumSubscribers() > 0)
    {
      generateMarkerArray(world_frame_, &occupied_nodes, &free_nodes);
    }

    if (occupied_nodes_pub_.getNumSubscribers() > 0)
    {
      occupied_nodes_pub_.publish(occupied_nodes);
    }

    if (free_nodes_pub_.getNumSubscribers() > 0)
    {
      free_nodes_pub_.publish(free_nodes);
    }

    octomap_msgs::Octomap binary_map, full_map;
    if (binary_map_pub_.getNumSubscribers() > 0 || full_map_pub_.getNumSubscribers() > 0)
    {
      getOctomapBinaryMsg(&binary_map);
      getOctomapFullMsg(&full_map);
    }

    binary_map.header.frame_id = world_frame_;
    full_map.header.frame_id = world_frame_;

    if (binary_map_pub_.getNumSubscribers() > 0)
    {
      binary_map_pub_.publish(binary_map);
    }
    if (full_map_pub_.getNumSubscribers() > 0)
    {
      full_map_pub_.publish(full_map);
    }
  }

  void OctomapManager::publishAllEvent(const ros::TimerEvent &e) { publishAll(); }

  bool OctomapManager::resetMapCallback(std_srvs::Empty::Request &request,
                                        std_srvs::Empty::Response &response)
  {
    resetMap();
    return true;
  }

  bool OctomapManager::publishAllCallback(std_srvs::Empty::Request &request,
                                          std_srvs::Empty::Response &response)
  {
    publishAll();
    return true;
  }

  bool OctomapManager::getOctomapCallback(
      octomap_msgs::GetOctomap::Request &request,
      octomap_msgs::GetOctomap::Response &response)
  {
    return getOctomapFullMsg(&response.map);
  }

  bool OctomapManager::loadOctomapCallback(
      volumetric_msgs_expl::LoadMap::Request &request,
      volumetric_msgs_expl::LoadMap::Response &response)
  {
    return loadOctomapFromFile(request.file_path);
  }

  bool OctomapManager::saveOctomapCallback(
      volumetric_msgs_expl::SaveMap::Request &request,
      volumetric_msgs_expl::SaveMap::Response &response)
  {
    return writeOctomapToFile(request.file_path);
  }

  bool OctomapManager::setBoxOccupancyCallback(
      volumetric_msgs_expl::SetBoxOccupancy::Request &request,
      volumetric_msgs_expl::SetBoxOccupancy::Response &response)
  {
    Eigen::Vector3d bounding_box_center;
    Eigen::Vector3d bounding_box_size;

    tf::vectorMsgToKindr(request.box_center, &bounding_box_center);
    tf::vectorMsgToKindr(request.box_size, &bounding_box_size);
    bool set_occupied = request.set_occupied;

    if (set_occupied)
    {
      setOccupied(bounding_box_center, bounding_box_size);
    }
    else
    {
      setFree(bounding_box_center, bounding_box_size);
    }
    publishAll();
    return true;
  }

  bool OctomapManager::setDisplayBoundsCallback(
      volumetric_msgs_expl::SetDisplayBounds::Request &request,
      volumetric_msgs_expl::SetDisplayBounds::Response &response)
  {
    params_.visualize_min_z = request.min_z;
    params_.visualize_max_z = request.max_z;
    publishAll();
    return true;
  }

  void OctomapManager::leftCameraInfoCallback(
      const sensor_msgs::CameraInfoPtr &left_info)
  {
    left_info_ = left_info;
    if (left_info_ && right_info_ && !Q_initialized_)
    {
      calculateQ();
    }
  }
  void OctomapManager::rightCameraInfoCallback(
      const sensor_msgs::CameraInfoPtr &right_info)
  {
    right_info_ = right_info;
    if (left_info_ && right_info_ && !Q_initialized_)
    {
      calculateQ();
    }
  }

  void OctomapManager::calculateQ()
  {
    Q_ = getQForROSCameras(*left_info_, *right_info_);
    full_image_size_.x() = left_info_->width;
    full_image_size_.y() = left_info_->height;
    Q_initialized_ = true;
  }

  void OctomapManager::insertDisparityImageWithTf(
      const stereo_msgs::DisparityImageConstPtr &disparity)
  {
    if (!Q_initialized_)
    {
      ROS_WARN_THROTTLE(
          1, "No camera info available yet, skipping adding disparity.");
      return;
    }

    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    // Look up transform from sensor frame to world frame.
    Transformation sensor_to_world;
    if (lookupTransform(disparity->header.frame_id, world_frame_,
                        disparity->header.stamp, &sensor_to_world))
    {
      insertDisparityImage(sensor_to_world, disparity, Q_, full_image_size_);
    }
  }

  void OctomapManager::insertPointcloudWithTf(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud)
  {
    boost::recursive_mutex::scoped_lock locker(interaction_mutex);
    ROS_INFO_STREAM("OctomapManager::insertPointcloudWithTf() - Got new pointcloud from " << pointcloud->header.frame_id << " - time " << (ros::Time::now() - time0_).toSec());
    // Look up transform from sensor frame to world frame.
    Transformation sensor_to_world;
    if (lookupTransform(pointcloud->header.frame_id, world_frame_,
                        pointcloud->header.stamp, &sensor_to_world))
    {
      insertPointcloud(sensor_to_world, pointcloud);
    }

    /*
  visualization_msgs::MarkerArray occupied, free;

  //generateMarkerArray(pointcloud->header.frame_id, &occupied, &free);
  generateMarkerArray(GetWorldFrame(), &occupied, &free);
  double occ_f = occupied.markers[getTreeDepth()].points.size();
  double free_f = free.markers[getTreeDepth()].points.size();

  // write values in a correspondent file =========================================================
  std::string path = "3dmr_devel/mapped/" + pointcloud->header.frame_id;        
  system(("mkdir -p ~/.ros/" + path).c_str());
  std::string filename = path + "/points_map.txt";
  std::ofstream file;
  file.open(filename, std::ios_base::app);
	file << pointcloud->header.stamp << "\t" << occ_f << "\t" << free_f << std::endl;     
	file.close();*/
  }

  bool OctomapManager::lookupTransform(const std::string &from_frame,
                                       const std::string &to_frame,
                                       const ros::Time &timestamp,
                                       Transformation *transform)
  {
    tf::StampedTransform tf_transform;

    ros::Time time_to_lookup = timestamp;

    // If this transform isn't possible at the time, then try to just look up
    // the latest (this is to work with bag files and static transform publisher,
    // etc).
    if (!p_tf_listener_->canTransform(to_frame, from_frame, time_to_lookup))
    {
#if 0          
      time_to_lookup = ros::Time(0);
      ROS_WARN("Using latest TF transform instead of timestamp match.");
#else
      // N.B.: the above mechanism cannot be used during an exploration process
      ROS_ERROR_STREAM("Cannot find TF transform " << from_frame << " - " << to_frame << " with timestamp match.");
      return false;
#endif
    }

    try
    {
      p_tf_listener_->lookupTransform(to_frame, from_frame, time_to_lookup,
                                      tf_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR_STREAM(
          "Error getting TF transform from sensor data: " << ex.what());
      return false;
    }

    tf::transformTFToKindr(tf_transform, transform);
    return true;
  }

} // namespace volumetric_mapping
