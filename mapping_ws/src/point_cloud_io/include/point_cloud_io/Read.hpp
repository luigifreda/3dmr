/*
 * Read.hpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace point_cloud_io {

class Read {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit Read(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Read() = default;

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initializes node.
   */
  void initialize();

  /*!
   * Read the point cloud from a .ply or .vtk file.
   * @param filePath the path to the .ply or .vtk file.
   * @param pointCloudFrameId the id of the frame of the point cloud data.
   * @return true if successful.
   */
  bool readFile(const std::string& filePath, const std::string& pointCloudFrameId);

  /*!
   * Timer callback function.
   * @param timerEvent the timer event.
   */
  void timerCallback(const ros::TimerEvent& timerEvent);

  /*!
   * Publish the point cloud as a PointCloud2.
   * @return true if successful.
   */
  bool publish();

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud message to publish.
  sensor_msgs::PointCloud2::Ptr pointCloudMessage_;

  //! Point cloud publisher.
  ros::Publisher pointCloudPublisher_;

  //! Timer for publishing the point cloud.
  ros::Timer timer_;

  //! Path to the point cloud file.
  std::string filePath_;

  //! Point cloud topic to be published at.
  std::string pointCloudTopic_;

  //! Point cloud frame id.
  std::string pointCloudFrameId_;

  /*!
   * If true, continuous publishing is used.
   * If false, point cloud is only published once.
   */
  bool isContinuouslyPublishing_ = false;

  //! Duration between publishing steps.
  ros::Duration updateDuration_;
};

}  // namespace point_cloud_io
