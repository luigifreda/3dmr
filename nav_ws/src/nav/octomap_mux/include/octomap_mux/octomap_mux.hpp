#ifndef OCTOMAP_MUX_OCTOMAP_MUX_HPP_
#define OCTOMAP_MUX_OCTOMAP_MUX_HPP_

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class OctomapMux {

 public:
  explicit OctomapMux(ros::NodeHandle& n);
  ~OctomapMux() {}
  
 protected:
  /// \brief Callback for the dynamic cloud.
  void dynamicCloudCallback(const sensor_msgs::PointCloud2& cloud_msg_in);
  
  /// \brief Callback for the point_map.
  void pointMapCallback(const sensor_msgs::PointCloud2& cloud_msg_in);
  
  /// \brief Callback for the cloud 3
  void cloudCallback(const sensor_msgs::PointCloud2& cloud_msg_in);
  

 private:
  // Node handle.
  ros::NodeHandle& nh_;

  // Subscribers.
  ros::Subscriber dynamic_cloud_sub_;
  ros::Subscriber point_map_sub_;
  ros::Subscriber cloud3_sub_;
  ros::Subscriber cloud4_sub_;
  ros::Subscriber cloud5_sub_;

  // Publishers.
  ros::Publisher mux_pub_;
  std::mutex mux_pub_mutex_;

  static constexpr unsigned int kSubMessageQueueSize = 10u;
  static constexpr unsigned int kPubMessageQueueSize = 10u;
}; // LaserMapper

#endif /* OCTOMAP_MUX_OCTOMAP_MUX_HPP_ */
