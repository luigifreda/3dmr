#include "octomap_mux/octomap_mux.hpp"

#include <sensor_msgs/PointCloud2.h>

OctomapMux::OctomapMux(ros::NodeHandle& n) : nh_(n) {
  
  dynamic_cloud_sub_ = nh_.subscribe("/dynamic_point_cloud", kSubMessageQueueSize,
                                     &OctomapMux::cloudCallback, this);
  point_map_sub_ = nh_.subscribe("/point_map", kSubMessageQueueSize,
                                 &OctomapMux::cloudCallback, this);
                                 
                                 
  cloud3_sub_ = nh_.subscribe("/cloud3", kSubMessageQueueSize,
                                 &OctomapMux::cloudCallback, this);       
  cloud4_sub_ = nh_.subscribe("/cloud4", kSubMessageQueueSize,
                                 &OctomapMux::cloudCallback, this);                                                            
  cloud5_sub_ = nh_.subscribe("/cloud5", kSubMessageQueueSize,
                                 &OctomapMux::cloudCallback, this);           
                                                        
  mux_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mux_point_cloud", kPubMessageQueueSize);
}  

void OctomapMux::dynamicCloudCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  std::cout << "OctomapMux::dynamicCloudCallback()" << std::endl; 
  mux_pub_mutex_.lock();
  mux_pub_.publish(cloud_msg_in);
  mux_pub_mutex_.unlock();
}


void OctomapMux::pointMapCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  std::cout << "OctomapMux::pointMapCallback()" << std::endl; 
  mux_pub_mutex_.lock();
  mux_pub_.publish(cloud_msg_in);
  mux_pub_mutex_.unlock();
}

/// \brief Callback for the cloud 3
void OctomapMux::cloudCallback(const sensor_msgs::PointCloud2& cloud_msg_in)
{
  std::cout << "OctomapMux::cloudCallback()" << std::endl; 
  mux_pub_mutex_.lock();
  mux_pub_.publish(cloud_msg_in);
  mux_pub_mutex_.unlock();
}
  
