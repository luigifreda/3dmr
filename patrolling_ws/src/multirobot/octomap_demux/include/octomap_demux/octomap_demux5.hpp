/**
* This file is part of the ROS package octomap_demux which belongs to the framework 3DMR. 
*
* Copyright (C) 2017-2019 Luigi Freda <luigifreda at gmail dot com>  
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

#ifndef OCTOMAP_DEMUX5_OCTOMAP_MUX_HPP_
#define OCTOMAP_DEMUX5_OCTOMAP_MUX_HPP_

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/message_filter.h>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
        {
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
        }
	return defaultValue;
}


class OctomapDeMux5 {

 public:
  explicit OctomapDeMux5(ros::NodeHandle& n);
  ~OctomapDeMux5() {}
  
 protected:
     
    /// \brief Callback for the point_map.
  void pointMapCallback(const sensor_msgs::PointCloud2& cloud_msg_in);
  
  std::string pcl_sub_name;
  
  std::string pcl1_pub_name;
  std::string pcl2_pub_name;
  std::string pcl3_pub_name;
  std::string pcl4_pub_name;
  std::string pcl5_pub_name;
  
  std::string global_ref;
  
  tf::TransformListener tf_;
  tf::MessageFilter<sensor_msgs::PointCloud2> * pcl_tf_filter_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_;
  
  ros::Subscriber pcl_normal_sub_;
  

 private:
  // Node handle.
  ros::NodeHandle& nh_;
  ros::NodeHandle n_;

  // Subscribers.
  //ros::Subscriber dynamic_cloud_sub_;
  //ros::Subscriber point_map_sub_;

  // Publishers.
  ros::Publisher pcl1_pub_;
  ros::Publisher pcl2_pub_;
  ros::Publisher pcl3_pub_;
  ros::Publisher pcl4_pub_;
  ros::Publisher pcl5_pub_;
  
  std::mutex mux_pub_mutex_;

  static constexpr unsigned int kSubMessageQueueSize = 20u;
  static constexpr unsigned int kPubMessageQueueSize = 20u;
}; // LaserMapper

#endif /* OCTOMAP_DEMUX_OCTOMAP_MUX_HPP_ */
