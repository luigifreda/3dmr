// Copyright (c) 2021, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static std::string topic = "/omron_ros_wheel/odom";
static std::string parent_frame = "/odom";
static std::string child_frame = "/base_link";

void odom_callback(const nav_msgs::OdometryConstPtr& odom){
  static tf::TransformBroadcaster br;
  tf::Transform tf;
  geometry_msgs::Pose odom_pose = odom->pose.pose;

//  tf.setOrigin(tf::Vector3(odom_pose.position.x, odom_pose.position.y, odom_pose.position.z));
//  tf::Quaternion quat;
//  tf::quaternionMsgToTF(odom_pose.orientation, quat);
//  tf.setRotation(quat);
  tf::poseMsgToTF(odom_pose, tf);

  tf::StampedTransform stamped_tf(tf, odom->header.stamp, parent_frame, child_frame);

  br.sendTransform(stamped_tf);
}

int main(int argc, char **argv){

  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string robot_topic_prefix;
  private_nh.getParam("odom_topic", topic);
  private_nh.getParam("parent_frame", parent_frame);
  private_nh.getParam("child_frame", child_frame);
  private_nh.getParam("robot_topic_prefix", robot_topic_prefix);  

  ROS_INFO_STREAM("Topic: " << topic);
  ROS_INFO_STREAM("Topic prefix: " << robot_topic_prefix);   
  ROS_INFO_STREAM("Parent frame: " << parent_frame);
  ROS_INFO_STREAM("Child frame: " << child_frame);
 

  ros::Subscriber odom_sub = nh.subscribe(topic, 10, odom_callback);
  ros::spin();

  return 0;
}