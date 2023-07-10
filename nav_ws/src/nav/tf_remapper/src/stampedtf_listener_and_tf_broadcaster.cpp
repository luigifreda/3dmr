/**
* This file is part of the ROS package tf_remapper which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

std::string global_frame_id;
std::string robot_frame_id;
std::string robot_name;

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

void poseCallback(const geometry_msgs::TransformStamped& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z));
    transform.setRotation(tf::Quaternion(msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w));
    br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, global_frame_id, robot_frame_id));
}


/// < this runs on robot for avoiding message looping, it receives a single robot current pose from a prefixed topic "/ugvi/stamped_transform" and broadcasts this to the local "/tf"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "stampedtf_listener_and_tf_broadcaster"); 
    
    ros::NodeHandle n("~");
    
    /// < get parameters
    global_frame_id = getParam<std::string>(n, "map_frame", "map");
    robot_name = getParam<std::string>(n, "robot_name", "ugv1");
    robot_frame_id = "/" + robot_name + "/base_link";
    
    ros::Subscriber sub = n.subscribe("/input_topic", 10, &poseCallback);

    ros::spin();
    return 0;
};
