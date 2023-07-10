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
#include <tf/transform_listener.h>

#include <std_msgs/Int16MultiArray.h>

#include <trajectory_control_msgs/MultiRobotPath.h>
#include <exploration_msgs/ExplorationRobotMessage.h>

int kMainLoopHz = 30;
int kPositionSubRate = 20;

tf::TransformListener *p_tf_listener_ = 0;

ros::Publisher core_multi_robot_paths_pub;
boost::recursive_mutex core_multi_robot_paths_pub_mutex;

ros::Publisher core_results_pub;
boost::recursive_mutex core_results_pub_mutex;

ros::Publisher core_positions_pub;
boost::recursive_mutex core_positions_pub_mutex;

ros::Publisher core_exploration_messages_pub;
boost::recursive_mutex core_exploration_messages_pub_mutex;

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

bool get(const std::string& parent, const std::string& child, tf::StampedTransform& transform)
{
    if (p_tf_listener_->waitForTransform(parent, child, ros::Time(), ros::Duration(1.0)))
    {
        try
        {
            p_tf_listener_->lookupTransform(parent, child, ros::Time(), transform);
            return true;
        }
        catch (tf::LookupException& ex)
        {
            ROS_WARN("no transform available: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_WARN("connectivity error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_WARN("extrapolation error: %s\n", ex.what());
            return false;
        }

        return true;
    }

    std::string error_message = "transformation not available between " + parent + " and " + child;
    ROS_ERROR_STREAM(error_message.c_str());

    return false;
}

void set_tf_stamped_msg(geometry_msgs::TransformStamped& tf_stamped_msg, const tf::StampedTransform& transform, const std::string& robot_frame_id, long int seq)
{
    tf_stamped_msg.child_frame_id = robot_frame_id;
    tf_stamped_msg.header.frame_id = transform.frame_id_;
    tf_stamped_msg.header.seq = seq;
    tf_stamped_msg.header.stamp = transform.stamp_;
    tf_stamped_msg.transform.rotation.w = transform.getRotation().getW();
    tf_stamped_msg.transform.rotation.x = transform.getRotation().getX();
    tf_stamped_msg.transform.rotation.y = transform.getRotation().getY();
    tf_stamped_msg.transform.rotation.z = transform.getRotation().getZ();
    tf_stamped_msg.transform.translation.x = transform.getOrigin().getX();
    tf_stamped_msg.transform.translation.y = transform.getOrigin().getY();
    tf_stamped_msg.transform.translation.z = transform.getOrigin().getZ();
}

void resultsCallback(const std_msgs::Int16MultiArray& msg)
{
    boost::recursive_mutex::scoped_lock locker(core_results_pub_mutex);
    // republish towards local robot 
    core_results_pub.publish(msg); 
}

void explorationMessagesCallback(const exploration_msgs::ExplorationRobotMessage& msg)
{
    boost::recursive_mutex::scoped_lock locker(core_exploration_messages_pub_mutex);
    // republish towards local robot 
    core_exploration_messages_pub.publish(msg); 
}

void positionsCallback(const std_msgs::Int16MultiArray& msg)
{
    boost::recursive_mutex::scoped_lock locker(core_positions_pub_mutex);
    // republish towards local robot 
    core_positions_pub.publish(msg); 
}

void multiRobotPathsCallback(const trajectory_control_msgs::MultiRobotPath& msg)
{
    boost::recursive_mutex::scoped_lock locker(core_multi_robot_paths_pub_mutex);
    // republish towards local robot 
    core_multi_robot_paths_pub.publish(msg); 
}

/// < this runs on core for avoiding message looping; it muxes messages from prefixed topics "ugv"i/topic" into a single "/core/topic" 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_listener_and_publisher");

    ros::NodeHandle n("~");

    /// < tf listener 
    p_tf_listener_ = new tf::TransformListener(ros::Duration(10.0));

    /// < get parameters
    std::string global_frame_id = getParam<std::string>(n, "map_frame", "map");
    std::string robot_frame_id1 = getParam<std::string>(n, "robot_frame_id1", "ugv1/base_link");
    std::string robot_frame_id2 = getParam<std::string>(n, "robot_frame_id2", "ugv2/base_link");
    std::string robot_frame_id3 = getParam<std::string>(n, "robot_frame_id3", "ugv3/base_link"); 

    std::string core_multi_robot_paths_topic   = getParam<std::string>(n, "core_multi_robot_paths_topic", "/core/multi_robot_paths"); 
    std::string core_results_topic = getParam<std::string>(n, "core_results_topic", "/core/results");
    std::string core_positions_topic = getParam<std::string>(n, "core_positions_topic", "/core/positions");
    
    std::string core_exploration_messages_topic = getParam<std::string>(n, "core_exploration_messages_topic", "/core/exploration_messages");
    
    /// < publishers
    ros::Publisher tf_msg_pub1 = n.advertise<geometry_msgs::TransformStamped>("/ugv1/stamped_transform", 1); /// < on core to all robots 
    ros::Publisher tf_msg_pub2 = n.advertise<geometry_msgs::TransformStamped>("/ugv2/stamped_transform", 1); /// < on core to all robots 
    ros::Publisher tf_msg_pub3 = n.advertise<geometry_msgs::TransformStamped>("/ugv3/stamped_transform", 1); /// < on core to all robots 
    
    core_multi_robot_paths_pub   = n.advertise<trajectory_control_msgs::MultiRobotPath>(core_multi_robot_paths_topic, 10); /// < on core to all robots
    core_results_pub   = n.advertise<std_msgs::Int16MultiArray>(core_results_topic, 10); /// < on core to all robots
    core_positions_pub = n.advertise<std_msgs::Int16MultiArray>(core_positions_topic, 10); /// < on core to all robots
    
    core_exploration_messages_pub   = n.advertise<exploration_msgs::ExplorationRobotMessage>(core_exploration_messages_topic, 10); /// < on core to all robots
    
    /// < subscribers 
    ros::Subscriber multi_robot_paths_sub1 = n.subscribe("/ugv1/multi_robot_paths", 1, multiRobotPathsCallback); /// < on core from robot 1
    ros::Subscriber multi_robot_paths_sub2 = n.subscribe("/ugv2/multi_robot_paths", 1, multiRobotPathsCallback); /// < on core from robot 2
    ros::Subscriber multi_robot_paths_sub3 = n.subscribe("/ugv3/multi_robot_paths", 1, multiRobotPathsCallback); /// < on core from robot 3
    
    ros::Subscriber multi_robot_results_sub0 = n.subscribe("/results", 100, resultsCallback); /// < on core from monitor
    ros::Subscriber multi_robot_results_sub1 = n.subscribe("/ugv1/results", 100, resultsCallback); /// < on core from robot 1
    ros::Subscriber multi_robot_results_sub2 = n.subscribe("/ugv2/results", 100, resultsCallback); /// < on core from robot 2
    ros::Subscriber multi_robot_results_sub3 = n.subscribe("/ugv3/results", 100, resultsCallback); /// < on core from robot 3
    
    ros::Subscriber multi_robot_positions_sub1 = n.subscribe("/ugv1/positions", 1, positionsCallback); /// < on core from robot 1
    ros::Subscriber multi_robot_positions_sub2 = n.subscribe("/ugv2/positions", 1, positionsCallback); /// < on core from robot 2
    ros::Subscriber multi_robot_positions_sub3 = n.subscribe("/ugv3/positions", 1, positionsCallback); /// < on core from robot 3
    
    //ros::Subscriber multi_robot_exploration_messages_sub0 = n.subscribe("/exploration_messages", 100, explorationMessagesCallback); /// < on core from monitor
    ros::Subscriber multi_robot_exploration_messages_sub1 = n.subscribe("/ugv1/exploration_messages", 100, explorationMessagesCallback); /// < on core from robot 1
    ros::Subscriber multi_robot_exploration_messages_sub2 = n.subscribe("/ugv2/exploration_messages", 100, explorationMessagesCallback); /// < on core from robot 2
    ros::Subscriber multi_robot_exploration_messages_sub3 = n.subscribe("/ugv3/exploration_messages", 100, explorationMessagesCallback); /// < on core from robot 3    
    
    /// < ======================================================================
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    tf::StampedTransform transform3;

    geometry_msgs::TransformStamped tf_stamped_msg1;
    geometry_msgs::TransformStamped tf_stamped_msg2;
    geometry_msgs::TransformStamped tf_stamped_msg3;

    ros::Rate rate(kMainLoopHz);

    long int seq = 0;

    while (ros::ok())
    {
        if( get(global_frame_id, robot_frame_id1, transform1) ) 
        {
            set_tf_stamped_msg(tf_stamped_msg1, transform1, robot_frame_id1, seq); 
            tf_msg_pub1.publish(tf_stamped_msg1);
        }
        
        if( get(global_frame_id, robot_frame_id2, transform2))
        {
            set_tf_stamped_msg(tf_stamped_msg2, transform2, robot_frame_id2, seq);
            tf_msg_pub2.publish(tf_stamped_msg2);
        }
        
        if( get(global_frame_id, robot_frame_id3, transform3))
        {
            set_tf_stamped_msg(tf_stamped_msg3, transform3, robot_frame_id3, seq);
            tf_msg_pub3.publish(tf_stamped_msg3);
        }

        ros::spinOnce();
        rate.sleep();

        seq++;
    }


    delete p_tf_listener_;

    return 0;
}
