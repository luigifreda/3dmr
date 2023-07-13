/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
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

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 


#include <ros/ros.h>
#include <signal.h>

#include <limits>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/geometry.h>


/// < PARAMETERS 


static const int kMaxNumberOfRobots = 16; 


ros::Publisher dynamic_cloud_router_pub;

ros::Subscriber other_dynamic_point_cloud_sub[kMaxNumberOfRobots];

std::string str_robot_name;
std::string robot_frame_id;

int number_of_robots = 2; 
int robot_id         = 0;


/// < functions 

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

void otherUgvDynamicPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
    // route teammate point cloud toward pointcloud2 of 'dense' octomap  
    dynamic_cloud_router_pub.publish(pointcloud_msg);
}


void mySigintHandler(int signum)
{
    std::cout << "mySigintHandler()" << std::endl;
    //boost::recursive_mutex::scoped_lock destroy_locker(destroy_mutex); 

    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_manager_node");

    //override default sigint handler 
    signal(SIGINT, mySigintHandler);

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    /// < get parameters
    
    str_robot_name = getParam<std::string>(nh_private, "robot_name", "ugv1");   /// < multi-robot
    std::string str_robot_prefix = "ugv"; 
    robot_id = atoi(str_robot_name.substr(3,str_robot_name.size()).c_str()) - 1;  
    
    std::string simulator_name  = getParam<std::string>(nh_private, "simulator", "");   /// < multi-robot    
    
    ROS_INFO_STREAM("==========================================================");    
    ROS_INFO_STREAM("exploration router node of robot " << robot_id << " alive");
    ROS_INFO_STREAM("==========================================================");     
    
    number_of_robots = getParam<int>(nh_private, "number_of_robots", 2);   /// < multi-robot    
    
    robot_frame_id = getParam<std::string>(nh_private, "robot_frame_name", "base_link");

          
    //std::string other_robot_dynamic_point_cloud_base_topic = getParam<std::string>(nh_private, "other_robot_dynamic_point_cloud_base_topic", "dynamic_point_cloud");
    std::string other_robot_dynamic_point_cloud_base_topic = getParam<std::string>(nh_private, "other_robot_dynamic_point_cloud_base_topic", "filtered_pointcloud");

    
    std::cout << "got parameters" << std::endl;

    /// ========================================================================
    
    
    /// < Publishers 

    dynamic_cloud_router_pub = nh.advertise<sensor_msgs::PointCloud2>("/volumetric_mapping/pointcloud2", 20); // toward this robot dense/traversability octomap 

    
    /// < Subscribers

    for(size_t id=0; id < kMaxNumberOfRobots; id++)
    {
        /// < N.B.: the individual robot scan is integrated by the ExplorationPlanner
        /// <       topic: "/expl_planner/exploration_pointcloud" => callback: ExplorationPlanner::insertPointcloudWithTf()

        // integrate teammate scans 
        if(id != robot_id)
        {
            std::stringstream topic_name;
            if(!simulator_name.empty())
            {
                //topic_name << "/"<< simulator_name;
                topic_name << simulator_name;
            }
            topic_name << "/" << str_robot_prefix << id+1 << "/" << other_robot_dynamic_point_cloud_base_topic;  
            std::cout << "topic name in: " << topic_name.str() << std::endl;
            
            other_dynamic_point_cloud_sub[id] = nh.subscribe(topic_name.str(), 20, otherUgvDynamicPointCloudCallback); /// < multi-robot            
        }
    }    

    //ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    //spinner.spin(); // spin() will not return until the node has been shutdown

    ros::spin();

    return 0;
}
