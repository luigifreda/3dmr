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
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <vector>
#include <limits>


/// < PARAMETERS 

const int kNumRobots = 2; 

const int kCircularBufferSize = 10; 
const float kMinUpdateTime = 0.03; // [s]

/// < STRUCTS/CLASSES

ros::Time last_pub_time = ros::TIME_MIN; // last publish time 

std::vector<tf::tfMessage> vec_msg(kNumRobots); 
boost::ptr_vector<boost::recursive_mutex> vec_mtx; //a ptr_vector takes ownership of its pointers so that they are deleted when appropriate without any of the overhead a smart pointer might introduce.

ros::Publisher merged_tf_pub;

size_t max_total_size = std::numeric_limits<size_t>::min();

/// < FUNCTIONS

void init_vec_mtx()
{
    for(int id=0;id<kNumRobots;id++)   vec_mtx.push_back(new boost::recursive_mutex);
}

void combine(tf::tfMessage& out)
{
    // update the max total size of the out msg
    size_t total_size = 0;
    for(int id = 0; id < kNumRobots; id++) 
    {
        boost::recursive_mutex::scoped_lock lock(vec_mtx[id]);
        total_size+= vec_msg[id].transforms.size();
    }
    if(max_total_size < total_size) max_total_size = total_size;
    
    // publish it
    out.transforms.reserve(max_total_size);
    for(int id = 0; id < kNumRobots; id++)
    {
        boost::recursive_mutex::scoped_lock lock(vec_mtx[id]);
        if(!vec_msg[id].transforms.empty())
        {
            out.transforms.insert(out.transforms.end(),vec_msg[id].transforms.begin(),vec_msg[id].transforms.end());
        }
    }
}

void publish()
{
    ros::Time time_now         = ros::Time::now();
    ros::Duration elapsed_time = time_now - last_pub_time;
    //if (elapsed_time.toSec() > kMinUpdateTime) // check min elapsed time 
    {
        // combine messages and publish 
        tf::tfMessage out;
        combine(out); 
        merged_tf_pub.publish(out);
        last_pub_time = time_now;
        //std::cout << "publish()" << std::endl; 
    }
}

void callback(const int id, const tf::tfMessageConstPtr& msg)
{
    //std::cout << "callback " << id << std::endl; 
    boost::recursive_mutex::scoped_lock lock(vec_mtx[id]);
    vec_msg[id] = *msg; 
    publish();
}

void callback1(const tf::tfMessageConstPtr& msg)
{
    static const int id = 1-1; 
    callback(id, msg);
}

void callback2(const tf::tfMessageConstPtr& msg)
{
    static const int id = 2-1; 
    callback(id, msg);
}

int main(int argc, char** argv)
{
  init_vec_mtx(); 
  
  ros::init(argc, argv, "tf_merger_node");

  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("/sim1_prefixed/tf", 100, callback1);
  ros::Subscriber sub2 = n.subscribe("/sim2_prefixed/tf", 100, callback2);
  
  merged_tf_pub = n.advertise<tf::tfMessage>("/tf", 100);

  ros::spin();

  return 0;
}
