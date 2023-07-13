/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Alcor Lab (La Sapienza University)
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

#include <ros/ros.h>
#include <signal.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "PathPlanner.h"  // stay before any pcl include, it contains PCL_NO_PRECOMPILE directive

#include <MarkerController.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <std_msgs/Bool.h>

#include <pcl/common/geometry.h>

#include <actionlib/client/simple_action_client.h>
#include <trajectory_control_msgs/TrajectoryControlAction.h>


/// < PARAMETERS 
const int kPathPlannerMaxNumAttempts = 10;
const float kAcceptable3DDistanceToGoal = 2.5*PathPlanner::kGoalPlanningCheckThreshold;
const float kAcceptable2DDistanceToGoal = 2.0*PathPlanner::kGoalPlanningCheckThreshold;
const float kRobotSize = PathPlanner::kRobotSize;

/// < GLOBAL VARS 

volatile bool is_wall_info_available = false;
volatile bool is_traversability_info_available = false;

boost::recursive_mutex wall_mutex;
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wall_pcl(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
pp::KdTreeFLANN<pcl::PointXYZRGBNormal> wall_kdtree;

boost::recursive_mutex traversability_mutex;
pcl::PointCloud<pcl::PointXYZI>::Ptr traversability_pcl(new pcl::PointCloud<pcl::PointXYZI>());
PathPlanner::KdTreeFLANN traversability_kdtree;

tf::TransformListener *p_tf;
ros::Publisher robot_global_path_pub;
ros::Publisher path_plan_stat_pub;

pcl::PointXYZI goal_point;
volatile bool is_goal_selected = false;


/// < MAIN CLASSES
boost::shared_ptr<MarkerController> p_marker_controller;
boost::shared_ptr<PathPlanner> p_planner;

boost::shared_ptr< actionlib::SimpleActionClient<trajectory_control_msgs::TrajectoryControlAction> > p_act_client;

// current path planner failures in computing a path towards the selected goal 
int path_planner_num_failures = 0;

boost::recursive_mutex do_path_planing_mutex;

//boost::recursive_mutex destroy_mutex; 

std::string robot_frame_id;
std::string goal_topic_name;
std::string goal_abort_topic_name; 
std::string int_marker_server_name;
std::string int_marker_name;

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

bool getRobotPose(tf::StampedTransform& robot_pose)
{

    //if (p_tf->waitForTransform("map", "base_link", ros::Time(), ros::Duration(1.0)))
    if (p_tf->waitForTransform("map", robot_frame_id, ros::Time(), ros::Duration(1.0)))
    {
        try
        {
            //p_tf->lookupTransform("map", "base_link", ros::Time(),robot_pose);
            p_tf->lookupTransform("map", robot_frame_id, ros::Time(),robot_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_INFO(
                     "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO(
                     "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_INFO(
                     "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_INFO("Transformation is not available");
        return false;
    }
}


void doPathPlanning()
{
    boost::recursive_mutex::scoped_lock path_planning_locker(do_path_planing_mutex);
                
    std_msgs::Bool status;
    if (is_goal_selected)
    {
        p_planner->setGoal(goal_point);
        p_marker_controller->setMarkerColor(Colors::LightYellow(), "Planning");
    }

    if (is_traversability_info_available && is_wall_info_available)
    {
        tf::StampedTransform robot_pose;
        getRobotPose(robot_pose);
        pcl::PointXYZI robot_position;
        robot_position.x = robot_pose.getOrigin().x();
        robot_position.y = robot_pose.getOrigin().y();
        robot_position.z = robot_pose.getOrigin().z();

        int starting_node_idx = -1;
        {
        boost::recursive_mutex::scoped_lock traversability_locker(traversability_mutex);
        starting_node_idx = PathPlanner::getClosestNodeIdx(robot_position, traversability_kdtree, 0.5 * kRobotSize + 0.1/*+tolerance*/);
        }

        if (starting_node_idx < 0)
        {
            ROS_WARN("cannot find a traversable node which is close enough to current robot position ");
            return;
        }

        ROS_INFO("path planner input set");
        { // start scope for locking mutex 
            boost::recursive_mutex::scoped_lock wall_locker(wall_mutex);
            boost::recursive_mutex::scoped_lock traversability_locker(traversability_mutex);
            
            p_planner->setInput(*traversability_pcl, *wall_pcl, wall_kdtree, traversability_kdtree, starting_node_idx);
            //is_wall_info_available = false;
        }// end scope for locking mutex

        if (is_goal_selected)
        {
            //if the goal has been reached then exit 
            pcl::PointXYZI goal_position = p_planner->getGoal();
            float distance3D_to_goal = PathPlanner::dist(goal_position, robot_position);
            float distance2D_to_goal = PathPlanner::dist2D(goal_position, robot_position);
            std::cout << "distance to goal -  3D: " << distance3D_to_goal << ", 2D: " << distance2D_to_goal << std::endl;
            if( (distance3D_to_goal < kAcceptable3DDistanceToGoal) && (distance2D_to_goal < kAcceptable2DDistanceToGoal) )
            {
                p_marker_controller->setMarkerColor(Colors::LightGrey(), "Arrived...select another goal");

                ROS_INFO("goal actually reached: attained distance %f", distance3D_to_goal);
                is_goal_selected = false;
                path_planner_num_failures = 0;
                return; /// < EXIT POINT
            }

            nav_msgs::Path path;

            ROS_INFO("planning path");
            if (is_goal_selected)
            {
                if (p_planner->planning(path))
                {
                    robot_global_path_pub.publish(path);
                    status.data = true;
                    path_plan_stat_pub.publish(status);

                    path_planner_num_failures = 0; // reset num failures  
                    ROS_INFO("path successfully computed");

                    p_marker_controller->setMarkerColor(Colors::Green(), "Success");


                    //ros::shutdown();  /// < TO REMOVE 
                }
                else
                {
                    ROS_INFO("no path exist for desired goal, trying again");
                    status.data = false;
                    path_plan_stat_pub.publish(status);
                    //is_goal_selected = true; // retry  

                    path_planner_num_failures++; // increase num failures 
                    ROS_INFO("path planner failure num %d", path_planner_num_failures);
                    if (path_planner_num_failures > kPathPlannerMaxNumAttempts)
                    {
                        is_goal_selected = false;
                        path_planner_num_failures = 0;
                        ROS_INFO("no path exist for desired goal, please choose another goal");
                        p_marker_controller->setMarkerColor(Colors::Red(), "Failure: select another goal");
                    }
                }

            }
        }
    }
    else
    {
        ROS_WARN("doPathPlanning() - traversability info: %d, wall info: %d", (int) is_traversability_info_available, (int) is_wall_info_available);
    }
}

void traversabilityCloudCallback(const sensor_msgs::PointCloud2& traversability_msg)
{
    boost::recursive_mutex::scoped_lock traversability_locker(traversability_mutex);
        
    pcl::fromROSMsg(traversability_msg, *traversability_pcl);
    
    if(traversability_pcl->size() > 0)
    {
        traversability_kdtree.setInputCloud(traversability_pcl);
        is_traversability_info_available = true;
    }
    
    doPathPlanning(); 
}           
 

void wallCloudCallback(const sensor_msgs::PointCloud2& wall_msg)
{
    boost::recursive_mutex::scoped_lock wall_locker(wall_mutex);

    pcl::fromROSMsg(wall_msg, *wall_pcl);
    wall_kdtree.setInputCloud(wall_pcl); 
    is_wall_info_available = true;
}

void goalSelectionCallback(geometry_msgs::PoseStamped goal_msg)
{
    std::cout << "goalSelectionCallback()" << std::endl;
    goal_point.x = goal_msg.pose.position.x;
    goal_point.y = goal_msg.pose.position.y;
    goal_point.z = goal_msg.pose.position.z;
    //p_planner->set_goal(goal_point);
    if(!is_goal_selected) is_goal_selected = true;
    ROS_INFO("goal selection");
    
    doPathPlanning();
}

void goalAbortCallback(std_msgs::Bool msg)
{
    std::cout << "goalAbortCallback()" << std::endl;

    if(msg.data == 1)
    {
        if(is_goal_selected) is_goal_selected = false;
        
        p_planner->setAbort(true); 
        
        p_marker_controller->setMarkerColor(Colors::Red(), "Abort");
                            
        ROS_INFO("goal abort");
        
        if(p_act_client)
        {
            p_act_client->waitForServer();
            p_act_client->cancelGoal();
        }
    }
}

void mySigintHandler(int signum)
{
    std::cout << "mySigintHandler()" << std::endl;
    //boost::recursive_mutex::scoped_lock destroy_locker(destroy_mutex); 

    //p_planner.reset();
    //p_marker_controller.reset(); 

    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");

    //override default sigint handler 
    signal(SIGINT, mySigintHandler);

    ros::NodeHandle n("~");

    /// < get parameters

    robot_frame_id = getParam<std::string>(n, "robot_frame_name", "/base_link");
    goal_topic_name = getParam<std::string>(n, "goal_topic_name", "/goal_topic");
    goal_abort_topic_name = getParam<std::string>(n, "goal_abort_topic_name", "/goal_abort_topic");
    int_marker_server_name = getParam<std::string>(n, "int_marker_server_name", "marker_controller");
    int_marker_name = getParam<std::string>(n, "int_marker_name", "Goal");
    
    std::string trajectory_control_node_name = getParam<std::string>(n, "trajectory_control_action_server_name", "trajectory_control_action_server_ugv1");

    tf::TransformListener tf;
    p_tf = &tf;

    // get a first robot pose in order to initialize the marker on robot starting position
    tf::StampedTransform robot_pose;
    getRobotPose(robot_pose);

    //tf::TransformListener tf_listener(ros::Duration(10.0));

    //PathPlanner new_planner(n);
    //p_planner = &new_planner;
    p_planner.reset(new PathPlanner);

    ros::Subscriber sub_trav = n.subscribe("/trav/traversability", 1, traversabilityCloudCallback);
    ros::Subscriber sub_wall = n.subscribe("/clustered_pcl/wall", 1, wallCloudCallback);
    
    //ros::Subscriber sub_goal = n.subscribe("/goal_topic", 1, goalSelectionCallback);
    ros::Subscriber sub_goal       = n.subscribe(goal_topic_name, 1, goalSelectionCallback);
    ros::Subscriber sub_goal_abort = n.subscribe(goal_abort_topic_name, 1, goalAbortCallback);

    robot_global_path_pub = n.advertise<nav_msgs::Path>("/robot_path", 1);
    path_plan_stat_pub = n.advertise<std_msgs::Bool>("/path_planning_status", 1);

    //MarkerController marker;
    p_marker_controller.reset(new MarkerController(robot_pose.getOrigin(), goal_topic_name, goal_abort_topic_name, int_marker_server_name, int_marker_name));

    p_act_client.reset(new actionlib::SimpleActionClient<trajectory_control_msgs::TrajectoryControlAction>(trajectory_control_node_name,true)); 
    
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    
    //ros::spin();

    //boost::recursive_mutex::scoped_lock destroy_locker(destroy_mutex); 

    //p_marker_controller->reset();

    return 0;
}
