/**
* This file is part of the ROS package path_planner which belongs to the framework 3DPATROLLING. 
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

#include "WaypointsToolSimUgv2.h"


#include "WaypointsTool.cpp" /// <  include the source code for the base class

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_planner_rviz_wp_plugin::WaypointsToolSimUgv2, rviz::Tool)

namespace path_planner_rviz_wp_plugin
{

WaypointsToolSimUgv2::WaypointsToolSimUgv2(
     const std::string& pcl_input_topic_name,
    const std::string& planner_task_feedback_topic_name,   
    const std::string& planner_task_append_topic_name,
    const std::string& planner_task_remove_topic_name,
    const std::string& planner_waypoints_server_name,
    const std::string& patrolling_task_append_topic_name,
    const std::string& patrolling_task_send_topic_name,
    const std::string& patrolling_task_stop_topic_name,
    const std::string& robot_frame_name,
    const std::string& world_frame_name, 
    const std::string& robot_name
):WaypointsTool(pcl_input_topic_name, 
                planner_task_feedback_topic_name, 
                planner_task_append_topic_name, 
                planner_task_remove_topic_name, 
                planner_waypoints_server_name,
                patrolling_task_append_topic_name,
                patrolling_task_send_topic_name,
                patrolling_task_stop_topic_name,
                robot_frame_name, 
                world_frame_name, 
                robot_name)
{
    shortcut_key_ = 'l';
    
    if(node_.hasParam(NAME_PARAMETER_ROBOT_NAME))
    {
        // ovverride robot frame id
        std::string robot_name;
        node_.getParam(NAME_PARAMETER_ROBOT_NAME,robot_name);
        robot_frame_id_ = "/"+robot_name + NAME_BASE_BASE_LINK;
    }
    
    // load mesh for 'interactive' markers
//    resource_interactive_ = "package://path_planner_rviz_wp_plugin/mesh/flag.dae";
//    if (rviz::loadMeshFromResource(resource_interactive_).isNull())
//    {
//        ROS_ERROR("WaypointsTool: failed to load model resource '%s'.", resource_interactive_.c_str());
//        return;
//    }
    
    // load mesh for 'static' markers with no failures
    resource_success_ = "package://path_planner_rviz_wp_plugin/mesh/barrier_success2.dae";
    if (rviz::loadMeshFromResource(resource_success_).isNull())
    {
        ROS_ERROR("WaypointsTool: failed to load model resource '%s'.", resource_success_.c_str());
        return;
    }
    
    
}

}
