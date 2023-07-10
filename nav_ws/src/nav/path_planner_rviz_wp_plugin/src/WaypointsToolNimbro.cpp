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

#include "WaypointsToolNimbro.h"


#include "WaypointsTool.cpp" /// <  include the source code for the base class

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_planner_rviz_wp_plugin::WaypointsToolNimbro, rviz::Tool)

namespace path_planner_rviz_wp_plugin
{

WaypointsToolNimbro::WaypointsToolNimbro(
    const std::string& pcl_input_topic_name,
    const std::string& planner_task_feedback_topic_name,   
    const std::string& planner_task_append_topic_name,
    const std::string& planner_task_remove_topic_name,
    const std::string& planner_waypoints_server_name,
    const std::string& patrolling_task_append_topic_name,
    const std::string& patrolling_task_pause_topic_name,
    const std::string& robot_frame_name,
    const std::string& world_frame_name, 
    const std::string& robot_name
):WaypointsTool(pcl_input_topic_name, planner_task_feedback_topic_name, planner_task_append_topic_name, 
                planner_task_remove_topic_name, planner_waypoints_server_name,
                patrolling_task_append_topic_name,patrolling_task_pause_topic_name,
                robot_frame_name, world_frame_name, robot_name)
{
    shortcut_key_ = 'n';
}

}
