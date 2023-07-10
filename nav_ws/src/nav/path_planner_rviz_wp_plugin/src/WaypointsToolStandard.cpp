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


/* Tell pluginlib about the class.  It is important
 * to do this in global scope, outside our package's namespace. */

#define WAYPOINT_REDEFINED_TOPICS

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_planner_rviz_wp_plugin::WaypointsTool, rviz::Tool)

static const std::string pcl_input_topic_name = "/dynjoinpcl_nn";
static const std::string planner_task_feedback_topic_name = "/planner/tasks/feedback";
static const std::string planner_task_append_topic_name = "/planner/tasks/append";
static const std::string planner_task_remove_topic_name = "/planner/tasks/remove";
static const std::string planner_waypoints_server_name = "/planner/waypoints/server";


#include "WaypointsTool.cpp"