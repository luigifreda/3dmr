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

#include "QueuePathPlanner.h"

//#define CANCEL_EXECUTED_PATH 1

void QueuePathPlanner::feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg)
{
    
    std::cout << "QueuePathPlanner::feedbackCallback()" << std::endl;
        
    // if this message have been published by the controller node, then
    if( !feedback_msg.node.compare("control") )
    {
	// controller busy? 
	if( !feedback_msg.task.compare("start") )
	{
	    controller_ready_flag_=false;

	    // stop timer for tasks processing
	    /// < task_timer_.stop();
	}
	else // we are receiving a stop or abort
        //if( !feedback_msg.task.compare("done") )
	{
	    controller_ready_flag_=true;

	    // (re)start timer for task's processing
	    /// < task_timer_.start();
            
#ifdef CANCEL_EXECUTED_PATH
            // init empty path message
            nav_msgs::Path path_msg;
            path_msg.header.stamp=ros::Time::now();
            path_msg.header.frame_id=reference_frame_;   
            // publish the empty path
            boost::recursive_mutex::scoped_lock locker(task_path_pub_mutex_); 
	    task_path_pub_.publish(path_msg);
#endif
        
	}
    }
}
