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

/// remove a task from the queue
void QueuePathPlanner::removeCallback(const trajectory_control_msgs::PlanningTask& task_msg)
{ 
    std::cout << "QueuePathPlanner::removeCallback()" << std::endl;
        
    boost::recursive_mutex::scoped_lock locker(task_queue_mutex_); 
        
    // iterate through all tasks of the queue
    for( std::list<TaskPtr>::iterator it=task_queue_.begin(); task_queue_.end()!=it; )
    {
        // get the task
        TaskPtr task = *it; 

        // identify task
        std::string name = (*task)[0]->segment_task.name;

        // decide if remove it or not
        if( !name.compare(task_msg.name) || !task_msg.name.compare("ALL") )
        {
            // remove the task
            // (I assume that there are no race conditions with the timer callback)

            // remove task from the queue
            it = task_queue_.erase(it); // this will not delete the pointers contained in the erased TaskPtr

            // for each task-segment of the task
            for( size_t i=0; task->size()>i; i++ )
            {
                TaskSegmentPtr segment = (*task)[i];

                segment->b_aborted = true; // we use it as a volatile flag, can be changed without locking mutex 
                
                // lock the segment 
                //boost::recursive_mutex::scoped_lock locker(segment->mutex); 
                                
                /// here we could force a stop of the planner
                if(segment->p_path_planner) segment->p_path_planner->setAbort(true); 
                
                // join the planning thread
                segment->thread.join();

                // delete the task's segment
                // (this line should not be commented but there is an
                // estrange issue related to a 'double free error'
                // with the pcl::KdTreeFLANN objects, so to avoid
                // the crash of the planner, we allow some memory leaks) 
                //delete segment;
            }

            // status information
            ROS_WARN("%s removed (by user)",name.c_str());
        }
        else
        {
            it++;
        }
    }
    
    if(!task_msg.name.compare("ALL"))
    {
        controller_ready_flag_ = true; 
    }
    
}
