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

void QueuePathPlanner::appendCallback(const trajectory_control_msgs::PlanningTask& task_msg)
{
    bool append_flag=true;

    if( !traversability_flag_ || !wall_flag_ )
    {
	ROS_WARN("no traversability or wall cloud available");
	append_flag=false;
    }

    // get robot pose with respect to reference_frame_
    tf::StampedTransform robot_pose;
    try
    {
	robot_pose  = transform_.get(reference_frame_,robot_frame_);
        append_flag = transform_.isOk();
    }
    catch(TransformException e )
    {
        ROS_WARN("transform exception");
	ROS_WARN("%s",e.what());
        append_flag=false;
    }

    /// < we cannot append the task 
    if(!append_flag ) 
    {
        // we are not ready for appending a new task 
        
	ROS_WARN("task %s dropped",task_msg.name.c_str());

	// create a new task object
	//std::vector<TaskSegment*> task;
        TaskPtr task(new Task); 

    	// we don't need to push_back all segments
	// since the task will be completely discarded

	// create a new task's segment object
	//TaskSegment *segment = new TaskSegment;
        TaskSegmentPtr segment(new TaskSegment); 

	// fill segment's data (only header with failure status) 
	segment->status=STATUS_FAILURE;
	segment->segment_task.header=task_msg.header;
	segment->segment_task.name=task_msg.name;
	segment->segment_task.segment_id=1;
	segment->segment_task.segment_count=task_msg.segment_count;

	// start planning thread
	// (this is needed because we synchronize with join())
	//segment->thread=boost::thread(&QueuePathPlanner::pathPlanningCallback,segment); 
        segment->thread=boost::thread(boost::bind(&QueuePathPlanner::pathPlanningCallback, this, segment, task)); /// < START NEW THREAD 

	// add segment to the task
	task->push_back(segment);

	// push task to the task queue
        { // start scope for locking mutex 
        boost::recursive_mutex::scoped_lock locker(task_queue_mutex_); 
	task_queue_.push_back(task);
        } // end scope for locking mutex 

	return; /// < EXIT POINT 
    }

    
    /// < we can actually append a new task 

    // extract robot position
    geometry_msgs::Point robot_position;
    robot_position.x=robot_pose.getOrigin().x();
    robot_position.y=robot_pose.getOrigin().y();
    robot_position.z=robot_pose.getOrigin().z();

    // generate the array of points including current robot position plus all received waypoints
    std::vector<geometry_msgs::Point> waypoints;
    waypoints.push_back(robot_position);
    waypoints.insert(waypoints.end(),task_msg.waypoints.begin(),task_msg.waypoints.end());
    
    int num_segments = task_msg.segment_count;
    if(task_msg.type == kPathCyclic)
    {
        ROS_INFO("cyclic path");
        waypoints.push_back(robot_position);
        num_segments++;
    }

    // create a new task object
    //std::vector<TaskSegment*> task;
    TaskPtr task(new Task); 
    task->type = (TaskType)task_msg.type; 

    // for each task-segment contained in the task 
    for(int i=0; i< num_segments; i++)
    {
	/// < create a new task-segment object
	//TaskSegment *segment = new TaskSegment;
        TaskSegmentPtr segment(new TaskSegment);

	// < fill main segment's data
	segment->status=STATUS_PLANNING;
	segment->segment_task.header=task_msg.header;
	segment->segment_task.name=task_msg.name;
	segment->segment_task.segment_id=i+1;
	segment->segment_task.segment_count=num_segments;//task_msg.segment_count;
        
        // < each task segment is composed just by TWO waypoints 
	segment->segment_task.waypoints.push_back(waypoints[i]);
	segment->segment_task.waypoints.push_back(waypoints[i+1]);
        
        
	// start planning thread
	//segment->thread=boost::thread(&QueuePathPlanner::pathPlanningCallback,segment); 
        segment->thread=boost::thread(boost::bind(&QueuePathPlanner::pathPlanningCallback, this, segment, task)); /// < START NEW THREAD 

	// add segment to the task
	task->push_back(segment);
    }
    
    // push planning task to the queue
    { // start scope for locking mutex 
    boost::recursive_mutex::scoped_lock locker(task_queue_mutex_); 
    task_queue_.push_back(task);
    } // end scope for locking mutex 

}
