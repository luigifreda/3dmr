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


// perform the planning on the input segment of the given input task
void QueuePathPlanner::pathPlanningCallback(TaskSegmentPtr segment, TaskPtr task)
{
    // planning information
#ifdef VERBOSE
    ROS_INFO("QueuePathPlanner::pathPlanningCallback() - %s %02d/%02d started", segment->segment_task.name.c_str(), segment->segment_task.segment_id, segment->segment_task.segment_count);
#endif

    // lock the segment 
    boost::recursive_mutex::scoped_lock locker(segment->mutex);

    { // start scope for locking status_mutex
        /// < test segment status flag before proceeding
        boost::recursive_mutex::scoped_lock locker_status(segment->status_mutex);
        {
            if (STATUS_FAILURE == segment->status)
            {
                // planning information
#ifdef VERBOSE
                ROS_INFO("%s %02d/%02d aborted", segment->segment_task.name.c_str(), segment->segment_task.segment_id, segment->segment_task.segment_count);
#endif

                // abort planning thread execution in the the case of failure
                return; /// < EXIT POINT 
            }
        }
    }// end scope for locking status_mutex

    /// < convert waypoints to the point representation of the planner

    if (segment->segment_task.waypoints.size() != 2)
    {
        {
            boost::recursive_mutex::scoped_lock locker_status(segment->status_mutex);
            segment->status = STATUS_FAILURE;
        }
        ROS_ERROR("QueuePathPlanner::pathPlanningCallback() - error in waypoints size");
        return;
    }

    pcl::PointXYZI start;
    start.x = segment->segment_task.waypoints[0].x;
    start.y = segment->segment_task.waypoints[0].y;
    start.z = segment->segment_task.waypoints[0].z;

    pcl::PointXYZI goal;
    goal.x = segment->segment_task.waypoints[1].x;
    goal.y = segment->segment_task.waypoints[1].y;
    goal.z = segment->segment_task.waypoints[1].z;

    /// < prepare wall PCL

    { // start scope for locking mutex     
        boost::recursive_mutex::scoped_lock locker(wall_pcl_mutex_);
        segment->p_wall_pcl = wall_pcl_->makeShared(); // necessary deep copy 
    } // end scope for locking mutex  
    
    // organize wall points
    segment->wall_kdtree.setInputCloud(segment->p_wall_pcl);

    /// < prepare intial traversability PCL (needed for computing the starting point ) and utilit_2d PCL
    segment->p_traversability_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
    segment->p_utility_2d_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
    
    //segment->crop_step = 0;
    segment->crop_step = PathPlannerManager::kCropBoxTakeAll;
    
    {
    boost::recursive_mutex::scoped_lock locker_traversability(traversability_pcl_mutex_); 
    boost::recursive_mutex::scoped_lock locker_utility(utility_2d_mutex_); 

    if (utility_2d_flag_)
    {
        PathPlannerManager::cropTwoPcls((PathPlannerManager::CropBoxMethod)segment->crop_step, 
                                         start, goal, traversability_pcl_, utility_2d_pcl_,
                                         segment->p_traversability_pcl, segment->p_utility_2d_pcl);
    }
    else
    {
        PathPlannerManager::cropPcl((PathPlannerManager::CropBoxMethod)segment->crop_step, 
                                     start, goal, 
                                     traversability_pcl_, segment->p_traversability_pcl);
    }
    
    }
    // organize cropped traversability points
    segment->traversability_kdtree.setInputCloud(segment->p_traversability_pcl);
    
 
    /// < compute starting point on the segment traversability map 
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNDistance(1);
    int found = segment->traversability_kdtree.nearestKSearch(start, 1, pointIdxNKNSearch, pointNKNDistance);
    if (found < 1)
    {
        ROS_WARN("%s %02d/%02d - cannot find a close starting node", segment->segment_task.name.c_str(), segment->segment_task.segment_id, segment->segment_task.segment_count);

        // allow queue manager to handle this path
        {
            boost::recursive_mutex::scoped_lock locker_status(segment->status_mutex);
            segment->status = STATUS_FAILURE;
        }
        return; /// < EXIT POINT 
    }

    /// < create the planner object and initialize it 
    boost::shared_ptr<PathPlanner> p_path_planner(new PathPlanner);
    segment->p_path_planner = p_path_planner;
    segment->p_path_planner->setCostFunctionType(cost_function_type_,lambda_trav_,lambda_utility_2d_,tau_exp_decay_);

    bool b_successful_planning = false;

    while ((segment->crop_step < PathPlannerManager::kNumCropBoxMethod) && (!b_successful_planning) && (!segment->b_aborted))
    {
        /// < first set input then the utility and the goal!
        p_path_planner->setInput(*segment->p_traversability_pcl, *segment->p_wall_pcl, segment->wall_kdtree, segment->traversability_kdtree, pointIdxNKNSearch[0]);
        if(utility_2d_flag_) p_path_planner->set2DUtility(*segment->p_utility_2d_pcl);
        p_path_planner->setGoal(goal);
        
#ifdef VERBOSE
        publishTestCropboxPcl(segment, task);
#endif

        /// < compute the path
        b_successful_planning = p_path_planner->planning(segment->path);
        if (!b_successful_planning)
        {
            ROS_INFO("%s %02d/%02d failed attempt %d", segment->segment_task.name.c_str(), segment->segment_task.segment_id, segment->segment_task.segment_count, segment->crop_step);

            segment->crop_step += 1;

            if (segment->crop_step < PathPlannerManager::kNumCropBoxMethod)
            {
                /// < generate a bigger traversability map and utility map
                segment->p_traversability_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
                segment->p_utility_2d_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
                {
                    boost::recursive_mutex::scoped_lock locker_traversability(traversability_pcl_mutex_); 
                    boost::recursive_mutex::scoped_lock locker_utility(utility_2d_mutex_); 

                    if (utility_2d_flag_)
                    {
                        PathPlannerManager::cropTwoPcls((PathPlannerManager::CropBoxMethod) segment->crop_step, 
                                                         start, goal, 
                                                         traversability_pcl_, utility_2d_pcl_,
                                                         segment->p_traversability_pcl, segment->p_utility_2d_pcl);
                    }
                    else
                    {
                        PathPlannerManager::cropPcl((PathPlannerManager::CropBoxMethod) segment->crop_step, 
                                                     start, goal, 
                                                     traversability_pcl_, segment->p_traversability_pcl);
                    }
                    
                    
                }
                
                // organize cropped traversability points
                segment->traversability_kdtree.setInputCloud(segment->p_traversability_pcl);
                
                int found = segment->traversability_kdtree.nearestKSearch(start, 1, pointIdxNKNSearch, pointNKNDistance);
                if (found < 1) 
                {
                    segment->b_aborted = true;
                    ROS_WARN("%s %02d/%02d - cannot find a close starting node - aborting", segment->segment_task.name.c_str(), segment->segment_task.segment_id, segment->segment_task.segment_count);
                }
            }
        }
    }


    if (b_successful_planning)
    {
        /// < cloud cropping visualization
#ifdef VERBOSE
        publishCropboxPcl(segment, task);
#endif

        // update header of the path message
        segment->path.header.stamp = segment->segment_task.header.stamp;
        segment->path.header.frame_id = segment->segment_task.header.frame_id;

        // allow queue manager to handle this path
        {
            boost::recursive_mutex::scoped_lock locker_status(segment->status_mutex);
            segment->status = STATUS_SUCCESS;
        }

        // planning information
        ROS_INFO("QueuePathPlanner::pathPlanningCallback() - %s %02d/%02d success, #attempts: %d", segment->segment_task.name.c_str(), segment->segment_task.segment_id, segment->segment_task.segment_count,segment->crop_step);
    }
    else
    {
        // allow queue manager to handle this path
        {
            boost::recursive_mutex::scoped_lock locker_status(segment->status_mutex);
            segment->status = STATUS_FAILURE;
        }


        // planning information
        ROS_WARN("QueuePathPlanner::pathPlanningCallback() - %s %02d/%02d failure", segment->segment_task.name.c_str(), segment->segment_task.segment_id, segment->segment_task.segment_count);
    }
}

void QueuePathPlanner::publishCropboxPcl(TaskSegmentPtr segment, TaskPtr task)
{
    boost::recursive_mutex::scoped_lock locker(task->cropbox_pcl_mutex);
    pcl::PointCloud<pcl::PointXYZI>& cropbox_pcl = task->cropbox_pcl;
    cropbox_pcl += *segment->p_traversability_pcl;

    // cloud cropping visualization
    sensor_msgs::PointCloud2 cropbox_msg;
    pcl::toROSMsg(cropbox_pcl, cropbox_msg);
    cropbox_msg.header.frame_id = reference_frame_;
    cropbox_msg.header.stamp = ros::Time::now();
    {
        boost::recursive_mutex::scoped_lock locker(cropbox_pub_mutex_);
        cropbox_pub_.publish(cropbox_msg);
    }
}

void QueuePathPlanner::publishTestCropboxPcl(TaskSegmentPtr segment, TaskPtr task)
{
    boost::recursive_mutex::scoped_lock locker(task->cropbox_pcl_mutex);
    pcl::PointCloud<pcl::PointXYZI>& cropbox_test_pcl = task->cropbox_test_pcl;
    cropbox_test_pcl = *segment->p_traversability_pcl;

    // cloud cropping visualization
    sensor_msgs::PointCloud2 cropbox_msg;
    pcl::toROSMsg(cropbox_test_pcl, cropbox_msg);
    cropbox_msg.header.frame_id = reference_frame_;
    cropbox_msg.header.stamp = ros::Time::now();
    {
        boost::recursive_mutex::scoped_lock locker(cropbox_pub_mutex_);
        cropbox_pub_.publish(cropbox_msg);
    }
}