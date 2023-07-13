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

class QueueTrajectoryController
{
public:
    ~QueueTrajectoryController(void);
    QueueTrajectoryController(void);

    void feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg);
    void taskCallback(const nav_msgs::Path& path_msg);
    void coreCallback(void);

protected:
    ros::NodeHandle node_;
    ros::Publisher task_feedback_pub_; // to say "hey, I'm ready!"
    ros::Subscriber task_feedback_sub_; // to know when to stop
    ros::Subscriber task_sub_; // path to follow

    bool core_busy_flag_;
    boost::mutex core_mutex_;
    boost::thread core_thread_;
};

QueueTrajectoryController::~QueueTrajectoryController(void)
{
}

QueueTrajectoryController::QueueTrajectoryController(void)
{
    // feedback between nodes: "tool", "planner" and "control"
    task_feedback_pub_ = node_.advertise<trajectory_control_msgs::PlanningFeedback>("/planner/tasks/feedback", 1);
    task_feedback_sub_ = node_.subscribe("/planner/tasks/feedback", 20, &QueueTrajectoryController::feedbackCallback, this);

    // get the task's paths
    task_sub_ = node_.subscribe("/planner/tasks/path", 1, &QueueTrajectoryController::taskCallback, this);

    // this code is used for the simulation of the controller
    core_busy_flag_ = false;
}

void QueueTrajectoryController::feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg)
{
    // if this message have been published by the tool node, then
    if (!feedback_msg.node.compare("tool"))
    {
        trajectory_control_msgs::PlanningFeedback feedback_msg;
        feedback_msg.header.stamp = ros::Time::now();
        feedback_msg.node = "control";
        feedback_msg.task = "stop";

        // the whole code bellow is used for the simulation of the controller
        if (core_busy_flag_ == false)
        {
            ROS_WARN("controller already stopped");
            feedback_msg.status = STATUS_FAILURE;
        }
        else
        {
            core_mutex_.lock();
            {
                core_busy_flag_ = false;
            }
            core_mutex_.unlock();

            core_thread_.join();

#ifdef VERBOSE
            ROS_INFO("controller stopped (ready)");
#endif
            feedback_msg.status = STATUS_SUCCESS;
        }

        task_feedback_pub_.publish(feedback_msg);
    }
}

void QueueTrajectoryController::taskCallback(const nav_msgs::Path& path_msg)
{
    trajectory_control_msgs::PlanningFeedback feedback_msg;
    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.node = "control";
    feedback_msg.task = "start";

    if (core_busy_flag_ == true)
    {
        ROS_WARN("controller already started, task dropped");
        feedback_msg.status = STATUS_FAILURE;
    }
    else
    {
        core_busy_flag_ = true;
        core_thread_ = boost::thread(&QueueTrajectoryController::coreCallback, this);

#ifdef VERBOSE
        ROS_INFO("controller started (busy)");
#endif
        feedback_msg.status = STATUS_SUCCESS;
    }

    task_feedback_pub_.publish(feedback_msg);
}

void QueueTrajectoryController::coreCallback(void)
{
    trajectory_control_msgs::PlanningFeedback feedback_msg;
    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.node = "control";
    feedback_msg.task = "done";

    bool flag;
    for (int i = 0; 100 > i; i++)
    {
        ros::Duration(0.1).sleep();

        core_mutex_.lock();
        {
            flag = core_busy_flag_;
        }
        core_mutex_.unlock();

        if (!flag)
        {
            break;
        }
    }

    if (!flag)
    {
        ROS_WARN("controller aborted (by user)");
        feedback_msg.status = STATUS_FAILURE;
    }
    else
    {
        core_mutex_.lock();
        {
            core_busy_flag_ = false;
        }
        core_mutex_.unlock();

#ifdef VERBOSE
        ROS_INFO("controller finished");
#endif
        feedback_msg.status = STATUS_SUCCESS;
    }

    task_feedback_pub_.publish(feedback_msg);
}

int main(int argc, char *argv[])
{
    // ros initialization
    ros::init(argc, argv, "queue_trajectory_controller");

    // queue planner
    QueueTrajectoryController controller;

    // ros loop
    ros::spin();

    return 0;
}
