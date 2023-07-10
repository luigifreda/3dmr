/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DPATROLLING. 
* This file is a VERY modified version of the corresponding file in patrolling_sim 
* http://wiki.ros.org/patrolling_sim see license below.
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> (La Sapienza University)
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

#ifndef PATROLLING_MARKER_CONTROLLER_H_
#define PATROLLING_MARKER_CONTROLLER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cstring>
#include <boost/thread/recursive_mutex.hpp>

#include <path_planner/Transform.h>

#include "ColorUtils.h"


class PatrollingMarkerController
{
    
    static const std::string kDefaultMakerName; 
    static const std::string kDefaultMakerDescription; 
    static const float kInitialVerticalOffset; 
    static const float kTextMessageHeight; 
    static const float kTextMessageVerticalOffset; 
    static const float kPoseUpdatePeriod; 
    
public:
    
    ros::NodeHandle nh_;
    
    visualization_msgs::InteractiveMarker int_marker_;
    ros::Publisher patrolling_pause_pub_;
    
    ros::Timer pose_timer_;    

    boost::recursive_mutex marker_server_mtx; 
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    interactive_markers::MenuHandler menu_handler_;

    std::string marker_name_; 
    std::string ugv_name_; 
    std::string robot_frame_id_; 
    std::string world_frame_id_; 
        
    Transform transform_robot_; 
    
public: 
    PatrollingMarkerController(const tf::Vector3& p0, 
                     const ros::NodeHandle& nh,
                     const std::string& ugv_name = "ugv1",
                     const std::string& robot_frame_id = "base_link",
                     const std::string& world_frame_id = "map",
                     const std::string& patrol_pause_topic_name = "/patrolling/task/pause", 
                     const std::string& int_marker_server_name = "patrolling_marker_controller",
                     const std::string& marker_name = kDefaultMakerName); 
    ~PatrollingMarkerController();
    
    visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker&, const Color& color = Colors::LightGrey()  );
    visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker&);
    void makeViewFacingMarker();
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    void reset();
    
    void setMarkerPosition(const geometry_msgs::Pose &pose); 
    void setMarkerColor(const Color& color, const std::string& text = std::string()); 
    
    void updateRobotPosition(const ros::TimerEvent& timer_msg);
    
};


#endif //MARKER_CONTROLLER
