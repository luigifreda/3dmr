/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>
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

#include "ExplorationMarkerController.h"


using namespace visualization_msgs;


const std::string ExplorationMarkerController::kDefaultMakerName = "expl_marker"; 
const std::string ExplorationMarkerController::kDefaultMakerDescription = "Exploration"; 

const float ExplorationMarkerController::kInitialVerticalOffset = 1.5; 
const float ExplorationMarkerController::kTextMessageHeight = 0.14; 
const float ExplorationMarkerController::kTextMessageVerticalOffset = 0.5; 

const float ExplorationMarkerController::kPoseUpdatePeriod = 10; 

ExplorationMarkerController::ExplorationMarkerController(const tf::Vector3& p0, 
                                   const ros::NodeHandle& nh, 
                                   const std::string& ugv_name,                                                         
                                   const std::string& robot_frame_id, 
                                   const std::string& world_frame_id,                                   
                                   const std::string& expl_pause_topic_name, 
                                   const std::string& int_marker_server_name, 
                                   const std::string& marker_name)
    :nh_(nh)
{
    ugv_name_ = ugv_name;
    robot_frame_id_ = robot_frame_id;
    world_frame_id_ = world_frame_id;
    transform_robot_.set(world_frame_id,robot_frame_id);
    
    //goal_pub_ = node.advertise<geometry_msgs::PoseStamped>("/goal_topic", 1);
    //expl_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(expl_topic_name, 1);
    expl_pause_pub_ = nh_.advertise<std_msgs::Bool>(expl_pause_topic_name, 1);
    
    //marker_server_.reset(new interactive_markers::InteractiveMarkerServer("marker_controller", "", false));
    marker_server_.reset(new interactive_markers::InteractiveMarkerServer(int_marker_server_name, "", false));
    
    menu_handler_.insert("Pause Exploration", boost::bind(&ExplorationMarkerController::processFeedback, this, _1));
    menu_handler_.insert("Restart Exploration", boost::bind(&ExplorationMarkerController::processFeedback, this, _1));
    //menu_handler_.insert("Select Goal", boost::bind(&MarkerController::processFeedback, this, _1));
        
    int_marker_.pose.position.x = p0.getX();
    int_marker_.pose.position.y = p0.getY();
    int_marker_.pose.position.z = p0.getZ() + kInitialVerticalOffset;
    
    marker_name_ = marker_name; 
    
    makeViewFacingMarker();
    
    
    pose_timer_ =  nh_.createTimer(ros::Duration(kPoseUpdatePeriod), &ExplorationMarkerController::updateRobotPosition, this); // the timer will automatically fire at startup   
}

ExplorationMarkerController::~ExplorationMarkerController()
{
    std::cout << "MarkerController::~MarkerController() - start " << std::endl; 
        
    //boost::recursive_mutex::scoped_lock locker(marker_server_mtx); 
    
   // remove all markers from the server
   //marker_server_->clear();
   //marker_server_->applyChanges();

    // reset the sever
    //marker_server_.reset();
    std::cout << "MarkerController::~MarkerController() - end " << std::endl;
}

void ExplorationMarkerController::reset()
{
    boost::recursive_mutex::scoped_lock locker(marker_server_mtx); 
        
    //marker_server_.reset();
}

Marker ExplorationMarkerController::makeBox(InteractiveMarker& msg, const Color& color)
{
    Marker marker;
    //marker.type = Marker::CUBE;
    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale * 0.4;
    marker.scale.y = msg.scale * 0.4;
    marker.scale.z = msg.scale * 0.4;
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;

    return marker;

}

InteractiveMarkerControl& ExplorationMarkerController::makeBoxControl(InteractiveMarker& msg)
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

void ExplorationMarkerController::makeViewFacingMarker()
{
    int_marker_.header.frame_id = "map";
    int_marker_.header.stamp = ros::Time::now();

//    int_marker_.pose.position.x = 0;
//    int_marker_.pose.position.y = 0;
//    int_marker_.pose.position.z = 0;
    int_marker_.scale = 1;

    int_marker_.name = marker_name_;
    int_marker_.description = ExplorationMarkerController::kDefaultMakerDescription + " " + ugv_name_;

    InteractiveMarkerControl control;
    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    //control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE; 
    control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
    control.independent_marker_orientation = true;
    control.name = "move";

    control.markers.push_back(makeBox(int_marker_));
    control.always_visible = true;
    int_marker_.controls.push_back(control);

    InteractiveMarkerControl control_menu;
    control_menu.interaction_mode = InteractiveMarkerControl::MENU;
    control_menu.name = "menu";
    //control.always_visible = false;
    int_marker_.controls.push_back(control_menu);

    {   
    boost::recursive_mutex::scoped_lock locker(marker_server_mtx); 
    marker_server_->insert(int_marker_);
    marker_server_->setCallback(int_marker_.name, boost::bind(&ExplorationMarkerController::processFeedback, this, _1));
    menu_handler_.apply(*marker_server_, int_marker_.name);
    marker_server_->applyChanges();
    }
}

void ExplorationMarkerController::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' " << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if (feedback->mouse_point_valid)
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                << ", " << feedback->mouse_point.y
                << ", " << feedback->mouse_point.z
                << " in frame " << feedback->header.frame_id;
    }

    switch (feedback->event_type)
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        ROS_INFO_STREAM(s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
        if (feedback->menu_entry_id == 1) // Pause Exporation
        {
            std_msgs::Bool msg;
            msg.data = 1; 
            expl_pause_pub_.publish(msg);
        }
        if (feedback->menu_entry_id == 2) // Restart Exporation
        {
            std_msgs::Bool msg;
            msg.data = 0; 
            expl_pause_pub_.publish(msg);
        }        
//        if (feedback->menu_entry_id == 2) // Select Goal
//        {
//            geometry_msgs::PoseStamped goal;
//            goal.header.frame_id = "map";
//            goal.header.stamp = feedback->header.stamp;
//            goal.pose = feedback->pose;
//            goal_pub_.publish(goal);
//        }        
        break;
        
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_INFO_STREAM(s.str() << ": pose changed"
                        << "\nposition = "
                        << feedback->pose.position.x
                        << ", " << feedback->pose.position.y
                        << ", " << feedback->pose.position.z
                        << "\norientation = "
                        << feedback->pose.orientation.w
                        << ", " << feedback->pose.orientation.x
                        << ", " << feedback->pose.orientation.y
                        << ", " << feedback->pose.orientation.z
                        << "\nframe: " << feedback->header.frame_id
                        << " time: " << feedback->header.stamp.sec << "sec, "
                        << feedback->header.stamp.nsec << " nsec");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
        break;
    }
    
    boost::recursive_mutex::scoped_lock locker(marker_server_mtx); 
    marker_server_->applyChanges();
}

void ExplorationMarkerController::setMarkerPosition(const geometry_msgs::Pose &pose)
{
    boost::recursive_mutex::scoped_lock locker(marker_server_mtx); 
        
    marker_server_->setPose(marker_name_,pose);
    marker_server_->applyChanges();
}

void ExplorationMarkerController::setMarkerColor(const Color& color, const std::string& text)
{
    //std::cout << "MarkerController::setMarkerColor() " << std::endl; 
    
    visualization_msgs::InteractiveMarker int_marker;

    {   
    boost::recursive_mutex::scoped_lock locker(marker_server_mtx); 
    // get the marker
    if (!marker_server_->get(marker_name_, int_marker))
    {
        ROS_ERROR("Interactive marker '%s' does not exist, but produces feedback!", marker_name_.c_str());
        return;
    }
    }
    // update the marker position
//    marker.pose.position.x = position.x;
//    marker.pose.position.y = position.y;    
//    marker.pose.position.z = position.z;
    
//    Marker marker = int_marker.controls[0].markers.back();
//    marker.color.r = color.r; 
//    marker.color.g = color.g; 
//    marker.color.b = color.b;
//    marker.color.a = color.a; 
    
    /// < NOTE: cannot succeed in changing int_marker description 
        
    int_marker.controls[0].markers.clear();// eliminate the old markers 
    int_marker.controls[0].markers.push_back(makeBox(int_marker,color)); // push the new marker 
    
    if(!text.empty())
    {
        Marker text_marker;
        text_marker.type = Marker::TEXT_VIEW_FACING;
        text_marker.text = text; 
        text_marker.scale.z = kTextMessageHeight;
        text_marker.pose.position.x = int_marker.controls[0].markers[0].pose.position.x; 
        text_marker.pose.position.y = int_marker.controls[0].markers[0].pose.position.y; 
        text_marker.pose.position.z = int_marker.controls[0].markers[0].pose.position.z + kTextMessageVerticalOffset; 
        text_marker.color.a = 1;
        text_marker.color.r = 1;
        text_marker.color.g = 1;
        text_marker.color.b = 1;
        int_marker.controls[0].markers.push_back(text_marker);
    }
    
    {   
    boost::recursive_mutex::scoped_lock locker(marker_server_mtx); 
    // insert (update) the marker on the server
    marker_server_->insert(int_marker);
    marker_server_->applyChanges();
    }
}


void ExplorationMarkerController::updateRobotPosition(const ros::TimerEvent& timer_msg)
{
    tf::StampedTransform robot_pose;
    bool is_ok_transform = false;
    try
    {
        robot_pose = transform_robot_.get();
        is_ok_transform = transform_robot_.isOk();

    }
    catch (TransformException e)
    {
        ROS_WARN("%s", e.what());
    }
    
    if(is_ok_transform)
    {
        int_marker_.pose.position.x = robot_pose.getOrigin().getX();
        int_marker_.pose.position.y = robot_pose.getOrigin().getY();
        int_marker_.pose.position.z = robot_pose.getOrigin().getZ() + kInitialVerticalOffset;
        setMarkerPosition(int_marker_.pose);
    }
}