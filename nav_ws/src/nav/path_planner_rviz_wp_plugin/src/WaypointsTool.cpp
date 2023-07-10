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

#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>
#include <rviz/window_manager_interface.h>
#include <rviz/panel_dock_widget.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <limits>  

#include <OgreCamera.h>

//#include <QShortcut>

#include <patrolling_build_graph_msgs/PatrollingPoints.h> 
#include <patrolling_build_graph_msgs/PriorityPoint.h>

#include <wireless_network_msgs/RequestRSS_Load_PC.h>
#include <wireless_network_msgs/RequestRSS_Save_PC.h>

#include "WaypointsTool.h"
#include "AdvancedWidgetUI.h"
#include "PriorityDialogUI.h"


//#define WAYPOINT_VERBOSE 1 

/* Tell pluginlib about the class.  It is important
 * to do this in global scope, outside our package's namespace. */

#ifndef DEFINING_ANOTHER_PLUGIN
    #include <pluginlib/class_list_macros.h>
    #if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    #include <qt4/QtGui/qwidget.h>
    #else
    #include <qt5/QtWidgets/qwidget.h>
    #endif 
    PLUGINLIB_EXPORT_CLASS(path_planner_rviz_wp_plugin::WaypointsTool, rviz::Tool)
#endif


namespace path_planner_rviz_wp_plugin   
{
        
        
        
static Ogre::Vector2 project3DPointToViewportXYwithDepth(const Ogre::Viewport* view, const Ogre::Vector3& pos, float& depth)
{
  Ogre::Camera* cam = view->getCamera();
  
  Ogre::Vector3 pos3D_cam = cam->getViewMatrix() * pos;
  Ogre::Vector3 pos2D = cam->getProjectionMatrix() * pos3D_cam;
  
  depth = -pos3D_cam.z; // the depth axis 

  Ogre::Real x = ((pos2D.x * 0.5) + 0.5);
  Ogre::Real y = 1 - ((pos2D.y * 0.5) + 0.5);

  return  Ogre::Vector2(x * view->getActualWidth(), y * view->getActualHeight());
}

/* BEGIN_TUTORIAL
 * Construction and destruction
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * The constructor must have no arguments, so we can't give the
 * constructor the parameters it needs to fully initialize.
 *
 * Here we set the "shortcut_key_" member variable defined in the
 * superclass to declare which key will activate the tool. */
WaypointsTool::WaypointsTool(
    const std::string& pcl_input_topic_name,
    const std::string& planner_task_feedback_topic_name,
    const std::string& planner_task_append_topic_name,
    const std::string& planner_task_remove_topic_name,
    const std::string& planner_waypoints_server_name,
    const std::string& patrolling_task_append_topic_name,
    const std::string& patrolling_task_send_topic_name,
    const std::string& patrolling_task_pause_topic_name,
    const std::string& robot_frame_name,
    const std::string& world_frame_name,
    const std::string& robot_name 
)
{
    b_added_new_node_ = false; 
    
    shortcut_key_ = 'm';
    
    pAdvancedWidget_ = 0;
    pPriorityDialog_ = 0;
    
    bInitManagers_ = false;
    bAddedAdvancedWidgetPanel_ = false;

    /// < qt connections 
    
    
    // planning text messages to be visualized in RVIZ
    planning_messages_pub_ = node_.advertise<visualization_msgs::MarkerArray>(NAME_PLANNING_MESSAGES_TOPIC,10);
            
    // point cloud update
    // use only one of the two following callbacks: once the first message is received on a topic then the other callback/subscriber is disabled!
    pcl_sub_ = node_.subscribe(pcl_input_topic_name, 1, &WaypointsTool::cloudCallback, this);
    pcl_normals_sub_ = node_.subscribe(pcl_input_topic_name+"_normals", 1, &WaypointsTool::cloudNormalsCallback, this);    

    //check_pcl_topics_timer_ = node_.createTimer(ros::Duration(5), &WaypointsTool::checkPclTopicsCallback, this);    
    
    // patrolling task, send and pause/restart  
    patrolling_points_task_pub_ = node_.advertise<patrolling_build_graph_msgs::PatrollingPoints>(patrolling_task_append_topic_name, 10);
    patrolling_send_pub_  = node_.advertise<std_msgs::Bool>(patrolling_task_send_topic_name, 10);
    patrolling_pause_pub_ = node_.advertise<std_msgs::Bool>(patrolling_task_pause_topic_name, 10);
    
    // exploration fence 
    exploration_bb_pub_    = node_.advertise<nav_msgs::Path>(NAME_EXPLORATION_FENCE_TOPIC, 1);
    exploration_pause_pub_ = node_.advertise<std_msgs::Bool>(NAME_EXPLORATION_PAUSE_TOPIC, 10);
    
    // single priority publisher 
    priority_point_pub_ =  node_.advertise<patrolling_build_graph_msgs::PriorityPoint>(NAME_PRIORITY_POINT_TOPIC, 10);
    
    // exploration priority point 
    exploration_priority_point_pub_ =  node_.advertise<exploration_msgs::ExplorationPriorityPoint>(NAME_EXPL_PRIORITY_POINT_TOPIC, 10);    
    
    // exploration priority point 
    exploration_priority_actions_pub_ = node_.advertise<exploration_msgs::ExplorationPriorityActions>(NAME_EXPL_PRIORITY_ACTIONS_TOPIC, 10);  
    exploration_priority_actions_sub_ = node_.subscribe(NAME_EXPL_PRIORITY_ACTIONS_TOPIC, 1, &WaypointsTool::explorationPriorityActionsCallback, this);    

    // planner/controller feedback
    task_feedback_pub_ = node_.advertise<trajectory_control_msgs::PlanningFeedback>(planner_task_feedback_topic_name, 10);
    task_feedback_sub_ = node_.subscribe(planner_task_feedback_topic_name, 10, &WaypointsTool::feedbackCallback, this);
    
    // publisher for the enabling/disabling velocity reduction depending on closer obstacle point  
    closest_obst_vel_reduction_enable_pub_ = node_.advertise<std_msgs::Bool>(NAME_CLOSEST_OBS_VEL_REDUCTION_ENABLE_TOPIC, 1);

    // publisher for the enabling/disabling rss map usage in the path planner  
    rss_enable_pub_ = node_.advertise<std_msgs::Bool>(NAME_RSS_ENABLE_TOPIC, 1);
    
    // publisher for setting the minimum critical rss value in the path planner 
    rss_min_signal_value_pub_ = node_.advertise<std_msgs::Int32>(NAME_RSS_MIN_VALUE_TOPIC, 1);
    
    rviz_message_string_sub_ = node_.subscribe(NAME_MESSAGE_STRING_TOPIC, 10, &WaypointsTool::rvizMessageStringCallback, this);
    
    // failures cleanup
    failures_cleanup_timer_ = node_.createTimer(ros::Duration(5), &WaypointsTool::failureCleanup, this);

    // internal counters
    planner_count_      = 1; // handles planning task ordering
    markers_count_      = 1; // handles markers naming
    text_markers_count_ = 1; 
    
    // markers frame 
    world_frame_id_ = world_frame_name;
    robot_frame_id_ = robot_frame_name;
    
    robot_name_ = robot_name;
    
    // publisher
    task_append_pub_ = node_.advertise<trajectory_control_msgs::PlanningTask>(planner_task_append_topic_name, 1);
    task_remove_pub_ = node_.advertise<trajectory_control_msgs::PlanningTask>(planner_task_remove_topic_name, 1);

    
    // right-click menu for 'interactive' markers (those orange)
    markers_interactive_menu_.insert("Append task", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert("Append cyclic task", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
        
    markers_interactive_menu_.insert("Remove this waypoint", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert("Remove all waypoints", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert("Set Priority", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    
    interactive_markers::MenuHandler::EntryHandle patrolling_sub_menu_handle = markers_interactive_menu_.insert( "Patrolling" ); /// < 6
    markers_interactive_menu_.insert(patrolling_sub_menu_handle,"Append task", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert(patrolling_sub_menu_handle,"Send task", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert(patrolling_sub_menu_handle,"Remove task", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert(patrolling_sub_menu_handle,"Pause", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert(patrolling_sub_menu_handle,"Restart", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));    
            
    interactive_markers::MenuHandler::EntryHandle exploration_sub_menu_handle = markers_interactive_menu_.insert( "Exploration" );/// < 12
    markers_interactive_menu_.insert(exploration_sub_menu_handle,"Append priority point", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert(exploration_sub_menu_handle,"Set BB", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert(exploration_sub_menu_handle,"Pause", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    markers_interactive_menu_.insert(exploration_sub_menu_handle,"Restart", boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));  
    
    
    // right-click menu for 'static' markers (those gray)
    markers_static_menu_.insert("Remove this task", boost::bind(&WaypointsTool::markerStaticCallback, this, _1));
    markers_static_menu_.insert("Remove all tasks", boost::bind(&WaypointsTool::markerStaticCallback, this, _1));

    
    // right-click menu for 'success' markers (those green)
    markers_success_menu_.insert("Stop the controller", boost::bind(&WaypointsTool::markerSuccessCallback, this, _1));
    markers_success_menu_.insert("Stop the controller and remove all tasks", boost::bind(&WaypointsTool::markerSuccessCallback, this, _1));

    
    // marker's server initialization
    markers_server_.reset(new interactive_markers::InteractiveMarkerServer(planner_waypoints_server_name, "", false));
    markers_server_->clear();
    markers_server_->applyChanges();

    // load mesh for 'interactive' markers
    resource_interactive_ = "package://path_planner_rviz_wp_plugin/mesh/barrier_interactive.dae";
    if (rviz::loadMeshFromResource(resource_interactive_).isNull())
    {
        ROS_ERROR("WaypointsTool: failed to load model resource '%s'.", resource_interactive_.c_str());
        return;
    }

    // load mesh for 'static' markers with failures
    resource_failure_ = "package://path_planner_rviz_wp_plugin/mesh/barrier_failure.dae";
    if (rviz::loadMeshFromResource(resource_failure_).isNull())
    {
        ROS_ERROR("WaypointsTool: failed to load model resource '%s'.", resource_failure_.c_str());
        return;
    }

    // load mesh for 'static' markers with no failures
    resource_success_ = "package://path_planner_rviz_wp_plugin/mesh/barrier_success.dae";
    if (rviz::loadMeshFromResource(resource_success_).isNull())
    {
        ROS_ERROR("WaypointsTool: failed to load model resource '%s'.", resource_success_.c_str());
        return;
    }

    // load mesh for 'static' markers without status feedback
    resource_static_ = "package://path_planner_rviz_wp_plugin/mesh/barrier_static.dae";
    if (rviz::loadMeshFromResource(resource_static_).isNull())
    {
        ROS_ERROR("WaypointsTool: failed to load model resource '%s'.", resource_static_.c_str());
        return;
    }
    
}

WaypointsTool::~WaypointsTool()
{
    // remove all markers from the server
    markers_server_->clear();
    markers_server_->applyChanges();

    // reset the sever
    markers_server_.reset();
}

// Use only one of the two following callback: once the first message is received the other callback/subscriber is disabled!
// NOTE: probably this is not the most elegant way, however this does NOT have any time wait for checking incoming message and so no impact on the GUI 
void WaypointsTool::cloudCallback(const PointCloud::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker(pcd_mtx_);

    // get last point cloud
    pcl::copyPointCloud(*msg, pcd_);

    if (pcl_normals_sub_.getNumPublishers() >= 0)  pcl_normals_sub_.shutdown();   // disconnect the other similar subscriber  
}
void WaypointsTool::cloudNormalsCallback(const PointCloud::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker(pcd_mtx_);

    // get last point cloud
    pcl::copyPointCloud(*msg, pcd_);

    if (pcl_sub_.getNumPublishers() >= 0)  pcl_sub_.shutdown();  // disconnect the other similar subscriber  
}

/* Handling mouse events
 * ^^^^^^^^^^^^^^^^^^^^^
 *
 * processMouseEvent() is sort of the main function of a Tool, because
 * mouse interactions are the point of Tools.
 *
 * We use the utility function rviz::project3DPointToViewportXY(...) to
 * project the points of the cloud  */
int WaypointsTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    // update viewport for projection of markers over the point cloud
    // (I believe that this should be done only once)
    viewport_ = event.viewport;

    // if the user pressed the left button of the mouse/touchpad, then
    if (event.leftDown())
    {

        bool ok = false;
        float depth = 0; 
            
        // find point (of the cloud) with the nearest projection to the pointer
        double distance = std::numeric_limits<double>::max();
        Ogre::Vector3 marker_position(0, 0, 0);

        { // start locking scope 
            boost::recursive_mutex::scoped_lock locker(pcd_mtx_);

            for (size_t i = 0; i < pcd_.size(); i++)  
            {
                Ogre::Vector3 point3D(pcd_.points[i].x, pcd_.points[i].y, pcd_.points[i].z);
                
                //Ogre::Vector2 point2D = rviz::project3DPointToViewportXY(viewport_, point3D);
                Ogre::Vector2 point2D = project3DPointToViewportXYwithDepth(viewport_, point3D, depth);

                if(depth > 0)
                {
                    ok = true; 
                    double d = sqrt(pow(event.x - point2D.x, 2) + pow(event.y - point2D.y, 2));
                    if (distance > d)
                    {
                        marker_position = point3D;
                        distance = d;
                    }
                }
                /*else
                {
                    ROS_INFO_STREAM("WaypointsTool::processMouseEvent() - depth: " << depth << std::endl); 
                }*/
            }

        }// end locking scope

        
        if(ok)
        {
            // create the marker at the above computed position
            markerAdd(marker_position);
        }

        return Render | Finished;
    }

    return Render;

}

int WaypointsTool::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )  
{
    int res = 0; 
    
    switch (event->key())
    {
      case Qt::Key_Q:
        patrollingTaskPause();
        explorationTaskPause();  
        markerAddRobotTextMsg("Patrolling/Exploration Pause");
        res= 1;
        break;
        
      case Qt::Key_W:
        patrollingTaskRestart();
        explorationTaskRestart();       
        markerAddRobotTextMsg("Patrolling/Exploration Restarted");
        res= 1;
        break;       
        
      case Qt::Key_H:
          OpenAdvancedWidget();
        break;           
        
    default:
        ; // NOP
    }
    
    return res; 
}

// add a new marker 
void WaypointsTool::markerAdd(const Ogre::Vector3& position, MarkerType marker_type, const std::string& description)
{
    visualization_msgs::InteractiveMarker marker;

    // header
    marker.header.frame_id = world_frame_id_;
    marker.header.stamp = ros::Time::now();

    // position
    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
    marker.scale = 1;
    
    // name and description
    std::ostringstream ss;
    ss << markers_count_;
    marker.name = ss.str();
    marker.description = ss.str();
    if(!description.empty()) marker.description = description; 
    
    {
    boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);           
    idMarkerMap_[marker.name] = markers_count_;
    priorityMarkerMap_[marker.name] = 1; // default priority 
    }
    
    markers_count_++; /// < increase marker count 
    
    // mesh resource
    visualization_msgs::Marker shape;
    shape.type = visualization_msgs::Marker::MESH_RESOURCE;
    switch(marker_type)
    {
    case kStatic:
        shape.mesh_resource = resource_static_;       
        break; 
    case kSuccess:
        shape.mesh_resource = resource_success_;       
        break; 
    case kFailure:
        shape.mesh_resource = resource_failure_;       
        break;     
    case kInteractive:
    default:
        shape.mesh_resource = resource_interactive_;
    }
    shape.mesh_use_embedded_materials = true;
    
    
    // interaction control
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    //control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    
    switch(marker_type)
    {
    case kStatic:
    case kSuccess:
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;   
        break;
    case kFailure:
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;       
        break;     
    case kInteractive:
    default:
        //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    }
    
    
    control.markers.push_back(shape);
    marker.controls.push_back(control);

    // insert marker on the server
    markers_server_->insert(marker);
    markers_server_->setCallback(marker.name, boost::bind(&WaypointsTool::markerInteractiveCallback, this, _1));
    
    switch(marker_type)
    {
    case kSuccess:
        markers_success_menu_.apply(*markers_server_, marker.name); 
        break;
    case kStatic:
    case kFailure:
        markers_static_menu_.apply(*markers_server_, marker.name);       
        break;     
    case kInteractive:
    default:
        markers_interactive_menu_.apply(*markers_server_, marker.name);
    }
    
    markers_server_->applyChanges();
    
    
#ifdef WAYPOINT_VERBOSE
        ROS_INFO_STREAM("marker " << marker.name << " added");
#endif
        
    b_added_new_node_ = true; 
    
    OpenAdvancedWidget();
}

// update marker position 
void WaypointsTool::markerUpdate(const std::string& name, const geometry_msgs::Point& reference)
{
    visualization_msgs::InteractiveMarker marker;

    // get the marker
    if (!markers_server_->get(name, marker))
    {
        ROS_ERROR("Interactive marker '%s' does not exist, but produces feedback!", name.c_str());
        return;
    }

    // if the marker is static, then do nothing
    if (marker.controls[0].interaction_mode == visualization_msgs::InteractiveMarkerControl::MENU)
    {
        return;
    }
    

    // marker update is also called everytime we add a new node (we avoid here this useless operation)
    if(b_added_new_node_)
    {
        b_added_new_node_ = false;
        return; 
    }

    // obtain pointer coordinates
    Ogre::Vector3 position = Ogre::Vector3(reference.x, reference.y, reference.z);
    Ogre::Vector2 pointer = rviz::project3DPointToViewportXY(viewport_, position);

    bool  ok = false; 
    
    { // start locking scope 
        boost::recursive_mutex::scoped_lock locker(pcd_mtx_);

        float depth = 0; 
        
        // find point (of the cloud) with the nearest projection of the pointer
        double distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < pcd_.size(); i++)
        {
            Ogre::Vector3 point3D(pcd_.points[i].x, pcd_.points[i].y, pcd_.points[i].z);
            //Ogre::Vector2 p = rviz::project3DPointToViewportXY(viewport_, P);
            Ogre::Vector2 point2D = project3DPointToViewportXYwithDepth(viewport_, point3D, depth);

            if(depth > 0)
            {
                ok = true; 
                double d = sqrt(pow(pointer.x - point2D.x, 2) + pow(pointer.y - point2D.y, 2));
                if (distance > d)
                {
                    position = point3D;
                    distance = d;
                }
            }
        }

    } // end locking scope 

    if(ok)
    {
        // update the marker position
        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z;

        // insert (update) the marker on the server
        markers_server_->insert(marker);
        markers_server_->applyChanges();
        
        if(pPriorityDialog_)
        {
            pPriorityDialog_->SetXYZ(position.x,position.y,position.z);
        }
    }
}


void WaypointsTool::markerAddText(const Ogre::Vector3& position, const std::string& text, const double duration)
{
    boost::recursive_mutex::scoped_lock locker(text_markers_array_mtx_);
            
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = world_frame_id_;
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "text_messages";
    text_marker.id = text_markers_count_++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = position.x;
    text_marker.pose.position.y = position.y;
    text_marker.pose.position.z = position.z;
    text_marker.pose.orientation.x = 0.0;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0; // Don't forget to set the alpha!
    text_marker.scale.x = 1;
    text_marker.scale.y = 1;
    text_marker.scale.z = 0.2;
    //text_marker.lifetime = ros::Duration(kTextMessagesLifeDuration);
    text_marker.lifetime = ros::Duration(duration); 
    text_marker.text = text.c_str();

    text_markers_array_.markers.clear(); 
    text_markers_array_.markers.push_back(text_marker);
    planning_messages_pub_.publish(text_markers_array_);
}


void WaypointsTool::markerAddRobotTextMsg(const std::string& msg, const double duration)
{
    Ogre::Vector3 position;
    if (getRobotPosition(position))
    {
        position.z += kTextMessageHeightAboveRobot;
        ROS_INFO_STREAM("adding robot text message: " << msg);
        markerAddText(position, msg, duration);
    }
}

// clear all interactive markers 
void WaypointsTool::markerClearAll(void)
{
    visualization_msgs::InteractiveMarker marker;

    // for each counted marker (or waypoint)
    for (unsigned long i = 1; markers_count_ > i; i++)
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss.str(), marker))
        {
            // verify that the marker is 'interactive'
            // (that is, has not been published in some planning task)
            if (!marker.controls[0].markers[0].mesh_resource.compare(resource_interactive_))
            {
                // in this case, erase (remove) marker from the server
//                markers_server_->erase(marker.name);
//                
//                boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);                   
//                idMarkerMap_.erase(marker.name);
//                priorityMarkerMap_.erase(marker.name);
                markerErase(marker.name);
            }
            
            /*if(marker.controls[0].markers[0].type = visualization_msgs::Marker::TEXT_VIEW_FACING)
            {
                // in this case, erase (remove) marker from the server
                markers_server_->erase(marker.name);
                priorityMarkerMap_.erase(marker.name);
            }*/
        }
    }

    // update markers states
    markers_server_->applyChanges();  
}

// N.B.: after calling this function you have to apply changes by using markers_server_->applyChanges()
void WaypointsTool::markerErase(const std::string marker_name)
{
    // in this case, erase (remove) marker from the server
    markers_server_->erase(marker_name);

    boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);                   
    idMarkerMap_.erase(marker_name);
    priorityMarkerMap_.erase(marker_name);    
}

// append a new task by pushing all the current interactive markers 
void WaypointsTool::taskAppend(const ros::Time& stamp, uint8_t task_type)
{
    int segment_count = 0;
    geometry_msgs::Pose marker_msg;
    trajectory_control_msgs::PlanningTask task_msg;
    visualization_msgs::InteractiveMarker marker;

    std::ostringstream ss;
    ss << "Task" << robot_name_ << " "; 
    if(task_type == kPathCyclic) ss << "Cyclic "; 
    ss << planner_count_++;
    task_msg.name = ss.str();
    task_msg.segment_id = -1;
    task_msg.type = task_type; 
    task_msg.header.stamp = stamp;
    task_msg.header.frame_id = world_frame_id_;
    
    // for each counted marker (or waypoint)
    for (unsigned long i = 1; markers_count_ > i; i++)
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss.str(), marker))
        {
            // verify that the marker does not belong to another task
            if (!marker.controls[0].markers[0].mesh_resource.compare(resource_interactive_))
            {
                // add the waypoint (marker position) to the task message
                task_msg.waypoints.push_back(marker.pose.position);

                // convert to a 'static' marker one
                marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
                marker.controls[0].markers[0].mesh_resource = resource_static_;

                // change marker description to identify the planning task
                // (which is based on the planner_count_)
                marker.description = task_msg.name;

                // insert (update) marker in the server
                markers_server_->insert(marker);
                markers_static_menu_.apply(*markers_server_, marker.name);

                // count number of waypoints inside the task
                segment_count++;
            }
        }
    }
    
    //if the path is cyclic add a new marker at the robot position 
    if(task_type == kPathCyclic) 
    {
        Ogre::Vector3 position;
        if(getRobotPosition(position))
        {
            markerAdd(position,kStatic,task_msg.name);
        }
    }

    // update markers states
    markers_server_->applyChanges();

    // append the planning task to the planner node queue
    task_msg.segment_count = segment_count;
    task_append_pub_.publish(task_msg);
}

void WaypointsTool::setPriority(const std::string& marker_name, const PriorityType val, const geometry_msgs::Point& point)
{
    boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);    
    priorityMarkerMap_[marker_name] = val;
    
    patrolling_build_graph_msgs::PriorityPoint msg;
    msg.id       = idMarkerMap_[marker_name]; // the id 
    msg.priority = priorityMarkerMap_[marker_name]; // the priority 
    msg.position = point; 
    
    priority_point_pub_.publish(msg);
}


void WaypointsTool::SetClosestObstVelReductionEnable(bool val)
{
    std_msgs::Bool msg;
    msg.data = val; 
    closest_obst_vel_reduction_enable_pub_.publish(msg);
}

void WaypointsTool::SetRssEnable(bool val)
{
    std_msgs::Bool msg;
    msg.data = val; 
    rss_enable_pub_.publish(msg);
}

void WaypointsTool::RssLoad()
{
    wireless_network_msgs::RequestRSS_Load_PC service;
            
    static std::string ss(std::string(NAME_RSS_SERVICE_LOAD));    
    // NOTE: keep "/" as a prefix since these behave like absolute topic names
    static std::string ss_ugv1(std::string("/ugv1") + std::string(NAME_RSS_SERVICE_LOAD));
    static std::string ss_ugv2(std::string("/ugv2") + std::string(NAME_RSS_SERVICE_LOAD));
    static std::string ss_ugv3(std::string("/ugv3") + std::string(NAME_RSS_SERVICE_LOAD));
    
    static std::vector<std::string> ss_array = {ss, ss_ugv1, ss_ugv2, ss_ugv3};
    
    for(size_t ii=0; ii<ss_array.size(); ii++)
    {    
        if(ros::service::exists(ss_array[ii],true))
        {
            if(ros::service::call(ss_array[ii], service))
            {
                markerAddRobotTextMsg("RSS map loaded");
                ROS_INFO_STREAM("WaypointsTool::RssLoad() - on " << ss_array[ii] );
            }
        }     
    }
}

void WaypointsTool::RssSave()
{
    boost::recursive_mutex::scoped_lock locker(pcd_mtx_);
    if(pcd_.empty()) return; 
        
    wireless_network_msgs::RequestRSS_Save_PC service;
    
    sensor_msgs::PointCloud2 pc_msg; 
    pcl::toROSMsg(pcd_, pc_msg);
    service.request.surveypoints = pc_msg;
    
    static std::string ss(std::string(NAME_RSS_SERVICE_SAVE));
    // NOTE: keep "/" as a prefix since these behave like absolute topic names 
    static std::string ss_ugv1(std::string("/ugv1") + std::string(NAME_RSS_SERVICE_SAVE));
    static std::string ss_ugv2(std::string("/ugv2") + std::string(NAME_RSS_SERVICE_SAVE));
    static std::string ss_ugv3(std::string("/ugv3") + std::string(NAME_RSS_SERVICE_SAVE));
    
    static std::vector<std::string> ss_array = {ss, ss_ugv1, ss_ugv2, ss_ugv3};
    
    for(size_t ii=0; ii<ss_array.size(); ii++)
    {           
        if(ros::service::exists(ss_array[ii],true))
        {
            if(ros::service::call(ss_array[ii], service))
            {
                markerAddRobotTextMsg("RSS map saved");
                ROS_INFO_STREAM("WaypointsTool::RssSave() - on " << ss_array[ii] << ", results: " << service.response.status );
            }
        } 
    }
}

void WaypointsTool::SetMinRssValueEnable(int val)
{
    std_msgs::Int32 msg;
    msg.data = val; 
    rss_min_signal_value_pub_.publish(msg);
}


// append a new patrolling task by pushing all the current interactive markers 
void WaypointsTool::patrollingTaskAppend(const ros::Time& stamp)
{
    patrolling_build_graph_msgs::PatrollingPoints patrolling_points_msg;
    patrolling_points_msg.header.frame_id = world_frame_id_;
    //geometry_msgs::PoseStamped patrolling_pose; // for pushing points in the navigation msg
    //patrolling_pose.header.frame_id = world_frame_id_;
    
    geometry_msgs::Point patrolling_point;
    
    int segment_count = 0;
    geometry_msgs::Pose marker_msg;
    trajectory_control_msgs::PlanningTask task_msg;
    
    visualization_msgs::InteractiveMarker marker;

    std::ostringstream ss;
    ss << "Patrolling Task" << planner_count_++;
    task_msg.name = ss.str();
    task_msg.segment_id = -1;
    task_msg.type = kPathNormal; 
    task_msg.header.stamp = stamp;
    task_msg.header.frame_id = world_frame_id_;
    
    patrolling_points_msg.num_nodes = 0;
    
    // for each counted marker (or waypoint)
    for (unsigned long i = 1; i < markers_count_; i++)
    {
        std::ostringstream ss_id;
        ss_id << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss_id.str(), marker))
        {
            // verify that the marker does not belong to another task
            if (!marker.controls[0].markers[0].mesh_resource.compare(resource_interactive_))
            {
                // add the waypoint (marker position) to the task message
                task_msg.waypoints.push_back(marker.pose.position);
                
                patrolling_point.x = marker.pose.position.x;
                patrolling_point.y = marker.pose.position.y;
                patrolling_point.z = marker.pose.position.z;
                
                patrolling_points_msg.node_id.push_back(idMarkerMap_[marker.name]);
                patrolling_points_msg.node_position.push_back(patrolling_point);
                patrolling_points_msg.node_priority.push_back(priorityMarkerMap_[marker.name]); 
                patrolling_points_msg.num_nodes++;

                // convert to a 'static' marker one
                ///marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;  // <-- convert to static
                ///marker.controls[0].markers[0].mesh_resource = resource_static_; // <-- convert to static

                // change marker description to identify the planning task
                // (which is based on the planner_count_)
                std::ostringstream ss_id_1;
                //ss_id_1 << segment_count;
                ss_id_1 << idMarkerMap_[marker.name];
                marker.description = task_msg.name + " id " + ss_id_1.str();

                // insert (update) marker in the server
                markers_server_->insert(marker); 
                
                ///markers_static_menu_.apply(*markers_server_, marker.name); // <-- convert to static

                // count number of waypoints inside the task
                segment_count++;
            }
        }
    }
    
    if(patrolling_points_msg.num_nodes != patrolling_points_msg.node_id.size())
    {
        ROS_ERROR_STREAM("ERROR in WaypointsTool::patrollingTaskAppend(): number of nodes is incoherent");
        return; 
    }

    // update markers states
    markers_server_->applyChanges();

    // append the planning task to the planner node queue
    task_msg.segment_count = segment_count;
    //task_append_pub_.publish(task_msg);
    patrolling_points_task_pub_.publish(patrolling_points_msg);
    
}



void WaypointsTool::patrollingTaskSend()
{
    ROS_INFO_STREAM("Patrolling Send");
    
    //first resend just to be sure
//    ros::Time stamp = ros::Time::now(); 
//    patrollingTaskAppend(stamp);
//    ros::Duration(1.).sleep(); 
        
    // then actually send the GO
    std_msgs::Bool msg_send;
    msg_send.data = true;
    patrolling_send_pub_.publish(msg_send);
    
    markerAddRobotTextMsg("Patrolling Sending Task");
}

void WaypointsTool::patrollingTaskPause()
{
    ROS_INFO_STREAM("Patrolling Paused");
        
    std_msgs::Bool msg_pause;
    msg_pause.data = true;
    
    patrolling_pause_pub_.publish(msg_pause);

    //markerAddRobotTextMsg("Patrolling Paused");
}

void WaypointsTool::patrollingTaskRestart()
{
    ROS_INFO_STREAM("Patrolling Restarted");
    
    std_msgs::Bool msg_pause;
    msg_pause.data = false;
    
    patrolling_pause_pub_.publish(msg_pause); 
    
    //markerAddRobotTextMsg("Patrolling Restarted");
}


void WaypointsTool::explorationSendPriorityPoint(const std::string& marker_name, uint8_t action)
{
    ROS_INFO_STREAM("Exploration Send Priority Point");

    visualization_msgs::InteractiveMarker marker;    
    
    exploration_msgs::ExplorationPriorityPoint msg; 
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.action = action; 
        
    // find marker
    if (markers_server_->get(marker_name, marker))
    {   
        std::ostringstream ss;
        ss << idMarkerMap_[marker.name] << " (" << priorityMarkerMap_[marker.name] << ")";
        marker.description = "Exploration Priority Point " + ss.str();

        markers_server_->insert(marker); 

        msg.position.x = marker.pose.position.x;
        msg.position.y = marker.pose.position.y;
        msg.position.z = marker.pose.position.z;    

        msg.id       = idMarkerMap_[marker_name];
        msg.priority = priorityMarkerMap_[marker_name]; // the priority       
    }    
        
    exploration_priority_point_pub_.publish(msg);    
    markers_server_->applyChanges();
}

void WaypointsTool::explorationSendFence()
{
    ROS_INFO_STREAM("Exploration Set BB");    
    
    nav_msgs::Path exploration_fence_path;
    exploration_fence_path.header.frame_id = world_frame_id_;
    geometry_msgs::PoseStamped pose; // for pushing points in the navigation msg
    pose.header.frame_id = world_frame_id_;

    int segment_count = 0;
//    geometry_msgs::Pose marker_msg;
//    trajectory_control_msgs::PlanningTask task_msg;
    
    visualization_msgs::InteractiveMarker marker;

    std::ostringstream ss;
    ss << "Exploration Fence"; // << planner_count_++;
//    task_msg.name = ss.str();
//    task_msg.segment_id = -1;
//    task_msg.type = kPathNormal; 
//    task_msg.header.stamp = stamp;
//    task_msg.header.frame_id = world_frame_id_;
    
    // for each counted marker (or waypoint)
    for (unsigned long i = 1; i < markers_count_; i++)
    {
        std::ostringstream ss_id;
        ss_id << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss_id.str(), marker))
        {
            // verify that the marker does not belong to another task
            if (!marker.controls[0].markers[0].mesh_resource.compare(resource_interactive_))
            {
                // add the waypoint (marker position) to the task message
                //task_msg.waypoints.push_back(marker.pose.position);
                
                pose.pose.position.x = marker.pose.position.x;
                pose.pose.position.y = marker.pose.position.y;
                pose.pose.position.z = marker.pose.position.z;
                exploration_fence_path.poses.push_back(pose);

                // convert to a 'static' marker one
                ///marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;  // <-- convert to static
                ///marker.controls[0].markers[0].mesh_resource = resource_static_; // <-- convert to static

                // change marker description to identify the planning task
                // (which is based on the planner_count_)
                std::ostringstream ss_id_1;
                ss_id_1 << segment_count;
                //marker.description = task_msg.name + " id " + ss_id_1.str();
                marker.description = ss.str() + " id " + ss_id_1.str();

                // insert (update) marker in the server
                markers_server_->insert(marker); 
                
                ///markers_static_menu_.apply(*markers_server_, marker.name); // <-- convert to static

                // count number of waypoints inside the task
                segment_count++;
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();

//    // append the planning task to the planner node queue
//    task_msg.segment_count = segment_count;
//    //task_append_pub_.publish(task_msg);
    exploration_bb_pub_.publish(exploration_fence_path);    
    
    markerAddRobotTextMsg("Exploration Set BB");    
}



void WaypointsTool::explorationTaskPause()
{
    ROS_INFO_STREAM("Exploration Paused");
        
    std_msgs::Bool msg_pause;
    msg_pause.data = true;
    
    exploration_pause_pub_.publish(msg_pause);
    
    //markerAddRobotTextMsg("Exploration Paused");
}

void WaypointsTool::explorationTaskRestart()
{
    ROS_INFO_STREAM("Exploration Restarted");
    
    std_msgs::Bool msg_pause;
    msg_pause.data = false;
    
    exploration_pause_pub_.publish(msg_pause);    
    
    //markerAddRobotTextMsg("Exploration Restarted");
}


void WaypointsTool::explorationPriorityActionsCallback(const exploration_msgs::ExplorationPriorityActions::ConstPtr msg)
{
    ROS_INFO_STREAM("rviz explorationPriorityActionsCallback()");    
    
    if(msg->sender == exploration_msgs::ExplorationPriorityActions::kSenderGUI) return; /// < EXIT POINT 
    
    visualization_msgs::InteractiveMarker marker;
           
    for(int jj=0; jj< msg->num_points; jj++)
    {           
        const exploration_msgs::ExplorationPriorityPoint& priorityPoint = msg->priority_points[jj];
        std::ostringstream ss;
        ss << priorityPoint.id;
            
        if(priorityPoint.action == exploration_msgs::ExplorationPriorityPoint::kActionRemove)
        {
            // find the existing ones. if marker '#i' exists, then
            if (markers_server_->get(ss.str(), marker))
            {
                // remove the marker 
                markerErase(marker.name);
            }
            
        }
    }
    markers_server_->applyChanges();
    
    //markerAddRobotTextMsg("Removed"); 
}


void WaypointsTool::markerInteractiveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    static const int kPatrollingMenuOffset = 6;
    static const int kExplorationMenuOffset = 12;    
    
    switch (feedback->event_type)
    {
        //case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //    break;

        // menu selection
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
#ifdef WAYPOINT_VERBOSE
        ROS_INFO(
                 "marker %s: MENU_SELECT item id %d",
                 feedback->marker_name.c_str(), feedback->menu_entry_id);
#endif
        switch (feedback->menu_entry_id)
        {
        case 1: // Send path to planner
            taskAppend(feedback->header.stamp, kPathNormal);
            break;
            
        case 2: // Append cyclic task
            taskAppend(feedback->header.stamp, kPathCyclic);
            break;

        case 3: // Remove this (interactive) waypoint
            explorationSendPriorityPoint(feedback->marker_name, exploration_msgs::ExplorationPriorityPoint::kActionRemove); 
//            markers_server_->erase(feedback->marker_name);
//            {
//            boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);                   
//            idMarkerMap_.erase(feedback->marker_name);
//            priorityMarkerMap_.erase(feedback->marker_name);
//            }
            markerErase(feedback->marker_name);
            markers_server_->applyChanges();
            break;

        case 4: // Remove all (interactive) waypoints
            explorationSendPriorityPoint(feedback->marker_name, exploration_msgs::ExplorationPriorityPoint::kActionRemoveAll);
            markerClearAll();
            break;
            
        case 5: // Remove all (interactive) waypoints
            OpenPriorityDialog(feedback->marker_name);
            break;            
            
        /// < Patrolling              
        case (kPatrollingMenuOffset+1): // Patrolling - append task
            patrollingTaskAppend(feedback->header.stamp); 
            break;
            
        case (kPatrollingMenuOffset+2): // Patrolling - send task
            patrollingTaskSend(); 
            break;
 
        case (kPatrollingMenuOffset+3): // Patrolling - remove task
            taskRemoveAll();
            patrollingTaskPause(); /// < PAUSE patrolling 
            break;
            
        case (kPatrollingMenuOffset+4): // Patrolling - pause
            patrollingTaskPause(); 
            break;
            
        case (kPatrollingMenuOffset+5): // Patrolling - restart
            patrollingTaskRestart(); 
            break;    
            
            
        /// < Exploration 
        case (kExplorationMenuOffset+1): // Exploration - set BB
            explorationSendPriorityPoint(feedback->marker_name, exploration_msgs::ExplorationPriorityPoint::kActionAdd); 
            break;   
            
        case (kExplorationMenuOffset+2): // Exploration - set BB
            explorationSendFence(); 
            break;      

        case (kExplorationMenuOffset+3): // Exploration - pause
            explorationTaskPause(); 
            break;
            
        case (kExplorationMenuOffset+4): // Exploration - restart
            explorationTaskRestart(); 
            break;              
            
        }
        break;

        //case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        //    break;

        // projects the marker to the point cloud
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
#ifdef WAYPOINT_VERBOSE
        ROS_INFO(
                 "marker %s: MOUSE_UP at (%.2f,%.2f,%.2f) in frame /%s",
                 feedback->marker_name.c_str(),
                 feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z,
                 feedback->header.frame_id.c_str());
#endif
        //if(!feedback->event_type )
        markerUpdate(feedback->marker_name, feedback->mouse_point);
        break;
    }
}

// remove the marker "name" task from the queue
void WaypointsTool::taskRemove(const std::string name)
{
    visualization_msgs::InteractiveMarker aux_marker;
    visualization_msgs::InteractiveMarker marker;
    trajectory_control_msgs::PlanningTask task_msg;

    if (!markers_server_->get(name, marker))
    {
        ROS_ERROR("Interactive marker '%s' does not exist, but produces feedback!", name.c_str());
        return;
    }

    // remove/stop the planning task from the planner node queue
    task_msg.name = marker.description;
    task_msg.header.stamp = ros::Time::now();
    task_msg.header.frame_id = world_frame_id_;
    task_remove_pub_.publish(task_msg);

    // for each counted marker (or waypoint)
    for (unsigned long i = 1; markers_count_ > i; i++)
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss.str(), aux_marker))
        {
            // verify that the marker belongs to the same planning task of the reference marker
            if (!marker.description.compare(aux_marker.description))
            {
                // in this case, erase (remove) marker from the server
//                markers_server_->erase(aux_marker.name);
//                {
//                boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);                       
//                idMarkerMap_.erase(aux_marker.name);
//                priorityMarkerMap_.erase(aux_marker.name);
//                }
                markerErase(aux_marker.name);
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();
}

// remove all the static marker task from the queue
void WaypointsTool::taskRemoveAll(void)
{
    visualization_msgs::InteractiveMarker marker;
    trajectory_control_msgs::PlanningTask task_msg;

    // remove/stop all planning tasks from the planner node queue
    task_msg.name = "ALL";
    task_msg.header.stamp = ros::Time::now();
    task_msg.header.frame_id = world_frame_id_;
    task_remove_pub_.publish(task_msg);

    // for each counted marker (or waypoint)
    for (unsigned long i = 1; markers_count_ > i; i++)
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss.str(), marker))
        {
            // verify that the marker is 'static'
            // (that is, has been published in some planning task)
            if (!marker.controls[0].markers[0].mesh_resource.compare(resource_static_))
            {
                // in this case, erase (remove) marker from the server
//                markers_server_->erase(marker.name);
//                {
//                boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);                       
//                idMarkerMap_.erase(marker.name);
//                priorityMarkerMap_.erase(marker.name);
//                }
                markerErase(marker.name);
            }
            
            /*if(marker.controls[0].markers[0].type = visualization_msgs::Marker::TEXT_VIEW_FACING)
            {
                // in this case, erase (remove) marker from the server
                markers_server_->erase(marker.name);
                priorityMarkerMap_.erase(marker.name);
            }*/
        }
    }

    // update markers states
    markers_server_->applyChanges();
}

// 
void WaypointsTool::markerStaticCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    switch (feedback->event_type)
    {
        //case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //    break;

        // menu selection
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
#ifdef WAYPOINT_VERBOSE
        ROS_INFO(
                 "marker %s: MENU_SELECT item id %d",
                 feedback->marker_name.c_str(), feedback->menu_entry_id);
#endif
        switch (feedback->menu_entry_id)
        {
        case 1: // remove/stop the given task
            taskRemove(feedback->marker_name);
            patrollingTaskPause();  /// < STOP patrolling 
            break;

        case 2: // remove/stop the given task
            taskRemoveAll();
            patrollingTaskPause(); /// < STOP patrolling 
            break;
        }
        break;

        //case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        //    break;
    }
}

// label the input marker "name" as failure 
void WaypointsTool::markerFailure(const std::string name)
{
    visualization_msgs::InteractiveMarker marker;

    // for each counted marker (or waypoint)
    for (unsigned long i = 1; markers_count_ > i; i++)
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss.str(), marker))
        {
            // identify those belonging to the given task
            if (!marker.description.compare(name))
            {
                // convert each marker to a 'failure' marker
                marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
                marker.controls[0].markers[0].mesh_resource = resource_failure_;

                // insert (update) marker in the server
                markers_server_->insert(marker);
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();

    // schedule failures cleanup
    // (we (re)start to ensure a lower bound of the visualization time of the markers) 
    failures_cleanup_timer_.stop();
    failures_cleanup_timer_.start();
}

// label the input maker "name" as success
void WaypointsTool::markerSuccess(const std::string name)
{
    visualization_msgs::InteractiveMarker marker;

    // for each counted marker (or waypoint)
    for (unsigned long i = 1; markers_count_ > i; i++)
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss.str(), marker))
        {
            // identify those belonging to the given task
            if (!marker.description.compare(name))
            {
                // convert each marker to a 'success' marker
                marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
                marker.controls[0].markers[0].mesh_resource = resource_success_;

                // insert (update) marker in the server
                markers_server_->insert(marker);
                markers_success_menu_.apply(*markers_server_, marker.name);
            }
            else
            {
                /// < we enter here when the controller communicates it has done (see feedbackCallback() )
                
                // if the marker belong to previous tasks already succeeded, then
                if (!marker.controls[0].markers[0].mesh_resource.compare(resource_success_))
                {
                    // remove marker form the server
//                    markers_server_->erase(marker.name);
//                    {
//                    boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);                           
//                    idMarkerMap_.erase(marker.name);                    
//                    priorityMarkerMap_.erase(marker.name);
//                    }
                    markerErase(marker.name);
                }
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();
}

// remove all the markers marked as failure 
void WaypointsTool::failureCleanup(const ros::TimerEvent& timer_msg)
{
    visualization_msgs::InteractiveMarker marker;

    // for each counted marker (or waypoint)
    for (unsigned long i = 1; markers_count_ > i; i++)
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if (markers_server_->get(ss.str(), marker))
        {
            // verify that the marker is 'failure'
            if (!marker.controls[0].markers[0].mesh_resource.compare(resource_failure_))
            {
                // in this case, erase (remove) marker from the server
//                markers_server_->erase(marker.name);
//                {
//                boost::recursive_mutex::scoped_lock locker(marker_maps_mtx_);                       
//                idMarkerMap_.erase(marker.name);                
//                priorityMarkerMap_.erase(marker.name);
//                }
                markerErase(marker.name);
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();

    // disable cleanup
    failures_cleanup_timer_.stop();
}


// stop the controller and label the input marker "name" as failure
void WaypointsTool::controllerStop(std::string name)
{
    visualization_msgs::InteractiveMarker marker;

    if (!markers_server_->get(name, marker))
    {
        ROS_ERROR("Interactive marker '%s' does not exist, but produces feedback!", name.c_str());
        return;
    }

    // identify the underlying task
    std::string task = marker.description;

    // advertise feedback to the controller
    trajectory_control_msgs::PlanningFeedback feedback_msg;
    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.node = "tool";
    feedback_msg.task = task;
    feedback_msg.action = 0; 
    feedback_msg.status = STATUS_FAILURE;
    task_feedback_pub_.publish(feedback_msg);

    // update markers of the given task as failure ones
    markerFailure(task);
}

void WaypointsTool::feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg)
{
    // if the feedback message comes from the planner, then
    if (!feedback_msg.node.compare("planner"))
    {
        switch (feedback_msg.status)
        {
        case STATUS_FAILURE:
            markerFailure(feedback_msg.task);
            break;

        case STATUS_SUCCESS:
            markerSuccess(feedback_msg.task);
            break;
        }
    }

    // if the feedback message comes from the controller, then
    if (!feedback_msg.node.compare("control") && !feedback_msg.task.compare("done"))
    {
        switch (feedback_msg.status)
        {
        case STATUS_FAILURE:
            // in this case should be a transition from green to red
            // (no time to implement this behavior)
        case STATUS_SUCCESS:
            markerSuccess("just remove the old green cones");
            break;
        }
    }
}

void WaypointsTool::markerSuccessCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    switch (feedback->event_type)
    {
        //case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //    break;

        // menu selection
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
#ifdef WAYPOINT_VERBOSE
        ROS_INFO(
                 "marker %s: MENU_SELECT item id %d",
                 feedback->marker_name.c_str(), feedback->menu_entry_id);
#endif
        switch (feedback->menu_entry_id)
        {
        case 1: // stop the controller
            controllerStop(feedback->marker_name);
            break;

        case 2: // stop controller and remove all planning task
            controllerStop(feedback->marker_name);
            taskRemoveAll();
            break;
        }
        break;

        //case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        //    break;
    }
}


bool WaypointsTool::getRobotPosition(Ogre::Vector3& position)
{
    Transform transform(world_frame_id_, robot_frame_id_);
    tf::StampedTransform robot_pose;
    bool is_ok_transform = false;
    try
    {
        robot_pose = transform.get();
        is_ok_transform = transform.isOk();

    }
    catch (TransformException e)
    {
        ROS_WARN("%s", e.what());
    }
    
    if(is_ok_transform)
    {
        position.x = robot_pose.getOrigin().getX();
        position.y = robot_pose.getOrigin().getY();
        position.z = robot_pose.getOrigin().getZ();
    }
    return is_ok_transform;
}


void WaypointsTool::InitGUIManagers()
{
    if(bInitManagers_) return;
    
    sceneManager_  = this->context_->getSceneManager();
    windowManager_ = this->context_->getWindowManager();
    if(windowManager_)
    {
        mainWidget_ = windowManager_->getParentWindow();      
    }
    else
    {
        mainWidget_ = 0;
    }
    bInitManagers_ = true;
}

void WaypointsTool::OpenAdvancedWidget()
{
    InitGUIManagers();
    
    std::cout << "WaypointsTool::OpenAdvancedWidget()()" << std::endl;

//    if (pAdvancedWidget_)
//    {
//        if (pAdvancedWidget_->isVisible())
//        {
//            pAdvancedWidget_->close();
//        }
//        else
//        {
//            pAdvancedWidget_->show();
//        }
//    }
//    else
    if(!pAdvancedWidget_)
    {
        pAdvancedWidget_ = new AdvancedWidgetUI(mainWidget_);
        pAdvancedWidget_->SetWpTool(this);
        pAdvancedWidget_->show();
    }


    if(mainWidget_)
    {
        //ROS_INFO_STREAM("set focus on main widget");
        //mainWidget_->activateWindow();
        //mainWidget_->setFocus();
        if(!pAdvancedWidget_->IsAddedToPane())
        {
            pAdvancedWidgetDockWidget_ = windowManager_->addPane("AdvancedWidget",pAdvancedWidget_);
            pAdvancedWidget_->SetAddedToPane(true);
        }
        else
        {
            pAdvancedWidgetDockWidget_->setVisible(true);
        }
    }
}

void WaypointsTool::OpenPriorityDialog(const std::string& name)
{
    InitGUIManagers();
    
    std::cout << "WaypointsTool::OpenPriorityWidget()()" << std::endl;

    if (pPriorityDialog_)
    {
        if (pPriorityDialog_->isVisible())
        {
            pPriorityDialog_->close();
        }
        else
        {
            pPriorityDialog_->show();
        }
    }
    else
    {
        pPriorityDialog_ = new PriorityDialogUI(mainWidget_);
        pPriorityDialog_->show();
        
        connect(pPriorityDialog_, SIGNAL(ValueChanged(double)), this, SLOT(priorityValueChanged(double)));
    }

    if(mainWidget_)
    {
        pPriorityDialog_->setVisible(true);
    }    
    
    // find the existing ones. if marker '#i' exists, then
    visualization_msgs::InteractiveMarker marker;    
    if (markers_server_->get(name, marker))
    {
        double x = marker.pose.position.x;
        double y = marker.pose.position.y;
        double z = marker.pose.position.z;
        pPriorityDialog_->SetXYZ(x,y,z);
    }
    
    pPriorityDialog_->SetMarkerName(name);
    pPriorityDialog_->SetValue(priorityMarkerMap_[name]);
}


void WaypointsTool::priorityValueChanged(double val)
{
    if(!pPriorityDialog_) return;
    
    geometry_msgs::Point point;
    point.x = pPriorityDialog_->GetX(); 
    point.y = pPriorityDialog_->GetY(); 
    point.z = pPriorityDialog_->GetZ();
    
    setPriority(pPriorityDialog_->GetMarkerName(), (PriorityType)lrint(val), point);
}


void WaypointsTool::rvizMessageStringCallback(const std_msgs::String::ConstPtr msg)
{   
    std::string text = msg->data; 
    markerAddRobotTextMsg(text,kQuickTextMessagesLifeDuration);
}


} // end namespace path_planner_rviz_wp_plugin


// END_TUTORIAL
