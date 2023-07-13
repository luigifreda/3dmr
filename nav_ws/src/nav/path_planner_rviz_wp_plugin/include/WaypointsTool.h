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

#ifndef WAYPOINTS_TOOL_H
#define WAYPOINTS_TOOL_H

#ifndef Q_MOC_RUN

// ros and rviz
#include <ros/ros.h>
#include <rviz/tool.h>

// ros messages
#include <geometry_msgs/Point.h>
#include <trajectory_control_msgs/PlanningTask.h>
#include <trajectory_control_msgs/PlanningFeedback.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/String.h>

// point clouds stuff
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// markers stuff
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// exploration messages
#include <exploration_msgs/ExplorationPriorityPoint.h>
#include <exploration_msgs/ExplorationPriorityActions.h>

// ogre stuff
#include <OGRE/OgreVector2.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreViewport.h>

// boost 
#include <boost/thread/recursive_mutex.hpp>

// stdlib
#include <unordered_map> /// < requires  C++11

#include <trajectory_control_msgs/message_enums.h> // for SegmentStatus and TaskType
#include <path_planner/Transform.h>

#endif

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/// < NB do not use a starting slash otherwise the configuration won't work with NIMBRO 

#define NAME_PATROLLING_TASK_APPEND_TOPIC "/patrolling/task/append"
#define NAME_PATROLLING_TASK_SEND_TOPIC   "/patrolling/task/send"
#define NAME_PATROLLING_TASK_PAUSE_TOPIC  "/patrolling/task/pause"
#define NAME_PLANNING_MESSAGES_TOPIC      "/planning/messages"   //planning text messages to be visualized in RVIZ
#define NAME_EXPLORATION_FENCE_TOPIC      "/exploration_fence"
#define NAME_EXPLORATION_PAUSE_TOPIC      "/expl_pause_topic"
#define NAME_PRIORITY_POINT_TOPIC         "/priority_point"              // for modifying priorities on patrolling graph
#define NAME_EXPL_PRIORITY_POINT_TOPIC    "/exploration_priority_point"  // for exploration 
#define NAME_EXPL_PRIORITY_ACTIONS_TOPIC  "/exploration_priority_actions"  // for exploration 

#define NAME_COMPUTE_NORMAL_ESTIMATION_DYNRECONF      "compute_normals/NormalEstimationPcl"
#define NAME_COMPUTE_NORMAL_ESTIMATION_UGV1_DYNRECONF "/compute_normals_ugv1/NormalEstimationPcl"
#define NAME_COMPUTE_NORMAL_ESTIMATION_UGV2_DYNRECONF "/compute_normals_ugv2/NormalEstimationPcl"
#define NAME_COMPUTE_NORMAL_ESTIMATION_UGV3_DYNRECONF "/compute_normals_ugv3/NormalEstimationPcl"

#define NAME_CLOSEST_OBS_VEL_REDUCTION_ENABLE_TOPIC  "/closest_obst_vel_reduction_enable"

#define NAME_RSS_ENABLE_TOPIC "/rss_enable"
#define NAME_RSS_SERVICE_SAVE "Request_RSS_Save_PointCloud"
#define NAME_RSS_SERVICE_LOAD "Request_RSS_Load_PointCloud"
#define NAME_RSS_MIN_VALUE_TOPIC "/rss_min_value"

#define NAME_MESSAGE_STRING_TOPIC "/rviz_message_string"

namespace Ogre
{
    class SceneManager;
}
namespace rviz
{
    class WindowManagerInterface;
    class PanelDockWidget;
}

namespace path_planner_rviz_wp_plugin   
{

class AdvancedWidgetUI;
class PriorityDialogUI;
  
typedef int PriorityType; 

/* BEGIN_TUTORIAL
 * Here we declare our new subclass of rviz::Tool.  Every tool
 * which can be added to the tool bar is a subclass of
 * rviz::Tool. */
class WaypointsTool: public rviz::Tool  
{
    Q_OBJECT
    
    enum MarkerType {kInteractive=0, kStatic, kSuccess, kFailure};
    
    static constexpr double kTextMessagesLifeDuration = 5.0; // [s] life duration for text messages 
    static constexpr double kTextMessageHeightAboveRobot = 1.5; // [m]
    
    static constexpr double kQuickTextMessagesLifeDuration = 2.0; // [s] life duration for text messages 
    
public:
    WaypointsTool(
    //const std::string& pcl_input_topic_name = "/dynjoinpcl_nn",  // old input for path planning with octomap 
    const std::string& pcl_input_topic_name = "volumetric_mapping/octomap_pcl",
    const std::string& planner_task_feedback_topic_name = "planner/tasks/feedback",
    const std::string& planner_task_append_topic_name   = "planner/tasks/append",                
    const std::string& planner_task_remove_topic_name   = "planner/tasks/remove",
    const std::string& planner_waypoints_server_name    = "planner/waypoints/server",
    const std::string& patrolling_task_append_topic_name = NAME_PATROLLING_TASK_APPEND_TOPIC,
    const std::string& patrolling_task_send_topic_name   = NAME_PATROLLING_TASK_SEND_TOPIC,
    const std::string& patrolling_task_pause_topic_name  = NAME_PATROLLING_TASK_PAUSE_TOPIC,
    const std::string& robot_frame_name = "/base_link", 
    const std::string& world_frame_name = "map",   
    const std::string& robot_name = ""
    );
    
    virtual ~WaypointsTool();

    virtual void onInitialize(){}

    virtual void activate(){}

    virtual void deactivate(){}

    virtual void load(const rviz::Config& config){}

    virtual void save(rviz::Config config) {}
    
    
    //  Process a mouse event.  This is the central function of all the tools, as it defines how the mouse is used
    virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
    
    // Process a key event.  Override if your tool should handle any other keypresses than the tool shortcuts, which are handled separately
    virtual int processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel );
    
public Q_SLOTS:
    
    void OpenAdvancedWidget();
    
    void OpenPriorityDialog(const std::string& name);
    
    void priorityValueChanged(double val);
    void setPriority(const std::string& name,const PriorityType val, const geometry_msgs::Point& point);   
    
    void SetClosestObstVelReductionEnable(bool val); 
    
    void SetRssEnable(bool val);

    void RssLoad();
    void RssSave();
    
    void SetMinRssValueEnable(int val);
    
public:
    // input point cloud
    void cloudCallback(const PointCloud::ConstPtr& msg);
    void cloudNormalsCallback(const PointCloud::ConstPtr& msg);    

    // procedures associated to the 'interactive' markers
    void markerAdd(const Ogre::Vector3& position, MarkerType marker_type = kInteractive, const std::string& description = std::string());
    void markerUpdate(const std::string& name, const geometry_msgs::Point& reference);
    void markerAddText(const Ogre::Vector3& position, const std::string& text, const double duration = kTextMessagesLifeDuration);
    void markerAddRobotTextMsg(const std::string& msg, const double duration = kTextMessagesLifeDuration);
    void markerClearAll(void);
    
    // N.B.: after calling this function you have to apply changes by using markers_server_->applyChanges(); 
    void markerErase(const std::string marker_name); 
    
    void taskAppend(const ros::Time& stamp, uint8_t task_type = 0 /*kNormal*/);
    
    void markerInteractiveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // procedures associated to the 'static' markers
    void taskRemove(const std::string name);
    void taskRemoveAll(void);
    void markerStaticCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // procedures associated to the planner/controller feedback
    void markerFailure(const std::string name);
    void markerSuccess(const std::string name);
    void failureCleanup(const ros::TimerEvent& timer_msg);
    void controllerStop(const std::string name);
    void feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg);
    void markerSuccessCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    void patrollingTaskAppend(const ros::Time& stamp);
    void patrollingTaskSend();
    void patrollingTaskPause();
    void patrollingTaskRestart();
    
    void explorationSendPriorityPoint(const std::string& name, uint8_t action = exploration_msgs::ExplorationPriorityPoint::kActionAdd);    
    void explorationSendFence();
    void explorationTaskPause();
    void explorationTaskRestart();    
    
    void explorationPriorityActionsCallback(const exploration_msgs::ExplorationPriorityActions::ConstPtr msg);

    void rvizMessageStringCallback(const std_msgs::String::ConstPtr msg);
    
protected:
    
    bool getRobotPosition(Ogre::Vector3& position);
    
    void InitGUIManagers();
    
protected: 
    
    // ros stuff
    ros::NodeHandle node_;    
    ros::Timer failures_cleanup_timer_;

    // subscribers
    ros::Subscriber pcl_sub_; // subscriber to the /dynjoinpcl_nn cloud 
    ros::Subscriber pcl_normals_sub_; // subscriber to the /dynjoinpcl_nn cloud     
    ros::Subscriber task_feedback_sub_; // subscriber to the planner/controller feedback
    
    ros::Subscriber exploration_priority_actions_sub_; // for managing exploration actions 
    
    ros::Subscriber rviz_message_string_sub_;    
        
    // publishers
    ros::Publisher task_append_pub_; // publisher for appending planning tasks
    ros::Publisher task_remove_pub_; // publisher for removing planning tasks
    ros::Publisher task_feedback_pub_; // publisher to the planner/controller feedback
    
    ros::Publisher patrolling_task_pub_; // publisher to patrolling/task
    ros::Publisher patrolling_points_task_pub_; // publisher to patrolling/task    
    
    ros::Publisher patrolling_pause_pub_; // publisher to patrolling/pause
    ros::Publisher patrolling_send_pub_; // publisher to patrolling send to robots
    
    ros::Publisher planning_messages_pub_; // publisher to planning messages 
    ros::Publisher exploration_bb_pub_; // publisher for exploration BB 
    ros::Publisher exploration_pause_pub_; // publisher to patrolling/pause    
    
    ros::Publisher priority_point_pub_; // for modifying priorities in patrolling graph
    
    ros::Publisher exploration_priority_point_pub_; // for generating/sending an exploration priority point 
    
    ros::Publisher exploration_priority_actions_pub_; // for managing exploration actions 
     
    ros::Publisher closest_obst_vel_reduction_enable_pub_; // publisher for the enabling/disabling velocity reduction depending on closer obstacle point    
    
    ros::Publisher rss_enable_pub_; // publisher for the enabling/disabling velocity reduction depending on closer obstacle point        
    
    ros::Publisher rss_min_signal_value_pub_ ; // publisher for setting the minimum critical rss value in the path planner   
        
    // input point cloud
    PointCloud pcd_;
    boost::recursive_mutex pcd_mtx_;  

    // marker associated data
    unsigned long markers_count_; // internal counter for markers names generation
    unsigned long planner_count_; // internal counter for tasks names generation
    
    std::string resource_interactive_; // file of the orange mesh
    std::string resource_failure_; // file of the red mesh
    std::string resource_success_; // file of the green mesh
    std::string resource_static_; // file of the gray mesh
    
    std::string world_frame_id_; // reference frame
    std::string robot_frame_id_; // robot frame
    
    std::string robot_name_;
    
    // menu of the 'interactive' markers
    interactive_markers::MenuHandler markers_interactive_menu_;
 
    // menu of the 'static' markers
    interactive_markers::MenuHandler markers_static_menu_;

    // menu of the 'success' markers
    interactive_markers::MenuHandler markers_success_menu_;

    // markers server
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> markers_server_;
    
    boost::recursive_mutex text_markers_array_mtx_;  
    visualization_msgs::MarkerArray text_markers_array_;
    unsigned long text_markers_count_; // internal counter for text markers names generation
    
    boost::recursive_mutex marker_maps_mtx_;  
    std::unordered_map<std::string,size_t> idMarkerMap_;       
    std::unordered_map<std::string,PriorityType> priorityMarkerMap_;   

    // bridge between the interactive markers and the rviz tool
    Ogre::Viewport *viewport_;
    
    bool b_added_new_node_; 
    
protected: // for managing additional widgets 
    
    bool bAddedAdvancedWidgetPanel_; 
    AdvancedWidgetUI* pAdvancedWidget_;
    PriorityDialogUI* pPriorityDialog_;
    
    bool bInitManagers_;
    Ogre::SceneManager* sceneManager_;
    rviz::WindowManagerInterface* windowManager_;
    QWidget* mainWidget_;     
    rviz::PanelDockWidget* pAdvancedWidgetDockWidget_;
    
    // END_TUTORIAL
};

} // end namespace path_planner_rviz_wp_plugin

#endif // WAYPOINTS_TOOL_H
