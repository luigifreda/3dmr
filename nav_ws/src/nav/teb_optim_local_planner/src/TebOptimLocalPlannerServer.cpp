#include <teb_optim_local_planner/TebOptimLocalPlannerServer.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
//#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


#define VERBOSE 1

namespace
{
  template<typename T>
  T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
  {
      T v;
      if (n.getParam(name, v))
      {
          ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
          return v;
      }
      else
      {
          ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
      }
      return defaultValue;
  }

  void tfConvert(const tf::StampedTransform& src, geometry_msgs::PoseStamped& dst)
  {
      dst.header.stamp    = src.stamp_; 
      dst.header.frame_id = src.frame_id_;
      dst.pose.position.x = src.getOrigin().getX();
      dst.pose.position.y = src.getOrigin().getY();
      dst.pose.position.z = src.getOrigin().getZ();
      dst.pose.orientation.x = src.getRotation().getX();
      dst.pose.orientation.y = src.getRotation().getY();
      dst.pose.orientation.z = src.getRotation().getZ();
      dst.pose.orientation.w = src.getRotation().getW();
  }

  void tfConvert(const tf::StampedTransform& src, geometry_msgs::TransformStamped& dst)
  {
      dst.header.stamp    = src.stamp_;
      dst.header.frame_id = src.frame_id_;
      dst.child_frame_id  = src.child_frame_id_;
      dst.transform.translation.x = src.getOrigin().getX();
      dst.transform.translation.y = src.getOrigin().getY();
      dst.transform.translation.z = src.getOrigin().getZ();
      dst.transform.rotation.x = src.getRotation().getX();
      dst.transform.rotation.y = src.getRotation().getY();
      dst.transform.rotation.z = src.getRotation().getZ();
      dst.transform.rotation.w = src.getRotation().getW();
  }
}

namespace teb_local_planner
{
  
TebOptimLocalPlannerServer::TebOptimLocalPlannerServer(const std::string& name): 
    name_(name), 
    param_node_(name),
    /*costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),*/
    /*costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),*/
    dynamic_recfg_(NULL), custom_via_points_active_(false), goal_reached_(false), no_infeasible_plans_(0),
    last_preferred_rotdir_(RotType::none), initialized_(false),
    action_name(name), act_server_(node_, name, boost::bind(&TebOptimLocalPlannerServer::executeCallback, this, _1), false),
    act_client_(name, true),
    b_local_path_(false),
    b_simple_rotation_(false)
{
  initialize();
}


TebOptimLocalPlannerServer::~TebOptimLocalPlannerServer()
{
}

void TebOptimLocalPlannerServer::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
  //ros::NodeHandle nh("~/" + name_);
  ros::NodeHandle& nh = param_node_;

  // lock the config mutex externally
  boost::mutex::scoped_lock lock(cfg_.configMutex());

  // create robot footprint/contour model for optimization 
#if USE_NEW_TEB_VERSION 
  cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
  planner_->updateRobotModel(cfg_.robot_model);
#else 
  RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh, cfg_);
  planner_->updateRobotModel(robot_model); 
#endif 
}

void TebOptimLocalPlannerServer::initialize()
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	
    //name_ = name;
 
    // create Node Handle with name of plugin (as used in move_base for loading)
    //ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle& nh = param_node_;
	        
    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);       
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);
    obstacles_stamps_.reserve(obstacles_.capacity());
        
    // create visualization instance	
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_)); 
        
#if USE_NEW_TEB_VERSION
    // create robot footprint/contour model for optimization
    cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
#else            
    // create robot footprint/contour model for optimization
    RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh, cfg_);
#endif 

    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
#if USE_NEW_TEB_VERSION      
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, visualization_, &via_points_));
#else 
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
#endif       
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    }
    else
    {
#if USE_NEW_TEB_VERSION       
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, visualization_, &via_points_));
#else 
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
#endif      
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    // tf_ = tf;
    // tf_->setUsingDedicatedThread(true);
    //costmap_ros_ = costmap_ros;
    //costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    //costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

    //global_frame_ = costmap_ros_->getGlobalFrameID();

    global_frame_ = getParam<std::string>(nh, name_ + "/global_frame", "map");
    robot_base_frame_ = getParam<std::string>(nh, name_ + "/robot_base_frame", "base_link");
    cfg_.map_frame = global_frame_; // TODO
    //robot_base_frame_ = costmap_ros_->getBaseFrameID();

    control_rate_ = getParam<double>(nh, name_ + "/control_rate", 10);

    transform_tolerance_ = getParam<double>(param_node_, "transform_tolerance", 0.2);

    max_time_for_evanescent_obstacles_ = getParam<double>(param_node_, "max_time_for_evanescent_obstacles", 2.0);
    factor_extending_plan_lookahead_dist_with_obs_ = getParam<double>(param_node_, "factor_extending_plan_lookahead_dist_with_obs", 1.5);

    global_path_topic_ = getParam<std::string>(param_node_, "global_path_topic", "/global_path");
    local_path_topic_ = getParam<std::string>(param_node_, "local_path_topic", "/local_path");
    
    cmd_topic_ = getParam<std::string>(param_node_, "cmd_vel_topic", "/cmd_vel");
    cmd_wheels_topic_ = getParam<std::string>(param_node_, "cmd_wheels_topic", "/cmd_wheels_topic");

    robot_path_topic_ = getParam<std::string>(param_node_, "robot_path_topic", "/robot_path");
    robot_local_path_topic_ = getParam<std::string>(param_node_, "robot_local_path_topic", "/robot_local_path");

    queue_task_feedback_topic_ = getParam<std::string>(param_node_, "queue_task_feedback_topic", "/planner/tasks/feedback");
    queue_task_path_topic_ = getParam<std::string>(param_node_, "queue_task_path_topic", "/planner/tasks/path");
    //queue_task_path_local_topic_ = getParam<std::string>(param_node_, "queue_task_path_local_topic", "/planner/tasks/local_path");
    
    goal_abort_topic_ = getParam<std::string>(param_node_, "goal_abort_topic", "/goal_abort_topic");
    trajectory_control_abort_topic_ = getParam<std::string>(param_node_, "trajectory_control_abort_topic", "/trajectory_control_abort_topic");
    
    const double tracks_distance_default_ = 0.397; // tracks distance for TRADR robot (used as default)
    tracks_distance_ = getParam<double>(param_node_, "tracks_distance", tracks_distance_default_);
    enable_track_cmds_ = getParam<bool>(param_node_, "enable_track_cmds", enable_track_cmds_); 
    tracks_vel_cmd_topic_ = getParam<std::string>(param_node_, "tracks_vel_cmd_topic", "/tracks_vel_cmd");

    /// < setup subcribers 
    
    robot_path_sub_ = node_.subscribe(robot_path_topic_, 1, &TebOptimLocalPlannerServer::robotPathCallBack, this);
    robot_loal_path_sub_ = node_.subscribe(robot_local_path_topic_, 1, &TebOptimLocalPlannerServer::robotLocalPathCallBack, this);
    //robot_rotation_sub_ = node_.subscribe(robot_rotation_topic_, 1, &TebOptimLocalPlannerServer::robotRotationCallBack, this);    

    // feedback between nodes: "tool", "planner" and "control"
    queue_task_feedback_sub_ = node_.subscribe(queue_task_feedback_topic_, 20, &TebOptimLocalPlannerServer::queueFeedbackCallback, this);

    // get the task's paths
    queue_task_path_sub_ = node_.subscribe(queue_task_path_topic_, 1, &TebOptimLocalPlannerServer::queueTaskCallback, this);
    //queue_task_path_local_sub_ = node_.subscribe(queue_task_path_local_topic_, 1, &TebOptimLocalPlannerServer::queueLocalTaskCallback, this);
    
    goal_abort_sub_ = node_.subscribe(goal_abort_topic_, 1, &TebOptimLocalPlannerServer::goalAbortCallback, this);
    trajectory_control_abort_sub_ = node_.subscribe(trajectory_control_abort_topic_, 1, &TebOptimLocalPlannerServer::goalAbortCallback, this); /// < use the same callback of goal abort!

    /// < setup publishers 

    global_path_pub_ = node_.advertise<nav_msgs::Path>("/traj_global_path", 1);

    local_path_pub_ = node_.advertise<nav_msgs::Path>("/traj_local_path", 1);

    cmd_pub_ = node_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
    cmd_wheels_pub_ = node_.advertise<trajectory_control_msgs::VelocityCommand>(cmd_wheels_topic_, 1);

    queue_task_feedback_pub_ = node_.advertise<trajectory_control_msgs::PlanningFeedback>(queue_task_feedback_topic_, 1);

    if(enable_track_cmds_)
    {
      tracks_vel_cmd_pub_ = node_.advertise<nifti_robot_driver_msgs::Tracks>(tracks_vel_cmd_topic_, 1);
    }

    /// < start the action server 
    act_server_.start();


    /// < init vars 

    global_path_msg_.header.frame_id = global_frame_;
    global_path_msg_.header.stamp = ros::Time::now();

    local_path_msg_.header.frame_id = global_frame_;
    local_path_msg_.header.stamp = ros::Time::now();

    // reset last command 
    last_cmd_twist_.linear.x = 0.0;
    last_cmd_twist_.linear.y = 0.0;
    last_cmd_twist_.angular.z = 0.0;

    //Initialize a costmap to polygon converter
    // if (!cfg_.obstacles.costmap_converter_plugin.empty())
    // {
    //   try
    //   {
    //     costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
    //     std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
    //     // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
    //     boost::replace_all(converter_name, "::", "/");
    //     costmap_converter_->setOdomTopic(cfg_.odom_topic);
    //     costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
    //     costmap_converter_->setCostmap2D(costmap_);
        
    //     costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
    //     ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");        
    //   }
    //   catch(pluginlib::PluginlibException& ex)
    //   {
    //     ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
    //     costmap_converter_.reset();
    //   }
    // }
    // else 
    //   ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
  
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    // footprint_spec_ = costmap_ros_->getRobotFootprint();
    // costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    
    
    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebOptimLocalPlannerServer::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
    
    // validate optimization footprint and costmap footprint
#if USE_NEW_TEB_VERSION
    validateFootprints(cfg_.robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);
#else     
    validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);
#endif         
    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("/obstacles", 1, &TebOptimLocalPlannerServer::customObstacleCB, this);

    // setup callback for custom via-points
    via_points_sub_ = nh.subscribe("via_points", 1, &TebOptimLocalPlannerServer::customViaPointsCB, this);
    
    // initialize failure detector
    // ros::NodeHandle nh_move_base("~");
    // double controller_frequency = 5;
    // nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    double controller_frequency = control_rate_;
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency));
    
    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("teb_optim_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_optim_local_planner has already been initialized, doing nothing.");
  }
}

bool TebOptimLocalPlannerServer::getRobotPose(tf::StampedTransform& robot_pose)
{
    if (tf_listener_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(transform_tolerance_)))
    {
        try
        {
            tf_listener_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(), robot_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_WARN("No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_WARN("Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_WARN("Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_WARN_STREAM("Transformation is not available: " << global_frame_ << " to " << robot_base_frame_);
        return false;
    }
}

bool TebOptimLocalPlannerServer::getTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& target_time, tf::StampedTransform& transform)
{
    if (tf_listener_.waitForTransform(target_frame, source_frame, target_time, ros::Duration(transform_tolerance_)))
    {
        try
        {
            tf_listener_.lookupTransform(target_frame, source_frame, target_time, transform);
        }
        catch (tf::LookupException& ex)
        {
            ROS_WARN("No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_WARN("Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_WARN("Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_WARN_STREAM("Transformation is not available: " << target_frame << " to " << source_frame);
        return false;
    }
}

bool TebOptimLocalPlannerServer::getTransform(const std::string& target_frame, const ros::Time& target_time,
                                            const std::string& source_frame, const ros::Time& source_time,
                                            const std::string& fixed_frame, tf::StampedTransform& transform)
{
    if (tf_listener_.waitForTransform(target_frame, target_time,
                                      source_frame, source_time,
                                      fixed_frame, ros::Duration(transform_tolerance_)))
    {
        try
        {
            tf_listener_.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform);
        }
        catch (tf::LookupException& ex)
        {
            ROS_WARN("No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_WARN("Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_WARN("Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_WARN_STREAM("Transformation is not available: " << target_frame << " to " << source_frame << ", fixed: " << fixed_frame);
        return false;
    }
}



bool TebOptimLocalPlannerServer::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_optim_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
            
  // reset goal_reached_ flag
  goal_reached_ = false;
  
  return true;
}

void TebOptimLocalPlannerServer::clearOldObstacles()
{
  ROS_ASSERT_MSG(obstacles_.size() == obstacles_stamps_.size(), "Wrong management in obstacles!");
  if(obstacles_.size() != obstacles_stamps_.size()) obstacles_.clear();
  auto ito = obstacles_.begin();
  auto itos = obstacles_stamps_.begin();
  ros::Time now = ros::Time::now(); 
#if VERBOSE  
  std::cout << "obstacles size: " << obstacles_.size() << std::endl; 
#endif   
  while(ito!=obstacles_.end())
  {
    ros::Duration elapsed_time = now - *itos;
    if(elapsed_time.toSec()>max_time_for_evanescent_obstacles_)
    {
      ito = obstacles_.erase(ito); 
      itos = obstacles_stamps_.erase(itos);
    }
    else
    {
      ito++;
      itos++;
    }
  }
#if VERBOSE   
  std::cout << "obstacles size (after cleaning): " << obstacles_.size() << std::endl;   
#endif   
}

bool TebOptimLocalPlannerServer::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t TebOptimLocalPlannerServer::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_optim_local_planner has not been initialized, please call initialize() before using this planner");
    message = "teb_optim_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;  
  
  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  //costmap_ros_->getRobotPose(robot_pose);
  getRobotPose(tf_robot_pose_);               
  tfConvert(tf_robot_pose_, robot_pose);

#if VERBOSE
  std::cout << "current robot p: (" << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << robot_pose.pose.position.z << ")" << std::endl; 
#endif 

  robot_pose_ = PoseSE2(robot_pose.pose);
    
  // Get robot velocity
#if 0  
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
#else
  // this is an approximation (should be reasonable though)
  robot_vel_.linear.x = last_cmd_twist_.linear.x;
  robot_vel_.linear.y = last_cmd_twist_.linear.y;
  robot_vel_.angular.z = last_cmd_twist_.angular.z;
#endif 

#if VERBOSE
  std::cout << "current robot v: (" << robot_vel_.linear.x << ", " << robot_vel_.linear.y << "), omega: " << robot_vel_.angular.z << std::endl; 
#endif 

  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

  double max_global_plan_lookahead_dist = cfg_.trajectory.max_global_plan_lookahead_dist;
  if(obstacles_.size()>0)
  {
    // in case there are obstacles let's double max_global_plan_lookahead_dist so as to have a larger horizon to reason about
    max_global_plan_lookahead_dist = factor_extending_plan_lookahead_dist_with_obs_ * cfg_.trajectory.max_global_plan_lookahead_dist;
  }  

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(global_plan_, robot_pose, /* *costmap_,*/ global_frame_, max_global_plan_lookahead_dist, 
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    std::cout << message << std::endl; 
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }

  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
#if VERBOSE
  std::cout << "global_goal: (" << global_goal.pose.position.x << ", " << global_goal.pose.position.y  << "), orientation: " << global_goal.pose.orientation << std::endl; 
#endif 
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );

  double error_2d = fabs(std::sqrt(dx*dx+dy*dy));
#if VERBOSE
  //std::cout << "error dp: (" << dx << ", " << dy << "), dtheta: " << delta_orient << std::endl; 
  std::cout << "error_2d: " <<  error_2d << ", dtheta: " << delta_orient << std::endl; 
#endif 

  if( error_2d < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
    //&& (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel) || cfg_.goal_tolerance.free_goal_vel)
    )
  {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);
  
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    std::cout << message << std::endl; 
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }
              
  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start
    
#if 0
  // clear currently existing obstacles  
  obstacles_.clear();
#else 
  // clear only old obstacles
  clearOldObstacles();
#endif   

  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
#if 0  
  if (costmap_converter_)
     updateObstacleContainerWithCostmapConverter();
   else
     updateObstacleContainerWithCostmap();
#endif 

  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  
    
  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());
    
  // Now perform the actual planning
  //   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("teb_optim_local_planner was not able to obtain a local plan for the current setting.");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_optim_local_planner was not able to obtain a local plan";
    std::cout << message << std::endl; 
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

#if USE_NEW_TEB_VERSION
  // Check for divergence  // New version of TEB

  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    ROS_WARN_THROTTLE(1.0, "TebOptimLocalPlannerServer: the trajectory has diverged. Resetting planner...");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
#endif          
  // // Check feasibility (but within the first few states only)
  // if(cfg_.robot.is_footprint_dynamic)
  // {
  //   // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
  //   footprint_spec_ = costmap_ros_->getRobotFootprint();
  //   costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  // }

  // bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  // if (!feasible)
  // {
  //   cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

  //   // now we reset everything to start again with the initialization of new trajectories.
  //   planner_->clearPlanner();
  //   ROS_WARN("TebOptimLocalPlannerServer: trajectory is not feasible. Resetting planner...");
    
  //   ++no_infeasible_plans_; // increase number of infeasible solutions in a row
  //   time_last_infeasible_plan_ = ros::Time::now();
  //   last_cmd_ = cmd_vel.twist;
  //   message = "teb_optim_local_planner trajectory is not feasible";
  //   return mbf_msgs::ExePathResult::NO_VALID_CMD;
  // }

  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses))

  {
    planner_->clearPlanner();
    ROS_WARN("TebOptimLocalPlannerServer: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_optim_local_planner velocity command invalid";
    std::cout << message << std::endl; 
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_x, cfg_.robot.max_vel_theta, 
                   cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
                                                                cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      ROS_WARN("TebOptimLocalPlannerServer: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      message = "teb_optim_local_planner steering angle is not finite";
      std::cout << message << std::endl; 
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
  }
  
  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;
  
  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;
  
  // Now visualize everything    
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  return mbf_msgs::ExePathResult::SUCCESS;
}


bool TebOptimLocalPlannerServer::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    return true;
  }
  return false;
}


#if 0
// disabled part relative to costmap 
void TebOptimLocalPlannerServer::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebOptimLocalPlannerServer::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}
#endif 


// TODO: add evanescent obstacles
void TebOptimLocalPlannerServer::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try 
    {
      //lookupTransform (const std::string &target_frame, const ros::Time &target_time, const std::string &source_frame, const ros::Time &source_time, const std::string &fixed_frame, const ros::Duration timeout)
      // geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
      //                                                                         custom_obstacle_msg_.header.frame_id, ros::Time(0),
      //                                                                         custom_obstacle_msg_.header.frame_id, ros::Duration(transform_tolerance_));
      tf::StampedTransform transform;
#if 0      
      getTransform(global_frame_, ros::Time(0),
                   custom_obstacle_msg_.header.frame_id, ros::Time(0),
                   custom_obstacle_msg_.header.frame_id, transform);
#else 
      getTransform(global_frame_, ros::Time(0),
                   custom_obstacle_msg_.header.frame_id, custom_obstacle_msg_.header.stamp,
                   custom_obstacle_msg_.header.frame_id, transform);
#endif                    
      geometry_msgs::TransformStamped obstacle_to_map; 
      tfConvert(transform, obstacle_to_map);

      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }
    
    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
        obstacles_stamps_.push_back(custom_obstacle_msg_.obstacles.at(i).header.stamp);
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
        obstacles_stamps_.push_back(custom_obstacle_msg_.obstacles.at(i).header.stamp);
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
        obstacles_stamps_.push_back(custom_obstacle_msg_.obstacles.at(i).header.stamp);                                                           
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
        obstacles_stamps_.push_back(custom_obstacle_msg_.obstacles.at(i).header.stamp);
      }

      // Set velocity, if obstacle is moving
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void TebOptimLocalPlannerServer::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}
      
Eigen::Vector2d TebOptimLocalPlannerServer::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
      
bool TebOptimLocalPlannerServer::pruneGlobalPlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    //geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    tf::StampedTransform transform;
    getTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0), transform);
    geometry_msgs::TransformStamped global_to_plan_transform;
    tfConvert(transform, global_to_plan_transform);

    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
      

bool TebOptimLocalPlannerServer::transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, /*const costmap_2d::Costmap2D& costmap,*/ const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global)
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    // geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
    //                                                                               plan_pose.header.frame_id, ros::Duration(transform_tolerance_));

    tf::StampedTransform transform;
    getTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, transform);
    geometry_msgs::TransformStamped plan_to_global_transform;
    tfConvert(transform, plan_to_global_transform);

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    //tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);
    tf::StampedTransform via_transform;
    getTransform(plan_pose.header.frame_id, tf2::getFrameId(global_pose), tf2::getTimestamp(robot_pose), via_transform);
    geometry_msgs::TransformStamped g_via_transform;
    tfConvert(via_transform, g_via_transform);
    tf2::doTransform(global_pose, robot_pose, g_via_transform);

    // //we'll discard points on the plan that are outside the local costmap
    // double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
    //                                  costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    // dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
    //                        // located on the border of the local costmap
    
    int i = 0;
    //double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist_threshold = 1e10;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }
    
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}

    
      
      
double TebOptimLocalPlannerServer::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
      
      
void TebOptimLocalPlannerServer::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_trans, double max_vel_theta, 
              double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebOptimLocalPlannerServer(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  //if (cfg_.robot.use_proportional_saturation)
  if(use_proportional_saturation_)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }

  double vel_linear = std::hypot(vx, vy);
  if (vel_linear > max_vel_trans)
  {
    double max_vel_trans_ratio = max_vel_trans / vel_linear;
    vx *= max_vel_trans_ratio;
    vy *= max_vel_trans_ratio;
  }
}
     
     
double TebOptimLocalPlannerServer::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  
  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius; 

  return std::atan(wheelbase / radius);
}
     

void TebOptimLocalPlannerServer::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}
   
   
   
void TebOptimLocalPlannerServer::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();
    
    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
        
        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        
        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                ROS_WARN("TebOptimLocalPlannerServer: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();  
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            ROS_INFO("TebOptimLocalPlannerServer: oscillation recovery disabled/expired.");
        }
    }

}
     
void TebOptimLocalPlannerServer::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;  
}

void TebOptimLocalPlannerServer::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  if (cfg_.trajectory.global_plan_viapoint_sep > 0)
  {
    ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}
     
RobotFootprintModelPtr TebOptimLocalPlannerServer::getRobotFootprintFromParamServer(const ros::NodeHandle& nh, const TebConfig& config)
{
  std::string model_name; 
  if (!nh.getParam("footprint_model/type", model_name))
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }
    
  // point  
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
#if USE_NEW_TEB_VERSION
    return boost::make_shared<PointRobotFootprint>(config.obstacles.min_obstacle_dist); // new version 
#else    
    return boost::make_shared<PointRobotFootprint>();
#endif     
  }
  
  // circular
  if (model_name.compare("circular") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  
  // line
  if (model_name.compare("line") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
#if USE_NEW_TEB_VERSION    
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
#else     
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
#endif     
  }
  
  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      } 
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
  }
  
  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}
         
       
       
       
Point2dContainer TebOptimLocalPlannerServer::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // Make sure we have an array of at least 3 elements.
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }
 
   Point2dContainer footprint;
   Eigen::Vector2d pt;
 
   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // Make sure each element of the list is an array of size 2. (x and y coordinates)
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double TebOptimLocalPlannerServer::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}


void setPosesOrientation(nav_msgs::Path& path)
{
    if(path.poses.empty()) 
    {
        ROS_WARN_STREAM("TebOptimLocalPlannerServer::executeCallback() - path is empty!");
        return; 
    }

    if(path.poses.size()>=2)
    {
        for(size_t ii=0; ii < path.poses.size()-1; ii++)
        {
            const double yaw = atan2(path.poses[ii+1].pose.position.y - path.poses[ii].pose.position.y, 
                                    path.poses[ii+1].pose.position.x - path.poses[ii].pose.position.x);
            path.poses[ii].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        path.poses[path.poses.size()-1].pose.orientation = path.poses[path.poses.size()-2].pose.orientation; // keept the second to last orientation 
    }
    else
    {
        ROS_WARN_STREAM("TebOptimLocalPlannerServer::executeCallback() - path contains just one pose!");
        path.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0);
    }
}


void TebOptimLocalPlannerServer::executeCallback(const trajectory_control_msgs::TrajectoryControlGoalConstPtr &goal_msg)
{
    nav_msgs::Path path = goal_msg->path; 

    // the path is empty and we pre-empt the action server
    if(ros::ok() && path.poses.empty())
    {
        ROS_WARN_STREAM("TebOptimLocalPlannerServer::executeCallback() - path is empty - action canceled ");
        act_server_.setPreempted();
    }

    std::vector<geometry_msgs::PoseStamped> global_plan;
#if 1
    setPosesOrientation(path);
    global_plan = path.poses;
#else
    // just insert the current robot pose and the final pose of the path     
    getRobotPose(tf_robot_pose_);               
    geometry_msgs::PoseStamped robot_pose;
    tfConvert(tf_robot_pose_, robot_pose);

    geometry_msgs::PoseStamped dest = path.poses.back();
    dest.header.stamp = tf_robot_pose_.stamp_; 
    dest.header.frame_id =  tf_robot_pose_.frame_id_;
    float yaw = atan2(dest.pose.position.y - robot_pose.pose.position.y, dest.pose.position.x - robot_pose.pose.position.x);
    dest.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    
    global_plan.push_back(robot_pose);
    global_plan.push_back(dest);
#endif 

    setPlan(global_plan);

    /// start queue path planner stuff
    trajectory_control_msgs::PlanningFeedback planning_feedback_msg;
    planning_feedback_msg.header.stamp = ros::Time::now();
    planning_feedback_msg.node = "control";
    planning_feedback_msg.task = "done";
    planning_feedback_msg.status = STATUS_FAILURE;
    /// end queue path planner stuff 

    /// < send local path
    local_path_msg_.poses.clear();
    local_path_msg_.header.stamp = ros::Time::now();
    local_path_pub_.publish(local_path_msg_);

    /// < send global path 
    //global_path_msg_.poses.clear(); 
    global_path_msg_.header = goal_msg->path.header;
    global_path_msg_.poses = global_plan;
    global_path_pub_.publish(global_path_msg_);

    ros::Rate rate(control_rate_);
    double timestep = rate.expectedCycleTime().nsec / 1e9;

    feedback_msg_.timestep = timestep;

    double estimated_duration = std::numeric_limits<double>::max(); // TODO

    // since we allow reference velocity to change time out cannot be reliably used 
    // moreover, the last GUIs allow to interrupt the trajectory control at any time 
    double time_out = std::numeric_limits<double>::max();

    const geometry_msgs::PoseStamped final_position = path.poses.back();
    
    double current_track_error_xy = 0;

    double time_counter = 0;

    bool b_done = false;
    bool b_time_out = false;
    bool b_path_end = path.poses.empty(); // check if the path is ok 
      
    double final_3D_error = 0; 
    double final_2D_error = 0;
    
    while (ros::ok() && !b_time_out && !b_done)
    {                
        //ROS_INFO(" ---- cycle step ----  ");
        if (act_server_.isPreemptRequested() || !ros::ok())
        {
            b_done = false;
            ROS_INFO("TebOptimLocalPlannerServer: %s: Preempted", action_name.c_str());
            global_plan_msg_.poses.clear();

            // send an empty path message
            local_path_msg_.poses.clear();
            local_path_pub_.publish(local_path_msg_);

            // send an empty path message
            global_path_msg_.poses.clear();
            global_path_pub_.publish(global_path_msg_);

            // stop 
            geometry_msgs::Twist cmd_twist;
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.angular.z = 0.0;
            sendVelCmd(cmd_twist);

            act_server_.setPreempted();

            planning_feedback_msg.status = STATUS_FAILURE;

            break; /// < EXIT FROM LOOP
        }
        else
        {


#if VERBOSE
            ROS_INFO("Computing commands vel....");
            //ROS_INFO("Timestep [%f]",timestep);
            //ROS_INFO("Duration [%f]",duration);
            //ROS_INFO("Counter [%f]",counter);
#endif 

            geometry_msgs::Twist cmd_twist;
            if(computeVelocityCommands(cmd_twist))
            {
              sendVelCmd(cmd_twist);
            }
            else
            {
              std::cout << "ERROR: failed to compute vel commands " << std::endl; 
            }

            final_3D_error = sqrt(pow(final_position.pose.position.x - tf_robot_pose_.getOrigin().getX(), 2) +
                                  pow(final_position.pose.position.y - tf_robot_pose_.getOrigin().getY(), 2) +
                                  pow(final_position.pose.position.z - tf_robot_pose_.getOrigin().getZ(), 2));

            final_2D_error = sqrt(pow(final_position.pose.position.x - tf_robot_pose_.getOrigin().getX(), 2) +
                                  pow(final_position.pose.position.y - tf_robot_pose_.getOrigin().getY(), 2));

            
            //ROS_INFO("Robot tracks vel computed");
                
            feedback_msg_.cmd_vel.linear.x   = cmd_twist.linear.x; //linear_vel_;
            feedback_msg_.cmd_vel.angular.z  = cmd_twist.angular.z; //angular_vel_;
            feedback_msg_.tracks_cmd.left    = 0; //tracks_cmd.left;
            feedback_msg_.tracks_cmd.right   = 0; //tracks_cmd.right;
            feedback_msg_.trajectory_error.x = 0; //pose_ref_B.position.x - tf_robot_pose_map_to_check.getOrigin().getX();
            feedback_msg_.trajectory_error.y = 0; //pose_ref_B.position.y - tf_robot_pose_map_to_check.getOrigin().getY();

            act_server_.publishFeedback(feedback_msg_);
            
            rate.sleep();
            time_counter += timestep;
        }

        b_time_out = time_counter > time_out;
        b_done = isGoalReached(); 

    } /// < end while

    if (b_time_out)
    {
        ROS_INFO("TebOptimLocalPlannerServer: %s: timeout!", action_name.c_str());
        planning_feedback_msg.status = STATUS_FAILURE;

        // stop 
        geometry_msgs::Twist cmd_twist;
        cmd_twist.linear.x = 0.0;
        cmd_twist.linear.y = 0.0;
        cmd_twist.angular.z = 0.0;
        sendVelCmd(cmd_twist);
    }

    if (b_done)
    {
        result_msg_.done = true;
        
        // stop 
        geometry_msgs::Twist cmd_twist;
        cmd_twist.linear.x = 0.0;
        cmd_twist.linear.y = 0.0;
        cmd_twist.angular.z = 0.0;
        sendVelCmd(cmd_twist);

        rate.sleep();
        
        ROS_INFO("TebOptimLocalPlannerServer: %s: Succeeded", action_name.c_str());
        global_plan_msg_.poses.clear();
        act_server_.setSucceeded(result_msg_);

        planning_feedback_msg.task = "done";
        planning_feedback_msg.status = STATUS_SUCCESS;

        getRobotPose(tf_robot_pose_);               
        
        /// < N.B.: controller work only considering x and y components; here we consider the z componenent for sake of completeness 
        final_3D_error = sqrt(pow(final_position.pose.position.x - tf_robot_pose_.getOrigin().getX(), 2) +
                              pow(final_position.pose.position.y - tf_robot_pose_.getOrigin().getY(), 2) +
                              pow(final_position.pose.position.z - tf_robot_pose_.getOrigin().getZ(), 2));

        final_2D_error = sqrt(pow(final_position.pose.position.x - tf_robot_pose_.getOrigin().getX(), 2) +
                              pow(final_position.pose.position.y - tf_robot_pose_.getOrigin().getY(), 2));

        ROS_INFO("TebOptimLocalPlannerServer: final 3D error %f", final_3D_error);
        ROS_INFO("TebOptimLocalPlannerServer: final 2D error %f", final_2D_error);

        if (!b_local_path_)
        {
            queue_task_feedback_pub_.publish(planning_feedback_msg);
        }


    }
}

void TebOptimLocalPlannerServer::robotPathCallBack(const nav_msgs::PathConstPtr &msg)
{
    std::cout << "TebOptimLocalPlannerServer::robotPathCallBack()" << std::endl;

    act_client_.waitForServer();
    trajectory_control_msgs::TrajectoryControlGoal track_goal;
    track_goal.path = *msg;
    ROS_INFO("TebOptimLocalPlannerServer: starting new path - path size: %d",(int)track_goal.path.poses.size());
    b_local_path_ = false;
    b_simple_rotation_ = false; 
    act_client_.sendGoal(track_goal);
}

void TebOptimLocalPlannerServer::robotLocalPathCallBack(const nav_msgs::PathConstPtr &msg)
{
    std::cout << "TebOptimLocalPlannerServer::robotLocalPathCallBack()" << std::endl;

    act_client_.waitForServer();
    trajectory_control_msgs::TrajectoryControlGoal track_goal;
    track_goal.path = *msg;
    ROS_INFO("TebOptimLocalPlannerServer: starting new path - path size: %d",(int)track_goal.path.poses.size());
    b_local_path_ = true;
    b_simple_rotation_ = false;     
    act_client_.sendGoal(track_goal);
}


void TebOptimLocalPlannerServer::queueFeedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg)
{
    // if this message have been published by the tool node, then
    if (!feedback_msg.node.compare("tool"))
    {
        trajectory_control_msgs::PlanningFeedback feedback_msg;
        feedback_msg.header.stamp = ros::Time::now();
        feedback_msg.node = "control";
        feedback_msg.task = "stop";

        act_client_.waitForServer();

        if (act_client_.getState() == actionlib::SimpleClientGoalState::LOST) // if no goal is running
        {
            ROS_WARN("TebOptimLocalPlannerServer: controller already stopped");
            feedback_msg.status = STATUS_FAILURE;
        }
        else
        {
            ROS_INFO("TebOptimLocalPlannerServer: cancelling current goal");

            act_client_.cancelGoal();

            ROS_INFO("TebOptimLocalPlannerServer: controller stopped (ready)");
            feedback_msg.status = STATUS_SUCCESS;
        }

        queue_task_feedback_pub_.publish(feedback_msg);
    }
}

void TebOptimLocalPlannerServer::queueTaskCallback(const nav_msgs::Path& path_msg)
{
    trajectory_control_msgs::PlanningFeedback feedback_msg;
    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.node = "control";
    feedback_msg.task = "start";

    act_client_.waitForServer();

    if (act_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE) // current goal is running
    {
        ROS_WARN("TebOptimLocalPlannerServer: controller already started, task dropped");
        feedback_msg.status = STATUS_FAILURE;
    }
    else
    {
        trajectory_control_msgs::TrajectoryControlGoal track_goal;
        track_goal.path = path_msg;
        act_client_.sendGoal(track_goal);

        ROS_INFO("TebOptimLocalPlannerServer: controller started (busy)");
        feedback_msg.status = STATUS_SUCCESS;
    }

    queue_task_feedback_pub_.publish(feedback_msg);
}

void TebOptimLocalPlannerServer::goalAbortCallback(const std_msgs::Bool& msg)
{
    act_client_.waitForServer();

    if (act_client_.getState() == actionlib::SimpleClientGoalState::LOST) // if no goal is running
    {
        ROS_WARN("TebOptimLocalPlannerServer: controller already stopped");
    }
    else
    {
        ROS_INFO("TebOptimLocalPlannerServer: cancelling current goal");

        act_client_.cancelGoal();

        ROS_INFO("TebOptimLocalPlannerServer: controller stopped (ready)");
    }
}

void TebOptimLocalPlannerServer::sendVelCmd(const geometry_msgs::Twist& cmd_twist)
{
    last_cmd_twist_ = cmd_twist;
    cmd_pub_.publish(cmd_twist);

    if(enable_track_cmds_)
    {
        nifti_robot_driver_msgs::Tracks tracks_cmd;
        getTracksVelCmd(cmd_twist, tracks_cmd);
        tracks_vel_cmd_pub_.publish(tracks_cmd);
    }
}

void TebOptimLocalPlannerServer::getTracksVelCmd(const geometry_msgs::Twist& cmd_twist, nifti_robot_driver_msgs::Tracks& tracks_cmd)
{
    const double linear_vel = cmd_twist.linear.x;
    const double angular_vel = cmd_twist.angular.z;
    const double d = 0.5*tracks_distance_;
    
    // compute tracks velocities from v and omega 
    tracks_cmd.left  = linear_vel - (d * angular_vel);
    tracks_cmd.right = linear_vel + (d * angular_vel);
}

} // end namespace teb_local_planner


