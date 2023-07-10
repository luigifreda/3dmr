/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DPATROLLING. 
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

#include <math.h> 
#include <TrajectoryControlActionServer.h>

#include <nifti_teleop/Acquire.h>
#include <nifti_teleop/Release.h>


#define VERBOSE 1


const double TrajectoryControlActionServer::kTrackDistance = 0.397; // [m] distance between tracks/wheels
const double TrajectoryControlActionServer::kWheelRadius = 0.07;    // [m] wheel radius 
const float TrajectoryControlActionServer::kRobotRadius = 0.5; //0.5; //0.45;  // [m] robot radius for computing the clearance
const float TrajectoryControlActionServer::kProximityDistanceThreshold = 0.2; // 0.5 // [m] distance point-to-robot to enable velocity reduction 
const float TrajectoryControlActionServer::kProximityActiveAngle = 60.*M_PI/180.0; // [rad]

const double TrajectoryControlActionServer::kDefaulRefVelocity = 0.2; //[m/s]  default linear velocity in m/s
const double TrajectoryControlActionServer::kMaxTrackVelocity = 0.3; //[m/s]  maximum track velocity in m/s
const double TrajectoryControlActionServer::kMaxTrackVelocityOnRotation = 0.2; //[m/s]  maximum track velocity in m/s
const double TrajectoryControlActionServer::kMaxAngularVelocity = 2*TrajectoryControlActionServer::kMaxTrackVelocityOnRotation/TrajectoryControlActionServer::kTrackDistance; //[m/s]  maximum track velocity in m/s

const double TrajectoryControlActionServer::kLaserProximityReducedVelocityFactor = 1.0;//0.5; // [frac %] reducing factor for linear velocity when laser proximity is detected  
const double TrajectoryControlActionServer::kLaserProximityReducedVelocityResetTime = 1.0; // [s] if elapsed time from last laser proximity check is > kLaserProximityReducedVelocityResetTime, the normal velocity is set back 


const int TrajectoryControlActionServer::kDefaultControlLaw = TrajectoryControlActionServer::kInputOutputFL; //kNonLinear; // default control law (should be one in the ControlLawType list)
const double TrajectoryControlActionServer::kDefaultControlFrequency = 15; //[Hz]  default control frequency 
//const double TrajectoryControlActionServer::kMinimumWaitingTimeforANewPath = 0.5; // [s] minimum time to wait for giving a new path as input 
const double TrajectoryControlActionServer::kTimeOutTolerance = 10; // [s] this time will be added to the expected trajectory execution time in order to define a time out

const double TrajectoryControlActionServer::kDefaultControlGainK1_IOL = 0.3; // default control gain k1 for IOL
const double TrajectoryControlActionServer::kDefaultControlGainK2_IOL = 0.3; // default control gain k2 for IOL
const double TrajectoryControlActionServer::kDefaultControlGainK1_NL = 0.3; // default control gain k1 for NL
const double TrajectoryControlActionServer::kDefaultControlGainK2_NL = 0.3; // default control gain k2 for NL
const double TrajectoryControlActionServer::kDefaultControlGainKv_CR = 0.3; // default control gain k1 for cartesian regulation
const double TrajectoryControlActionServer::kDefaultControlGainKw_CR = 0.2; // default control gain k2 for catersian regulation 
const double TrajectoryControlActionServer::kDefaultAngularGainKw = 0.2; // default control angular gain kw: a pure rotation control is applied if the angular error is above kRefAngularErrorForPureRotationControl
const double TrajectoryControlActionServer::kRefAngularErrorForPureRotationControl = M_PI / 4;
const double TrajectoryControlActionServer::kRefXYErrorForPureRotationControl = 0.5;

const double TrajectoryControlActionServer::kAngularErrorThreshold = 4 * M_PI/180.; // [rad] threshold for stopping rotational control 

const double TrajectoryControlActionServer::kDefaultDistanceOffsetPointB = 0.1; // [m] distance offset of point B from robot center (this is used in the control law) 
// position of point B from robot pose (x,y,theta)
// xB = x + displacement * cos(theta)
// yB = y + displacement * sin(theta)

const double TrajectoryControlActionServer::kAdpativeTravFreq = 5; // [Hz] default refresh frequency for adaptive traversability velocity
const double TrajectoryControlActionServer::kAdpativeTravFilterWn = 2 * M_PI*kAdpativeTravFreq / 18.; // [rad/s] default natural frequency for the adaptive traversability velocity filter  
// 2*M_PI*kAdpativeTravFreq/18. corresponds to a rise time of 1 sec approximatively 

const double TrajectoryControlActionServer::kRefErrorForStoppingPathManagerStep = 5 * kDefaultDistanceOffsetPointB; // max reference error for allowing path manager to step  
const double TrajectoryControlActionServer::kFinalRefErrorForStoppingTrajControl = 1 * kDefaultDistanceOffsetPointB; // final reference error for stopping trajectory control   
//const double TrajectoryControlActionServer::k3DDistanceArrivedToGoal = 4 * kDefaultDistanceOffsetPointB; // [m]
//const double TrajectoryControlActionServer::k2DDistanceArrivedToGoal = 2 * kDefaultDistanceOffsetPointB; // [m]

const std::string TrajectoryControlActionServer::kTeleopMuxPriorityName = "/nav/cmd_vel"; 

TrajectoryControlActionServer::TrajectoryControlActionServer(std::string name) :
param_node_("~"),
action_name(name),
act_server_(node_, name, boost::bind(&TrajectoryControlActionServer::executeCallback, this, _1), false),
act_client_(name, true),
//odom_frame_id("/odom"),
//global_frame_id("map"),
//robot_frame_id("/base_link"),
//fl_frame_id("/front_left_flipper"),
//fr_frame_id("/front_right_flipper"),
//rl_frame_id("/rear_left_flipper"),
//rr_frame_id("/rear_right_flipper"),
//imu_frame_id("/imu"),
//displacement(0.2),
//vel_reference(0.15),
//imu_odom_topic("/imu_odom"),
//tracks_vel_cmd_topic("/tracks_vel_cmd"),
linear_vel_(0),
angular_vel_(0),
robot_width_(kTrackDistance),
wheel_radius_(kWheelRadius),
ref_error_for_stopping_path_manager_step(kRefErrorForStoppingPathManagerStep),
//k1(0.3),
//k2(0.3),
//delay(10),
//global_path_topic("global_path"),
//local_path_topic("local_path"),
//cmd_topic("cmd_vel"),
//imu_topic("/imu/data")
filter_vel_trav_(1. / kAdpativeTravFreq, kAdpativeTravFilterWn),
b_local_path_(false),
b_simple_rotation_(false),
b_decreased_vel_(false),
use_max_angular_vel_check_(false),
velocity_before_reduction_(0.)
{

    /// <  get parameters 
    
    str_robot_name_ = getParam<std::string>(param_node_, "robot_name", "ugv1");   /// < multi-robot
    robot_id_ = atoi(str_robot_name_.substr(3,str_robot_name_.size()).c_str()) - 1;

    std::string simulator_name  = getParam<std::string>(param_node_, "simulator", "");   /// < multi-robot
    
    bool b_use_at  = getParam<bool>(param_node_, "use_at", false);   /// < use adaptive traversability    
    if(b_use_at)
    {
        p_path_manager_.reset(new PathManagerKdt);
        ROS_INFO_STREAM("TrajectoryControlActionServer() - using ADAPTIVE TRAVERSABILITY!");
    }
    else
    {
        p_path_manager_.reset(new PathManager);        
    }
                
    odom_frame_id_ = getParam<std::string>(param_node_, "odom_frame_id", "/odom");
    global_frame_id_ = getParam<std::string>(param_node_, "global_frame_id", "map");
    robot_frame_id_ = getParam<std::string>(param_node_, "robot_frame_id", "/base_link");
    fl_frame_id_ = getParam<std::string>(param_node_, "fl_frame_id", "/front_left_flipper");
    fr_frame_id_ = getParam<std::string>(param_node_, "fr_frame_id", "/front_right_flipper");
    rl_frame_id_ = getParam<std::string>(param_node_, "rl_frame_id", "/rear_left_flipper");
    rr_frame_id_ = getParam<std::string>(param_node_, "rr_frame_id", "/rear_right_flipper");
    imu_frame_id_ = getParam<std::string>(param_node_, "imu_frame_id", "/imu");

    transform_tolerance_ = getParam<double>(param_node_, "transform_tolerance", 0.2);

    robot_width_ = getParam<double>(param_node_, "robot_width", kTrackDistance);
    wheel_radius_ = getParam<double>(param_node_, "wheel_radius", kWheelRadius);

    enable_flippers_ = getParam<bool>(param_node_, "enable_flippers", enable_flippers_);

    displacement_ = getParam<double>(param_node_, "displacement", kDefaultDistanceOffsetPointB);

    cruise_vel_ = getParam<double>(param_node_, "cruise_vel", kDefaulRefVelocity);
    curr_vel_ = cruise_vel_; 

    rise_time_ = getParam<double>(param_node_, "rise_time", 2.0);

    k1_IOL_ = getParam<double>(param_node_, "gain_k1_IOL", kDefaultControlGainK1_IOL);
    k2_IOL_ = getParam<double>(param_node_, "gain_k2_IOL", kDefaultControlGainK2_IOL);
    k1_NL_ = getParam<double>(param_node_, "gain_k1_NL", kDefaultControlGainK1_NL);
    k2_NL_ = getParam<double>(param_node_, "gain_k2_NL", kDefaultControlGainK2_NL);
    kv_CR_ = getParam<double>(param_node_, "gain_kv_CR", kDefaultControlGainKv_CR);
    kw_CR_ = getParam<double>(param_node_, "gain_kw_CR", kDefaultControlGainKw_CR);
    kw_ = getParam<double>(param_node_, "gain_kw", kDefaultAngularGainKw);
    control_frequency_ = getParam<double>(param_node_, "control_frequency", kDefaultControlFrequency);

    imu_odom_topic_ = getParam<std::string>(param_node_, "imu_odom_topic", "/imu_odom");
    tracks_vel_cmd_topic_ = getParam<std::string>(param_node_, "tracks_vel_cmd_topic", "/tracks_vel_cmd");
    global_path_topic_ = getParam<std::string>(param_node_, "global_path_topic", "/global_path");
    local_path_topic_ = getParam<std::string>(param_node_, "local_path_topic", "/local_path");
    cmd_topic_ = getParam<std::string>(param_node_, "cmd_vel_topic", "/cmd_vel");
    cmd_wheels_topic_ = getParam<std::string>(param_node_, "cmd_wheels_topic", "/cmd_wheels_topic");
    imu_topic_ = getParam<std::string>(param_node_, "imu_topic", "/imu/data");

    robot_path_topic_ = getParam<std::string>(param_node_, "robot_path_topic", "/robot_path");
    robot_local_path_topic_ = getParam<std::string>(param_node_, "robot_local_path_topic", "/robot_local_path");
    robot_rotation_topic_ = getParam<std::string>(param_node_, "robot_rotation_topic", "/robot_rotation");    

    vel_max_tracks_ = getParam<double>(param_node_, "vel_max_tracks", kMaxTrackVelocity);
    if(vel_max_tracks_ < cruise_vel_) vel_max_tracks_ = cruise_vel_;

    use_max_angular_vel_check_ =  getParam<bool>(param_node_, "use_max_angular_vel_check", false);

    adapt_trav_vel_topic_ = getParam<std::string>(param_node_, "adapt_trav_vel_topic", "/adapt_trav_vel");
    adaptive_trav_vel_freq_ = getParam<double>(param_node_, "adaptive_trav_vel_freq", kAdpativeTravFreq);

    queue_task_feedback_topic_ = getParam<std::string>(param_node_, "queue_task_feedback_topic", "/planner/tasks/feedback");
    queue_task_path_topic_ = getParam<std::string>(param_node_, "queue_task_path_topic", "/planner/tasks/path");
    //queue_task_path_local_topic_ = getParam<std::string>(param_node_, "queue_task_path_local_topic", "/planner/tasks/local_path");
    
    goal_abort_topic_ = getParam<std::string>(param_node_, "goal_abort_topic", "/goal_abort_topic");
    trajectory_control_abort_topic_ = getParam<std::string>(param_node_, "trajectory_control_abort_topic", "/trajectory_control_abort_topic");
    
    laser_proximity_topic_ = getParam<std::string>(param_node_, "laser_proximity_topic", "/laser_proximity_topic");
    
    closest_obst_point_topic_ = getParam<std::string>(param_node_, "closest_obst_point_topic", "/closest_obst_point"); 
    
    closest_obst_vel_reduction_enable_topic_ = getParam<std::string>(param_node_, "closest_obst_vel_reduction_enable_topic_", "/closest_obst_vel_reduction_enable"); 
    
    multi_robot_paths_topic_ = getParam<std::string>(param_node_, "multi_robot_paths_topic", "/multi_robot_paths");

    //control_law_type_ = (ControlLawType) kNonLinear;//kDefaultControlLaw;    
    control_law_type_ = (ControlLawType) getParam<int>(param_node_, "control_law_type", (int)kDefaultControlLaw);    

    // path smoother type 
    path_smoother_type = (PathSmoother::PathSmootherType) getParam<int>(param_node_, "path_smoother_type", (int)PathSmoother::PathSmootherType::kNoSmoother);    

    ref_error_for_stopping_path_manager_step =  getParam<double>(param_node_, "ref_error_for_stopping_path_manager_step", kRefErrorForStoppingPathManagerStep);


    /// < setup subcribers 

    robot_pp_path_sub_ = node_.subscribe("/robot_pp_path", 1, &TrajectoryControlActionServer::robotPathCallBack, this);

    imu_odom_sub_ = node_.subscribe(imu_odom_topic_, 1, &TrajectoryControlActionServer::imuOdomCallback, this);

    robot_path_sub_ = node_.subscribe(robot_path_topic_, 1, &TrajectoryControlActionServer::pathCallBack, this);
    robot_loal_path_sub_ = node_.subscribe(robot_local_path_topic_, 1, &TrajectoryControlActionServer::localPathCallBack, this);
    robot_rotation_sub_ = node_.subscribe(robot_rotation_topic_, 1, &TrajectoryControlActionServer::rotationCallBack, this);    

    adapt_trav_vel_sub_ = node_.subscribe(adapt_trav_vel_topic_, 1, &TrajectoryControlActionServer::adaptTravVelCallback, this);

    if(enable_flippers_)
    {
        //imu_sub = node.subscribe(imu_topic,1,&TrajectoryControlActionServer::imuDataCallback,this);
    }

    // feedback between nodes: "tool", "planner" and "control"
    queue_task_feedback_sub_ = node_.subscribe(queue_task_feedback_topic_, 20, &TrajectoryControlActionServer::queueFeedbackCallback, this);

    // get the task's paths
    queue_task_path_sub_ = node_.subscribe(queue_task_path_topic_, 1, &TrajectoryControlActionServer::queueTaskCallback, this);
    //queue_task_path_local_sub_ = node_.subscribe(queue_task_path_local_topic_, 1, &TrajectoryControlActionServer::queueLocalTaskCallback, this);
    
    goal_abort_sub_ = node_.subscribe(goal_abort_topic_, 1, &TrajectoryControlActionServer::goalAbortCallback, this);
    trajectory_control_abort_sub_ = node_.subscribe(trajectory_control_abort_topic_, 1, &TrajectoryControlActionServer::goalAbortCallback, this); /// < use the same callback of goal abort!

    laser_proximity_sub_ = node_.subscribe(laser_proximity_topic_, 1, &TrajectoryControlActionServer::laserProximityCallback, this);
    
    closest_obst_point_sub_ = node_.subscribe(closest_obst_point_topic_, 1, &TrajectoryControlActionServer::closestObstaclePointCallback, this);
    
    closest_obst_vel_reduction_enable_sub_ = node_.subscribe(closest_obst_vel_reduction_enable_topic_, 1, &TrajectoryControlActionServer::closestObstacleVelReductionEnableCallback, this);
                

    /// < setup publishers 

    tracks_vel_cmd_pub_ = node_.advertise<nifti_robot_driver_msgs::Tracks>(tracks_vel_cmd_topic_, 1);

    global_path_pub_ = node_.advertise<nav_msgs::Path>("/traj_global_path", 1);

    local_path_pub_ = node_.advertise<nav_msgs::Path>("/traj_local_path", 1);

    smoothed_path_pub_ = node_.advertise<nav_msgs::Path>("/traj_smoothed_path", 1);

    cmd_pub_ = node_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
    cmd_wheels_pub_ = node_.advertise<trajectory_control_msgs::VelocityCommand>(cmd_wheels_topic_, 1);

    // feedback between nodes: "tool", "planner" and "control"
    queue_task_feedback_pub_ = node_.advertise<trajectory_control_msgs::PlanningFeedback>(queue_task_feedback_topic_, 1);

    multi_robot_paths_pub_ = node_.advertise<trajectory_control_msgs::MultiRobotPath>(multi_robot_paths_topic_, 1);

    /// < wait the first robot pose 
    //while(!getRobotPose(current_real_robot_pose_odom,real_robot_pose_map,from_odom_to_map))
    while (!getRobotPose2(tf_robot_pose_odom_, tf_robot_pose_map_, tf_odom_to_map_, tf_front_left_flipper_, tf_front_right_flipper, tf_rear_left_flipper_, tf_rear_right_flipper_, tf_imu_t))
    {
        ROS_INFO("Waiting for transformation");
    }
    getRealRobotPoseB(displacement_, tf_robot_pose_map_, tf_robot_poseB_map_);

    /// < start the action server 
    act_server_.start();

    /// < init vars 
    adapt_trav_vel_ = 0;

    global_path_msg_.header.frame_id = global_frame_id_;
    global_path_msg_.header.stamp = ros::Time::now();

    local_path_msg_.header.frame_id = global_frame_id_;
    local_path_msg_.header.stamp = ros::Time::now();
    
    b_closest_obst_vel_reduction_enable_ = true;
}

TrajectoryControlActionServer::~TrajectoryControlActionServer()
{
}

void TrajectoryControlActionServer::odomMsgToStampedTransform(nav_msgs::Odometry pose_odometry_msg, tf::StampedTransform& pose_stamped_tf)
{
    pose_stamped_tf.stamp_ = pose_odometry_msg.header.stamp;
    pose_stamped_tf.frame_id_ = pose_odometry_msg.header.frame_id;
    pose_stamped_tf.child_frame_id_ = pose_odometry_msg.child_frame_id;

    tf::Vector3 v;
    v.setX(pose_odometry_msg.pose.pose.position.x);
    v.setY(pose_odometry_msg.pose.pose.position.y);
    v.setZ(pose_odometry_msg.pose.pose.position.z);

    pose_stamped_tf.setOrigin(v);

    tf::Quaternion q;
    q.setX(pose_odometry_msg.pose.pose.orientation.x);
    q.setY(pose_odometry_msg.pose.pose.orientation.y);
    q.setZ(pose_odometry_msg.pose.pose.orientation.z);
    q.setW(pose_odometry_msg.pose.pose.orientation.w);

    pose_stamped_tf.setRotation(q);
}

// compute the position of point B from robot pose (x,y,theta)
// xB = x + displacement * cos(theta)
// yB = y + displacement * sin(theta)
void TrajectoryControlActionServer::getRealRobotPoseB(double displacement, tf::StampedTransform real_robot_pose, tf::StampedTransform& real_robot_poseB)
{
    real_robot_poseB.frame_id_       = real_robot_pose.frame_id_;
    real_robot_poseB.stamp_          = real_robot_pose.stamp_;
    real_robot_poseB.child_frame_id_ = real_robot_pose.child_frame_id_;

    tf::Vector3 v;
    double roll, pitch, yaw;
    real_robot_pose.getBasis().getRPY(roll, pitch, yaw);

    v.setX(real_robot_pose.getOrigin().getX() + displacement * cos(yaw));
    v.setY(real_robot_pose.getOrigin().getY() + displacement * sin(yaw));
    v.setZ(real_robot_pose.getOrigin().getZ());
    real_robot_poseB.setOrigin(v);

    real_robot_poseB.setRotation(real_robot_pose.getRotation());
}

bool TrajectoryControlActionServer::getRobotPose(tf::StampedTransform& robot_pose_odom, tf::StampedTransform& robot_pose_map, tf::StampedTransform& from_odom_to_map)
{

    if (tf_listener_.waitForTransform(global_frame_id_, robot_frame_id_, ros::Time(), ros::Duration(transform_tolerance_)))
    {
        try
        {
            tf_listener_.lookupTransform(global_frame_id_, robot_frame_id_, ros::Time(), robot_pose_map);
            tf_listener_.lookupTransform(odom_frame_id_, robot_frame_id_, ros::Time(), robot_pose_odom);
            tf_listener_.lookupTransform(global_frame_id_, odom_frame_id_, ros::Time(), from_odom_to_map);
        }
        catch (tf::LookupException& ex)
        {
            ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_INFO("Transformation is not available");
        return false;
    }
}

bool TrajectoryControlActionServer::getRobotPose2(tf::StampedTransform& robot_pose_odom, tf::StampedTransform& robot_pose_map, tf::StampedTransform& from_odom_to_map, tf::StampedTransform& frontLeftF, tf::StampedTransform& frontRightF, tf::StampedTransform& rearLeftF, tf::StampedTransform& rearRightF, tf::StampedTransform& imu_t)
{

    if (tf_listener_.waitForTransform(global_frame_id_, robot_frame_id_, ros::Time(), ros::Duration(1.0)))
    {
        try
        {
            tf_listener_.lookupTransform(global_frame_id_, robot_frame_id_, ros::Time(), robot_pose_map);
            tf_listener_.lookupTransform(odom_frame_id_, robot_frame_id_, ros::Time(), robot_pose_odom);
            tf_listener_.lookupTransform(global_frame_id_, odom_frame_id_, ros::Time(), from_odom_to_map);
            
            if(enable_flippers_)
            {
                tf_listener_.lookupTransform(global_frame_id_, fl_frame_id_, ros::Time(), frontLeftF);
                tf_listener_.lookupTransform(global_frame_id_, fr_frame_id_, ros::Time(), frontRightF);
                tf_listener_.lookupTransform(global_frame_id_, rl_frame_id_, ros::Time(), rearLeftF);
                tf_listener_.lookupTransform(global_frame_id_, rr_frame_id_, ros::Time(), rearRightF);
            }
            //tf_listener_.lookupTransform(global_frame_id,imu_frame_id,ros::Time(),imu_t);
        }
        catch (tf::LookupException& ex)
        {
            ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_INFO("Transformation is not available");
        return false;
    }
}

// resample the trajectory in order to have a point for each delta = vel*Ts

void TrajectoryControlActionServer::resampleAndSmoothPath(const nav_msgs::Path& path_in, nav_msgs::Path& path_out)
{
    double vel = cruise_vel_;
    ros::Rate rate(control_frequency_);
    double time_step = rate.expectedCycleTime().nsec / 1e9; // durata dell'intervallo temporale

    path_out.header.frame_id = path_in.header.frame_id;
    path_out.header.stamp = path_in.header.stamp;

    double dist_step;

    geometry_msgs::PoseStamped current;
    current.header = path_in.poses[0].header;
    current.pose.position = path_in.poses[0].pose.position; // start from first point 

    geometry_msgs::PoseStamped new_point;

    double current_dist_to_next;
    double yaw;
    double offset = 0.0;
    bool is_change_point = false;

    int i = 1; // this is the index of next point 
    //while (i < (path_in.poses.size() - 1))
    while (i < path_in.poses.size())
    {
        //if (current.pose.position.x == path_in.poses[i].pose.position.x && current.pose.position.y == path_in.poses[i].pose.position.y)
        if (FEQUAL(current.pose.position.x, path_in.poses[i].pose.position.x, 1e-4) && FEQUAL(current.pose.position.y, path_in.poses[i].pose.position.y, 1e-4))
        {
            i++; // get the new point in the list 
        }
        else
        {
            if (is_change_point)
            {
                dist_step = fabs(vel * time_step - offset); // get a shorter step 
                current_dist_to_next = sqrt(pow(path_in.poses[i].pose.position.x - new_point.pose.position.x, 2) + pow(path_in.poses[i].pose.position.y - new_point.pose.position.y, 2));
                //ROS_INFO("changePoint current distance [%f]",current_dist);
                //ROS_INFO("changePoint distance [%f]",distance);
                //ROS_INFO("Offset: [%f]",offset);
                //ROS_INFO_STREAM("change point event");

                yaw = atan2(path_in.poses[i].pose.position.y - new_point.pose.position.y, path_in.poses[i].pose.position.x - new_point.pose.position.x);
                current.pose.orientation = tf::createQuaternionMsgFromYaw(yaw); // added 
            }
            else
            {
                dist_step = vel * time_step;
                current_dist_to_next = sqrt(pow(path_in.poses[i].pose.position.x - current.pose.position.x, 2) + pow(path_in.poses[i].pose.position.y - current.pose.position.y, 2));

                yaw = atan2(path_in.poses[i].pose.position.y - current.pose.position.y, path_in.poses[i].pose.position.x - current.pose.position.x);
                current.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

                offset = 0.0;
            }
            //ROS_INFO(" offset [%f]",offset);
            //ROS_INFO("distance [%f]",distance);
            //ROS_INFO("curr distance [%f]",current_dist);
            if (current_dist_to_next >= dist_step)
            {
                geometry_msgs::PoseStamped temp;
                temp.header = path_in.poses[i].header;
                temp.pose.position.x = current.pose.position.x + (dist_step * cos(yaw));
                temp.pose.position.y = current.pose.position.y + (dist_step * sin(yaw));
                temp.pose.position.z = 0;

                if (is_change_point)
                {
                    yaw = atan2(temp.pose.position.y - current.pose.position.y, temp.pose.position.x - current.pose.position.x);
                    current.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                }

                temp.pose.orientation = current.pose.orientation;

                path_out.poses.push_back(current);
                //ROS_INFO_STREAM("added point: current_dist: "<< current_dist << " maggiore di distance: " << distance);
                //ROS_INFO_STREAM("new distance from added point: "<< sqrt(pow(in.poses[i].pose.position.x - temp.pose.position.x,2) + pow(in.poses[i].pose.position.y - temp.pose.position.y,2)));
                current.header = temp.header;
                current.pose = temp.pose;
                is_change_point = false;
                //add to global_pose
            }
            else
            {
                // the dist_step brings us beyond path_in.poses[i]
                // => we take path_in.poses[i] as intermediate point 
                // => we store the offset and we recover the missed distance in the next iteration 
                offset += fabs(current_dist_to_next - dist_step); // absolute value!
                new_point.header = path_in.poses[i].header;
                new_point.pose = path_in.poses[i].pose;
                is_change_point = true;

                i++; // get the new point in the list 
            }

        }
        //ROS_INFO_STREAM("Index "<<i<< "");
    }

    ROS_INFO("Number of points input path [%ld]", path_in.poses.size());
    ROS_INFO("Number of points exe path [%ld]", path_out.poses.size());

}

// cruise_vel_, global_plan_msg_.poses[index], poseB, velB
void TrajectoryControlActionServer::buildReferenceTrajectory(double vel_ref, const geometry_msgs::PoseStamped& pose_ref, geometry_msgs::Pose& pose_ref_B, geometry_msgs::Twist& vel_ref_B)
{
#if VERBOSE
    std::cout <<  "TrajectoryControlActionServer::buildReferenceTrajectory()" << std::endl; 
#endif 

    pose_ref_B = pose_ref.pose; // the reference trajectory is applied on point B as is

    double yaw = tf::getYaw(pose_ref.pose.orientation);

    // we compute the components of the reference velocity 
    vel_ref_B.linear.x = vel_ref * cos(yaw);
    vel_ref_B.linear.y = vel_ref * sin(yaw);
    vel_ref_B.angular.z = p_path_manager_->getAngularVel();
}

// compute control law and return the current error - Input-Output Linearization 
double TrajectoryControlActionServer::computeControlLawIOLin(const tf::StampedTransform& tf_robot_pose_B, double k1, double k2, const geometry_msgs::Pose& ref_pose_B, const geometry_msgs::Twist& ref_vel_B, double& linear_vel, double& angular_vel)
{
    double roll, pitch, yaw;
    tf_robot_pose_B.getBasis().getRPY(roll, pitch, yaw);

    double error_yaw = diffS1(tf::getYaw(ref_pose_B.orientation), yaw);

    double err_x = (ref_pose_B.position.x - tf_robot_pose_B.getOrigin().getX());
    double err_y = (ref_pose_B.position.y - tf_robot_pose_B.getOrigin().getY());
    double error_xy = sqrt(err_x * err_x + err_y * err_y);

    if ((fabs(error_yaw) > kRefAngularErrorForPureRotationControl) && (error_xy > kRefXYErrorForPureRotationControl))
    {
#if VERBOSE
        std::cout << "computeControlLawIOLin() - pure rotational control " << std::endl;
#endif
        // pure rotational control 
        linear_vel = 0;
        angular_vel = kw_*error_yaw;
    }
    else
    {
#if VERBOSE
        std::cout << "computeControlLawIOLin() - normal control - 2D err:  " << error_xy << std::endl;
#endif
        double cyaw = cos(yaw);
        double syaw = sin(yaw);

        double a = cyaw;
        double b = syaw;
        double c = -syaw / displacement_;
        double d = cyaw / displacement_;

        double u1 = ref_vel_B.linear.x + k1 * err_x;
        double u2 = ref_vel_B.linear.y + k2 * err_y;

        //double u1 = k1 * (poseB.position.x - robot_pose.getOrigin().getX());
        //double u2 = k2 * (poseB.position.y - robot_pose.getOrigin().getY());

        linear_vel  = a * u1 + b * u2;
        angular_vel = c * u1 + d * u2;
    }

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = tf_robot_pose_B.frame_id_;
    robot_pose.header.stamp    = tf_robot_pose_B.stamp_;
    robot_pose.pose.position.x = tf_robot_pose_B.getOrigin().getX() - displacement_ * cos(yaw); // compute the position of the robot center starting from point B info
    robot_pose.pose.position.y = tf_robot_pose_B.getOrigin().getY() - displacement_ * sin(yaw);
    robot_pose.pose.position.z = tf_robot_pose_B.getOrigin().getZ();
    robot_pose.pose.orientation.x = tf_robot_pose_B.getRotation().getX();
    robot_pose.pose.orientation.y = tf_robot_pose_B.getRotation().getY();
    robot_pose.pose.orientation.z = tf_robot_pose_B.getRotation().getZ();
    robot_pose.pose.orientation.w = tf_robot_pose_B.getRotation().getW();
    local_path_msg_.poses.push_back(robot_pose);
    local_path_pub_.publish(local_path_msg_);

    return error_xy;
}

// compute control law and return the current error - non-linear position control (Cartesian Regulation) 
double TrajectoryControlActionServer::computeControlLawPosition(const tf::StampedTransform& tf_robot_pose, double kv, double kw, const geometry_msgs::PoseStamped& target_pose, double& linear_vel, double& angular_vel)
{
    double roll, pitch, yaw;
    tf_robot_pose.getBasis().getRPY(roll, pitch, yaw);

    //double error_yaw = diffS1(tf::getYaw(ref_pose_B.orientation), yaw);

    double err_x = (target_pose.pose.position.x - tf_robot_pose.getOrigin().getX());
    double err_y = (target_pose.pose.position.y - tf_robot_pose.getOrigin().getY());
    double error_xy = sqrt(err_x * err_x + err_y * err_y);
     
    // point towards the target 
    double target_yaw = atan2(err_y, err_x);
    
    double err_yaw = diffS1(target_yaw, yaw);   

    if ((fabs(err_yaw) > kRefAngularErrorForPureRotationControl) && (error_xy > kRefXYErrorForPureRotationControl))
    {
#if VERBOSE
        std::cout << "computeControlLawPosition() - pure rotational control " << std::endl;
#endif
        // pure rotational control 
        linear_vel  = 0;
        angular_vel = kw_*err_yaw;
    }
    else
    {
#if VERBOSE
        std::cout << "computeControlLawPosition() - normal control - 2D err:  " << error_xy << std::endl;
#endif

        linear_vel = kv * (err_x * cos(yaw) + err_y * sin(yaw));
        if (error_xy > 1e-2)
        {
            angular_vel = kw * err_yaw;
        }
        else
        {
            angular_vel = 0;
        }
    }

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = tf_robot_pose.frame_id_;
    robot_pose.header.stamp = tf_robot_pose.stamp_;
    robot_pose.pose.position.x = tf_robot_pose.getOrigin().getX(); 
    robot_pose.pose.position.y = tf_robot_pose.getOrigin().getY();
    robot_pose.pose.position.z = tf_robot_pose.getOrigin().getZ();
    robot_pose.pose.orientation.x = tf_robot_pose.getRotation().getX();
    robot_pose.pose.orientation.y = tf_robot_pose.getRotation().getY();
    robot_pose.pose.orientation.z = tf_robot_pose.getRotation().getZ();
    robot_pose.pose.orientation.w = tf_robot_pose.getRotation().getW();
    local_path_msg_.poses.push_back(robot_pose);
    local_path_pub_.publish(local_path_msg_);

    return error_xy;
}

// compute control law and return the current error - Non-Linear control law  
double TrajectoryControlActionServer::computeControlLawNonLin(const tf::StampedTransform& tf_robot_pose, double k1, double k2, const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& ref_vel, double& linear_vel, double& angular_vel)
{
    double roll, pitch, yaw;
    tf_robot_pose.getBasis().getRPY(roll, pitch, yaw);

    double error_yaw = diffS1(tf::getYaw(ref_pose.orientation), yaw);

    double err_x = (ref_pose.position.x - tf_robot_pose.getOrigin().getX());
    double err_y = (ref_pose.position.y - tf_robot_pose.getOrigin().getY());
    double error_xy = sqrt(err_x * err_x + err_y * err_y);


    if ((fabs(error_yaw) > kRefAngularErrorForPureRotationControl) && (error_xy > kRefXYErrorForPureRotationControl) )
    {
#if VERBOSE
        std::cout << "computeControlLawNonLin() - pure rotational control " << std::endl;
#endif
        // pure rotational control 
        linear_vel = 0;
        angular_vel = kw_*error_yaw;
    }
    else
    {
        double cyaw = cos(yaw);
        double syaw = sin(yaw);

        double e1 = cyaw * err_x + syaw * err_y;
        double e2 = -syaw * err_x + cyaw * err_y;
        double e3 = error_yaw;

        double vd = sqrt(ref_vel.linear.x * ref_vel.linear.x + ref_vel.linear.y * ref_vel.linear.y);

        //double k3 = k1;
        
        // from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        double zita = 0.7;  // 
        double k3 = k1 = 2*zita*sqrt(ref_vel.angular.z*ref_vel.angular.z + k2*vd*vd);

        double u1 = -k1 * e1;
        double u2 = -k2 * vd * sinc(e3) * e2 - k3*e3;
        
#if VERBOSE
        std::cout << "computeControlLawNonLin() - normal control - 2D err:  " << error_xy << ", k1=k3: " << k1 << std::endl;
#endif        

        linear_vel  = vd * cos(e3) - u1;
        angular_vel = ref_vel.angular.z - u2;
    }

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = tf_robot_pose.frame_id_;
    robot_pose.header.stamp    = tf_robot_pose.stamp_;
    robot_pose.pose.position.x = tf_robot_pose.getOrigin().getX();
    robot_pose.pose.position.y = tf_robot_pose.getOrigin().getY();
    robot_pose.pose.position.z = tf_robot_pose.getOrigin().getZ();
    robot_pose.pose.orientation.x = tf_robot_pose.getRotation().getX();
    robot_pose.pose.orientation.y = tf_robot_pose.getRotation().getY();
    robot_pose.pose.orientation.z = tf_robot_pose.getRotation().getZ();
    robot_pose.pose.orientation.w = tf_robot_pose.getRotation().getW();
    local_path_msg_.poses.push_back(robot_pose);
    local_path_pub_.publish(local_path_msg_);

    return error_xy;
}


// compute control law for rotation, it returns the current yaw error 
double TrajectoryControlActionServer::computeControlLawRotation(const tf::StampedTransform& tf_robot_pose, const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& ref_vel, double& linear_vel, double& angular_vel)
{
#if VERBOSE
    std::cout <<  "TrajectoryControlActionServer::computeControlLawRotation()" << std::endl; 
#endif 

    double roll, pitch, yaw;
    tf_robot_pose.getBasis().getRPY(roll, pitch, yaw);

    double error_yaw = diffS1(tf::getYaw(ref_pose.orientation), yaw);

    if (fabs(error_yaw) >= kAngularErrorThreshold)
    {
#if VERBOSE
        std::cout << "computeControlLawRotation() - pure rotational control - yaw error:  " << error_yaw << std::endl;
#endif
        // pure rotational control 
        linear_vel = 0;
        angular_vel = kw_*error_yaw;
    }

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = tf_robot_pose.frame_id_;
    robot_pose.header.stamp    = tf_robot_pose.stamp_;
    robot_pose.pose.position.x = tf_robot_pose.getOrigin().getX();
    robot_pose.pose.position.y = tf_robot_pose.getOrigin().getY();
    robot_pose.pose.position.z = tf_robot_pose.getOrigin().getZ();
    robot_pose.pose.orientation.x = tf_robot_pose.getRotation().getX();
    robot_pose.pose.orientation.y = tf_robot_pose.getRotation().getY();
    robot_pose.pose.orientation.z = tf_robot_pose.getRotation().getZ();
    robot_pose.pose.orientation.w = tf_robot_pose.getRotation().getW();
    local_path_msg_.poses.push_back(robot_pose);
    local_path_pub_.publish(local_path_msg_);

    return error_yaw;
}

void TrajectoryControlActionServer::getTracksVelCmd(double linear_vel, double angular_vel, double robot_width, nifti_robot_driver_msgs::Tracks& tracks_cmd)
{
    double d = robot_width / 2;
    
    /// < saturate omega (this is for helping SLAM: fast rotations can give SLAM a hard time)
    if ( use_max_angular_vel_check_ && (fabs(angular_vel) > kMaxAngularVelocity) )
    {
        // here we maintain the curvature radius 
        double scale = fabs(kMaxAngularVelocity / angular_vel);
        angular_vel  *= scale;
        linear_vel   *= scale;
    }
    
    /// < scale velocity on the basis of closest obstacle point 
    if(b_decreased_vel_ && b_closest_obst_vel_reduction_enable_)
    {
        boost::recursive_mutex::scoped_lock locker(closest_obst_point_mutex);

        float clostest_obst_point_dist = sqrt(closest_obst_point.x()*closest_obst_point.x() + closest_obst_point.y()*closest_obst_point.y());
        double  distance_from_robot    = std::max(clostest_obst_point_dist - kRobotRadius, 0.f);
#if VERBOSE        
        ROS_INFO_STREAM("getTracksVelCmd() - distance  from robot: " << distance_from_robot); 
#endif
        if ( (distance_from_robot >= 0.) && (distance_from_robot < kProximityDistanceThreshold) )
        {
            float obst_point_unit_vecx = (clostest_obst_point_dist > 0.f) ? closest_obst_point.x()/clostest_obst_point_dist : 1.; // this is 'cos(theta)' where theta is the relative direction to the obstacle point 
            double theta = acos(obst_point_unit_vecx);
            std::cout << "getTracksVelCmd() - theta: " << 180./M_PI * theta << std::endl;
            if(theta < kProximityActiveAngle)
            {
                float vel_towards_obst     = obst_point_unit_vecx * linear_vel; // this is 'v * cos(theta)'
                float vel_max_towards_obst = (distance_from_robot/kProximityDistanceThreshold)*curr_vel_;
                float scale = (vel_towards_obst > 0.f) ? std::min( vel_max_towards_obst/vel_towards_obst , 1.0f) : 0.f;
#if VERBOSE               
            ROS_INFO_STREAM("getTracksVelCmd() - scale: " << scale); 
#endif
                linear_vel *= scale; 
            }
        }
    }
    
    /// < compute tracks velocities from v and omega 
    tracks_cmd.left  = linear_vel - (d * angular_vel);
    tracks_cmd.right = linear_vel + (d * angular_vel);

    /// < scale the commands by maintaining the orginal velocities ratio (i.e. maintainingt the curvature radius)
    //tracks_cmd.left = sat(tracks_cmd.left, -vel_max_tracks_, vel_max_tracks_);
    if (fabs(tracks_cmd.left) > vel_max_tracks_)
    {
        double scale = fabs(vel_max_tracks_ / tracks_cmd.left);
        tracks_cmd.left  *= scale;
        tracks_cmd.right *= scale;
    }

    //tracks_cmd.right = sat(tracks_cmd.right, -vel_max_tracks_, vel_max_tracks_);
    if (fabs(tracks_cmd.right) > vel_max_tracks_)
    {
        double scale = fabs(vel_max_tracks_ / tracks_cmd.right);
        tracks_cmd.left  *= scale;
        tracks_cmd.right *= scale;
    }
}

void TrajectoryControlActionServer::imuDataCallback(const sensor_msgs::ImuConstPtr &msg)
{
    if (tf_listener_.waitForTransform(global_frame_id_, robot_frame_id_, msg->header.stamp, ros::Duration(1.0)))
    {
        try
        {
            tf_listener_.lookupTransform(global_frame_id_, fl_frame_id_, msg->header.stamp, tf_front_left_flipper_);
            tf_listener_.lookupTransform(global_frame_id_, fr_frame_id_, msg->header.stamp, tf_front_right_flipper);
            tf_listener_.lookupTransform(global_frame_id_, rl_frame_id_, msg->header.stamp, tf_rear_left_flipper_);
            tf_listener_.lookupTransform(global_frame_id_, rr_frame_id_, msg->header.stamp, tf_rear_right_flipper_);
            //tf_listener_.lookupTransform(global_frame_id,imu_frame_id,msg->header.stamp,imu_t);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR_THROTTLE(30, "No Transform available Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR_THROTTLE(30, "Connectivity Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR_THROTTLE(30, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        }
    }
    else
    {
        ROS_INFO("Transformation is not available");
    }

    //    tipOverAxis(tf_front_left_flipper_, tf_front_right_flipper, tf_rear_left_flipper_, tf_rear_right_flipper_, tip_over_axes_coeffs_);
    //    tf::Vector3 gravity(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    //    tf::Vector3 toa_f(tip_over_axes_coeffs_[0], tip_over_axes_coeffs_[1], tip_over_axes_coeffs_[2]);
    //    tf::Vector3 toa_l(tip_over_axes_coeffs_[3], tip_over_axes_coeffs_[4], tip_over_axes_coeffs_[5]);
    //    tf::Vector3 toa_b(tip_over_axes_coeffs_[6], tip_over_axes_coeffs_[7], tip_over_axes_coeffs_[8]);
    //    tf::Vector3 toa_r(tip_over_axes_coeffs_[9], tip_over_axes_coeffs_[10], tip_over_axes_coeffs_[11]);

    tipOverAxis(tf_front_left_flipper_, tf_front_right_flipper, tf_rear_left_flipper_, tf_rear_right_flipper_, tip_over_axes_vecs_);
    tf::Vector3 gravity(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    tf::Vector3 toa_f(tip_over_axes_vecs_[0]);
    tf::Vector3 toa_l(tip_over_axes_vecs_[1]);
    tf::Vector3 toa_b(tip_over_axes_vecs_[2]);
    tf::Vector3 toa_r(tip_over_axes_vecs_[3]);

    // compute unit vectors 
    gravity = gravity / gravity.length();
    toa_f = toa_f / toa_f.length();
    toa_l = toa_l / toa_l.length();
    toa_b = toa_b / toa_b.length();
    toa_r = toa_r / toa_r.length();

    double cos_alpha[4];
    cos_alpha[1] = tf::tfDot(gravity, toa_f);
    cos_alpha[2] = tf::tfDot(gravity, toa_l);
    cos_alpha[3] = tf::tfDot(gravity, toa_b);
    cos_alpha[4] = tf::tfDot(gravity, toa_r);

    double max_fc = -std::numeric_limits<double>::max();
    for (int i = 0; i < 4; i++)
    {
        double fc = fabs(cos_alpha[i]);
        if (fc > max_fc)
        {
            max_fc = fc;
        }
    }
    double cost = max_fc; // |cos(alpha)| the more g and v are far from being perpendicular the bigger the cost  
    ROS_INFO("Force-Angle Stability Margin: [%f]", cost);
}

void TrajectoryControlActionServer::imuOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker(tf_robot_pose_map_mutex);
        
    odomMsgToStampedTransform(*msg, tf_robot_pose_odom_);

    double yaw = tf::getYaw(tf_robot_pose_odom_.getRotation());

    if (tf_listener_.waitForTransform(global_frame_id_, odom_frame_id_, msg->header.stamp, ros::Duration(transform_tolerance_)))
    {
        try
        {
            tf_listener_.lookupTransform(global_frame_id_, robot_frame_id_, msg->header.stamp, tf_robot_pose_map_);
            tf_listener_.lookupTransform(global_frame_id_, odom_frame_id_, msg->header.stamp, tf_odom_to_map_);

            if(enable_flippers_)
            {
                tf_listener_.lookupTransform(global_frame_id_, fl_frame_id_, msg->header.stamp, tf_front_left_flipper_);
                tf_listener_.lookupTransform(global_frame_id_, fr_frame_id_, msg->header.stamp, tf_front_right_flipper);
                tf_listener_.lookupTransform(global_frame_id_, rl_frame_id_, msg->header.stamp, tf_rear_left_flipper_);
                tf_listener_.lookupTransform(global_frame_id_, rr_frame_id_, msg->header.stamp, tf_rear_right_flipper_);
            }
        }

        catch (tf::LookupException& ex)
        {
            ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ExtrapolationException& ex)
        {
            //ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
        }
    }
    else
    {

        tf::Transform transformation = tf_odom_to_map_ * tf_robot_pose_odom_;
        
        tf_robot_pose_map_.stamp_ = tf_robot_pose_odom_.stamp_;
        tf_robot_pose_map_.setBasis(transformation.getBasis());
        tf_robot_pose_map_.setOrigin(transformation.getOrigin());
        tf_robot_pose_map_.setRotation(transformation.getRotation());
    }

    getRealRobotPoseB(displacement_, tf_robot_pose_map_, tf_robot_poseB_map_);
}

void TrajectoryControlActionServer::executeCallback(const trajectory_control_msgs::TrajectoryControlGoalConstPtr &goal_msg)
{
    boost::recursive_mutex::scoped_lock locker(action_mutex);
    
    if(b_simple_rotation_)
    {
        executeRotation(goal_msg);
        return; /// < EXIT POINT! 
    }
    
    /// start queue path planner stuff
    trajectory_control_msgs::PlanningFeedback planning_feedback_msg;
    planning_feedback_msg.header.stamp = ros::Time::now();
    planning_feedback_msg.node = "control";
    planning_feedback_msg.task = "done";
    planning_feedback_msg.status = STATUS_FAILURE;
    /// end queue path planner stuff 

    nifti_robot_driver_msgs::Tracks tracks_cmd;

    /// < send local path
    local_path_msg_.poses.clear();
    local_path_msg_.header.stamp = ros::Time::now();
    local_path_pub_.publish(local_path_msg_);

    /// < send global path 
    //global_path_msg_.poses.clear(); 
    global_path_msg_ = goal_msg->path;
    global_path_pub_.publish(global_path_msg_);
    
    /// < send multi- path 
    sendMultiRobotPath(global_path_msg_);

    ros::Rate rate(control_frequency_);
    double timestep = rate.expectedCycleTime().nsec / 1e9;

    feedback_msg_.timestep = timestep;

    double actual_rise_time = rise_time_;
    if(!need_start_vel_ramp_) actual_rise_time = 0; // check if we need or not a starting vel ramp 
    p_path_manager_->init(timestep, cruise_vel_, actual_rise_time, goal_msg->path, path_smoother_type);

    smoothed_path_pub_.publish(p_path_manager_->getPathIn()); // the input path may have been smoothed depending on the input parameters

    double estimated_duration = p_path_manager_->getEstimatedTime();

    // since we allow reference velocity to change time out cannot be reliably used 
    // moreover, the last GUIs allow to interrupt the trajectory control at any time 
    double time_out = std::numeric_limits<double>::max();

    const geometry_msgs::Pose final_position = p_path_manager_->getFinalPose().pose;

    double current_track_error_xy = 0;

    double time_counter = 0;
    int index = 0;

    CmdVels cmd_vels(robot_width_, 1./*chi*/, wheel_radius_);
    
    geometry_msgs::Pose pose_ref_B;
    geometry_msgs::Twist vel_ref_B;

    bool b_done = false;
    bool b_time_out = false;
    bool b_path_end = p_path_manager_->isPathEnd(); // check  if the path is ok 
    
    // the path is empty and we pre-empt the action server
    if(ros::ok() && b_path_end)
    {
        ROS_WARN_STREAM("TrajectoryControlActionServer::executeCallback() - path is empty - action canceled ");
        act_server_.setPreempted();
    }
    
    bool b_do_step = true;
    
    double final_3D_error = 0; 
    double final_2D_error = 0;
    
    //ROS_INFO("counter=%f, duration=%f, index=%d, global_plan.poses.size()=%ld", (float)counter, (float)duration, index, global_plan.poses.size());

    while (ros::ok() && !b_time_out && !b_path_end)
    {
        checkLaserProximityAndUpdateVelocity();
                
#if 1        
        while (!getRobotPose(tf_robot_pose_odom_, tf_robot_pose_map_, tf_odom_to_map_))
        {
            ROS_INFO("Waiting for transformation");
        }
        getRealRobotPoseB(displacement_, tf_robot_pose_map_, tf_robot_poseB_map_);
#endif 
                
        //ROS_INFO(" ---- cycle step ----  ");
        if (act_server_.isPreemptRequested() || !ros::ok())
        {
            b_done = false;
            ROS_INFO("Trajectory Control: %s: Preempted", action_name.c_str());
            global_plan_msg_.poses.clear();

            // send an empty path message
            local_path_msg_.poses.clear();
            local_path_pub_.publish(local_path_msg_);

            // send an empty path message
            global_path_msg_.poses.clear();
            global_path_pub_.publish(global_path_msg_);
            
            //sendMultiRobotPath(global_path_msg_);

            geometry_msgs::Twist cmd_twist;
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.angular.z = 0.0;
            //cmd_pub_.publish(cmd_twist);

            tracks_cmd.left = 0;
            tracks_cmd.right = 0;
            //tracks_vel_cmd_pub_.publish(tracks_cmd);

            cmd_vels.setZero();
            sendVelCommands(tracks_cmd, cmd_vels);

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
            // set the current robot position in the path manager 
            { // start scope for locking tf_robot_pose_map_mutex
                boost::recursive_mutex::scoped_lock locker(tf_robot_pose_map_mutex);
                tf::StampedTransform&  tf_robot_pose_map_to_check = (control_law_type_ == kInputOutputFL) ? (tf_robot_poseB_map_) : (tf_robot_pose_map_);            
                tf::Vector3& robot_origin_to_check = tf_robot_pose_map_to_check.getOrigin();
                p_path_manager_->setRobotPosition(robot_origin_to_check.getX(),robot_origin_to_check.getY(),robot_origin_to_check.getZ());
             }

            // if the error is not too big we can generate a new point ahead with the path manager 
            b_do_step = (current_track_error_xy < ref_error_for_stopping_path_manager_step);
            //b_do_step = true;

            if (b_do_step)
            {
                p_path_manager_->step(ros::Time::now().toSec());
            }
            else
            {
                ROS_WARN_STREAM("TrajectoryControlActionServer::executeCallback() - waiting error decreasing for doing next step - 2D err: " << current_track_error_xy);
            }

            geometry_msgs::PoseStamped pose_ref = p_path_manager_->getCurrentPose();

            //const double current_vel = cruise_vel_;
            curr_vel_ = p_path_manager_->getCurrentVel(); 
            buildReferenceTrajectory(curr_vel_, pose_ref, pose_ref_B, vel_ref_B); /// < N.B.: the reference trajectory is applied on point B as is

            //ROS_INFO("BuildRefTrajectory done");

            { // start scope for locking tf_robot_pose_map_mutex
                boost::recursive_mutex::scoped_lock locker(tf_robot_pose_map_mutex);
                tf::StampedTransform&  tf_robot_pose_map_to_check = (control_law_type_ == kInputOutputFL) ? (tf_robot_poseB_map_) : (tf_robot_pose_map_);

                final_3D_error = sqrt(pow(final_position.position.x - tf_robot_pose_map_to_check.getOrigin().getX(), 2) +
                                             pow(final_position.position.y - tf_robot_pose_map_to_check.getOrigin().getY(), 2) +
                                             pow(final_position.position.z - tf_robot_pose_map_to_check.getOrigin().getZ(), 2));

                final_2D_error = sqrt(pow(final_position.position.x - tf_robot_pose_map_to_check.getOrigin().getX(), 2) +
                                             pow(final_position.position.y - tf_robot_pose_map_to_check.getOrigin().getY(), 2));

                if (b_do_step && (final_3D_error > kFinalRefErrorForStoppingTrajControl))
                {
                    switch (control_law_type_)
                    {
                    case kInputOutputFL:
                        current_track_error_xy = computeControlLawIOLin(tf_robot_poseB_map_, k1_IOL_, k2_IOL_, pose_ref_B, vel_ref_B, linear_vel_, angular_vel_);
                        break;
                    case kNonLinear:
                        default:
                        current_track_error_xy = computeControlLawNonLin(tf_robot_pose_map_, k1_NL_, k2_NL_, pose_ref_B, vel_ref_B, linear_vel_, angular_vel_);
                        break;
                    }
                }
                else
                {
                    current_track_error_xy = computeControlLawPosition(tf_robot_pose_map_, kv_CR_, kw_CR_, p_path_manager_->getCurrentPose(), linear_vel_, angular_vel_);
                }

                //ROS_INFO("Robot commands computed");
                getTracksVelCmd(linear_vel_, angular_vel_, robot_width_, tracks_cmd);
                
                // once we have processed tracks_cmd.left and tracks_cmd.right lets transform the result back to linear and angular velocities 
                cmd_vels.vl_ = tracks_cmd.left;
                cmd_vels.vr_ = tracks_cmd.right;
                cmd_vels.updateVOmega();
                cmd_vels.updateOmegalOmegar();
                linear_vel_  = cmd_vels.v_; 
                angular_vel_ = cmd_vels.omega_;
                
                //ROS_INFO("Robot tracks vel computed");
                
                feedback_msg_.cmd_vel.linear.x   = linear_vel_;
                feedback_msg_.cmd_vel.angular.z  = angular_vel_;
                feedback_msg_.tracks_cmd.left    = tracks_cmd.left;
                feedback_msg_.tracks_cmd.right   = tracks_cmd.right;
                feedback_msg_.trajectory_error.x = pose_ref_B.position.x - tf_robot_pose_map_to_check.getOrigin().getX();
                feedback_msg_.trajectory_error.y = pose_ref_B.position.y - tf_robot_pose_map_to_check.getOrigin().getY();

            } // end scope for locking tf_robot_pose_map_mutex

            act_server_.publishFeedback(feedback_msg_);

            /// < publish the velocity command
            //tracks_vel_cmd_pub_.publish(tracks_cmd);
            sendVelCommands(tracks_cmd, cmd_vels);
            
            
            rate.sleep();
            time_counter += timestep;
            index++;
        }
       
        b_path_end = (p_path_manager_->isPathEnd());
        b_time_out = time_counter > time_out;

        b_done = b_path_end;

    } /// < end while

    if (b_time_out)
    {
        ROS_INFO("Trajectory Control: %s: timeout!", action_name.c_str());
        planning_feedback_msg.status = STATUS_FAILURE;

        cmd_vels.setZero();
        sendVelCommands(tracks_cmd, cmd_vels);
    }

    if (b_done)
    {
        result_msg_.done = true;
        
        getTracksVelCmd(0, 0, robot_width_, tracks_cmd);
        //tracks_vel_cmd_pub_.publish(tracks_cmd);
        cmd_vels.setZero();
        sendVelCommands(tracks_cmd, cmd_vels); 
        rate.sleep();        
        
        ROS_INFO("Trajectory Control: %s: Succeeded", action_name.c_str());
        global_plan_msg_.poses.clear();
        act_server_.setSucceeded(result_msg_);

        planning_feedback_msg.task = "done";
        planning_feedback_msg.status = STATUS_SUCCESS;

        {
            boost::recursive_mutex::scoped_lock locker(tf_robot_pose_map_mutex);
            tf::StampedTransform&  tf_robot_pose_map_to_check = (control_law_type_ == kInputOutputFL) ? (tf_robot_poseB_map_) : (tf_robot_pose_map_);

            /// < N.B.: controller work only considering x and y components; here we consider the z componenent for sake of completeness 
            final_3D_error = sqrt(pow(final_position.position.x - tf_robot_pose_map_to_check.getOrigin().getX(), 2) +
                                         pow(final_position.position.y - tf_robot_pose_map_to_check.getOrigin().getY(), 2) +
                                         pow(final_position.position.z - tf_robot_pose_map_to_check.getOrigin().getZ(), 2));

            final_2D_error = sqrt(pow(final_position.position.x - tf_robot_pose_map_to_check.getOrigin().getX(), 2) +
                                         pow(final_position.position.y - tf_robot_pose_map_to_check.getOrigin().getY(), 2));

            ROS_INFO("Trajectory Control: final 3D error %f", final_3D_error);
            ROS_INFO("Trajectory Control: final 2D error %f", final_2D_error);
        }
        

        if (!b_local_path_)
        {
            queue_task_feedback_pub_.publish(planning_feedback_msg);
        }
    }
}


void TrajectoryControlActionServer::executeRotation(const trajectory_control_msgs::TrajectoryControlGoalConstPtr &goal_msg)
{
    std::cout << "TrajectoryControlActionServer::executeRotation()" << std::endl; 
    
    /// start queue path planner stuff
    trajectory_control_msgs::PlanningFeedback planning_feedback_msg;
    planning_feedback_msg.header.stamp = ros::Time::now();
    planning_feedback_msg.node = "control";
    planning_feedback_msg.task = "rotation";
    planning_feedback_msg.status = STATUS_FAILURE;
    /// end queue path planner stuff 

    nifti_robot_driver_msgs::Tracks tracks_cmd;

    /// < send local path
    local_path_msg_.poses.clear();
    local_path_msg_.header.stamp = ros::Time::now();       
    local_path_pub_.publish(local_path_msg_);

    /// < send global path 
    //global_path_msg_.poses.clear();
    global_path_msg_.header.stamp = ros::Time::now();       
    global_path_msg_ = goal_msg->path;
    std::cout << "global_path_msg: "  << global_path_msg_ << std::endl; 
    global_path_pub_.publish(global_path_msg_);
    
    /// < send multi- path 
    sendMultiRobotPath(global_path_msg_);
    
    ros::Rate rate(control_frequency_);
    double timestep = rate.expectedCycleTime().nsec / 1e9;

    feedback_msg_.timestep = timestep;

    double current_yaw_error = 0;

    double time_counter = 0;

    CmdVels cmd_vels(robot_width_, 1./*chi*/, wheel_radius_);
    
    geometry_msgs::Pose pose_ref_B;
    geometry_msgs::Twist vel_ref_B;

    bool b_done = false;
    bool b_time_out = false;
    bool b_path_end = false; 

    while (ros::ok() && !b_time_out && !b_path_end)
    {

#if 1        
        while (!getRobotPose(tf_robot_pose_odom_, tf_robot_pose_map_, tf_odom_to_map_))
        {
            ROS_INFO("Waiting for transformation");
        }
        getRealRobotPoseB(displacement_, tf_robot_pose_map_, tf_robot_poseB_map_);
#endif 

        //ROS_INFO(" ---- cycle step ----  ");
        if (act_server_.isPreemptRequested() || !ros::ok())
        {
            b_done = false;
            ROS_INFO("Trajectory Control: %s: Preempted", action_name.c_str());
            global_plan_msg_.poses.clear();

            // send an empty path message
            local_path_msg_.poses.clear();
            local_path_pub_.publish(local_path_msg_);

            // send an empty path message
            global_path_msg_.poses.clear();
            global_path_pub_.publish(global_path_msg_);
            
            //sendMultiRobotPath(global_path_msg_);

            geometry_msgs::Twist cmd_twist;
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.angular.z = 0.0;
            //cmd_pub_.publish(cmd_twist);

            tracks_cmd.left = 0;
            tracks_cmd.right = 0;
            //tracks_vel_cmd_pub_.publish(tracks_cmd);

            cmd_vels.setZero();
            sendVelCommands(tracks_cmd, cmd_vels); 

            act_server_.setPreempted();

            planning_feedback_msg.status = STATUS_FAILURE;

            break; /// < EXIT FROM LOOP
        }
        else
        {
            /// < N.B.: the path is assumed to be just a single pose (containing the reference yaw)
            //geometry_msgs::PoseStamped pose_ref = global_plan_msg_.poses[0]; // this gives problem!
            geometry_msgs::PoseStamped pose_ref = goal_msg->path.poses[0];
            
            buildReferenceTrajectory(curr_vel_, pose_ref, pose_ref_B, vel_ref_B); /// < N.B.: the reference trajectory is applied on point B as is

            //ROS_INFO("BuildRefTrajectory done");

            { // start scope for locking tf_robot_pose_map_mutex
                boost::recursive_mutex::scoped_lock locker(tf_robot_pose_map_mutex);
                tf::StampedTransform&  tf_robot_pose_map_to_check = (control_law_type_ == kInputOutputFL) ? (tf_robot_poseB_map_) : (tf_robot_pose_map_);


                current_yaw_error = computeControlLawRotation(tf_robot_pose_map_, pose_ref_B, vel_ref_B, linear_vel_, angular_vel_);
                b_path_end = (fabs(current_yaw_error) < kAngularErrorThreshold);

                //ROS_INFO("Robot commands computed");
                getTracksVelCmd(linear_vel_, angular_vel_, robot_width_, tracks_cmd);
                
                // once we have processed tracks_cmd.left and tracks_cmd.right lets transform the result back to linear and angular velocities 
                cmd_vels.vl_ = tracks_cmd.left;
                cmd_vels.vr_ = tracks_cmd.right;
                cmd_vels.updateVOmega();
                cmd_vels.updateOmegalOmegar();
                linear_vel_  = cmd_vels.v_; 
                angular_vel_ = cmd_vels.omega_;
                
                //ROS_INFO("Robot tracks vel computed");
                
                feedback_msg_.cmd_vel.linear.x   = linear_vel_;
                feedback_msg_.cmd_vel.angular.z  = angular_vel_;
                feedback_msg_.tracks_cmd.left    = tracks_cmd.left;
                feedback_msg_.tracks_cmd.right   = tracks_cmd.right;
                feedback_msg_.trajectory_error.x = pose_ref_B.position.x - tf_robot_pose_map_to_check.getOrigin().getX();
                feedback_msg_.trajectory_error.y = pose_ref_B.position.y - tf_robot_pose_map_to_check.getOrigin().getY();

            } // end scope for locking tf_robot_pose_map_mutex

            act_server_.publishFeedback(feedback_msg_);

            /// < publish the velocity command
            sendVelCommands(tracks_cmd, cmd_vels);
            
            
            rate.sleep();
            time_counter += timestep;
        }

        b_done = b_path_end;

    } /// < end while

    if (b_time_out)
    {
        ROS_INFO("Trajectory Control: %s: timeout!", action_name.c_str());
        planning_feedback_msg.status = STATUS_FAILURE;

        cmd_vels.setZero();
        sendVelCommands(tracks_cmd, cmd_vels); 
    }

    if (b_done)
    {
        result_msg_.done = true;
        
        getTracksVelCmd(0, 0, robot_width_, tracks_cmd);
        //tracks_vel_cmd_pub_.publish(tracks_cmd);
        cmd_vels.setZero();
        sendVelCommands(tracks_cmd, cmd_vels);         
        rate.sleep();
        
        ROS_INFO("Trajectory Control: %s: Succeeded", action_name.c_str());
        global_plan_msg_.poses.clear();
        act_server_.setSucceeded(result_msg_);

        //planning_feedback_msg.task = "done";
        planning_feedback_msg.status = STATUS_SUCCESS;
        
        ROS_INFO("Trajectory Control: final rotation error %f", fabs(current_yaw_error)); 

        if (!b_local_path_)
        {
            queue_task_feedback_pub_.publish(planning_feedback_msg);
        }
    }
}

void TrajectoryControlActionServer::robotPathCallBack(const trajectory_control_msgs::RobotPathConstPtr &msg)
{
    std::cout << "=============================================================" << std::endl;
    std::cout << "TrajectoryControlActionServer::robotPathCallBack()" << std::endl;

    act_client_.waitForServer();
    trajectory_control_msgs::TrajectoryControlGoal track_goal;
    track_goal.path = msg->path;
    b_local_path_ = (msg->type == trajectory_control_msgs::RobotPath::kLocal);
    need_start_vel_ramp_ = msg->need_start_vel_ramp;
    b_simple_rotation_ = false; 
    ROS_INFO("Trajectory Control: starting new path - path size: %d, local: %d, ramp: %d",(int)track_goal.path.poses.size(),(int)b_local_path_.load(),(int)need_start_vel_ramp_.load());    
    act_client_.sendGoal(track_goal);
}

void TrajectoryControlActionServer::pathCallBack(const nav_msgs::PathConstPtr &msg)
{
    std::cout << "=============================================================" << std::endl;
    std::cout << "TrajectoryControlActionServer::pathCallBack()" << std::endl;

    act_client_.waitForServer();
    trajectory_control_msgs::TrajectoryControlGoal track_goal;
    track_goal.path = *msg;
    ROS_INFO("Trajectory Control: starting new path - path size: %d",(int)track_goal.path.poses.size());
    b_local_path_ = false; // this is a global path
    need_start_vel_ramp_ = true; 
    b_simple_rotation_ = false; 
    act_client_.sendGoal(track_goal);
}

void TrajectoryControlActionServer::localPathCallBack(const nav_msgs::PathConstPtr &msg)
{
    std::cout << "=============================================================" << std::endl;
    std::cout << "TrajectoryControlActionServer::localPathCallBack()" << std::endl;

    act_client_.waitForServer();
    trajectory_control_msgs::TrajectoryControlGoal track_goal;
    track_goal.path = *msg;
    ROS_INFO("Trajectory Control: starting new path - path size: %d",(int)track_goal.path.poses.size());
    b_local_path_ = true; // this is a local path
    need_start_vel_ramp_ = false;
    b_simple_rotation_ = false;     
    act_client_.sendGoal(track_goal);
}


void TrajectoryControlActionServer::rotationCallBack(const nav_msgs::PathConstPtr &msg)
{
    std::cout << "=============================================================" << std::endl;
    std::cout << "TrajectoryControlActionServer::rotationCallBack()" << std::endl;

    act_client_.waitForServer();
    trajectory_control_msgs::TrajectoryControlGoal track_goal;
    track_goal.path = *msg;
    ROS_INFO("Trajectory Control: starting new path - path size: %d",(int)track_goal.path.poses.size());
    b_local_path_ = false; // a rotation is considered as a global path 
    b_simple_rotation_ = true; 
    act_client_.sendGoal(track_goal);
}

void TrajectoryControlActionServer::adaptTravVelCallback(const geometry_msgs::TwistStampedPtr& vel_msg)
{
    adapt_trav_vel_ = sat(vel_msg->twist.linear.x, -1. * vel_max_tracks_, vel_max_tracks_); // sat the input filter 
    curr_vel_ = filter_vel_trav_.step(adapt_trav_vel_);
    curr_vel_ = sat(curr_vel_.load(), -1. * vel_max_tracks_, vel_max_tracks_); // sat the output of the filter 

    //curr_vel_ = filter(adapt_trav_vel_);
}

void TrajectoryControlActionServer::queueFeedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg)
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
            ROS_WARN("Trajectory Control: controller already stopped");
            feedback_msg.status = STATUS_FAILURE;
        }
        else
        {
            ROS_INFO("Trajectory Control: cancelling current goal");

            act_client_.cancelGoal();

            ROS_INFO("Trajectory Control: controller stopped (ready)");
            feedback_msg.status = STATUS_SUCCESS;
        }

        queue_task_feedback_pub_.publish(feedback_msg);
    }
}

void TrajectoryControlActionServer::queueTaskCallback(const nav_msgs::Path& path_msg)
{
    trajectory_control_msgs::PlanningFeedback feedback_msg;
    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.node = "control";
    feedback_msg.task = "start";

    act_client_.waitForServer();

    if (act_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE) // current goal is running
    {
        ROS_WARN("Trajectory Control: controller already started, task dropped");
        feedback_msg.status = STATUS_FAILURE;
    }
    else
    {
        trajectory_control_msgs::TrajectoryControlGoal track_goal;
        track_goal.path = path_msg;
        act_client_.sendGoal(track_goal);

        ROS_INFO("Trajectory Control: controller started (busy)");
        feedback_msg.status = STATUS_SUCCESS;
    }

    queue_task_feedback_pub_.publish(feedback_msg);
}

void TrajectoryControlActionServer::goalAbortCallback(const std_msgs::Bool& msg)
{
    act_client_.waitForServer();

    if (act_client_.getState() == actionlib::SimpleClientGoalState::LOST) // if no goal is running
    {
        ROS_WARN("Trajectory Control: controller already stopped");
    }
    else
    {
        ROS_INFO("Trajectory Control: cancelling current goal");

        act_client_.cancelGoal();
        
        nifti_robot_driver_msgs::Tracks tracks_cmd;
        CmdVels cmd_vels(robot_width_, 1./*chi*/, wheel_radius_);        
        sendVelCommands(tracks_cmd, cmd_vels); 

        ROS_INFO("Trajectory Control: controller stopped (ready)");
    }
}

/// < DISABLED!!
void TrajectoryControlActionServer::laserProximityCallback(const std_msgs::Bool& msg)
{
    return; /// < disabled EXIT POINT 
    
    decrease_vel_activation_time_ = ros::Time::now(); 
        
    if(b_decreased_vel_) return; /// < EXIT POINT 
    
    ROS_INFO("Trajectory Control: laser proximity on - reducing velocity ");
    
    b_decreased_vel_ = true; 

    velocity_before_reduction_ = curr_vel_; // store the currently set reference velocity 
    
    curr_vel_ = curr_vel_* kLaserProximityReducedVelocityFactor; //  reduce velocity by kLaserProximityReducedVelocityFactor
}

void TrajectoryControlActionServer::closestObstaclePointCallback(const std_msgs::Float32MultiArray& msg)
{
    decrease_vel_activation_time_ = ros::Time::now(); 
 
    if(!b_closest_obst_vel_reduction_enable_) return; /// < EXIT POINT 
            
    ROS_INFO_STREAM("Trajectory Control: laser proximity on - reducing velocity - vel: " << curr_vel_);
        
    {
    boost::recursive_mutex::scoped_lock locker(closest_obst_point_mutex);
    //std_msgs::Float32MultiArray closest_point_msg; // <x,y,z,dist>
    closest_obst_point.setX(msg.data[0]);
    closest_obst_point.setY(msg.data[1]);
    closest_obst_point.setZ(msg.data[2]);
    closest_obst_dist = msg.data[3];
    }
    
#if 0    // just for testing 
    ///  scale velocity on the basis of closest obstacle point 
    if(b_decreased_vel_ && b_closest_obst_vel_reduction_enable_)
    {
        boost::recursive_mutex::scoped_lock locker(closest_obst_point_mutex);

        float clostest_obst_point_dist = sqrt(closest_obst_point.x()*closest_obst_point.x() + closest_obst_point.y()*closest_obst_point.y());
        double  distance_from_robot    = std::max(clostest_obst_point_dist - kRobotRadius, 0.f);
    
        ROS_INFO_STREAM("closestObstaclePointCallback() - distance  from robot: " << distance_from_robot); 

        if ( (distance_from_robot >= 0.) && (distance_from_robot < kProximityDistanceThreshold) )
        {
            float obst_point_unit_vecx = (clostest_obst_point_dist > 0.f) ? closest_obst_point.x()/clostest_obst_point_dist : 1.; // this is 'cos(theta)' where theta is the relative direction to the obstacle point 
            double theta = acos(obst_point_unit_vecx);
            std::cout << "getTracksVelCmd() - theta: " << 180./M_PI * theta << std::endl;
            if(theta < kProximityActiveAngle)
            {
                float vel_towards_obst     = obst_point_unit_vecx * curr_vel_; // this is 'v * cos(theta)'
                float vel_max_towards_obst = (distance_from_robot/kProximityDistanceThreshold)*curr_vel_;
                float scale = (vel_towards_obst > 0.f) ? std::min( vel_max_towards_obst/vel_towards_obst , 1.0f) : 0.f;
            }
           
            ROS_INFO_STREAM("closestObstaclePointCallback() - scale: " << scale); 
        }
    } 
#endif  
    
    
    if(b_decreased_vel_) return; /// < EXIT POINT 
        
    b_decreased_vel_ = true; 

    velocity_before_reduction_ = curr_vel_; // store the currently set reference velocity 
    
    curr_vel_ = curr_vel_* kLaserProximityReducedVelocityFactor; //  reduce velocity by kLaserProximityReducedVelocityFactor    
}

void TrajectoryControlActionServer::closestObstacleVelReductionEnableCallback(const std_msgs::Bool& msg)
{    
    b_closest_obst_vel_reduction_enable_ = (bool) msg.data;
    
    ROS_INFO_STREAM("Trajectory Control: velocity reduction enable " << b_closest_obst_vel_reduction_enable_);    
}


void TrajectoryControlActionServer::checkLaserProximityAndUpdateVelocity()
{   
    if(!b_decreased_vel_) return; /// < EXIT POINT 
    
    ROS_INFO("Trajectory Control: laser proximity on - checking velocity ");
        
    if(!b_closest_obst_vel_reduction_enable_)
    {
        b_decreased_vel_ = false;
        curr_vel_ = velocity_before_reduction_; // restore the last set reference velocity 
    }
    
    ros::Duration elapsed_time_since_last_message = ros::Time::now() - decrease_vel_activation_time_;
    if(elapsed_time_since_last_message.toSec() > kLaserProximityReducedVelocityResetTime)
    {
        b_decreased_vel_ = false;
        curr_vel_ = velocity_before_reduction_; // restore the last set reference velocity 
    }
}

// Now: this is managed by the local planner 
//void TrajectoryControlActionServer::queueLocalTaskCallback(const nav_msgs::Path& path_msg)
//{
//    trajectory_control_msgs::PlanningFeedback feedback_msg;
//    feedback_msg.header.stamp = ros::Time::now();
//    feedback_msg.node = "control";
//    feedback_msg.task = "start";
//
//    b_local_path_ = true; 
//            
//    act_client_.waitForServer();
//
////    if (act_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE) // current goal is running
////    {
////        ROS_WARN("Trajectory Control: controller already started, task dropped");
////        //feedback_msg.status = STATUS_FAILURE;
////    }
////    else
//    {
//        trajectory_control_msgs::TrajectoryControlGoal track_goal;
//        track_goal.path = path_msg;
//        act_client_.sendGoal(track_goal);
//
//        ROS_INFO("Trajectory Control: restarted for local path planning");
//        //feedback_msg.status = STATUS_SUCCESS;
//    }
//
//    //queue_task_feedback_pub_.publish(feedback_msg);
//}


/// < other functions 

void TrajectoryControlActionServer::tipOverAxis(tf::StampedTransform& fl, tf::StampedTransform& fr, tf::StampedTransform& rl, tf::StampedTransform& rr, std::vector<double>& coeff)
{
    coeff.clear();

    double l1 = fl.getOrigin().getX() - fr.getOrigin().getX();
    double m1 = fl.getOrigin().getY() - fr.getOrigin().getY();
    double n1 = fl.getOrigin().getZ() - fr.getOrigin().getZ();

    coeff.push_back(l1);
    coeff.push_back(m1);
    coeff.push_back(n1);

    double l2 = rl.getOrigin().getX() - fl.getOrigin().getX();
    double m2 = rl.getOrigin().getY() - fl.getOrigin().getY();
    double n2 = rl.getOrigin().getZ() - fl.getOrigin().getZ();

    coeff.push_back(l2);
    coeff.push_back(m2);
    coeff.push_back(n2);

    double l3 = rr.getOrigin().getX() - rl.getOrigin().getX();
    double m3 = rr.getOrigin().getY() - rl.getOrigin().getY();
    double n3 = rr.getOrigin().getZ() - rl.getOrigin().getZ();

    coeff.push_back(l3);
    coeff.push_back(m3);
    coeff.push_back(n3);

    double l4 = fr.getOrigin().getX() - rr.getOrigin().getX();
    double m4 = fr.getOrigin().getY() - rr.getOrigin().getY();
    double n4 = fr.getOrigin().getZ() - rr.getOrigin().getZ();

    coeff.push_back(l4);
    coeff.push_back(m4);
    coeff.push_back(n4);
}

void TrajectoryControlActionServer::tipOverAxis(tf::StampedTransform& fl, tf::StampedTransform& fr, tf::StampedTransform& rl, tf::StampedTransform& rr, std::vector<tf::Vector3>& vecs)
{
    vecs.clear();

    tf::Vector3 v1 = fl.getOrigin() - fr.getOrigin();
    tf::Vector3 v2 = rl.getOrigin() - fl.getOrigin();
    tf::Vector3 v3 = rr.getOrigin() - rl.getOrigin();
    tf::Vector3 v4 = fr.getOrigin() - rr.getOrigin();

    vecs.push_back(v1);
    vecs.push_back(v2);
    vecs.push_back(v3);
    vecs.push_back(v4);
}


void TrajectoryControlActionServer::sendMultiRobotPath(const nav_msgs::Path& path_msg)
{
    trajectory_control_msgs::MultiRobotPath multi_robot_path_msg; 
    multi_robot_path_msg.header   = path_msg.header;
    multi_robot_path_msg.robot_id = robot_id_; 
    multi_robot_path_msg.poses    = path_msg.poses;
    multi_robot_paths_pub_.publish(multi_robot_path_msg);
}

void TrajectoryControlActionServer::sendVelCommands(const nifti_robot_driver_msgs::Tracks& tracks_cmd, const CmdVels& cmd_vel)
{
    geometry_msgs::Twist cmd_twist;
    cmd_twist.linear.x = cmd_vel.v_;
    cmd_twist.linear.y = 0.0;
    cmd_twist.angular.z = cmd_vel.omega_;
    cmd_pub_.publish(cmd_twist);
    
    tracks_vel_cmd_pub_.publish(tracks_cmd);

    trajectory_control_msgs::VelocityCommand ang_cmd_msg;
    ang_cmd_msg.angular_velocity_left = cmd_vel.omegal_;
    ang_cmd_msg.angular_velocity_right = cmd_vel.omegar_;
    cmd_wheels_pub_.publish(ang_cmd_msg);
}
