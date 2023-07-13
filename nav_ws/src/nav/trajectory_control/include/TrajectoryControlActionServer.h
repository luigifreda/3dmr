/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
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

#pragma once 

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <trajectory_control_msgs/TrajectoryControlAction.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nifti_robot_driver_msgs/Tracks.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <trajectory_control_msgs/PlanningFeedback.h>
#include <trajectory_control_msgs/PlanningTask.h>
#include <trajectory_control_msgs/MultiRobotPath.h> 
#include <trajectory_control_msgs/message_enums.h> // for SegmentStatus and TaskType
#include <trajectory_control_msgs/RobotPath.h> 
#include <trajectory_control_msgs/VelocityCommand.h>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>

#include "SignalUtils.h"
#include "PathManager.h"
#include "LowPassFilter.h"
#include "CmdVels.h"

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

///	\class TrajectoryControlActionServer
///	\author Luigi Freda (2016-Present) and Alcor (2016)
///	\brief Trajectory control node
///	\note 
/// 	\todo 
///	\date
///	\warning
class TrajectoryControlActionServer
{
    
public:

    enum ControlLawType
    {
        kInputOutputFL = 0, 
        kNonLinear = 1
    }; 
    
public:
    
    static const double kTrackDistance; // [m] distance between tracks 
    static const double kWheelRadius; // [m] radius of the wheels 

    static const float kRobotRadius;  // [m] robot radius for computing the clearance  
    static const float kProximityDistanceThreshold; // [m] distance point-to-robot 
    static const float kProximityActiveAngle; // [rad]
        
    static const double kDefaulRefVelocity; // [m/s] default linear velocity 
    static const double kDefaultDistanceOffsetPointB; // [m] distance offset of point B from robot center (this is used in the control law) 
                                                // position of point B from robot pose (x,y,theta)
                                                // xB = x + displacement * cos(theta)
                                                // yB = y + displacement * sin(theta)
    static const double kLaserProximityReducedVelocityFactor; // [frac %] reducing factor for linear velocity when laser proximity is detected 
    static const double kLaserProximityReducedVelocityResetTime; // [s] if elapsed time from last laser proximity check is > kLaserProximityReducedVelocityResetTime, the normal velocity is set back 
    
    static const double kDefaultControlGainK1_IOL; // default control gain k1 for input-output linearization 
    static const double kDefaultControlGainK2_IOL; // default control gain k2 for input-output linearization 
    static const double kDefaultControlGainK1_NL; // default control gain k1 for non-linear control 
    static const double kDefaultControlGainK2_NL; // default control gain k2 for non-linear control
    static const double kDefaultControlGainKv_CR; // default control gain kv for Cartesian regulation  
    static const double kDefaultControlGainKw_CR; // default control gain kw for Cartesian regulation 
    static const double kDefaultAngularGainKw;  // default control angular gain kw: a pure rotatation control is applied if the angular error is above kRefAngularErrorForPureRotationControl
    static const double kRefAngularErrorForPureRotationControl; 
    static const double kRefXYErrorForPureRotationControl; 
    
    static const double kAngularErrorThreshold;    
    
    static const int kDefaultControlLaw; // default control law (should be one in the ControlLawType list)
    static const double kDefaultControlFrequency; // [Hz] default control frequency 
    static const double kMaxTrackVelocity; // [m/s] maximum track velocity 
    static const double kMaxTrackVelocityOnRotation; //[m/s]  maximum track velocity in m/s on rotations 
    static const double kMaxAngularVelocity; //[rad/s]  maximum angular velocity 

    
    
    //static const double kMinimumWaitingTimeforANewPath; // [s] minimum time to wait for giving a new path as input 
    
    static const double kTimeOutTolerance; // [s], this time will be added to the expected trajectory execution time in order to define a time out
    
    static const double kAdpativeTravFreq; // [Hz] default refresh frequency for adaptive traversability velocity
    static const double kAdpativeTravFilterWn; // [rad/s] default natural frequency for the adaptive traversability velocity filter  
    
    static const double kRefErrorForStoppingPathManagerStep; // max reference error for allowing path manager to step  (default)
    static const double kFinalRefErrorForStoppingTrajControl; // final reference error for stopping trajectory control   
    //static const double k3DDistanceArrivedToGoal; // [m]
    //static const double k2DDistanceArrivedToGoal; // [m]
    
    
    static const std::string kTeleopMuxPriorityName; 
    
protected:
    
    ros::NodeHandle node_;
    ros::NodeHandle param_node_;
    
    std::string action_name;
    actionlib::SimpleActionServer<trajectory_control_msgs::TrajectoryControlAction> act_server_;
    actionlib::SimpleActionClient<trajectory_control_msgs::TrajectoryControlAction> act_client_;

    trajectory_control_msgs::TrajectoryControlFeedback feedback_msg_;
    trajectory_control_msgs::TrajectoryControlResult result_msg_;

    std::string odom_frame_id_;
    std::string global_frame_id_;
    std::string robot_frame_id_;

    std::string fl_frame_id_;
    std::string fr_frame_id_;
    std::string rl_frame_id_;
    std::string rr_frame_id_;
    std::string imu_frame_id_;

    //std::vector<double> tip_over_axes_coeffs_;
    std::vector<tf::Vector3> tip_over_axes_vecs_;

    tf::TransformListener tf_listener_;

    double transform_tolerance_ = 0.2; // [s] time tolerance for waiting a transform  

    std::string imu_odom_topic_;
    ros::Subscriber imu_odom_sub_;

    std::string tracks_vel_cmd_topic_;
    ros::Publisher tracks_vel_cmd_pub_;

    tf::StampedTransform tf_robot_pose_map_;
    tf::StampedTransform tf_robot_pose_odom_;
    tf::StampedTransform tf_odom_to_map_;
    tf::StampedTransform tf_robot_poseB_map_; // point B with offset
    boost::recursive_mutex tf_robot_pose_map_mutex;

    tf::StampedTransform tf_front_left_flipper_;
    tf::StampedTransform tf_front_right_flipper;
    tf::StampedTransform tf_rear_left_flipper_;
    tf::StampedTransform tf_rear_right_flipper_;
    tf::StampedTransform tf_imu_t;

    double displacement_; // axis displacement of poin B (feedback linearization with offset)
    double cruise_vel_; 
    std::atomic<double> curr_vel_;
    double rise_time_;
    
    double linear_vel_;  
    double angular_vel_;
    double robot_width_;
    double wheel_radius_;
    double k1_IOL_; // for input-output linearization control law
    double k2_IOL_; // for input-output linearization control law
    double k1_NL_; // for non linear control law
    double k2_NL_; // for non linear control law 
    double kv_CR_; // for Cartesian regulation
    double kw_CR_; // for Cartesian regulation  
    double kw_; // gain for pure rotational control 
    double control_frequency_;
    ControlLawType control_law_type_;

    double ref_error_for_stopping_path_manager_step; 

    PathSmoother::PathSmootherType path_smoother_type; 
    
    double vel_max_tracks_; // maximum allowed track velocity

    bool use_max_delta_v_check_; 
    double max_delta_v_check_; 

    std::string global_path_topic_;
    ros::Publisher global_path_pub_;

    std::string local_path_topic_;
    ros::Publisher local_path_pub_;

    ros::Publisher smoothed_path_pub_;
    
    nav_msgs::Path global_path_msg_;
    nav_msgs::Path local_path_msg_;
    nav_msgs::Path global_plan_msg_;

    std::string cmd_topic_;
    ros::Publisher cmd_pub_;

    std::string cmd_wheels_topic_;
    ros::Publisher cmd_wheels_pub_;

    std::string imu_topic_;
    ros::Subscriber imu_sub_;

    ros::Subscriber robot_pp_path_sub_;
    
    std::string robot_path_topic_;
    ros::Subscriber robot_path_sub_;
    
    std::string robot_local_path_topic_;
    ros::Subscriber robot_loal_path_sub_;
    
    std::string robot_rotation_topic_;
    ros::Subscriber robot_rotation_sub_;    
    
    std::string goal_abort_topic_;
    ros::Subscriber goal_abort_sub_;
    
    std::string trajectory_control_abort_topic_;
    ros::Subscriber trajectory_control_abort_sub_;
    
    std::string multi_robot_paths_topic_; 
    ros::Publisher multi_robot_paths_pub_; 
    
    boost::shared_ptr<PathManager> p_path_manager_; 
    //PathManagerKdt p_path_manager_;
    
    boost::recursive_mutex action_mutex;    
    
public:   /// <  laser proximity checker integration 
        
    std::string laser_proximity_topic_; 
    ros::Subscriber laser_proximity_sub_; // subscriber for the laser proximity checker on/off message
    
    std::string closest_obst_point_topic_; 
    ros::Subscriber closest_obst_point_sub_; // subscriber for the closer obstacle point
    
    std::string closest_obst_vel_reduction_enable_topic_; 
    ros::Subscriber closest_obst_vel_reduction_enable_sub_; // subscriber for the enabling/disabling velocity reduction depending on closer obstacle point
    
    boost::recursive_mutex closest_obst_point_mutex;
    tf::Vector3 closest_obst_point;
    double closest_obst_dist; 
    
    bool b_closest_obst_vel_reduction_enable_;
    
    std::atomic<bool> b_decreased_vel_; // for decreasing the velocity when the robot is close to an obstacle
    ros::Time decrease_vel_activation_time_; // time the last laser proximity msg was received 
    double velocity_before_reduction_; 

    bool use_max_angular_vel_check_; 
        
#if 0
    //PathManager path_manager_; 
    boost::shared_ptr<BasePathManager> p_ext_path_manager_;
    
    // for executing a path on demand 
    ros::Timer path_task_timer_;
#endif 

public:   /// < adpative traversability stuff 
    
    std::string adapt_trav_vel_topic_; 
    ros::Subscriber adapt_trav_vel_sub_; // subscriber for maximum velocity coming from adaptive traversability module
    double adapt_trav_vel_; 
    LowPassFilter filter_vel_trav_; 
    double adaptive_trav_vel_freq_; // refresh frequency 
    
protected: /// < queue controller stuff 
    
    ros::Publisher queue_task_feedback_pub_; // to say "hey, I'm ready!"
    ros::Subscriber queue_task_feedback_sub_; // to know when to stop
    ros::Subscriber queue_task_path_sub_; // path to follow (global path planning)
    ros::Subscriber queue_task_path_local_sub_; // path to follow (local path planning)
    
    std::string queue_task_feedback_topic_;
    std::string queue_task_path_topic_; 
    
    std::string queue_task_path_local_topic_; 
    
    std::atomic<bool> b_local_path_;
    std::atomic<bool> b_simple_rotation_;

    std::atomic<bool> need_start_vel_ramp_ = {true}; // need to use vel ramp? 

    bool enable_flippers_ = true; 
        
    int robot_id_; 
    std::string str_robot_name_;

public:
    
    TrajectoryControlActionServer(std::string);
    ~TrajectoryControlActionServer();
    
    // convert and odo msg to stamped transform 
    void odomMsgToStampedTransform(nav_msgs::Odometry pose_odometry_msg, tf::StampedTransform& pose_stamped_tf);
    
    /// < compute the position of point B from robot pose (x,y,theta)
    // xB = x + displacement * cos(theta)
    // yB = y + displacement * sin(theta)
    void getRealRobotPoseB(double displacement, tf::StampedTransform real_robot_pose, tf::StampedTransform& real_robot_poseB);
    
    // two methods for getting current useful transforms
    // robot_pose_odom, robot_pose_map, from_odom_to_map
    bool getRobotPose(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&);
    // robot_pose_odom, robot_pose_map, from_odom_to_map, frontLeftF,frontRightF, rearLeftF, rearRightF, imu_t
    bool getRobotPose2(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&);
    
    // resample the trajectory in order to have a point for each delta = vel*Ts
    void resampleAndSmoothPath(const nav_msgs::Path&, nav_msgs::Path&);
    
    // compute ref signals on point B starting from ref signals on robot center 
    void buildReferenceTrajectory(double, const geometry_msgs::PoseStamped&, geometry_msgs::Pose&, geometry_msgs::Twist&);
    
    // compute control law and return the current error
    double computeControlLawIOLin(const tf::StampedTransform& tf_robot_pose_B, double k1, double k2, const geometry_msgs::Pose& ref_pose_B, const geometry_msgs::Twist& ref_vel_B, double& linear_vel, double& angular_vel);
    double computeControlLawNonLin(const tf::StampedTransform& tf_robot_pose, double k1, double k2, const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& ref_vel, double& linear_vel, double& angular_vel);
    double computeControlLawPosition(const tf::StampedTransform& tf_robot_pose_B, double kv, double kw, const geometry_msgs::PoseStamped& target_pose, double& linear_vel, double& angular_vel);
    
    double computeControlLawRotation(const tf::StampedTransform& tf_robot_pose, const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& ref_vel, double& linear_vel, double& angular_vel);

    // compute and saturate track commands 
    void getTracksVelCmd(double, double, double, nifti_robot_driver_msgs::Tracks&);

#if 0    
    void setExternalPathManager(boost::shared_ptr<BasePathManager>& p_ext_path_manager) 
    { 
        p_ext_path_manager_ = p_ext_path_manager; 
        path_task_timer_ = node_.createTimer(ros::Duration(1), &TrajectoryControlActionServer::executeExtPathCallback, this, true/*one shot*/); // the timer will automatically fire at startup 
    }

    void executeExtPathCallback(const ros::TimerEvent& timer_msg)
    {
        if (p_ext_path_manager_)
        {
            std::cout << "TrajectoryControlActionServer::executeExtPathCallback() - starting the trajectory " << std::endl;
            executePath(dynamic_cast<BasePathManager*> (p_ext_path_manager_.get()));
        }
        else
        {
            ROS_ERROR("TrajectoryControlActionServer::executeExtPathCallback() - you did not init the path manager!");
        }
    }
#endif 

public: /// < callbacks     
    
    void imuOdomCallback(const nav_msgs::OdometryConstPtr&);
    void imuDataCallback(const sensor_msgs::ImuConstPtr&);
        
    void executeCallback(const trajectory_control_msgs::TrajectoryControlGoalConstPtr&);
    
    void robotPathCallBack(const trajectory_control_msgs::RobotPathConstPtr&);

    void pathCallBack(const nav_msgs::PathConstPtr&);
    void localPathCallBack(const nav_msgs::PathConstPtr&);
    
    void rotationCallBack(const nav_msgs::PathConstPtr&);
    
    void adaptTravVelCallback(const geometry_msgs::TwistStampedPtr&); 
        
    void queueFeedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg);
    void queueTaskCallback(const nav_msgs::Path& path_msg);
    void queueLocalTaskCallback(const nav_msgs::Path& path_msg);
    
    void goalAbortCallback(const std_msgs::Bool& msg); 
    
    void laserProximityCallback(const std_msgs::Bool& msg); 
    
    void closestObstaclePointCallback(const std_msgs::Float32MultiArray& msg); 
    
    void closestObstacleVelReductionEnableCallback(const std_msgs::Bool& msg);
    
protected: /// < other functions          
    
    void executeRotation(const trajectory_control_msgs::TrajectoryControlGoalConstPtr&);
    
    void checkLaserProximityAndUpdateVelocity();
    
    void tipOverAxis(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, std::vector<double>&);
    void tipOverAxis(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, std::vector<tf::Vector3>&);
    
    void sendMultiRobotPath(const nav_msgs::Path& path_msg); 
    
    void sendVelCommands(const nifti_robot_driver_msgs::Tracks& tracks_cmd, const CmdVels& cmd_vel);
};

