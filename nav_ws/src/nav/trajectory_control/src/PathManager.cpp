/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> 
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

#include <limits>
#include <visualization_msgs/MarkerArray.h>

#include "PathManager.h"

//#define VERBOSE 1

#define DEBUG_REFERENCE_POINT 1

#if DEBUG_REFERENCE_POINT 
visualization_msgs::MarkerArray debugReferencePointMarkerArray;
ros::Publisher debugReferencePointPub;

visualization_msgs::MarkerArray debugRobotPointMarkerArray;
ros::Publisher debugRobotPointPub;

visualization_msgs::MarkerArray debugTrajPointsMarkerArray;
ros::Publisher debugTrajPointsPub;

#endif // DEBUG_REFERENCE_POINT


const double PathManager::kDistanceToRecomputeIndex = 0.2;


PathManager::PathManager() : b_init_(false), b_jump_point_(false), b_end_(true), d_Ts_(0), d_vel_lin_(0), d_vel_ang_(0), d_yaw_last_(0), d_step_offset_(0), i_index_(0), d_estimated_time_(0), d_estimated_distance_(0), smoother_type_(PathSmoother::kNoSmoother)
{
}

PathManager::~PathManager()
{
}

void PathManager::init(double Ts, double vel, double rise_time, const nav_msgs::Path& path_in, const PathSmoother::PathSmootherType& smoother_type)
{
    boost::recursive_mutex::scoped_lock locker(mutex_); 
    
    vel_ramp_ = VelRamp(vel, rise_time); 

#if DEBUG_REFERENCE_POINT
    debugReferencePointPub = node_.advertise<visualization_msgs::MarkerArray>("/trajectory_reference_point", 1);
    debugRobotPointPub     = node_.advertise<visualization_msgs::MarkerArray>("/trajectory_robot_point", 1);
    debugTrajPointsPub     = node_.advertise<visualization_msgs::MarkerArray>("/trajectory_points", 1);
#endif
    
    // let's smooth the input path 
    nav_msgs::Path path_smoothed = smoother_.smooth(path_in, smoother_type); 
    size_t input_path_length = path_in.poses.size(); 
        
    ROS_INFO("PathManager::init() - Ts:%f, vel:%f, path size: %d", Ts, vel,(int)input_path_length);
    
    b_init_ = true;
    b_end_ = false;
    b_jump_point_ = false;

    path_in_  = path_smoothed;
    path_out_ = nav_msgs::Path();

    d_Ts_ = Ts;
    
    d_vel_lin_ = vel;
    d_vel_ang_ = 0; 
    d_yaw_last_ = 0;
    
    d_estimated_time_ = 0;
    d_estimated_distance_ = 0;

    current_pose_ = geometry_msgs::PoseStamped();
    jumped_pose_  = geometry_msgs::PoseStamped();
    final_pose_   = geometry_msgs::PoseStamped();
    
    if(input_path_length == 0)
    {
        b_end_ = true; 
        return; /// < EXIT POINT 
    }

    path_out_.header.frame_id = path_in_.header.frame_id;
    path_out_.header.stamp = path_in_.header.stamp;

    current_pose_.header = path_in_.poses[0].header;
    current_pose_.pose.position = path_in_.poses[0].pose.position; // start from first point 
    current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);


    //std::cout << "getting final pose " << std::endl;    
    final_pose_.header = path_in_.poses.back().header;
    final_pose_.pose.position = path_in_.poses.back().pose.position; 
    final_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    
    
    i_index_ = 1; /// < start from second point, the current_ one has been store in  current_
    d_step_offset_ = 0;


    std::cout << "computing estimated distance " << std::endl;
    if(input_path_length>1)
    {
        for (int ii = 0; ii < (input_path_length - 1); ii++)
        {
            d_estimated_distance_ += sqrt(pow(path_in_.poses[ii + 1].pose.position.x - path_in_.poses[ii].pose.position.x, 2) +
                                          pow(path_in_.poses[ii + 1].pose.position.y - path_in_.poses[ii].pose.position.y, 2) +
                                          pow(path_in_.poses[ii + 1].pose.position.z - path_in_.poses[ii].pose.position.z, 2));
        }
    }
    d_estimated_time_ = ((fabs(d_vel_lin_) > std::numeric_limits<double>::epsilon()) ? d_estimated_distance_ / d_vel_lin_ : 0);


    ROS_INFO("PathManager::init() - #points: %ld, estimated time: %f, estimated distance: %f", path_in_.poses.size(), d_estimated_time_, d_estimated_distance_);
}


// resample the trajectory in order to have a point for each delta = vel*Ts

bool PathManager::step(const double current_time)
{
    boost::recursive_mutex::scoped_lock locker(mutex_);     
    if (!b_init_)
    {
        ROS_ERROR("PathManager::Step() - you did not init!");
        b_end_ = true;
        d_vel_ang_ = 0; 
        
        return b_end_; /// < EXIT POINT 
    }

    d_vel_ang_ = 0; 
    
    //double vel = d_vel_lin_;
    d_vel_curr_ = vel_ramp_.vel(current_time);
    double vel = d_vel_curr_; //d_vel_lin_;

    //    ros::Rate rate(control_frequency_);
    //    double d_Ts_ = rate.expectedCycleTime().nsec / 1e9; // durata dell'intervallo temporale
    //
    //    path_out_.header.frame_id = path_in_.header.frame_id;
    //    path_out_.header.stamp = path_in_.header.stamp;

    double dist_step = 0;

    //    geometry_msgs::PoseStamped current_;
    //    current_.header = path_in_.poses[0].header;
    //    current_.pose.position = path_in_.poses[0].pose.position; // start from first point 

    //    geometry_msgs::PoseStamped next_;

    double current_dist_to_next = 0;
    double yaw = 0;
    //double d_offset_ = 0.0;
    //bool b_change_point = false;

#ifdef VERBOSE
    std::cout << "index : " << i_index_ << std::endl;
#endif

    bool b_step_done = false;
    b_jump_point_ = false; 

    // int i_index_ = 1; // this is the index of next point 
    while (!b_step_done && (i_index_ < path_in_.poses.size()))
    {

#ifdef VERBOSE
        std::cout << "iteration : " << i_index_ << std::endl;
#endif

        //if (current_.pose.position.x == path_in_.poses[i_index_].pose.position.x && current_.pose.position.y == path_in_.poses[i_index_].pose.position.y)
        if (FEQUAL(current_pose_.pose.position.x, path_in_.poses[i_index_].pose.position.x, 1e-4) && FEQUAL(current_pose_.pose.position.y, path_in_.poses[i_index_].pose.position.y, 1e-4))
        {
            i_index_++; // get the new point in the list 
        }
        else
        {
            /// <  check step distance from next pose poses[i_index_]
            if(!b_jump_point_)
            {
                dist_step = d_vel_curr_ * d_Ts_;
                current_dist_to_next = sqrt(pow(path_in_.poses[i_index_].pose.position.x - current_pose_.pose.position.x, 2) + pow(path_in_.poses[i_index_].pose.position.y - current_pose_.pose.position.y, 2));

                yaw = atan2(path_in_.poses[i_index_].pose.position.y - current_pose_.pose.position.y, path_in_.poses[i_index_].pose.position.x - current_pose_.pose.position.x);
                current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

                d_step_offset_ = 0.0;
            }
            else
            {
                dist_step = fabs(d_vel_curr_ * d_Ts_ - d_step_offset_); // get a shorter step 
                current_dist_to_next = sqrt(pow(path_in_.poses[i_index_].pose.position.x - jumped_pose_.pose.position.x, 2) + pow(path_in_.poses[i_index_].pose.position.y - jumped_pose_.pose.position.y, 2));
#ifdef VERBOSE                
                ROS_INFO("changePoint current_dist_to_next: %f", current_dist_to_next);
                ROS_INFO("changePoint dist_step: %f", dist_step);
                ROS_INFO("Offset: %f", d_step_offset_);
                ROS_INFO_STREAM("change point event");
#endif
                yaw = atan2(path_in_.poses[i_index_].pose.position.y - jumped_pose_.pose.position.y, path_in_.poses[i_index_].pose.position.x - jumped_pose_.pose.position.x);
                current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw); 
            }

#ifdef VERBOSE            
            ROS_INFO("offset [%f]", d_step_offset_);
            ROS_INFO("dist-step [%f]", dist_step);
            ROS_INFO("current_dist_to_next [%f]", current_dist_to_next);
#endif

            /// < if current dist_step does not bring us beyond poses[i_index_] generate a new pose
            if (current_dist_to_next >= dist_step)
            {
                geometry_msgs::PoseStamped temp;
                temp.header = path_in_.poses[i_index_].header;
                temp.pose.position.x = current_pose_.pose.position.x + (dist_step * cos(yaw));
                temp.pose.position.y = current_pose_.pose.position.y + (dist_step * sin(yaw));
                //temp.pose.position.z = 0;
                temp.pose.position.z = current_pose_.pose.position.z;

                if (b_jump_point_)
                {
                    yaw = atan2(temp.pose.position.y - current_pose_.pose.position.y, temp.pose.position.x - current_pose_.pose.position.x);
                    current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                }
                
                temp.pose.orientation = current_pose_.pose.orientation;
                
                path_out_.poses.push_back(current_pose_);
                //ROS_INFO_STREAM("added point: current_dist: "<< current_dist << " maggiore di distance: " << distance);
                //ROS_INFO_STREAM("new distance from added point: "<< sqrt(pow(in.poses[i_index_].pose.position.x - temp.pose.position.x,2) + pow(in.poses[i_index_].pose.position.y - temp.pose.position.y,2)));
                current_pose_.header = temp.header;
                current_pose_.pose = temp.pose;
                
                b_jump_point_ = false;
                
                //add to global_pose

                b_step_done = true; /// < we generated a new point, we can actually exit
            }
            else
            {
                // the dist_step brings us beyond path_in_.poses[i_index_]
                // => we take path_in_.poses[i_index_] as intermediate point 
                // => we store the offset and we recover the missed distance in the next iteration 
                d_step_offset_ += fabs(current_dist_to_next - dist_step); // absolute value!
                jumped_pose_.header = path_in_.poses[i_index_].header;
                jumped_pose_.pose = path_in_.poses[i_index_].pose;
                b_jump_point_ = true;

                if(i_index_ == (path_in_.poses.size() - 1)) // this is the last point, push it inside the output path 
                {
#ifdef VERBOSE
                    std::cout << "adding last point as intermediate" << std::endl; 
#endif
                    jumped_pose_.pose.orientation = current_pose_.pose.orientation;
                    current_pose_ = jumped_pose_;
                    path_out_.poses.push_back(current_pose_);
                }

                i_index_++; // get the new point in the list 
            }

        }
        //ROS_INFO_STREAM("Index "<<i_index_<< "");
    }

    if (i_index_ >= path_in_.poses.size())
    {
        b_end_ = true;
#ifdef VERBOSE
        std::cout << "final path index" << std::endl;
#endif
    }
    
    d_vel_ang_ = diffS1(yaw,d_yaw_last_)/d_Ts_;
    d_yaw_last_ = yaw; 
    if(i_index_ == 1) d_vel_ang_ = 0; // we are still on the first segment

    /*
    for(int i_index_=1;i_index_<global_plan.poses.size()-1;i_index_++){

            global_plan.poses[i_index_].pose.position.x = 0.3*global_plan.poses[i_index_-1].pose.position.x + 0.4*global_plan.poses[i_index_].pose.position.x + 0.3*global_plan.poses[i_index_+1].pose.position.x;
            global_plan.poses[i_index_].pose.position.y = 0.3*global_plan.poses[i_index_-1].pose.position.y + 0.4*global_plan.poses[i_index_].pose.position.y + 0.3*global_plan.poses[i_index_+1].pose.position.y;
    }
     */
    
    
    sendMarkers();
    
    
    return b_end_;
}


void PathManager::sendMarkers()
{
#if DEBUG_REFERENCE_POINT    
    
    debugReferencePointMarkerArray.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.position.x = current_pose_.pose.position.x;
    marker.pose.position.y = current_pose_.pose.position.y;
    marker.pose.position.z = current_pose_.pose.position.z;
    marker.lifetime = ros::Duration(5);
    marker.id = 1;
    debugReferencePointMarkerArray.markers.push_back(marker);        
    debugReferencePointPub.publish(debugReferencePointMarkerArray);
    
    
    debugRobotPointMarkerArray.markers.clear();
    visualization_msgs::Marker marker_robot;
    marker_robot.header.frame_id = "map";
    marker_robot.header.stamp = ros::Time::now();
    marker_robot.type = visualization_msgs::Marker::SPHERE;
    marker_robot.action = visualization_msgs::Marker::ADD;
    marker_robot.scale.x = marker_robot.scale.y = marker_robot.scale.z = 0.1;
    marker_robot.color.a = 1.0;
    marker_robot.color.r = 1.0;
    marker_robot.color.g = 1.0;
    marker_robot.color.b = 1.0;
    marker_robot.pose.position.x = robot_position_.x;
    marker_robot.pose.position.y = robot_position_.y;
    marker_robot.pose.position.z = robot_position_.z;
    marker_robot.lifetime = ros::Duration(5);
    marker_robot.id = 2;
    debugRobotPointMarkerArray.markers.push_back(marker_robot);
    debugRobotPointPub.publish(debugRobotPointMarkerArray);    
    
    
    size_t input_path_length = path_in_.poses.size();     
    debugTrajPointsMarkerArray.markers.clear();
    {
        for(size_t i=0; i<input_path_length;i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(5);
        
            marker.pose.position.x = path_in_.poses[i].pose.position.x;
            marker.pose.position.y = path_in_.poses[i].pose.position.y;
            marker.pose.position.z = path_in_.poses[i].pose.position.z;
            marker.id = 3 + i;
            debugTrajPointsMarkerArray.markers.push_back(marker);  
        }                                                                
        debugTrajPointsPub.publish(debugTrajPointsMarkerArray);
    }
#endif    
}

void PathManager::setRobotPosition(double x, double y, double z)
{
    robot_position_.x = x;
    robot_position_.y = y;
    robot_position_.z = z;
}



// =============================================================================

/// Init
void PathManagerKdt::init(double Ts, double vel,  double rise_time, const nav_msgs::Path& path_in)
{    
    boost::recursive_mutex::scoped_lock locker(mutex_);   
 
    ROS_INFO("PathManagerKdt::init()");
        
    PathManager::init(Ts, vel, rise_time, path_in);
    
    size_t input_path_length = path_in_.poses.size(); 
    
    if(input_path_length == 0)
    {
        return; 
    }
    
    plc_points_.clear(); 
    
    PointNode point; 
    for(size_t i=0; i<input_path_length;i++)
    {
        point.x     = path_in_.poses[i].pose.position.x;
        point.y     = path_in_.poses[i].pose.position.y;
        point.z     = path_in_.poses[i].pose.position.z;
        point.label = i;
        plc_points_.push_back(point);
    }
    kdtree_points_.setInputCloud(plc_points_.makeShared());    
    
}

/// basic step for generating next ref point 
/// return true when done 
bool PathManagerKdt::step(const double current_time)
{
    std::vector<int> pointIdxSearch(1);
    std::vector<float> pointSquaredDistance(1,std::numeric_limits<float>::max());
    
    // check if we have to override the i_index_ 
    if( i_index_ < (path_in_.poses.size()-1) ) 
    {   
        //int n_found = kdtree_points_.radiusSearch(robot_position_,kDistanceToRecomputeIndex,pointIdxSearch,pointSquaredDistance);
        int n_found = kdtree_points_.nearestKSearch(robot_position_, 1, pointIdxSearch, pointSquaredDistance);
               
        PointNode& closest_point = plc_points_[pointIdxSearch[0]];
        double closest_dist = sqrt(pointSquaredDistance[0]);
        
        if(closest_dist > kDistanceToRecomputeIndex)
        {
            ROS_INFO_STREAM("PathManagerKdt::step() - path index override - dist: " << closest_dist);
            i_index_ = closest_point.label;
            current_pose_.pose.position = path_in_.poses[i_index_].pose.position; 
            current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            if(i_index_ > 0)
            {
                geometry_msgs::Point prev_point = path_in_.poses[i_index_-1].pose.position; 
                double yaw = atan2(current_pose_.pose.position.y-prev_point.y, current_pose_.pose.position.x-prev_point.x);
                current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);                
            }
            
            // check if next point is in the range, if yes use it as current point 
            geometry_msgs::Point next_point = path_in_.poses[i_index_+1].pose.position;
            double dist_next_point = sqrt( pow(next_point.x - robot_position_.x,2) + pow(next_point.y - robot_position_.y,2) + pow(next_point.z - robot_position_.z,2) );
            if(dist_next_point < kDistanceToRecomputeIndex)
            {
                i_index_++;
                current_pose_.pose.position = path_in_.poses[i_index_].pose.position;                 
            }
            
            d_step_offset_ = 0;
        }
        else
        {
            if(i_index_  <  closest_point.label)
            {
                ROS_INFO_STREAM("PathManagerKdt::step() - path index override - dist: " << closest_dist);
                i_index_ = closest_point.label;

                current_pose_.pose.position = path_in_.poses[i_index_].pose.position; // start from first point 
                current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                d_step_offset_ = 0;                
            }
        }   
    }
    
    PathManager::step(ros::Time::now().toSec());
}


