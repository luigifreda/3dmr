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

#pragma once 

#include <boost/shared_ptr.hpp>

#include "PathManager.h"
#include "CSVReader.h"


///	\class CSVTrajectoryManager
///	\author Luigi Freda
///	\brief Class for reading a trajectory from cvs data 
///	\note
/// 	\todo 
///	\date
///	\warning

class CSVTrajectoryManager: PathManager
{
public:

    CSVTrajectoryManager() : i_traj_length_(0)
    {
    }

    CSVTrajectoryManager(const std::string& csv_file, double Ts, double vel)
    {
        init(csv_file, Ts, vel);
    }

    ~CSVTrajectoryManager()
    {
    }

    /// Init

    void init(const std::string& csv_file, double Ts, double vel, const std::string& base_link="base_link")
    {        
        boost::recursive_mutex::scoped_lock locker(mutex_); 

        p_csv_reader_.reset(new CSVReader(csv_file));
        std::vector< std::vector<double> > mat_data; 
        p_csv_reader_->convertDataToDoubles(mat_data);
        i_traj_length_ = mat_data.size();

        ROS_INFO("CSVTrajectoryManager::init() - Ts:%f, vel:%f, path size: %d", Ts, vel,(int)i_traj_length_);

        b_init_ = true;
        b_end_  = false;
        b_jump_point_ = false;

        path_in_  = nav_msgs::Path();
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

        if(i_traj_length_ == 0)
        {
            b_end_ = true; 
            return; /// < EXIT POINT 
        }

        double first_timestamp = mat_data[0][0];  //labels: timestamp x_d y_d z_d roll_d pitch_d yaw_d 
        path_in_.header.frame_id = base_link;
        path_in_.header.stamp = ros::Time(first_timestamp);

        path_out_.header.frame_id = base_link;
        path_out_.header.stamp = ros::Time(first_timestamp);

        path_in_.poses.reserve(i_traj_length_);
        for (int i = 0; i < i_traj_length_; i++)
        {
            //labels: timestamp x_d y_d z_d roll_d pitch_d yaw_d 
            //idx:            0   1   2   3      4      5      6
            timestamp_.push_back(mat_data[i][0]);
            x_.push_back(mat_data[i][1]);
            y_.push_back(mat_data[i][2]);
            z_.push_back(mat_data[i][3]);            
            roll_.push_back(mat_data[i][4]);
            pitch_.push_back(mat_data[i][5]);
            yaw_.push_back(mat_data[i][6]);          


            geometry_msgs::PoseStamped new_pose;
            new_pose.header.frame_id = base_link;
            new_pose.header.stamp = ros::Time(timestamp_[i]);
            new_pose.pose.position.x = x_[i];
            new_pose.pose.position.y = y_[i];
            new_pose.pose.position.z = z_[i];
            new_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_[i],pitch_[i],yaw_[i]);
            path_in_.poses.push_back(new_pose);
        }

        initPoses();
    }

    inline void initPoses();

    inline void addTrajectoryOffset(double x0, double y0, double z0);
    
    inline void print();

public: // setters 

public: // getters

protected:

    boost::shared_ptr<CSVReader> p_csv_reader_;

    // current assumed label sequence timestamp x_d y_d z_d roll_d pitch_d yaw_d  
    std::vector<double> timestamp_;
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;    
    std::vector<double> roll_;
    std::vector<double> pitch_;
    std::vector<double> yaw_; // theta

    int i_traj_length_;

};

void CSVTrajectoryManager::initPoses()
{
    current_pose_.header = path_in_.poses[0].header;
    // current_pose_.pose.position.x = x_[0]; // start from first point 
    // current_pose_.pose.position.y = y_[0];
    // current_pose_.pose.position.z = z_[0];
    current_pose_.pose.position = path_in_.poses[0].pose.position; // start from first point 
    //current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta_[0]);
    current_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_[0],pitch_[0],yaw_[0]);

    std::cout << "getting final pose " << std::endl;  
    // final_pose_.pose.position.x = x_[i_traj_length_ - 1]; // start from first point 
    // final_pose_.pose.position.y = y_[i_traj_length_ - 1];
    // final_pose_.pose.position.z = z_[i_traj_length_ - 1];
    final_pose_.pose.position = path_in_.poses.back().pose.position; 
    //final_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta_[i_traj_length_ - 1]);
    final_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_[i_traj_length_ - 1],pitch_[i_traj_length_ - 1],yaw_[i_traj_length_ - 1]);

    //    size_t input_path_length = path_in_.poses.size();
    //    final_pose_.header = path_in_.poses[input_path_length - 1].header;
    //    final_pose_.pose.position = path_in_.poses[input_path_length - 1].pose.position;
    //    final_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    i_index_ = 1; /// < start from second point, the current_ one has been stored in current_
    d_step_offset_ = 0;

    d_estimated_time_     = 0;
    d_estimated_distance_ = 0;
    for (int ii = 0; ii < (i_traj_length_ - 2); ii++)
    {
        d_estimated_distance_ += sqrt(pow(x_[ii + 1] - x_[ii], 2) +
                                      pow(y_[ii + 1] - y_[ii], 2) +
                                      pow(z_[ii + 1] - z_[ii], 2));
    }
    d_estimated_time_ = ((fabs(d_vel_lin_) > std::numeric_limits<double>::epsilon()) ? d_estimated_distance_ / d_vel_lin_ : 0);
    //d_estimated_time_ = d_Ts_ * i_traj_length_;


    //ROS_INFO("PathManager::init() - #points: %ld, estimated time: %f, estimated distance: %f", path_in_.poses.size(), d_estimated_time_, d_estimated_distance_);
}

void CSVTrajectoryManager::addTrajectoryOffset(double x0, double y0, double z0)
{
    ROS_ASSERT_MSG(x_.size() == path_in_.poses.size(), "These two guys should be of the same size!"); 

    for (int i = 0; i < i_traj_length_; i++)
    {
        x_[i] += x0;
        y_[i] += y0;
        z_[i] += z0;

        path_in_.poses[i].pose.position.x = x_[i]; 
        path_in_.poses[i].pose.position.y = y_[i]; 
        path_in_.poses[i].pose.position.z = z_[i]; 
    }
}

void CSVTrajectoryManager::print()
{
    for (int i = 0; i < i_traj_length_; i++)
    {
        std::cout << x_[i] <<", " << y_[i] << ", " << z_[i] << ", " << roll_[i] << ", " << pitch_[i] << ", " << yaw_[i] << ", " << std::endl;        
    }
}

