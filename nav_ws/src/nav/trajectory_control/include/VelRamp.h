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

#pragma once 

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include <boost/thread/recursive_mutex.hpp>

#include <path_planner/KdTreeFLANN.h>

#include "SignalUtils.h"


class VelRamp
{
public: 
    VelRamp():cruise_vel_(0), rise_time_(0), current_vel_(0), start_time_(NAN)
    {}

    VelRamp(const double cruise_vel, const double rise_time = 2.0): 
        cruise_vel_(cruise_vel), rise_time_(rise_time), current_vel_(0), start_time_(NAN)
    {}
    
    double vel(const double current_time)
    {
        if( std::isnan(start_time_)) start_time_ = current_time;
        const double t = current_time - start_time_;
        if(t >= rise_time_)
        {
            return cruise_vel_;
        }
        else
        {
            std::cout << "vel ramp " << std::endl; 
            return cruise_vel_*t/rise_time_;
        }
    }

protected: 

    double cruise_vel_; 
    double rise_time_;

    double current_vel_;  
    double start_time_; 
};
