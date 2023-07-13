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

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "Exception.h" 


DEFINE_EXCEPTION(TransformException, Exception)	


///	\class Transform
///	\author Luigi Freda
///	\brief 
///	\note 
/// 	\todo 
///	\date
///	\warning
class Transform
{
public:
    
    Transform(const std::string& parent = "map", const std::string& child="base_link");
    ~Transform();
        
public: // getters 
    
    // get transform for input frames 
    tf::StampedTransform get(const std::string& parent, const std::string& child);

    // get transform for set frames 
    tf::StampedTransform get();
    
    bool isOk() const {return b_ok_;}
    
public: // setters 
    
    // set frames 
    void set(const std::string& parent, const std::string& child);
        
protected:
    tf::TransformListener *p_tf_listener_;
    
    std::string parent_;
    std::string child_;
    bool b_frames_set_; 
    bool b_ok_; // true if we received a valid transform, false otherwise 
};


#endif
