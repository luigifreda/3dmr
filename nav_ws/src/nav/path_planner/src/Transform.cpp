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

#include "Transform.h"


Transform::Transform(const std::string& parent, const std::string& child):parent_(parent),child_(child),b_frames_set_(false),b_ok_(true)
{
    if( (!parent_.empty()) && (!child_.empty()) )
    {
        b_frames_set_ = true; 
    }
    p_tf_listener_ = new tf::TransformListener(ros::Duration(10.0));
}

Transform::~Transform()
{
    delete p_tf_listener_;
}

tf::StampedTransform Transform::get(const std::string& parent, const std::string& child)
{
    tf::StampedTransform transform;
    b_ok_ = true;

    if( p_tf_listener_->waitForTransform(parent,child,ros::Time(),ros::Duration(1.0)) )
    {
	try
	{
	    p_tf_listener_->lookupTransform(parent,child,ros::Time(),transform);
	}
	catch( tf::LookupException& ex )
	{
	    ROS_WARN("no transform available: %s\n",ex.what());
            b_ok_ = false; 
	    return tf::StampedTransform();
	}
	catch( tf::ConnectivityException& ex )
	{
	    ROS_WARN("connectivity error: %s\n",ex.what());
            b_ok_ = false; 
	    return tf::StampedTransform();
	}
	catch( tf::ExtrapolationException& ex )
	{
	    ROS_WARN("extrapolation error: %s\n",ex.what());
            b_ok_ = false;            
	    return tf::StampedTransform();
	}

	return transform;
    }

    std::string error_message = "transformation not available between " + parent +" and " + child;
    ROS_ERROR_STREAM(error_message.c_str());
    THROW_EXCEPTION(TransformException, error_message)	 
    //throw "transformation not available";
    b_ok_ = false;
    
    return tf::StampedTransform();
}

void Transform::set(const std::string& parent, const std::string& child)
{ 
    if( (!parent_.empty()) && (!child_.empty()) )
    {
        b_frames_set_ = true; 
    }
    else
    {
        b_frames_set_ = false; 
    }
    parent_ = parent.c_str(); 
    child_ = child.c_str();
}
    
tf::StampedTransform Transform::get()
{
    if(!b_frames_set_)
    {
        ROS_ERROR("frames not set, transformation not available");
        return tf::StampedTransform();
    }
    else
    {
        return get(parent_, child_);
    }
}