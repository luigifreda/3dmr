/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Tiago Novo
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

#include <math.h> 
#include <SpaceTimeFilter.h>


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

int main(int argc, char** argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    /// < get parameters
    
    std::string str_robot_name   = getParam<std::string>(nh_private, "robot_name", "ugv1");   /// < multi-robot
    std::string str_robot_prefix = "ugv"; 
    int robot_id = atoi(str_robot_name.substr(3,str_robot_name.size()).c_str()) - 1;  
    
    std::string simulator_name  = getParam<std::string>(nh_private, "simulator", "");   /// < multi-robot    
    
    ROS_INFO_STREAM("==========================================================");    
    ROS_INFO_STREAM("scan filter node of robot " << robot_id << " alive");
    ROS_INFO_STREAM("==========================================================");    

    std::cout << "got parameters" << std::endl;

    /// ========================================================================


    SpaceTimeFilter scan_filter(nh, nh_private);
    scan_filter.setRobotId(robot_id);

    ros::spin();

    return 0;
}
