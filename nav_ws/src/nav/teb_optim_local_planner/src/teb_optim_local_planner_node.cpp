/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2017-present Luigi Freda <luigifreda at gmail dot com>
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

#include <teb_optim_local_planner/TebOptimLocalPlannerServer.h>

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

    ros::NodeHandle nh("~");
    std::string node_name = getParam<std::string>(nh, "node_name", "teb_optim_local_planner_node"); 
    std::string robot_name = getParam<std::string>(nh, "robot_name", "ugv1");     

    teb_local_planner::TebOptimLocalPlannerServer optimLocalPlanner("/" + node_name);
    ros::spin();

    return 0;
}
