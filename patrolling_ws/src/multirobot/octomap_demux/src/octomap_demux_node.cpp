/**
* This file is part of the ROS package octomap_demux which belongs to the framework 3DMR. 
*
* Copyright (C) 2017-2019 Luigi Freda <luigifreda at gmail dot com>  
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

#include <ros/ros.h>

#include "octomap_demux/octomap_demux.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "octomap_demux");
  ros::NodeHandle node_handle("~");

  OctomapDeMux octomap_demux(node_handle);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  return 0;
}
