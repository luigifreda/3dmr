#include <ros/ros.h>

#include "octomap_mux/octomap_mux.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "LaserMapper");
  ros::NodeHandle node_handle("~");

  OctomapMux octomap_mux(node_handle);

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
