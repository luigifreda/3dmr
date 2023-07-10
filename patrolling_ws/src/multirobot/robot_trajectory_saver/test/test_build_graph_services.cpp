#include <string>
#include <cstdlib>
#include "ros/ros.h"
#include <robot_trajectory_saver_msgs/SaveRobotTrajectories.h>
#include <robot_trajectory_saver_msgs/LoadRobotTrajectories.h>
#include <robot_trajectory_saver_msgs/GetRobotTrajectories.h>
#include <robot_trajectory_saver_msgs/CheckPath.h>
#include "robot_trajectory_saver/robot_trajectory_saver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_build_graph_services");
  

  ros::NodeHandle n("~");
  
  
  std::string service_name_SaveRobotTrajectories = getParam<std::string>(n, "SaveRobotTrajectories_service_name", "/robot_trajectory_saver_node/save_robot_trajectories_only");
  std::string service_name_LoadRobotTrajectories = getParam<std::string>(n, "LoadRobotTrajectories_service_name", "/robot_trajectory_saver_node/load_robot_trajectories_only");
  std::string service_name_GetRobotTrajectories = getParam<std::string>(n, "GetRobotTrajectories_service_name", "/robot_trajectory_saver_node/get_robot_trajectories_nav_msgs");
  std::string service_name_CheckPath = getParam<std::string>(n, "CheckPath_service_name", "/robot_trajectory_saver_node/check_path");

  ros::ServiceClient client_SaveRobotTrajectories = n.serviceClient<robot_trajectory_saver_msgs::SaveRobotTrajectories>(service_name_SaveRobotTrajectories);
  ros::ServiceClient client_LoadRobotTrajectories = n.serviceClient<robot_trajectory_saver_msgs::LoadRobotTrajectories>(service_name_LoadRobotTrajectories);
  ros::ServiceClient client_GetRobotTrajectories = n.serviceClient<robot_trajectory_saver_msgs::GetRobotTrajectories>(service_name_GetRobotTrajectories);
  ros::ServiceClient client_CheckPath = n.serviceClient<robot_trajectory_saver_msgs::CheckPath>(service_name_CheckPath);
  
  robot_trajectory_saver_msgs::SaveRobotTrajectories m1;
  robot_trajectory_saver_msgs::LoadRobotTrajectories m2;
  robot_trajectory_saver_msgs::GetRobotTrajectories m3;
  robot_trajectory_saver_msgs::CheckPath m4;
  
  
#if 0 
  if (client_SaveRobotTrajectories.call(m1))
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service client_SaveRobotTrajectories");
    return 1;
  }
  
  if (client_LoadRobotTrajectories.call(m2))
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service client_LoadRobotTrajectories");
    return 1;
  }
  
#endif
  
  if (client_GetRobotTrajectories.waitForExistence(ros::Duration(5.0))) {

        if (client_GetRobotTrajectories.call(m3)) {
            ROS_INFO("OK");
            std::cout << "Number of robot poses red: " << m3.response.trajectories.poses.size() << std::endl;
        } else {
            ROS_ERROR("Failed to call service client_GetRobotTrajectories");
            return 1;
        }
    }

  return 0;
}