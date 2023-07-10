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
