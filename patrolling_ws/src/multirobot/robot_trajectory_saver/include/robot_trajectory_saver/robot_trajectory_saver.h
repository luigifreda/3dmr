/**
* This file is part of the ROS package robot_trajectory_saver which belongs to the framework 3DMR. 
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

#ifndef ROBOT_TRAJECTORY_SAVER_H
#define ROBOT_TRAJECTORY_SAVER_H

#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <robot_trajectory_saver_msgs/SaveRobotTrajectories.h>
#include <robot_trajectory_saver_msgs/LoadRobotTrajectories.h>
#include <robot_trajectory_saver_msgs/GetRobotTrajectories.h>
#include <robot_trajectory_saver_msgs/CheckPath.h>

#include <nav_msgs/Odometry.h> 

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>

#include "pcl_ros/point_cloud.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <pcl/conversions.h>

#include <boost/graph/connected_components.hpp>
#include "boost/graph/graph_traits.hpp"

#include <visualization_msgs/Marker.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>

inline bool exist_file(const std::string& name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

inline bool empty_file(std::ifstream& pFile)
{
    return pFile.peek() == std::ifstream::traits_type::eof();
}

using namespace boost;

enum vertex_properties_t
{
    vertex_properties
};

namespace boost
{
BOOST_INSTALL_PROPERTY(vertex, properties);
}

struct Point3D
{
    double x;
    double y;
    double z;
};

typedef adjacency_list < listS, vecS, undirectedS, property<vertex_properties_t, Point3D>, property < edge_weight_t, double > > Graph;
typedef graph_traits<Graph>::edge_iterator edge_iterator;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef property_map<Graph, vertex_properties_t>::type VertexPositions;
typedef graph_traits<Graph>::adjacency_iterator n_iterator;
typedef graph_traits<Graph>::edge_descriptor Edge;
typedef graph_traits<Graph>::vertex_iterator vertex_iterator;


typedef boost::shared_ptr< Graph > GraphPtr;

struct RobotTrajectory
{
    std::string robot_frame_;
    std::string odom_frame_;
    std::string imu_odom_topic_;
    std::string traj_laser_mapper_topic_;
    std::vector<tf::StampedTransform> global_robot_poses;
    std::vector<tf::StampedTransform> odom_robot_poses;
    std::vector<tf::StampedTransform> from_odoms_to_map;
};

template <class Name>
class WriteVertexPosition
{
public:

    WriteVertexPosition(Name _name):name(_name)
    {
        v_positions = get(vertex_properties, name);
    }

    template <class VertexOrEdge>
    void operator()(std::ostream& out, const VertexOrEdge& v) const
    {
        Point3D p = v_positions[v];
        out << " [x = " << p.x << ", y = " << p.y << ", z = " << p.z << "]";
    }
private:
    Name name;
    VertexPositions v_positions;
};

template <class Name>
class WriteEdgeWeight
{
public:

    WriteEdgeWeight(Name _name):name(_name)
    {
        weights = get(edge_weight, name);
    }

    template <class VertexOrEdge>
    void operator()(std::ostream& out, const VertexOrEdge& v) const
    {
        double wt = get(weights, v);
        out << " [weight = \"" << wt << "\"]";
    }
private:
    Name name;
    typename property_map<Name, edge_weight_t>::type weights;
};

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

class RobotTrajectorySaver
{
public:

    /* Constructor */
    RobotTrajectorySaver();

    /* De-constructor */
    ~RobotTrajectorySaver();

    /* Initialize Stamped Transform Structures in Constructor for Robot i */
    bool initRobotTrajectory(int i);

    /* Initialize Stamped Transform Structures in Constructor for Robot i */
    void initRobotTrajectories();

    /* Update pose of Robot i */
    bool updateRobotPose(int i);

    /* Update poses of all robots */
    void updateRobotPoses();

    /* callback for msgs coming from imu_odom UGV 1 */
    void imuOdomCallbackUgv1(const nav_msgs::OdometryConstPtr&);

    /* callback for msgs coming from imu_odom UGV 2 */
    void imuOdomCallbackUgv2(const nav_msgs::OdometryConstPtr&);

    /* callback for msgs coming from laser mapper UGV 1 */
    void trajFromLaserMapperUgv1(const nav_msgs::PathConstPtr&);

    /* callback for msgs coming from laser UGV 2 */
    void trajFromLaserMapperUgv2(const nav_msgs::PathConstPtr&);

    /* Service Callback responsible of saving robot trajectories onto a file */
    bool saveRobotTrajectoriesCallback(robot_trajectory_saver_msgs::SaveRobotTrajectories::Request& request, robot_trajectory_saver_msgs::SaveRobotTrajectories::Response& response);

    /* Service Callback responsible of loading robot trajectories from a file and return them as nav_msgs::Path */
    bool loadRobotTrajectoriesCallback(robot_trajectory_saver_msgs::LoadRobotTrajectories::Request& request, robot_trajectory_saver_msgs::LoadRobotTrajectories::Response& response);

    /* Service Callback responsible of saving robot trajectories onto a file */
    bool getRobotTrajectoriesCallback(robot_trajectory_saver_msgs::GetRobotTrajectories::Request& request, robot_trajectory_saver_msgs::GetRobotTrajectories::Response& response);

    /* Service Callback responsible for checking if a path exists (Dijkstra) between two nodes and retrieve the cost (Euclidean distance) */
    bool checkPath(robot_trajectory_saver_msgs::CheckPath::Request& request, robot_trajectory_saver_msgs::CheckPath::Response& response);

    /* Spin at fixed rate */
    void ros_spin();

    /* For visualizing the robot trajectory graph */
    void visualize(visualization_msgs::Marker&);

    /* Function which transform a boost graph into a nav_msgs::Path */
    void fromBoostGraphToNavMsgPath(nav_msgs::Path& path);

    /* If a file of trajectories already exists then it will be loaded at the beginning */
    void initTrajectoriesFromExistingDotFile();


protected:

    /* PARAM if true the flag state that this node will run in simulation */
    bool run_in_simulation;

    /* PARAM if true the flag state that we are using laser_mapper */
    bool run_laser_mapper;

    /* PARAM Number of robots */
    int num_robots;

    /* if true then a file with robot trajectories already existed and it has been loaded */
    bool read_file;

    /* PARAM Sample time of robot trajectories (default 1 Hz) */
    double frequency;

    /* PARAM Leaf size for downsampling robot trajectories */
    double leaf_size;

    /* PARAM Parameter of kdtree radius serch for building the graph of robot poses */
    double min_radius;

    /* PARAM Parameter of kdtree radius serch for building the graph of robot poses */
    double max_radius;

    /* PARAM Parameter of kdtree radius serch for building the graph of robot poses */
    double radius_incr;

    /* PARAM Parameter of kdtree radius serch for building the graph of robot poses */
    double scaled_factor;

    /* PARAM frame name robot frame prefix (needed in simulation) */
    std::string robot_name_prefix;

    /* PARAM frame name of the odom (default odom in simulation) */
    std::string odom_frame;

    /* PARAM frame name of the robot (default base_link in simulation) */
    std::string robot_frame;

    /* PARAM Name of the frame of the global reference frame */
    std::string global_frame;

    /* PARAM Name of both the directory and the file where the trajectories are saved as dot file */
    std::string map_filename;

    /* PARAM Name of topic of inso node publishing imu_odom */
    std::string imu_odom_topic;

    /* PARAM Name of the topic where robot is publishing trajectories */
    std::string traj_ugv_topic_name_from_laser_mapper;


    /* PARAM Name of topic where the graph as markers is published */
    std::string trajectories_marker_topic;

    /* PARAM Name of the service to be called to save into a graph.dot file only the robot trajectories */
    std::string save_robot_trajectories_service_name;

    /* PARAM Name of the service to be called to load from graph.dot file the robot trajectories in the form of a nav_msgs::Path */
    std::string load_robot_trajectories_service_name;

    /* PARAM Name of the service to be called to retrieve as a nav_msgs::Path the robot trajectories */
    std::string get_robot_trajectories_service_name;

    /* PARAM Name of the service to check if a path exists (Dijkstra) between two nodes and retrieve the cost (Euclidean distance) */
    std::string check_path_service_name;

    /* Data structure representing a list of robot trajectories */
    std::vector<RobotTrajectory> ugv_trajectories;

    /* Boost Trajectory graph */
    Graph* graph_ptr;

    /* It will be filled with the vertexes of the graph red from file ate the beginning if exists */
    pcl::PointCloud<pcl::PointXYZ> *cloud_in_init_graph;

    /* visualization_msgs Trajectory graph */
    visualization_msgs::Marker line_list;

    /* Public ROS Node Handle for both Publishers and Subscribers */
    ros::NodeHandle node;

    /* TF */
    tf::TransformListener tf_;

    /* Publisher of the graph as markers built from robot trajectories */
    ros::Publisher trajectories_marker_pub;

    /* Subscriber to msgs coming from imu_odom of ugv1 */
    ros::Subscriber ugv1_imu_odom_sub;

    /* Subscriber to msgs coming from imu_odom of ugv2 */
    ros::Subscriber ugv2_imu_odom_sub;

    /* Subscriber to msgs coming from laser_mapper publishing ugv1 trajectory in /map frame as nav_msgs::Path */
    ros::Subscriber ugv1_traj_ugv_topic_name_from_laser_mapper_sub;

    /* Subscriber to msgs coming from laser_mapper publishing ugv2 trajectory in /map frame as nav_msgs::Path */
    ros::Subscriber ugv2_traj_ugv_topic_name_from_laser_mapper_sub;

    /* Ros Service for saving into a file the robot trajectories dounwsampled */
    ros::ServiceServer save_robot_trajectories;

    /* Ros Service for loading from a file the robot trajectories dounwsampled */
    ros::ServiceServer load_robot_trajectories;

    /* Ros Service for retrieving the robot trajectories dounwsampled as a nav_msgs::Path */
    ros::ServiceServer get_robot_trajectories;

    /* Ros Service for checking if a path exists (Dijkstra) between two nodes and retrieve the cost (Euclidean distance) */
    ros::ServiceServer check_path;

private:

    /* Private ROS Node Handle for parameters setting */
    ros::NodeHandle n_;

    /* Function to transform odom msgs into stamped transform */
    void odomMsgToStampedTransform(nav_msgs::Odometry, tf::StampedTransform&);

    /* Function to transform nav msgs path (sequences of robot poses) coming from laser mapper into stamped transform */
    void navMsgToStampedTransform(int, const nav_msgs::PathConstPtr&);

    /* Size of the queues of the subscribers */
    static const int kSubMessageQueueSize = 10;

    /* Maximum distance between nodes */
    static const double MAX_DISTANCE;

    /* Mutex to be used when save robt trajectories service is called */
    pthread_mutex_t lock_robot_trajectories;

    /* Function which initialize the process of graph building */
    void initialize(double, int, pcl::PointCloud<pcl::PointXYZ>*, pcl::KdTreeFLANN<pcl::PointXYZ>*);

    /* Step for connecting connected components in the neighborhood */
    void linkConnectedComponents(double, double, double, int, pcl::PointCloud<pcl::PointXYZ>*, pcl::KdTreeFLANN<pcl::PointXYZ>*);

    /* Function used for computing a path given the robot trajectories graph and two nodes */
    bool findVertex(int i, Vertex& vertex);

    /* Function which computes the path given the robot trajectories graph and two nodes */
    void computeShortestPath(int i, int j, std::vector<int>& path);

    /* Find the node of the graph with minimum distance from a poi */
    void findClosestVertex(int n_point_of_interest, geometry_msgs::Point s, int& i, Point3D& p, double& d);

};

const double RobotTrajectorySaver::MAX_DISTANCE = 1000000;



#endif /* ROBOT_TRAJECTORY_SAVER_H */

