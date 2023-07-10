/**
* This file is part of the ROS package patrolling_build_graph which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BUILD_GRAPH_H
#define BUILD_GRAPH_H

#include <ros/ros.h>
#include <string>
#include <stdio.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/thread/recursive_mutex.hpp>

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <cmath>

#include <pcl/conversions.h>

#include <trajectory_control_msgs/PlanningTask.h>
#include <trajectory_control_msgs/PathPlanning.h>

#include <patrolling_build_graph_msgs/BuildGraphEvent.h>
#include <patrolling_build_graph_msgs/Graph.h>
#include <patrolling_build_graph_msgs/PatrollingPoints.h>
#include <patrolling_build_graph_msgs/PriorityPoint.h>

#include <boost/foreach.hpp>
#include <robot_trajectory_saver_msgs/CheckPath.h>

#include "build_graph_utils.h"


///	\class Vertex3D
///	\author Luigi Freda (2016-present) and Mario Gianni (2016)
///	\brief A class for representing a generic 3D Vertex 
///	\note 
/// 	\todo 
///	\date
///	\warning
class Vertex3D  
{
public:

    int id;    // identifier as generated on RVIZ
    int index; // index  N.B.1: indices ARE consecutive; conversely, ids ARE NOT guaranteed to be consecutive; 
               //        N.B.2: we build the graph, with consecutive ids! Check the method GraphBuilder::buildVertexGraph()
    
    float priority; // priority of the node
    int num_neighbours;
    double x;
    double y; //pass these attributes in meters
    double z;
    std::vector<int> id_neighbour;
    std::vector<int> cost; // float cost is converted to int by using the scale factor patrolling_build_graph_msgs::Graph::kNavCostConversionFloatToUint (=10 now)
    std::vector<std::string> dir;
    int number_of_visits;
    double current_idleness;
    double last_visit;

    Vertex3D(){};

    ~Vertex3D(){};

    void setNumVisits(int n)
    {
        number_of_visits = n;
    }

    void setCurrentIdleness(double n)
    {
        current_idleness = n;
    }

    void setLastVisit(double n)
    {
        last_visit = n;
    }
};



///	\class GraphBuilder
///	\author Alcor
///	\brief A class for building a graph representation of an environment by using user inputs
///	\note 
/// 	\todo 
///	\date
///	\warning
class GraphBuilder
{
public:

    static const int kMaxAllowedBranchingFactor; // see the implementation in getgraph.h
    static const double kZOffset;
    static const double kMaxEdgePitch;
    static const double kHeighEdgeCost; 
    static const double kMaxDistPointPriority;

public: 
    
    GraphBuilder();
    ~GraphBuilder();

    void buildVertexGraph();
    void writeVertexGraphOnFile();
    void publishGraph();

    void initMatGraphDataStructures();
    void printAdjacencyMatrix();
    void printDirectionsMatrix();

    void publishBuildGraphEvent();

    bool checkPitchAngle(pcl::PointXYZ& s, pcl::PointXYZ& t);
    bool checkEdgeMaxPitch(int i, int j, pcl::PointXYZ& s, pcl::PointXYZ& t);

    bool checkEdgeNoGroundIntersection(int i, int j, pcl::PointXYZ& s, pcl::PointXYZ& e);
    bool checkEdgeNoWallIntersection(int node_id, int other_node_id, pcl::PointXYZ& node_pt, pcl::PointXYZ& other_node_pt);

public:  /// < callbacks 
    
    // callback which is invoked when a new patrolling task is appended 
    void nodesCallback(const patrolling_build_graph_msgs::PatrollingPoints::ConstPtr&); 
    
    void pcdWallCallback(const sensor_msgs::PointCloud2ConstPtr&);

    void pcdTraversabilityCallback(const sensor_msgs::PointCloud2ConstPtr&);
    
    void cancelCallback(const trajectory_control_msgs::PlanningTask::ConstPtr&);
    
    bool callRobotTrajSaverNodeCheckPathService(nav_msgs::Path&, int, int, int&);
    
    void sendToRobotsCallback(const std_msgs::Bool& msg);
    
    void priorityPointCallback(const patrolling_build_graph_msgs::PriorityPoint::ConstPtr&);

public:
    
    pcl::PointCloud<pcl::PointXYZ>  *p_nodes_pcd_;   // point cloud containing the input nodes 
    std::vector<float> nodes_priority;
    std::vector<size_t> nodes_id;
    
    pcl::PointCloud<pcl::PointXYZ>  *p_nodes_filtered_pcd_; // point cloud containing the filtered input nodes (no too close nodes)
    std::vector<float> nodes_filtered_priority;
    std::vector<size_t> nodes_filtered_id;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> *p_kdtree_;
    boost::recursive_mutex mutex_nodes_pcd_;
    
    pcl::PointCloud<pcl::PointXYZ> *p_wall_pcd_;
    boost::recursive_mutex mutex_wall_pcd_;
    bool b_wall_pcd_ready_;
        
    pcl::PointCloud<pcl::PointXYZ> *p_traversability_pcd_; 
    boost::recursive_mutex mutex_traversability_pcd_;
    bool b_traversability_pcd_ready_;
    
public: /// < file management     

    std::string filename_;
    boost::recursive_mutex mutex_file_;
        
public: /// < ROS pubs, subs 
    
    ros::NodeHandle node_;
        
    // subscribers 
    std::string nodes_topic_name_;
    ros::Subscriber nodes_topic_sub_;
    
    std::string send_to_robot_topic_name_;
    ros::Subscriber send_to_robot_topic_sub_;
    
    std::string pcl_wall_topic_;
    ros::Subscriber pcl_wall_sub_;

    std::string pcl_trav_topic_;
    ros::Subscriber pcl_trav_sub_;

    std::string cancel_graph_topic_;
    ros::Subscriber cancel_graph_sub_;
    
    std::string priority_point_topic_;
    ros::Subscriber priority_point_sub_;    
    
    // service clients
    std::string path_planner_service_name_;
    ros::ServiceClient path_planner_service_client_;

    std::string traj_robot_saver_check_path_service_name_;
    ros::ServiceClient traj_robot_saver_check_path_service_client_;
        
    // publishers 
    std::string edges_marker_topic_;
    ros::Publisher edges_marker_pub_;
    
    std::string build_graph_event_topic_;
    ros::Publisher build_graph_event_pub_;
    
    std::string graph_topic_;
    ros::Publisher graph_pub_;
    
    // visualization messages 
    visualization_msgs::Marker marker_lines_list_;
    boost::recursive_mutex mutex_marker_lines_list_;

public: /// < collision checking

    double collision_check_line_resolution_; // It has to be within [0,1]
    double ground_collisions_check_radius_;
    double wall_collisions_check_radius_;
    int max_num_line_collisions_;

public: /// < graph structures and infos
    
    bool b_interactive_; // if true actually build the graph on call
    int node_branching_factor_;
    int node_max_dist_neighbours_;

    // graph matrices 
    utils::MatrixInt mat_adj_;
    utils::MatrixDouble mat_edge_costs_;
    utils::MatrixString mat_directions_;
    int num_nodes_;
    boost::recursive_mutex mutex_mat_graph_;
    
    // vertex graph struct  
    std::vector<Vertex3D> vertex_graph_;
    boost::recursive_mutex mutex_vertex_graph_;

private:

    ros::NodeHandle n_;
};



#endif /* BUILD_GRAPH_H */

