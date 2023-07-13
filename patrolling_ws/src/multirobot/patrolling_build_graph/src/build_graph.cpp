/**
* This file is part of the ROS package patrolling_build_graph which belongs to the framework 3DMR. 
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

#include "patrolling_build_graph/build_graph.h"

const int GraphBuilder::kMaxAllowedBranchingFactor = 8; // must be < 8 given the current graph data structure 
const double GraphBuilder::kZOffset = 0.3;
const double GraphBuilder::kMaxEdgePitch = M_PI/6;
const double GraphBuilder::kHeighEdgeCost = 1e+10; 
const double GraphBuilder::kMaxDistPointPriority = 0.2;


GraphBuilder::GraphBuilder() : n_("~")
{
    //mat_adj_ = 0;
    //mat_edge_costs_ = 0;
    
    b_wall_pcd_ready_           = false;
    b_traversability_pcd_ready_ = false;
    

    p_nodes_pcd_          = new pcl::PointCloud<pcl::PointXYZ>();
    p_nodes_filtered_pcd_ = new pcl::PointCloud<pcl::PointXYZ>();
    p_kdtree_             = new pcl::KdTreeFLANN<pcl::PointXYZ>();
    p_wall_pcd_           = new pcl::PointCloud<pcl::PointXYZ>();
    p_traversability_pcd_ = new pcl::PointCloud<pcl::PointXYZ>();

    num_nodes_ = 0;

    nodes_topic_name_ = getParam<std::string>(n_, "nodes_topic_name", "/patrolling/task/append");
    nodes_topic_sub_ = node_.subscribe(nodes_topic_name_, 50, &GraphBuilder::nodesCallback, this);
    
    node_branching_factor_ = getParam<int>(n_, "node_branching_factor", 3);
    // max allowed branchig factor is 8
    node_branching_factor_ = std::min(node_branching_factor_,GraphBuilder::kMaxAllowedBranchingFactor);
    
    node_max_dist_neighbours_ = getParam<int>(n_, "node_max_dist_neighbours", 5);

    edges_marker_topic_ = getParam<std::string>(n_, "edges_marker_topic", "/edges_markers");
    edges_marker_pub_ = node_.advertise<visualization_msgs::Marker>(edges_marker_topic_, 10);

    pcl_wall_topic_ = getParam<std::string>(n_, "pcl_wall_topic", "/clustered_pcl/wall");
    pcl_wall_sub_ = node_.subscribe(pcl_wall_topic_, 10, &GraphBuilder::pcdWallCallback, this);

    pcl_trav_topic_ = getParam<std::string>(n_, "pcl_traversability_topic", "/trav/traversability"); // CHECK THE NAME
    pcl_trav_sub_ = node_.subscribe(pcl_trav_topic_, 10, &GraphBuilder::pcdTraversabilityCallback, this);

    build_graph_event_topic_ = getParam<std::string>(n_, "build_graph_event_topic", "/build_graph_event");
    build_graph_event_pub_ = node_.advertise<patrolling_build_graph_msgs::BuildGraphEvent>(build_graph_event_topic_, 10);
    
    graph_topic_ = getParam<std::string>(n_, "graph_topic", "/patrolling/graph");
    graph_pub_ = node_.advertise<patrolling_build_graph_msgs::Graph>(graph_topic_, 10);
    
    priority_point_topic_ = getParam<std::string>(n_, "priority_point_topic_", "/priority_point");
    priority_point_sub_ = node_.subscribe(priority_point_topic_, 10, &GraphBuilder::priorityPointCallback, this);
    

    collision_check_line_resolution_ = getParam<double>(n_, "collision_check_line_resolution", 0.01);
    ground_collisions_check_radius_  = getParam<double>(n_, "ground_collisions_check_radius", 0.1); // 0.25
    wall_collisions_check_radius_    = getParam<double>(n_, "ground_collisions_check_radius", 0.25); 

    max_num_line_collisions_ = getParam<int>(n_, "max_num_line_collisions", 1); // 5

    filename_ = getParam<std::string>(n_, "filename", "vrep.graph");

    cancel_graph_topic_ = getParam<std::string>(n_, "cancel_graph_topic", "/planner/tasks/remove");
    cancel_graph_sub_ = node_.subscribe(cancel_graph_topic_, 10, &GraphBuilder::cancelCallback, this);
    
    send_to_robot_topic_name_  = getParam<std::string>(n_, "send_to_robot_topic", "/patrolling/task/send");
    send_to_robot_topic_sub_   = node_.subscribe(send_to_robot_topic_name_, 10, &GraphBuilder::sendToRobotsCallback, this);

    marker_lines_list_.action = visualization_msgs::Marker::ADD;
    marker_lines_list_.pose.orientation.w = 1.0;
    marker_lines_list_.ns = "edges";
    marker_lines_list_.id = 1;
    marker_lines_list_.type = visualization_msgs::Marker::LINE_LIST;
    marker_lines_list_.scale.x = 0.1;
    marker_lines_list_.color.b = 1.0;
    marker_lines_list_.color.a = 1.0;

    path_planner_service_name_ = getParam<std::string>(n_, "path_planner_service_name", "path_planning_service");
    path_planner_service_client_ = node_.serviceClient<trajectory_control_msgs::PathPlanning>(path_planner_service_name_);

    traj_robot_saver_check_path_service_name_ = getParam<std::string>(n_, "traj_robot_saver_check_path_service_name", "traj_robot_saver_check_path_service");
    traj_robot_saver_check_path_service_client_ = node_.serviceClient<robot_trajectory_saver_msgs::CheckPath>(traj_robot_saver_check_path_service_name_);

    b_interactive_ = getParam<bool>(n_, "interactive", true);
}

void GraphBuilder::printAdjacencyMatrix()
{
    boost::recursive_mutex::scoped_lock locker(mutex_mat_graph_);
    
    std::cout << "......................................................." << std::endl;
    std::cout << "Adiacency Matrix = [ " << "\n";
    for (int h = 0; h < num_nodes_; h++)
    {
        std::cout << " [ ";
        for (int k = 0; k < num_nodes_; k++)
        {
            std::cout << mat_adj_[h][k] << " ";
        }
        std::cout << "]" << "\n";
    }
    std::cout << " ] " << std::endl;

    std::cout << "......................................................." << std::endl;
}

void GraphBuilder::printDirectionsMatrix()
{
    boost::recursive_mutex::scoped_lock locker(mutex_mat_graph_);

    std::cout << ".........................................................\n";
    std::cout << "Directions = [\n";
    for (int r = 0; r < num_nodes_; r++)
    {
        std::cout << "[ ";
        for (int s = 0; s < num_nodes_; s++)
        {
            std::cout << mat_directions_[r][s] << " ";
        }
        std::cout << "]\n";
    }
    std::cout << ".........................................................\n";
}

GraphBuilder::~GraphBuilder()
{
}

void GraphBuilder::pcdWallCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
    boost::recursive_mutex::scoped_lock locker(mutex_wall_pcd_);
        
    if (p_wall_pcd_) delete p_wall_pcd_;
    p_wall_pcd_ = new pcl::PointCloud<pcl::PointXYZ>();
    pcl::fromROSMsg(*cloud_in, *p_wall_pcd_);
    b_wall_pcd_ready_ = true; 
}

void GraphBuilder::pcdTraversabilityCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
    boost::recursive_mutex::scoped_lock locker(mutex_traversability_pcd_);
        
    pcl::PointCloud<pcl::PointXYZI> *temp_pcd;
    temp_pcd = new pcl::PointCloud<pcl::PointXYZI>();

    if (p_traversability_pcd_) delete p_traversability_pcd_;
    p_traversability_pcd_ = new pcl::PointCloud<pcl::PointXYZ>();

    pcl::fromROSMsg(*cloud_in, *temp_pcd);

    BOOST_FOREACH(const pcl::PointXYZI& pt, temp_pcd->points)
    {
        pcl::PointXYZ q;
        q.x = pt.x;
        q.y = pt.y;
        q.z = pt.z;
        p_traversability_pcd_->points.push_back(q);
    }
    
    b_traversability_pcd_ready_ = true; 

}

bool GraphBuilder::callRobotTrajSaverNodeCheckPathService(nav_msgs::Path& pois, int s_node_id, int d_node_id, int& path_cost)
{
    if (traj_robot_saver_check_path_service_client_.waitForExistence(ros::Duration(5.0)))
    {
        robot_trajectory_saver_msgs::CheckPath srv;
        srv.request.point_of_interest = pois;
        srv.request.source_node_id = s_node_id;
        srv.request.destination_node_id = d_node_id;
        if (traj_robot_saver_check_path_service_client_.call(srv))
        {
            path_cost = srv.response.path_cost;
            return srv.response.result;
        }
        else
        {
            ROS_ERROR("Failed to call service provided by robot_trajectory_saver_node");
            return false;
        }
    }
    else
    {
        ROS_ERROR("robot_traj_service check_path is not UP!!! There is an issue");
        return false;
    }
}


void GraphBuilder::initMatGraphDataStructures()
{
    boost::recursive_mutex::scoped_lock locker(mutex_mat_graph_);
        
    ROS_INFO("Initialize adiacency matrix");
    mat_adj_ = utils::MatrixInt(num_nodes_, num_nodes_,0);  

    ROS_INFO("Initialize edge-costs matrix");
    mat_edge_costs_ = utils::MatrixDouble(num_nodes_, num_nodes_,0); 

    ROS_INFO("Initialize directions matrix");
    mat_directions_ = utils::MatrixString(num_nodes_, num_nodes_,"-");  
}


// Not USED
bool GraphBuilder::checkPitchAngle(pcl::PointXYZ& s, pcl::PointXYZ& t)
{
    double epsilon = 0.15;
    // FILL DIRECTIONS
    pcl::PointXYZ d;
    d.x = t.x - s.x;
    d.y = t.y - s.y;
    d.z = t.z - s.z;

    //        double den = sqrt(pow(d.x,2) + pow(d.y,2) + pow(d.z,2));
    //        
    //        pcl::PointXYZ d_norm;
    //        d_norm.x = d.x / den;
    //        d_norm.y = d.y / den;
    //        d_norm.z = d.z / den;   

    double l = sqrt(pow(d.x, 2) + pow(d.y, 2));

    double pitch = 0.0;
    if (d.z < 0)
    {
        pitch = -atan2(d.z, l);
    }
    else
    {
        pitch = atan2(d.z, l);
    }
    double pitch02PI = normalize_angle_positive(pitch);

    std::cout << "Angle between the considered pair of pois " << pitch << std::endl;
    std::cout << "Angle between the considered pair of pois normalized " << pitch02PI << std::endl;

    if ((pitch02PI >= 0) && (pitch02PI < (M_PI / 4 - epsilon)))
    {
        return true;
    }
    else if ((pitch02PI > (3 * M_PI / 4 + epsilon)) && (pitch02PI < (5 * M_PI / 4 - epsilon)))
    {
        return true;
    }
    else if ((pitch02PI > (7 * M_PI / 4 + epsilon)) && (pitch02PI <= 2 * M_PI))
    {
        return true;
    }
    else
    {
        return false;
    }

}

// basically it checks if two nodes are on two different floors; compute the pitch angle of the unit vector joining the two nodes
bool GraphBuilder::checkEdgeMaxPitch(int i, int j, pcl::PointXYZ& s, pcl::PointXYZ& t)
{
    // FILL DIRECTION VECTOR UN_NORMALIZED
    pcl::PointXYZ delta;
    delta.x = t.x - s.x;
    delta.y = t.y - s.y;
    delta.z = t.z - s.z;

    if (delta.z < 0)
    {
        std::cout << "checkEdgeMaxPitch() - NEIGHBORHOOD_ID " << j << " is UNDER CURRENT_NODE_ID " << i << std::endl;
    }
    else
    {
        std::cout << "checkEdgeMaxPitch() - NEIGHBORHOOD_ID " << j << " is ON_TOP_OF CURRENT_NODE_ID " << i << std::endl;
    }

    double norm = sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2));

    pcl::PointXYZ unit_vec;
    unit_vec.x = delta.x / norm;
    unit_vec.y = delta.y / norm;
    unit_vec.z = delta.z / norm;

    double pitch = asin(unit_vec.z);

    if ((pitch > -kMaxEdgePitch) && (pitch < kMaxEdgePitch))
    {
        std::cout << "checkEdgeMaxPitch() - pitch(CURRENT_NODE_ID, NEIGHBORHOOD_ID) = (" << i << ", " << j << ") = " << pitch << " --> OK " << std::endl;
        return true;

    }
    else
    {
        std::cout << "Pitch(CURRENT_NODE_ID, NEIGHBORHOOD_ID) = (" << i << ", " << j << ") = " << pitch << " --> TOO STEEP " << std::endl;
        return false;
    }

}

// check if edge has ground intersection 
bool GraphBuilder::checkEdgeNoGroundIntersection(int i, int j, pcl::PointXYZ& s, pcl::PointXYZ& e)
{
    if(!b_traversability_pcd_ready_)
    {
        ROS_WARN_STREAM("GraphBuilder::checkEdgeNoGroundIntersection() - no traversability received");   
        return false;
    }
    
    double delta_perc = 0;

    double z_delta = 0.0;
    double z_increment = 0.01;
    double z_delta_max = 0.1;

    std::vector<int> num_intersections;
    pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree_collision = new pcl::KdTreeFLANN<pcl::PointXYZ>();
    kdtree_collision->setInputCloud(p_traversability_pcd_->makeShared());

    pcl::PointXYZ d;
    d.x = e.x - s.x;
    d.y = e.y - s.y;
    d.z = e.z - s.z;

    pcl::PointXYZ start;
    pcl::PointXYZ end;

    if (d.z < 0)
    {
        //std::cout << "checkEdgeGroundIntersection - NEIGHBORHOOD_ID " << j << " is UNDER CURRENT_NODE_ID " << i << std::endl;
        start.x = s.x;
        start.y = s.y;
        start.z = s.z + kZOffset;
        end.x = e.x;
        end.y = e.y;
        end.z = e.z + kZOffset;
    }
    else
    {
        //std::cout << "checkEdgeGroundIntersection - NEIGHBORHOOD_ID " << j << " is ON_TOP_OF CURRENT_NODE_ID " << i << std::endl;
        start.x = e.x;
        start.y = e.y;
        start.z = e.z + kZOffset;
        end.x = s.x;
        end.y = s.y;
        end.z = s.z + kZOffset;
    }

    bool b_done = false;
    while (z_delta <= z_delta_max && !b_done)
    {
        start.z = start.z + z_delta;

        while (delta_perc <= 1.0)
        {

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            int neighborhoods = kdtree_collision->radiusSearch(start, ground_collisions_check_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            num_intersections.push_back(neighborhoods);

            //std::cout << "Current num of neighborhood: " << neighborhoods << " at resolution: " << resolution << "\n";

            delta_perc = delta_perc + collision_check_line_resolution_;
            start.x = start.x + delta_perc * (end.x - start.x);
            start.y = start.y + delta_perc * (end.y - start.y);
            start.z = start.z + delta_perc * (end.z - start.z);
        }

        int max_intersec = compute_max_num_intersections(num_intersections);

        if (max_intersec <= max_num_line_collisions_)
        {
            return true;
        }
        else
        {
            b_done = false;
        }

        z_delta = z_delta + z_increment;
    }

    return b_done;
}

// check if edge has wall intersection 
bool GraphBuilder::checkEdgeNoWallIntersection(int node_id, int other_node_id, pcl::PointXYZ& node_pt, pcl::PointXYZ& other_node_pt)
{
    if(!b_wall_pcd_ready_) 
    {
        ROS_WARN_STREAM("GraphBuilder::checkEdgeNoWallIntersection() - no wall received");   
        return false;
    }
        
    /// < check wall intersection 
    pcl::PointXYZ t;
    t.x = node_pt.x;
    t.y = node_pt.y;
    t.z = node_pt.z + kZOffset;

    double delta_perc = 0;
    std::vector<int> num_intersections;

    //ROS_INFO("Collision checking test START");

    pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree_collision = new pcl::KdTreeFLANN<pcl::PointXYZ>();
    kdtree_collision->setInputCloud(p_wall_pcd_->makeShared());

    while (delta_perc <= 1.0)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        int neighborhoods = kdtree_collision->radiusSearch(t, wall_collisions_check_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        num_intersections.push_back(neighborhoods);

        //std::cout << "Current num of neighborhood: " << neighborhoods << " at resolution: " << resolution << "\n";

        delta_perc = delta_perc + collision_check_line_resolution_;
        t.x = t.x + delta_perc * (other_node_pt.x - t.x);
        t.y = t.y + delta_perc * (other_node_pt.y - t.y);
        t.z = t.z + delta_perc * (other_node_pt.z + kZOffset - t.z);
    }

    //ROS_INFO("Collision checking test END");

    int max_num_intersections = compute_max_num_intersections(num_intersections);

    std::cout << "The edge <" << node_id << "," << other_node_id << ">" << " hit " << max_num_intersections << " points belonging to wall pcd" << std::endl;
    
    return max_num_intersections <= max_num_line_collisions_;
}


void GraphBuilder::nodesCallback(const patrolling_build_graph_msgs::PatrollingPoints::ConstPtr& nodes_msg) 
{  
    boost::recursive_mutex::scoped_lock locker_mat(mutex_mat_graph_);
    boost::recursive_mutex::scoped_lock locker_vert(mutex_vertex_graph_);
    boost::recursive_mutex::scoped_lock locker_nodes(mutex_nodes_pcd_);    
    
    if(nodes_msg->num_nodes != nodes_msg->node_id.size())
    {
        ROS_ERROR_STREAM("ERROR in GraphBuilder::nodesCallback(): number of nodes is incoherent");
        return; 
    }    
    
    if (b_interactive_)
    {
        std::cout << "============================================================" << std::endl; 
        ROS_INFO("Building the graph on START");

        ROS_INFO("Cleaning point cloud and kdtree data structure");
        
        if (p_nodes_pcd_) delete p_nodes_pcd_;
        p_nodes_pcd_ = new pcl::PointCloud<pcl::PointXYZ>();
        p_nodes_pcd_->header.frame_id = nodes_msg->header.frame_id;
        //p_nodes_pcd_->header.stamp = nodes_msg->header.stamp;
        nodes_priority.clear();
        nodes_id.clear();                
                
        if (p_nodes_filtered_pcd_) delete p_nodes_filtered_pcd_;
        p_nodes_filtered_pcd_ = new pcl::PointCloud<pcl::PointXYZ>(); 
        p_nodes_filtered_pcd_->header.frame_id = p_nodes_pcd_->header.frame_id;        
        nodes_filtered_priority.clear();
        nodes_filtered_id.clear();        
        
        if (p_kdtree_) delete p_kdtree_;
        p_kdtree_ = new pcl::KdTreeFLANN<pcl::PointXYZ>();        

        ROS_INFO_STREAM("Building the pcd from " << nodes_msg->num_nodes << " received waypoints");
        for (int i = 0; i < nodes_msg->num_nodes; i++)
        {
//            geometry_msgs::Point pose = nodes_msg->poses[i];
//            pcl::PointXYZ point;
//            point.x = pose.pose.position.x;
//            point.y = pose.pose.position.y;
//            point.z = pose.pose.position.z;
            
            geometry_msgs::Point node_position = nodes_msg->node_position[i];
            pcl::PointXYZ point;
            point.x = node_position.x;
            point.y = node_position.y;
            point.z = node_position.z;
            
            p_nodes_pcd_->points.push_back(point);
            
            nodes_priority.push_back(nodes_msg->node_priority[i]);
            nodes_id.push_back(nodes_msg->node_id[i]);
        }

        ROS_INFO("Removing of duplicate waypoints");
        //p_nodes_filtered_pcd_ = check_all_pts_different(p_nodes_pcd_);
        check_all_pts_different(*p_nodes_pcd_, nodes_priority, nodes_id,
                                *p_nodes_filtered_pcd_, nodes_filtered_priority, nodes_filtered_id);
                                            
        num_nodes_ = p_nodes_filtered_pcd_->points.size();
        ROS_INFO("Number of actual patrolling nodes %d", num_nodes_);

        initMatGraphDataStructures();

        boost::recursive_mutex::scoped_lock locker_maker(mutex_marker_lines_list_);
                    
        ROS_INFO("Clearing of the MarkerArray graph");
        marker_lines_list_.header.frame_id = nodes_msg->header.frame_id;
        marker_lines_list_.header.stamp = ros::Time::now();
        marker_lines_list_.points.clear();


        std::vector<int> pointIdxNKNSearch(num_nodes_);
        std::vector<float> pointNKNSquaredDistance(num_nodes_);

        p_kdtree_->setInputCloud(p_nodes_filtered_pcd_->makeShared());


        for (int node_id = 0; node_id < num_nodes_; node_id++)
        {
            std::cout << "**************************************************************" << std::endl; 
            std::cout << "**************************************************************" << std::endl; 
            std::cout << "Current Node ID: " << node_id <<  std::endl; 
            
            pcl::PointXYZ node_pt = p_nodes_filtered_pcd_->points[node_id];

            int num_neighbours = p_kdtree_->nearestKSearch(node_pt, num_nodes_, pointIdxNKNSearch, pointNKNSquaredDistance);

            // remove nodes which are far than node_max_dist_neighbours_
            remove_far_nodes(pointIdxNKNSearch, pointNKNSquaredDistance, node_max_dist_neighbours_ * node_max_dist_neighbours_);
            
            int num_potential_neighbours = pointNKNSquaredDistance.size();
            num_potential_neighbours = std::max(std::min(node_branching_factor_, num_potential_neighbours - 1), 0); // -1 since we have to remove the first found neighbor which is the node under analysis itself

            std::cout << " \tMax allowed number of neighbours (=node_branching_factor param): " << node_branching_factor_ << "\n";
            std::cout << " \tMax number of potential neighbours(returned by kdtree and max dist check): " << num_potential_neighbours << "\n";
            //std::cout << " \tNumber of actual neighborhoods : " << num_actual_neighborhoods << std::endl;

            std::cout << "Order of processing of neighborhoods:\n";
            for (int l = 1; l < pointIdxNKNSearch.size(); l++)
            {
                std::cout << "\tNode neigh_ID: " << pointIdxNKNSearch[l] << " at distance " << sqrt(pointNKNSquaredDistance[l]) << " from Node ID: " << node_id << "\n";
            }

            boost::recursive_mutex::scoped_lock locker_wall(mutex_wall_pcd_);
            boost::recursive_mutex::scoped_lock locker_trav(mutex_traversability_pcd_);

            int nearest_ksearch_id = 1; // we have to discard point in [0] since it is the same node

            bool b_done = false;
            int actual_num_neighbours = 0;
            std::vector<int> neighbours_list; 

            while( (!b_done ) && (nearest_ksearch_id < pointNKNSquaredDistance.size()) )
            {
                // if we have found a number of neighbours > num_potential_neighbours then exit!
                actual_num_neighbours = compute_num_neighbours(mat_adj_, node_id, neighbours_list);
                std::cout << "current number neighbours: " << actual_num_neighbours << " [";  
                for(int ii=0; ii < neighbours_list.size(); ii++){ std::cout << neighbours_list[ii] <<",";} 
                std::cout << "]" << std::endl;
                
                if (actual_num_neighbours >= num_potential_neighbours)
                {
                    b_done = true;
                }

                if (!b_done)
                {
                    std::cout << "Neighborhood ID: " << pointIdxNKNSearch[nearest_ksearch_id] << std::endl;

                    if (mat_adj_[node_id][pointIdxNKNSearch[nearest_ksearch_id]] == 0)
                    {
                        /// < check edge pitch 
                        bool b_edge_pitch_is_ok = checkEdgeMaxPitch(node_id, pointIdxNKNSearch[nearest_ksearch_id], node_pt, p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]]);

                        if (b_edge_pitch_is_ok)
                        {
                            std::cout << "It does not exist an edge between node id " << node_id << " and " << pointIdxNKNSearch[nearest_ksearch_id] << " let's see if we can add it!!!" << std::endl;

                            /// < check ground intersection 
                            bool b_edge_no_ground_intersections = checkEdgeNoGroundIntersection(node_id, pointIdxNKNSearch[nearest_ksearch_id], node_pt, p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]]);

                            if (b_edge_no_ground_intersections)
                            {
                                /// < check wall intersection 
                                bool b_edge_no_wall_intersections = checkEdgeNoWallIntersection(node_id, pointIdxNKNSearch[nearest_ksearch_id], node_pt, p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]]);

                                if(b_edge_no_wall_intersections)    
                                {
                                    /// < call path planner service to check the cost and traversability of the edge 
                                    
                                    trajectory_control_msgs::PathPlanning srv_msg;
                                    srv_msg.request.start.header.stamp = ros::Time::now();
                                    srv_msg.request.start.header.frame_id = "map";

                                    srv_msg.request.goal.header.stamp = srv_msg.request.start.header.stamp;
                                    srv_msg.request.goal.header.frame_id = srv_msg.request.start.header.frame_id;

                                    srv_msg.request.start.pose.position.x = node_pt.x;
                                    srv_msg.request.start.pose.position.y = node_pt.y;
                                    srv_msg.request.start.pose.position.z = node_pt.z;

                                    srv_msg.request.goal.pose.position.x = p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]].x;
                                    srv_msg.request.goal.pose.position.y = p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]].y;
                                    srv_msg.request.goal.pose.position.z = p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]].z;

                                    if (path_planner_service_client_.call(srv_msg))
                                    {
                                        if (srv_msg.response.success)
                                        {
                                            std::cout << "Fill cost matrix by adding to edge <" << node_id << "," << pointIdxNKNSearch[nearest_ksearch_id] << "> cost: " << srv_msg.response.path_cost << std::endl;
                                            mat_edge_costs_[node_id][pointIdxNKNSearch[nearest_ksearch_id]] = srv_msg.response.path_cost;
                                            std::cout << "Fill cost matrix by adding to edge <" << pointIdxNKNSearch[nearest_ksearch_id] << "," << node_id << "> cost: " << srv_msg.response.path_cost << std::endl;
                                            mat_edge_costs_[pointIdxNKNSearch[nearest_ksearch_id]][node_id] = srv_msg.response.path_cost;

                                            std::cout << "Fill adiacency matrix by adding the edge <" << node_id << "," << pointIdxNKNSearch[nearest_ksearch_id] << ">" << std::endl;
                                            mat_adj_[node_id][pointIdxNKNSearch[nearest_ksearch_id]] = 1;
                                            std::cout << "Fill adiacency matrix by adding the edge <" << pointIdxNKNSearch[nearest_ksearch_id] << "," << node_id << ">" << std::endl;
                                            mat_adj_[pointIdxNKNSearch[nearest_ksearch_id]][node_id] = 1;

                                            printAdjacencyMatrix();

                                            geometry_msgs::Point pointA; // Point A of the line
                                            pointA.x = node_pt.x;
                                            pointA.y = node_pt.y;
                                            pointA.z = node_pt.z + kZOffset;

                                            geometry_msgs::Point pointB; // Point B of the line
                                            pointB.x = p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]].x;
                                            pointB.y = p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]].y;
                                            pointB.z = p_nodes_filtered_pcd_->points[pointIdxNKNSearch[nearest_ksearch_id]].z + kZOffset;

                                            // FILL GRAPH MARKER LINES
                                            marker_lines_list_.points.push_back(pointA);
                                            marker_lines_list_.points.push_back(pointB);

                                            // FILL DIRECTIONS
                                            geometry_msgs::Point d;
                                            d.x = pointB.x - pointA.x;
                                            d.y = pointB.y - pointA.y;
                                            d.z = pointB.z - pointA.z;

                                            std::cout << "Computing the direction of node " << pointIdxNKNSearch[nearest_ksearch_id] << " with respect to node " << node_id << std::endl;
                                            double angle = atan2(d.y, d.x);
                                            std::cout << "atan2(d.y, d.x)=" << angle << "\n";
                                            double angle02PI = normalize_angle_positive(angle);
                                            std::cout << "normalize_angle_positive(angle)=" << angle02PI << "\n";
                                            std::string dir = compute_direction(angle02PI);

                                            //printDirectionsMatrix();

                                            mat_directions_[node_id][pointIdxNKNSearch[nearest_ksearch_id]] = dir;
                                            mat_directions_[pointIdxNKNSearch[nearest_ksearch_id]][node_id] = compute_inverse_direction(dir);

                                            printDirectionsMatrix();

                                            //std::cout << "\tID = " << pointIdxNKNSearch[j] << " DIR = " << dir << "\n";
                                            std::cout << "Edge <" << node_id << "," << pointIdxNKNSearch[nearest_ksearch_id] << ">" << "\n";
                                            std::cout << "Angle " << angle02PI << " Direction " << dir << "\n";
                                            std::cout << "Edge <" << pointIdxNKNSearch[nearest_ksearch_id] << "," << node_id << ">" << "\n";
                                            std::cout << "Angle " << normalize_angle_positive(angle02PI + 0.5 * M_PI) << " Direction " << mat_directions_[pointIdxNKNSearch[nearest_ksearch_id]][node_id] << "\n";

                                        }
                                        else
                                        {
                                            std::cout << "A path between <" << node_id << "," << pointIdxNKNSearch[nearest_ksearch_id] << "> does not exist!" << std::endl;
                                        }

                                    }
                                    else
                                    {
                                        std::cout << "Path planner service call FAILURE\n";
                                    }

                                }
                                else
                                {
                                    //std::cout << "Collision checking test: (" << max_num_line_collisions_ << " <= " << max_num_intersections << ") -> TRUE" << std::endl;
                                    std::cout << "The edge between node id " << node_id << " and node id " << pointIdxNKNSearch[nearest_ksearch_id] << " crosses a wall!\n";
                                }

                            }
                            else
                            {
                                std::cout << "The edge between node id " << node_id << " and node id " << pointIdxNKNSearch[nearest_ksearch_id] << " crosses the ground!\n";
                            }

                        }// end if edge pitch is ok 
                        else
                        {
                            std::cout << "A possible edge between node id " << node_id << " and node id " << pointIdxNKNSearch[nearest_ksearch_id] << " might be too steep!!!\n";
                        }

                    } // end if mat_adj_[node_id][pointIdxNKNSearch[other_node_id]] == 0)
                    else
                    {
                        //std::cout << "\tID = " << pointIdxNKNSearch[j] << " DIR = " << directions[i][pointIdxNKNSearch[j]] << "\n";
                        std::cout << "An edge between node id " << node_id << " and " << pointIdxNKNSearch[nearest_ksearch_id] << " already exits" << std::endl;
                        printAdjacencyMatrix();
                    }

                    nearest_ksearch_id++;

                } // end  if (!b_done)

            } // end while (!b_done && other_node_id < num_nodes_)

            std::cout << "......................................................." << std::endl;
        }

        edges_marker_pub_.publish(marker_lines_list_);

        ROS_INFO("Building the graph END");

        buildVertexGraph();
        writeVertexGraphOnFile();

        //publishBuildGraphEvent();
        
        //publishGraph(); 
    }
}

void GraphBuilder::publishBuildGraphEvent()
{
    patrolling_build_graph_msgs::BuildGraphEvent msg;
    msg.event = patrolling_build_graph_msgs::BuildGraphEvent::GRAPH_BUILT;
    build_graph_event_pub_.publish(msg);
}

void GraphBuilder::buildVertexGraph()
{
    boost::recursive_mutex::scoped_lock locker_mat(mutex_mat_graph_);
    boost::recursive_mutex::scoped_lock locker_vert(mutex_vertex_graph_);
    boost::recursive_mutex::scoped_lock locker_nodes(mutex_nodes_pcd_);
    
    const int navCostConversionFloatToUint = patrolling_build_graph_msgs::Graph::kNavCostConversionFloatToUint;
        
    vertex_graph_.clear();
    for (int i = 0; i < num_nodes_; i++)
    {
        Vertex3D v;
        
        v.index    = i;
        v.id       = i;//nodes_filtered_id[i]; // N.B.: here we guarantee that ids are consecutive and equal to index!!!
        
        v.priority = nodes_filtered_priority[i];
        
        v.x = p_nodes_filtered_pcd_->points[i].x;
        v.y = p_nodes_filtered_pcd_->points[i].y;
        v.z = p_nodes_filtered_pcd_->points[i].z;

        int n = 0;
        for (int j = 0; j < num_nodes_; j++)
        {
            if (mat_adj_[i][j] == 1)
            {
                v.id_neighbour.push_back(j);
                //v.id_neighbour.push_back(nodes_filtered_id[j]);
                
                v.dir.push_back(mat_directions_[i][j]);

                v.cost.push_back(mat_edge_costs_[i][j]*navCostConversionFloatToUint);// convert it to int type (as represented in the patrol agent)
                n++;
            }
        }
        v.num_neighbours = n;
        vertex_graph_.push_back(v);
    }
}

void GraphBuilder::writeVertexGraphOnFile()
{
    boost::recursive_mutex::scoped_lock locker_mat(mutex_mat_graph_);
    boost::recursive_mutex::scoped_lock locker_vert(mutex_vertex_graph_);
    
    boost::recursive_mutex::scoped_lock locker_file(mutex_file_);
    
    FILE *file = fopen(filename_.c_str(), "w");
    fprintf(file, "%lu\n", vertex_graph_.size());   // number of nodes 
    fprintf(file, "%d\n", 25); // WIDTH_PX
    fprintf(file, "%d\n", 25); // HEIGHT_PX
    fprintf(file, "%d\n", 1);  // RESOLUTION
    fprintf(file, "\n");

    for (int i = 0; i < vertex_graph_.size(); i++)
    {
        fprintf(file, "%d\n",  vertex_graph_[i].id);
        
        fprintf(file, "%lf\n", vertex_graph_[i].x);
        fprintf(file, "%lf\n", vertex_graph_[i].y);
        fprintf(file, "%lf\n", vertex_graph_[i].z);
        
        fprintf(file, "%d\n", vertex_graph_[i].num_neighbours);
        for (int j = 0; j < vertex_graph_[i].num_neighbours; j++)
        {
            fprintf(file, "%d\n", vertex_graph_[i].id_neighbour[j]);
            fprintf(file, "%s\n", vertex_graph_[i].dir[j].c_str());
            fprintf(file, "%d\n", vertex_graph_[i].cost[j]);
        }
        fprintf(file, "\n");
    }
    fclose(file);
}


void GraphBuilder::publishGraph()
{
    boost::recursive_mutex::scoped_lock locker_mat(mutex_mat_graph_);
    boost::recursive_mutex::scoped_lock locker_vert(mutex_vertex_graph_);
    
    patrolling_build_graph_msgs::Graph msg;
    
//    # general information
//    uint32   num_nodes  # graph number of nodes
//
//    # lists (i-th item of each list contains information concerning the i-th node)
//    uint32[] node_id    # node id
//    geometry_msgs/Point[] node_position  # node position
//    uint32[] num_neighbours  # number of neighbours
//
//    # matrices of size num_nodes x num_nodes in row-major order 
//    bool[]  adjacency_matrix
//    string[]  direction_matrix
//    float32[] cost_matrix
   
    int num_nodes  = vertex_graph_.size();
    int num_nodes2 = num_nodes*num_nodes;
   
    std::cout << "GraphBuilder::publishGraph(): num nodes " << num_nodes << std::endl; 
    
    msg.num_nodes  = num_nodes;
    
    msg.node_id.resize(num_nodes);
    msg.node_priority.resize(num_nodes);
    msg.node_position.resize(num_nodes);
    msg.num_neighbours.resize(num_nodes);
    
    msg.adjacency_matrix.resize(num_nodes2,0);
    msg.direction_matrix.resize(num_nodes2,"-");
    msg.cost_matrix.resize(num_nodes2,0.);

    for (int i = 0; i < num_nodes; i++)
    {
        msg.node_id[i]         = vertex_graph_[i].id;
        msg.node_priority[i]   = vertex_graph_[i].priority;
        
        msg.node_position[i].x = vertex_graph_[i].x;
        msg.node_position[i].y = vertex_graph_[i].y;
        msg.node_position[i].z = vertex_graph_[i].z;
        
        msg.num_neighbours[i]  = vertex_graph_[i].num_neighbours;
        
        std::cout << "id: " << i << " num_neighbours: " << vertex_graph_[i].num_neighbours << std::endl; 
        //std::cout << "id: " << vertex_graph_[i].id << " num_neighbours: " << vertex_graph_[i].num_neighbours << std::endl;         
        
        for (int j = 0; j < vertex_graph_[i].num_neighbours; j++)
        {
            // find the index of the node with id = vertex_graph_[i].id_neighbour[j]
            int found_index; 
            for(found_index = 0; found_index < num_nodes; found_index++) 
            {
                if(vertex_graph_[found_index].id == vertex_graph_[i].id_neighbour[j]) break; 
            }
            if(found_index == num_nodes)
            {
                ROS_ERROR_STREAM("node " << vertex_graph_[i].id << " has neighbour " << vertex_graph_[i].id_neighbour[j] << " not found");
                ROS_ERROR_STREAM("GraphBuilder::publishGraph() - strange graph was not published");
                return; 
            }
            
            // row-major order
            int index     = i*num_nodes + found_index;
            int index_sim = found_index*num_nodes + i;
            msg.adjacency_matrix[index] = msg.adjacency_matrix[index_sim] = 1;
            msg.direction_matrix[index] = msg.direction_matrix[index_sim] = vertex_graph_[i].dir[j].c_str();
            msg.cost_matrix[index]      = msg.cost_matrix[index_sim] = vertex_graph_[i].cost[j];
        }
    }
        
    std::cout << "......................................................." << std::endl;
    std::cout << "msg adiacency Matrix = [ " << "\n";
    for (int h = 0; h < num_nodes_; h++)
    {
        for (int k = 0; k < num_nodes_; k++)
        {
            int index = h*num_nodes_ + k;
            std::cout << (int)msg.adjacency_matrix[index] << " ";
        }
        std::cout << "]" << "\n";
    }
    std::cout << "......................................................." << std::endl;
    std::cout << "msg direction Matrix = [ " << "\n";
    for (int h = 0; h < num_nodes_; h++)
    {
        for (int k = 0; k < num_nodes_; k++)
        {
            int index = h*num_nodes_ + k;
            std::cout << msg.direction_matrix[index] << " ";
        }
        std::cout << "]" << "\n";
    }
    std::cout << "......................................................." << std::endl;
    std::cout << "msg cost Matrix = [ " << "\n";
    for (int h = 0; h < num_nodes_; h++)
    {
        for (int k = 0; k < num_nodes_; k++)
        {
            int index = h*num_nodes_ + k;
            std::cout << msg.cost_matrix[index] << " ";
        }
        std::cout << "]" << "\n";
    }
    std::cout << "......................................................." << std::endl;
    
    graph_pub_.publish(msg);
    ROS_INFO_STREAM("GraphBuilder::publishGraph() - published graph");
}


void GraphBuilder::cancelCallback(const trajectory_control_msgs::PlanningTask::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker_mat(mutex_mat_graph_);
    boost::recursive_mutex::scoped_lock locker_vert(mutex_vertex_graph_);
    
    boost::recursive_mutex::scoped_lock locker_maker(mutex_marker_lines_list_);
    
    marker_lines_list_.points.clear();
    edges_marker_pub_.publish(marker_lines_list_);

    mat_directions_.clear();

    boost::recursive_mutex::scoped_lock locker_file(mutex_file_);
        
    num_nodes_ = 0;
    if (remove(filename_.c_str()) != 0)
        std::cout << "Error deleting file" << std::endl;
    else
        std::cout << "File successfully deleted" << std::endl;
}

void GraphBuilder::sendToRobotsCallback(const std_msgs::Bool& msg)
{
    boost::recursive_mutex::scoped_lock locker_mat(mutex_mat_graph_);
    boost::recursive_mutex::scoped_lock locker_vert(mutex_vertex_graph_);
    boost::recursive_mutex::scoped_lock locker_nodes(mutex_nodes_pcd_);
    
    if(!vertex_graph_.empty())
    {
        publishGraph();    
    }
}


void GraphBuilder::priorityPointCallback(const patrolling_build_graph_msgs::PriorityPoint::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock locker_mat(mutex_mat_graph_);
    boost::recursive_mutex::scoped_lock locker_vert(mutex_vertex_graph_);
    boost::recursive_mutex::scoped_lock locker_nodes(mutex_nodes_pcd_);    

    if(vertex_graph_.empty())
    {
        ROS_WARN_STREAM("GraphBuilder::setPriorityPointCallback() - graph is empty! cannot insert priority point");
        return;
    }
    
    double px = msg->position.x; 
    double py = msg->position.y; 
    double pz = msg->position.z;     
            
    double dist_min = std::numeric_limits<double>::max();
    int index_min = -1; 
    for(size_t ii=0; ii < vertex_graph_.size(); ii++)
    {
        double dist = sqrt( pow(px-vertex_graph_[ii].x,2) + pow(py-vertex_graph_[ii].y,2) + pow(pz-vertex_graph_[ii].z,2) );
        if(dist < dist_min)
        {
            dist_min  = dist;
            index_min = ii;
        }
    }
    
    if( (dist_min < kMaxDistPointPriority) &&  (index_min > -1) )
    {
        vertex_graph_[index_min].priority = msg->priority; 
        ROS_INFO_STREAM("set priority on node: " << msg->id << ", closest point distance: " << dist_min);        
    }
    else
    {
        if(index_min > -1)
        {
            ROS_WARN_STREAM("received priority point far from graph");
        }
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "build_graph");

    GraphBuilder bg;

    ros::spin();

    return (0);

}
