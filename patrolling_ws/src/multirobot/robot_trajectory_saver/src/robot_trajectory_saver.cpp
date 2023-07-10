/**
* This file is part of the ROS package robot_trajectory_saver which belongs to the framework 3DPATROLLING. 
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

#include <robot_trajectory_saver/robot_trajectory_saver.h>

void print(Graph* graph_ptr)
{

    VertexPositions v_positions = get(vertex_properties, *graph_ptr);
    property_map<Graph, vertex_index_t>::type index = get(vertex_index, *graph_ptr);
    std::pair<vertex_iterator, vertex_iterator> vp;
    for (vp = vertices(*graph_ptr); vp.first != vp.second; ++vp.first)
    {

        Vertex v = *vp.first;
        Point3D p = v_positions[v];
        //        ps.pose.position.x = p.x;
        //        ps.pose.position.y = p.y;
        //        ps.pose.position.z = p.z;
        //        ps.pose.orientation.w = 1;

        std::cout << "p: " << p.x << ", " << p.y << ", " << p.z << std::endl;
    }
}

void RobotTrajectorySaver::fromBoostGraphToNavMsgPath(nav_msgs::Path& path)
{

    path.header.frame_id = global_frame;
    path.header.stamp = ros::Time::now();

    VertexPositions v_positions = get(vertex_properties, *graph_ptr);
    property_map<Graph, vertex_index_t>::type index = get(vertex_index, *graph_ptr);
    std::pair<vertex_iterator, vertex_iterator> vp;
    for (vp = vertices(*graph_ptr); vp.first != vp.second; ++vp.first)
    {

        Vertex v = *vp.first;
        Point3D p = v_positions[v];
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = global_frame;
        ps.header.stamp = path.header.stamp;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.position.z = p.z;
        ps.pose.orientation.w = 1;
        path.poses.push_back(ps);
    }
}

bool RobotTrajectorySaver::findVertex(int i, Vertex& vertex)
{
    property_map<Graph, vertex_index_t>::type index = get(vertex_index, *graph_ptr);

    std::pair<vertex_iterator, vertex_iterator> vp;
    for (vp = vertices(*graph_ptr); vp.first != vp.second; ++vp.first)
    {
        if (index[*vp.first] == i)
        {
            vertex = *vp.first;
            return true;
        }
        //std::cout << index[*vp.first] << std::endl;
    }
    return false;
}

void RobotTrajectorySaver::computeShortestPath(int i, int j, std::vector<int>& path)
{
    property_map<Graph, vertex_index_t>::type index = get(vertex_index, *graph_ptr);
    Vertex source;
    bool source_found = findVertex(i, source);

    Vertex destination;
    bool destination_found = findVertex(j, destination);

    if (source_found && destination_found)
    {
        path.push_back(j);

        std::vector<Vertex> p(num_vertices(*graph_ptr));
        std::vector<int> d(num_vertices(*graph_ptr));

        dijkstra_shortest_paths(*graph_ptr, source, predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, *graph_ptr))).distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, *graph_ptr))));

        Vertex pred = p[destination];
        Edge e1;
        bool found;
        tie(e1, found) = edge(destination, pred, *graph_ptr);

        if (pred != destination)
        {
            while (pred != source)
            {
                int k = index[pred];

                path.insert(path.begin(), k);
                tie(e1, found) = edge(p[pred], pred, *graph_ptr);
                pred = p[pred];
            }
        }
    }
    else
    {
        std::cout << "Invalid source and destination nodes" << std::endl;
    }
}

void RobotTrajectorySaver::visualize(visualization_msgs::Marker& line_list)
{

    std::cout << "Visualize: num of vertexes found " << num_vertices(*graph_ptr) << std::endl;

    VertexPositions v_positions = get(vertex_properties, *graph_ptr);

    std::pair<edge_iterator, edge_iterator> ei = edges(*graph_ptr);
    property_map<Graph, edge_weight_t>::type weight = get(edge_weight, *graph_ptr);
    double max_wt = -100, wt;
    double min_wt = 100;

    for (edge_iterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
    {
        wt = get(weight, *edge_iter);
        if (wt > max_wt)
        {
            max_wt = wt;
        }
        if (wt < min_wt)
        {
            min_wt = wt;
        }
    }

    for (edge_iterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
    {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        Vertex s = source(*edge_iter, *graph_ptr);
        Vertex t = target(*edge_iter, *graph_ptr);

        Point3D p_s = v_positions[s];
        Point3D p_t = v_positions[t];

        p1.x = p_s.x;
        p1.y = p_s.y;
        p1.z = p_s.z;

        //std::cout << "(" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;

        p2.x = p_t.x;
        p2.y = p_t.y;
        p2.z = p_t.z;
        //std::cout << "(" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        property_map<Graph, edge_weight_t>::type weight = get(edge_weight, *graph_ptr);
        wt = get(weight, *edge_iter);
        std_msgs::ColorRGBA colorA;
        if (wt > 100)
        {
            colorA.g = 0.0;
            colorA.b = 0.0;
            colorA.r = 1;
            line_list.colors.push_back(colorA);
            line_list.colors.push_back(colorA);
        }
        else
        {
            colorA.g = 1 / (wt - min_wt) / (max_wt - min_wt);
            colorA.b = 0.0;
            colorA.r = (wt - min_wt) / (max_wt - min_wt);
            colorA.a = 1;
            line_list.colors.push_back(colorA);
            line_list.colors.push_back(colorA);
        }

    }
    // OK write_graphviz(std::cout, *graph_ptr, write_vertex_position<Graph>(*graph_ptr), write_edge_weight<Graph>(*graph_ptr));

}

void RobotTrajectorySaver::initialize(double radius, int total_nr_points, pcl::PointCloud<pcl::PointXYZ>* point_cloud_in_total, pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree_total)
{

    VertexPositions v_positions = get(vertex_properties, *graph_ptr);

    std::vector<int> pointIdxRadiusSearch;

    std::vector<float> pointRadiusSquaredDistance;

    int i = 0;
    int j = 0;

    while (i < total_nr_points)
    {

        pcl::PointXYZ searchPoint = point_cloud_in_total->points[i];

        int n_points = kdtree_total->radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        if (n_points > 1)
        { // The first point is the searchPoint itself

            for (unsigned int k = 1; k < pointIdxRadiusSearch.size(); k++)
            {

                j = pointIdxRadiusSearch[k];

                geometry_msgs::Point tail;
                tail.x = point_cloud_in_total->points[j].x;
                tail.y = point_cloud_in_total->points[j].y;
                tail.z = point_cloud_in_total->points[j].z;


                double scaled_distance = sqrt(pow(tail.x - searchPoint.x, 2) + pow(tail.y - searchPoint.y, 2) + scaled_factor * pow(tail.z - searchPoint.z, 2));
                //ROS_INFO("SCLAED DISTANCE: %f - RADIUS: %f",scaled_distance,radius);
                if (scaled_distance < radius)
                {

                    Edge e = add_edge(i, j, scaled_distance, *graph_ptr).first;

                    Vertex s = source(e, *graph_ptr);
                    Point3D p_s;
                    p_s.x = searchPoint.x;
                    p_s.y = searchPoint.y;
                    p_s.z = searchPoint.z;
                    v_positions[s] = p_s;

                    Vertex t = target(e, *graph_ptr);
                    Point3D p_t;
                    p_t.x = point_cloud_in_total->points[j].x;
                    p_t.y = point_cloud_in_total->points[j].y;
                    p_t.z = point_cloud_in_total->points[j].z;
                    v_positions[t] = p_t;

                }
            }
        }

        i++;

    }

}

void RobotTrajectorySaver::linkConnectedComponents(double min, double incr, double rmax, int total_nr_points, pcl::PointCloud<pcl::PointXYZ>* point_cloud_in_total, pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree_total)
{
    double count = min;
    VertexPositions v_positions = get(vertex_properties, *graph_ptr);

    while (count < rmax)
    {
        count += incr;

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std::vector<int> component(num_vertices(*graph_ptr));
        int num = connected_components(*graph_ptr, &component[0]);

        int i = 0;
        int j = 0;

        while (i < total_nr_points)
        {

            pcl::PointXYZ searchPoint = point_cloud_in_total->points[i];

            int n_points = kdtree_total->radiusSearch(searchPoint, count, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            if (n_points > 0)
            {

                for (unsigned int k = 0; k < pointIdxRadiusSearch.size(); k++)
                {
                    j = pointIdxRadiusSearch[k];

                    if (component[i] != component[j])
                    {

                        geometry_msgs::Point tail;
                        tail.x = point_cloud_in_total->points[j].x;
                        tail.y = point_cloud_in_total->points[j].y;
                        tail.z = point_cloud_in_total->points[j].z;


                        double scaled_distance = sqrt(pow(tail.x - searchPoint.x, 2) + pow(tail.y - searchPoint.y, 2) + pow(scaled_factor * tail.z - searchPoint.z, 2));
                        //ROS_INFO("SCLAED DISTANCE: %f - RADIUS: %f",scaled_distance,rmax);
                        if (scaled_distance < count)
                        {

                            Edge e = add_edge(i, j, scaled_distance, *graph_ptr).first;

                            Vertex s = source(e, *graph_ptr);
                            Point3D p_s;
                            p_s.x = searchPoint.x;
                            p_s.y = searchPoint.y;
                            p_s.z = searchPoint.z;
                            v_positions[s] = p_s;

                            Vertex t = target(e, *graph_ptr);
                            Point3D p_t;
                            p_t.x = point_cloud_in_total->points[j].x;
                            p_t.y = point_cloud_in_total->points[j].y;
                            p_t.z = point_cloud_in_total->points[j].z;
                            v_positions[t] = p_t;

                        }
                    }
                }
            }

            i++;

        }

    }
}

void RobotTrajectorySaver::findClosestVertex(int n_point_of_interest, geometry_msgs::Point s, int& i, Point3D& p, double& d)
{

    double min_dist = MAX_DISTANCE;

    property_map<Graph, vertex_index_t>::type index = get(vertex_index, *graph_ptr);
    VertexPositions v_positions = get(vertex_properties, *graph_ptr);

    for (int i = 0; i < n_point_of_interest; i++)
    {

        std::pair<vertex_iterator, vertex_iterator> vp;
        for (vp = vertices(*graph_ptr); vp.first != vp.second; ++vp.first)
        {

            //Vertex v = *vp.first;
            //Point3D p = v_positions[v];
            double dist = sqrt(pow(s.x - v_positions[*vp.first].x, 2) + pow(s.y - v_positions[*vp.first].y, 2) + pow(s.z - v_positions[*vp.first].z, 2));
            if (dist < min_dist)
            {
                min_dist = dist;
                d = min_dist;
                i = index[*vp.first];
                p = v_positions[*vp.first];
            }

        }
    }

}

bool RobotTrajectorySaver::checkPath(robot_trajectory_saver_msgs::CheckPath::Request& request, robot_trajectory_saver_msgs::CheckPath::Response& response)
{

    /* create a new graph  starting from the graph of the trajectories and augmenting them with new nodes and edges */
    /* nodes are the point of interest in the req message */
    Graph* augmented_graph_ptr = graph_ptr;
    int n_nodes = num_vertices(*graph_ptr);

    VertexPositions v_positions = get(vertex_properties, *graph_ptr);

    int n_point_of_interest = request.point_of_interest.poses.size();

    for (int i = 0; i < n_point_of_interest; i++)
    {

        int j;
        Point3D p_t;
        double weight;
        findClosestVertex(n_point_of_interest, request.point_of_interest.poses[i].pose.position, j, p_t, weight);
        int k = n_nodes + i;
        Edge e = add_edge(k, j, weight, *augmented_graph_ptr).first;

        Vertex s = source(e, *augmented_graph_ptr);
        Point3D p_s;
        p_s.x = request.point_of_interest.poses[i].pose.position.x;
        p_s.y = request.point_of_interest.poses[i].pose.position.y;
        p_s.z = request.point_of_interest.poses[i].pose.position.z;
        v_positions[s] = p_s;

        Vertex t = target(e, *graph_ptr);
        v_positions[t] = p_t;
    }
    std::vector<int> node_id_path;
    computeShortestPath(request.source_node_id, request.destination_node_id, node_id_path);

    if (node_id_path.size() > 0)
    {
        response.path_cost = node_id_path.size();
        response.result = true;
    }
    else
    {
        response.path_cost = 0;
        response.result = false;
    }

    return true;
}

bool RobotTrajectorySaver::getRobotTrajectoriesCallback(robot_trajectory_saver_msgs::GetRobotTrajectories::Request& request, robot_trajectory_saver_msgs::GetRobotTrajectories::Response& response)
{

    std::cout << "Service callback called: getRobotTrajectoriesCallback " << std::endl;
    pcl::PointCloud<pcl::PointXYZ> *cloud_in;
    pcl::PointCloud<pcl::PointXYZ>* cloud_in_filtered;

    cloud_in = new pcl::PointCloud<pcl::PointXYZ>();
    cloud_in_filtered = new pcl::PointCloud<pcl::PointXYZ>();

    //if (read_file) {
    for (int i = 0; i < cloud_in_init_graph->points.size(); i++)
    {
        cloud_in->points.push_back(cloud_in_init_graph->points[i]);
    }
    //    read_file = false;
    //}

    pthread_mutex_lock(&lock_robot_trajectories);

    for (int i = 0; i < num_robots; i++)
    {
        for (int j = 0; j < ugv_trajectories[i].global_robot_poses.size(); j++)
        {
            pcl::PointXYZ q;
            q.x = ugv_trajectories[i].global_robot_poses[j].getOrigin().getX();
            q.y = ugv_trajectories[i].global_robot_poses[j].getOrigin().getY();
            q.z = ugv_trajectories[i].global_robot_poses[j].getOrigin().getZ();
            cloud_in->points.push_back(q);
        }
    }

    pthread_mutex_unlock(&lock_robot_trajectories);

#if 0
    if (read_file)
    {
        property_map<Graph, vertex_index_t>::type index = get(vertex_index, *graph_ptr);
        VertexPositions v_positions = get(vertex_properties, *graph_ptr);
        std::cout << "Num of vertices of the graph red from file: " << num_vertices(*graph_ptr) << std::endl;
        std::pair<vertex_iterator, vertex_iterator> vp;
        for (vp = vertices(*graph_ptr); vp.first != vp.second; ++vp.first)
        {

            Vertex v = *vp.first;
            Point3D p = v_positions[v];
            pcl::PointXYZ q;
            q.x = p.x;
            q.y = p.y;
            q.z = p.z;
            cloud_in->points.push_back(q);

        }

    }
#endif

    pcl::VoxelGrid<pcl::PointXYZ> *sor;
    sor = new pcl::VoxelGrid<pcl::PointXYZ>();
    sor->setInputCloud(cloud_in->makeShared());
    sor->setLeafSize(leaf_size, leaf_size, leaf_size);
    sor->filter(*cloud_in_filtered);

    pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree;
    kdtree = new pcl::KdTreeFLANN<pcl::PointXYZ>();

    kdtree->setInputCloud(cloud_in_filtered->makeShared());
    int total_nr_points = cloud_in_filtered->points.size();

    if (graph_ptr) delete graph_ptr;
    graph_ptr = new Graph(total_nr_points);

#if 1
    initialize(min_radius, total_nr_points, cloud_in_filtered, kdtree);
    linkConnectedComponents(min_radius, radius_incr, max_radius, total_nr_points, cloud_in_filtered, kdtree);
#endif

    line_list.header.stamp = ros::Time::now();
    line_list.points.clear();
    line_list.colors.clear();
    visualize(line_list);
    trajectories_marker_pub.publish(line_list);

    fromBoostGraphToNavMsgPath(response.trajectories);

    return true;

}

bool RobotTrajectorySaver::loadRobotTrajectoriesCallback(robot_trajectory_saver_msgs::LoadRobotTrajectories::Request& request, robot_trajectory_saver_msgs::LoadRobotTrajectories::Response& response)
{

    Graph* graph_ptr = new Graph(0);
    VertexPositions v_positions = get(vertex_properties, *graph_ptr);

    std::vector<int> indexes;
    std::vector<Point3D> coordinates;

    nav_msgs::Path path;
    path.header.frame_id = global_frame;
    path.header.stamp = ros::Time::now();

    std::ifstream graph_filename;
    graph_filename.open(request.file_path.c_str());

    std::string line;
    while (std::getline(graph_filename, line))
    {

        int id;
        double x, y, z;
        // Nodes are under the form 163 [x = 7.59997, y = 2.56912, z = 2.39076];
        if (sscanf(line.c_str(), "%d %*s = %lf, %*s = %lf, %*s = %lf %*s", &id, &x, &y, &z) == 4)
        {
            //std::cout << "Node ID " << id << " x = " << x << " y = " << y << " z = " << z << "\n";
            indexes.push_back(id);
            Point3D p;
            p.x = x;
            p.y = y;
            p.z = z;
            coordinates.push_back(p);

            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = global_frame;
            ps.header.stamp = path.header.stamp;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.position.z = p.z;
            ps.pose.orientation.w = 1;
            path.poses.push_back(ps);
        }

        int i, j;
        double w;
        // Edges are under the form 8--18  [weight = "0.718538"];
        if (sscanf(line.c_str(), "%d--%d %*s = \"%lf\" %*s", &i, &j, &w) == 3)
        {
            //std::cout << "Edge=["<< i << "," << j << "] weight="<< w <<"\n"; 

            Edge e = add_edge(i, j, w, *graph_ptr).first;
            Vertex s = source(e, *graph_ptr);

            int i_source, i_target;
            for (int k = 0; k < indexes.size(); k++)
            {
                if (indexes[k] == i)
                {
                    i_source = k;
                }
                if (indexes[k] == j)
                {
                    i_target = k;
                }
            }

            v_positions[s] = coordinates[i_source];

            Vertex t = target(e, *graph_ptr);
            v_positions[t] = coordinates[i_target];



        }
    }
    graph_filename.close();
    response.trajectories = path;
    return true;
}

bool RobotTrajectorySaver::saveRobotTrajectoriesCallback(robot_trajectory_saver_msgs::SaveRobotTrajectories::Request& request, robot_trajectory_saver_msgs::SaveRobotTrajectories::Response& response)
{

    pcl::PointCloud<pcl::PointXYZ> *cloud_in;
    pcl::PointCloud<pcl::PointXYZ>* cloud_in_filtered;

    cloud_in = new pcl::PointCloud<pcl::PointXYZ>();
    cloud_in_filtered = new pcl::PointCloud<pcl::PointXYZ>();

    //    if (read_file) {
    for (int i = 0; i < cloud_in_init_graph->points.size(); i++)
    {
        cloud_in->points.push_back(cloud_in_init_graph->points[i]);
    }
    //       read_file = false;
    //   }


    pthread_mutex_lock(&lock_robot_trajectories);

    for (int i = 0; i < num_robots; i++)
    {
        for (int j = 0; j < ugv_trajectories[i].global_robot_poses.size(); j++)
        {
            pcl::PointXYZ q;
            q.x = ugv_trajectories[i].global_robot_poses[j].getOrigin().getX();
            q.y = ugv_trajectories[i].global_robot_poses[j].getOrigin().getY();
            q.z = ugv_trajectories[i].global_robot_poses[j].getOrigin().getZ();
            cloud_in->points.push_back(q);
        }
    }

    pthread_mutex_unlock(&lock_robot_trajectories);

    pcl::VoxelGrid<pcl::PointXYZ> *sor;
    sor = new pcl::VoxelGrid<pcl::PointXYZ>();
    sor->setInputCloud(cloud_in->makeShared());
    sor->setLeafSize(leaf_size, leaf_size, leaf_size);
    sor->filter(*cloud_in_filtered);

    pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree;
    kdtree = new pcl::KdTreeFLANN<pcl::PointXYZ>();

    kdtree->setInputCloud(cloud_in_filtered->makeShared());
    int total_nr_points = cloud_in_filtered->points.size();

    if (graph_ptr) delete graph_ptr;
    graph_ptr = new Graph(total_nr_points);

    initialize(min_radius, total_nr_points, cloud_in_filtered, kdtree);
    linkConnectedComponents(min_radius, radius_incr, max_radius, total_nr_points, cloud_in_filtered, kdtree);

    line_list.header.stamp = ros::Time::now();
    line_list.points.clear();
    line_list.colors.clear();
    visualize(line_list);
    trajectories_marker_pub.publish(line_list);

    std::ofstream dotfile(request.file_path.c_str());
    write_graphviz(dotfile, *graph_ptr, WriteVertexPosition<Graph>(*graph_ptr), WriteEdgeWeight<Graph>(*graph_ptr));
    return true;

}

void RobotTrajectorySaver::navMsgToStampedTransform(int robot_id, const nav_msgs::PathConstPtr& msg)
{
    for (int i = 0; msg->poses.size(); i++)
    {
        tf::StampedTransform robot_pose_map;
        tf::Vector3 v;
        v.setX(msg->poses[i].pose.position.x);
        v.setY(msg->poses[i].pose.position.y);
        v.setZ(msg->poses[i].pose.position.z);
        robot_pose_map.setOrigin(v);


        tf::Quaternion q;
        q.setX(msg->poses[i].pose.orientation.x);
        q.setY(msg->poses[i].pose.orientation.y);
        q.setZ(msg->poses[i].pose.orientation.z);
        q.setW(msg->poses[i].pose.orientation.w);
        robot_pose_map.setRotation(q);

        ugv_trajectories[robot_id].global_robot_poses.push_back(robot_pose_map);
    }
}

void RobotTrajectorySaver::odomMsgToStampedTransform(nav_msgs::Odometry pose_odometry, tf::StampedTransform& pose_stamped)
{
    pose_stamped.stamp_ = pose_odometry.header.stamp;
    pose_stamped.frame_id_ = pose_odometry.header.frame_id;
    pose_stamped.child_frame_id_ = pose_odometry.child_frame_id;

    tf::Vector3 v;
    v.setX(pose_odometry.pose.pose.position.x);
    v.setY(pose_odometry.pose.pose.position.y);
    v.setZ(pose_odometry.pose.pose.position.z);

    pose_stamped.setOrigin(v);

    tf::Quaternion q;
    q.setX(pose_odometry.pose.pose.orientation.x);
    q.setY(pose_odometry.pose.pose.orientation.y);
    q.setZ(pose_odometry.pose.pose.orientation.z);
    q.setW(pose_odometry.pose.pose.orientation.w);

    pose_stamped.setRotation(q);
}

void RobotTrajectorySaver::trajFromLaserMapperUgv1(const nav_msgs::PathConstPtr& msg)
{
    navMsgToStampedTransform(0, msg);
}

void RobotTrajectorySaver::trajFromLaserMapperUgv2(const nav_msgs::PathConstPtr& msg)
{
    navMsgToStampedTransform(1, msg);
}

void RobotTrajectorySaver::imuOdomCallbackUgv1(const nav_msgs::OdometryConstPtr& msg)
{
    tf::StampedTransform current_real_robot_pose_odom;
    odomMsgToStampedTransform(*msg, current_real_robot_pose_odom);
    tf::StampedTransform robot_pose_map;

    if (tf_.waitForTransform(global_frame, ugv_trajectories[0].odom_frame_, msg->header.stamp, ros::Duration(0.05)))
    {
        try
        {

            tf::StampedTransform from_odom_to_map;
            tf_.lookupTransform(global_frame, ugv_trajectories[0].robot_frame_, msg->header.stamp, robot_pose_map);
            tf_.lookupTransform(global_frame, ugv_trajectories[0].odom_frame_, msg->header.stamp, from_odom_to_map);
            ugv_trajectories[0].global_robot_poses.push_back(robot_pose_map);
            ugv_trajectories[0].from_odoms_to_map.push_back(from_odom_to_map);
        }
        catch (tf::LookupException& ex)
        {
            ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ExtrapolationException& ex)
        {
            //ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
        }
    }
    else
    {

        int last = ugv_trajectories[0].from_odoms_to_map.size() - 1;
        tf::Transform transformation = ugv_trajectories[0].from_odoms_to_map[last] * current_real_robot_pose_odom;
        robot_pose_map.stamp_ = current_real_robot_pose_odom.stamp_;
        robot_pose_map.setBasis(transformation.getBasis());
        robot_pose_map.setOrigin(transformation.getOrigin());
        robot_pose_map.setRotation(transformation.getRotation());
        ugv_trajectories[0].global_robot_poses.push_back(robot_pose_map);
    }
}

void RobotTrajectorySaver::imuOdomCallbackUgv2(const nav_msgs::OdometryConstPtr& msg)
{
    tf::StampedTransform current_real_robot_pose_odom;
    odomMsgToStampedTransform(*msg, current_real_robot_pose_odom);
    tf::StampedTransform robot_pose_map;

    if (tf_.waitForTransform(global_frame, ugv_trajectories[1].odom_frame_, msg->header.stamp, ros::Duration(0.05)))
    {
        try
        {

            tf::StampedTransform from_odom_to_map;
            tf_.lookupTransform(global_frame, ugv_trajectories[1].robot_frame_, msg->header.stamp, robot_pose_map);
            tf_.lookupTransform(global_frame, ugv_trajectories[1].odom_frame_, msg->header.stamp, from_odom_to_map);
            ugv_trajectories[1].global_robot_poses.push_back(robot_pose_map);
            ugv_trajectories[1].from_odoms_to_map.push_back(from_odom_to_map);
        }
        catch (tf::LookupException& ex)
        {
            ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
        }
        catch (tf::ExtrapolationException& ex)
        {
            //ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
        }
    }
    else
    {

        int last = ugv_trajectories[1].from_odoms_to_map.size() - 1;
        tf::Transform transformation = ugv_trajectories[0].from_odoms_to_map[last] * current_real_robot_pose_odom;
        robot_pose_map.stamp_ = current_real_robot_pose_odom.stamp_;
        robot_pose_map.setBasis(transformation.getBasis());
        robot_pose_map.setOrigin(transformation.getOrigin());
        robot_pose_map.setRotation(transformation.getRotation());
        ugv_trajectories[1].global_robot_poses.push_back(robot_pose_map);
    }
}

void RobotTrajectorySaver::initTrajectoriesFromExistingDotFile()
{

    if (cloud_in_init_graph) delete cloud_in_init_graph;
    cloud_in_init_graph = new pcl::PointCloud<pcl::PointXYZ>();

    graph_ptr = new Graph(0);

    VertexPositions v_positions = get(vertex_properties, *graph_ptr);

    std::vector<int> indexes;
    std::vector<Point3D> coordinates;

    std::ifstream graph_filename;

    if (exist_file(map_filename))
    {

        graph_filename.open(map_filename.c_str());

        if (!empty_file(graph_filename))
        {

            std::string line;
            while (std::getline(graph_filename, line))
            {

                int id;
                double x, y, z;
                // Nodes are under the form 163 [x = 7.59997, y = 2.56912, z = 2.39076];
                if (sscanf(line.c_str(), "%d %*s = %lf, %*s = %lf, %*s = %lf %*s", &id, &x, &y, &z) == 4)
                {
                    //std::cout << "Node ID " << id << " x = " << x << " y = " << y << " z = " << z << "\n";
                    indexes.push_back(id);
                    Point3D p;
                    p.x = x;
                    p.y = y;
                    p.z = z;
                    coordinates.push_back(p);
                    pcl::PointXYZ q;
                    q.x = p.x;
                    q.y = p.y;
                    q.z = p.z;
                    cloud_in_init_graph->points.push_back(q);
                }

                int i, j;
                double w;
                // Edges are under the form 8--18  [weight = "0.718538"];
                if (sscanf(line.c_str(), "%d--%d %*s = \"%lf\" %*s", &i, &j, &w) == 3)
                {
                    //std::cout << "Edge=[" << i << "," << j << "] weight=" << w << "\n";

                    Edge e = add_edge(i, j, w, *graph_ptr).first;
                    Vertex s = source(e, *graph_ptr);

                    int i_source, i_target;
                    for (int k = 0; k < indexes.size(); k++)
                    {
                        if (indexes[k] == i)
                        {
                            i_source = k;
                        }
                        if (indexes[k] == j)
                        {
                            i_target = k;
                        }
                    }

                    v_positions[s] = coordinates[i_source];

                    Vertex t = target(e, *graph_ptr);
                    v_positions[t] = coordinates[i_target];

                }
            }
            graph_filename.close();

            line_list.header.stamp = ros::Time::now();
            line_list.points.clear();
            line_list.colors.clear();
            visualize(line_list);
            trajectories_marker_pub.publish(line_list);
            //write_graphviz(std::cout, *graph_ptr, write_vertex_position<Graph>(*graph_ptr), write_edge_weight<Graph>(*graph_ptr));
            //read_file = true;
        }
        else
        {
            std::cout << "Trajectory Graph can not initialized. File is empty!" << std::endl;
            if (cloud_in_init_graph) delete cloud_in_init_graph;
            cloud_in_init_graph = new pcl::PointCloud<pcl::PointXYZ>();
            cloud_in_init_graph->points.resize(0);
        }

    }
    else
    {
        std::cout << "Trajectory Graph can not initialized. File does not exists!" << std::endl;
        if (cloud_in_init_graph) delete cloud_in_init_graph;
        cloud_in_init_graph = new pcl::PointCloud<pcl::PointXYZ>();
        cloud_in_init_graph->points.resize(0);
    }
}

void RobotTrajectorySaver::initRobotTrajectories()
{
    for (int i = 0; i < num_robots; i++)
    {
        while (!initRobotTrajectory(i))
        {
            ROS_INFO("Waiting for transformation");
        }
    }
}

bool RobotTrajectorySaver::initRobotTrajectory(int i)
{

    if (tf_.waitForTransform(global_frame, ugv_trajectories[i].robot_frame_, ros::Time(), ros::Duration(1.0)))
    {
        try
        {
            tf::StampedTransform robot_pose_map;
            tf::StampedTransform robot_pose_odom;
            tf::StampedTransform from_odom_to_map;
            tf_.lookupTransform(global_frame, ugv_trajectories[i].robot_frame_, ros::Time(), robot_pose_map);
            tf_.lookupTransform(ugv_trajectories[i].odom_frame_, ugv_trajectories[i].robot_frame_, ros::Time(), robot_pose_odom);
            tf_.lookupTransform(global_frame, ugv_trajectories[i].odom_frame_, ros::Time(), from_odom_to_map);
            
            ugv_trajectories[i].global_robot_poses.push_back(robot_pose_map);
            ugv_trajectories[i].odom_robot_poses.push_back(robot_pose_odom);
            ugv_trajectories[i].from_odoms_to_map.push_back(from_odom_to_map);

        }
        catch (tf::LookupException& ex)
        {
            ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_INFO("Transformation is not available");
        return false;
    }
}

void RobotTrajectorySaver::updateRobotPoses()
{
    for (int i = 0; i < num_robots; i++)
    {
        while (!updateRobotPose(i))
        {
            ROS_INFO("Waiting for updating transformation");
        }
    }
}

bool RobotTrajectorySaver::updateRobotPose(int i)
{

    if (tf_.waitForTransform(global_frame, ugv_trajectories[i].robot_frame_, ros::Time(), ros::Duration(1.0)))
    {
        try
        {
            tf::StampedTransform robot_pose_map;
            tf::StampedTransform robot_pose_odom;
            tf::StampedTransform from_odom_to_map;
            tf_.lookupTransform(global_frame, ugv_trajectories[i].robot_frame_, ros::Time(), robot_pose_map);
            tf_.lookupTransform(ugv_trajectories[i].odom_frame_, ugv_trajectories[i].robot_frame_, ros::Time(), robot_pose_odom);
            tf_.lookupTransform(global_frame, ugv_trajectories[i].odom_frame_, ros::Time(), from_odom_to_map);
            ugv_trajectories[i].global_robot_poses.push_back(robot_pose_map);
            ugv_trajectories[i].odom_robot_poses.push_back(robot_pose_odom);
            ugv_trajectories[i].from_odoms_to_map.push_back(from_odom_to_map);

        }
        catch (tf::LookupException& ex)
        {
            ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }
    else
    {
        ROS_INFO("Transformation is not available");
        return false;
    }
}

void RobotTrajectorySaver::ros_spin()
{
    ros::Rate rate(frequency);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

RobotTrajectorySaver::RobotTrajectorySaver() : n_("~"), tf_(), graph_ptr(0L), cloud_in_init_graph(0L)
{

    read_file = false;

    run_in_simulation = getParam<bool>(n_, "run_in_simulation", true);
    run_laser_mapper = getParam<bool>(n_, "run_laser_mapper", false);

    num_robots = getParam<int>(n_, "num_robots", 2);
    odom_frame = getParam<std::string>(n_, "odom_frame", "odom");
    robot_name_prefix = getParam<std::string>(n_, "robot_name_prefix", "ugv");
    robot_frame = getParam<std::string>(n_, "robot_frame", "base_link");
    global_frame = getParam<std::string>(n_, "global_frame", "map");
    imu_odom_topic = getParam<std::string>(n_, "imu_odom_topic", "odom");
    traj_ugv_topic_name_from_laser_mapper = getParam<std::string>(n_, "traj_ugv_topic_name_from_laser_mapper", "/laser_mapper/trajectory");

    map_filename = getParam<std::string>(n_, "map", "vrep3D.dot");

    ugv_trajectories.clear();
    for (int i = 0; i < num_robots; i++)
    {

        RobotTrajectory trajectory;

        std::stringstream ss;
        ss << i + 1;

        std::string temp;
        std::string temp2;
        std::string temp3;
        std::string temp4;

        if (run_in_simulation)
        {
            temp3 = "/vrep";
            temp4 = "/vrep";
        }
        else
        {
            temp3 = "";
            temp4 = "";
        }

        if (robot_name_prefix[0] == '/')
        {
            temp = robot_name_prefix + ss.str();
            temp2 = robot_name_prefix + ss.str();
            temp3 = temp3 + robot_name_prefix + ss.str();
            temp4 = temp4 + robot_name_prefix + ss.str();
        }
        else
        {
            temp = "/" + robot_name_prefix + ss.str();
            temp2 = "/" + robot_name_prefix + ss.str();
            temp3 = temp3 + "/" + robot_name_prefix + ss.str();
            temp4 = temp4 + "/" + robot_name_prefix + ss.str();
        }

        if (robot_frame[0] == '/')
        {
            temp = temp + robot_frame;
        }
        else
        {
            temp = temp + "/" + robot_frame;
        }

        trajectory.robot_frame_ = temp;
        std::cout << "ROBOT_ID " << i << " robot_frame " << trajectory.robot_frame_ << std::endl;

        if (odom_frame[0] == '/')
        {
            temp2 = temp2 + odom_frame;
        }
        else
        {
            temp2 = temp2 + "/" + odom_frame;
        }

        trajectory.odom_frame_ = temp2;
        std::cout << "ROBOT_ID " << i << " odom_frame " << trajectory.odom_frame_ << std::endl;

        if (imu_odom_topic[0] == '/')
        {
            temp3 = temp3 + imu_odom_topic;
        }
        else
        {
            temp3 = temp3 + "/" + imu_odom_topic;
        }

        if (traj_ugv_topic_name_from_laser_mapper[0] = '/')
        {
            temp4 = temp4 + traj_ugv_topic_name_from_laser_mapper;
        }
        else
        {
            temp4 = temp4 + "/" + traj_ugv_topic_name_from_laser_mapper;
        }



        trajectory.imu_odom_topic_ = temp3;
        trajectory.traj_laser_mapper_topic_ = temp4;

        std::cout << "ROBOT_ID " << i << " imu_odom_topic " << trajectory.imu_odom_topic_ << std::endl;
        std::cout << "ROBOT_ID " << i << " traj_ugv_topic_name_from_laser_mapper " << trajectory.traj_laser_mapper_topic_ << std::endl;

        ugv_trajectories.push_back(trajectory);
    }

    frequency = getParam<double>(n_, "frequency", 1);

    leaf_size = getParam<double>(n_, "leaf_size", 1);

    min_radius = getParam<double>(n_, "min_radius", 1);
    max_radius = getParam<double>(n_, "max_radius", 3.5);
    radius_incr = getParam<double>(n_, "radius_incr", 0.25);
    scaled_factor = getParam<double>(n_, "scaled_factor", 4. / 3.);

    trajectories_marker_topic = getParam<std::string>(n_, "trajectories_marker_topic", "robot_trajectories_marker");
    trajectories_marker_pub = node.advertise<visualization_msgs::Marker>(trajectories_marker_topic, 10);

    if (!run_laser_mapper)
    {
        ugv1_imu_odom_sub = node.subscribe(ugv_trajectories[0].imu_odom_topic_, kSubMessageQueueSize, &RobotTrajectorySaver::imuOdomCallbackUgv1, this);
        ugv2_imu_odom_sub = node.subscribe(ugv_trajectories[1].imu_odom_topic_, kSubMessageQueueSize, &RobotTrajectorySaver::imuOdomCallbackUgv2, this);
    }
    else
    {
        ugv1_traj_ugv_topic_name_from_laser_mapper_sub = node.subscribe(ugv_trajectories[0].traj_laser_mapper_topic_, kSubMessageQueueSize, &RobotTrajectorySaver::trajFromLaserMapperUgv1, this);
        ugv2_traj_ugv_topic_name_from_laser_mapper_sub = node.subscribe(ugv_trajectories[1].traj_laser_mapper_topic_, kSubMessageQueueSize, &RobotTrajectorySaver::trajFromLaserMapperUgv2, this);
    }
    save_robot_trajectories_service_name = getParam<std::string>(n_, "save_robot_trajectories_service_name", "save_robot_trajectories_only");
    save_robot_trajectories = n_.advertiseService(save_robot_trajectories_service_name, &RobotTrajectorySaver::saveRobotTrajectoriesCallback, this);

    load_robot_trajectories_service_name = getParam<std::string>(n_, "load_robot_trajectories_service_name", "load_robot_trajectories_only");
    load_robot_trajectories = n_.advertiseService(load_robot_trajectories_service_name, &RobotTrajectorySaver::loadRobotTrajectoriesCallback, this);

    get_robot_trajectories_service_name = getParam<std::string>(n_, "get_robot_trajectories_service_name", "get_robot_trajectories_nav_msgs");
    get_robot_trajectories = n_.advertiseService(get_robot_trajectories_service_name, &RobotTrajectorySaver::getRobotTrajectoriesCallback, this);

    check_path_service_name = getParam<std::string>(n_, "check_path_service_name", "check_path");
    check_path = n_.advertiseService(check_path_service_name, &RobotTrajectorySaver::checkPath, this);

    pthread_mutex_init(&lock_robot_trajectories, NULL);


    initRobotTrajectories();

    line_list.header.frame_id = global_frame;
    line_list.ns = "robot_trajectories";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.03;
    // Line list is red
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_list.points.clear();
    line_list.colors.clear();

    initTrajectoriesFromExistingDotFile();
}

RobotTrajectorySaver::~RobotTrajectorySaver()
{
}

/*
 * 
 */
int main(int argc, char** argv)
{

    ros::init(argc, argv, "robot_trajectory_saver");
    RobotTrajectorySaver robot_trajectory_saver;
    robot_trajectory_saver.ros_spin();

    return 0;
}

