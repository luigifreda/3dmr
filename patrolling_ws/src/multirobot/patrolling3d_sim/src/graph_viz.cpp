/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DMR. 
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



#include "graph_viz.h"

const double GraphViz::kMaxIdlenessForDrawing  = 100;
const double GraphViz::kMaxIdlenessRadius = 0.5;
const double GraphViz::kMarkerTextHeight = 0.15;

GraphViz::GraphViz(): node_("~")
{
    file_ = NULL;
    listener_ = NULL;
    time_zero_ = ros::Time::now();
    graph_filename_ = getParam<std::string>(node_, "map", "vrep");
    
    nodes_topic_name_ = getParam<std::string>(node_, "nodes_topic_name", "/patrolling_nodes_markers");
    nodes_topic_pub_ = node_.advertise<visualization_msgs::MarkerArray>(nodes_topic_name_, 10);
    
    edges_topic_name_ = getParam<std::string>(node_, "edges_topic_name", "/edges_markers");
    edges_topic_pub_ = node_.advertise<visualization_msgs::Marker>(edges_topic_name_, 10);
    
    num_robots_ = getParam<int>(node_, "num_robots", 2);
    global_frame_ = getParam<std::string>(node_, "global_frame", "map");
    robot_frame_ = getParam<std::string>(node_, "robot_frame", "base_link");
    safety_radius_ = getParam<double>(node_, "radius", 1);
    boundaries_topic_name_ = getParam<std::string>(node_, "boundaries_topic_name", "/patrolling_robot_boudaries_markers");
    boundaries_topic_pub_ = node_.advertise<visualization_msgs::MarkerArray>(boundaries_topic_name_, 10);
    listener_ = new tf::TransformListener();
    
    init();
    
    buildPatrollingNodesAsMarkers();
    buildPatrollingEdgesAsMarkers();
}

GraphViz::GraphViz(std::string& gf, std::string& ntn, int n, std::string& glf, std::string& rf, double r, std::string& btn): node_("~")
{
    file_ = NULL;
    listener_ = NULL;
    time_zero_ = ros::Time::now();
    graph_filename_ = gf;
    nodes_topic_name_ = ntn;
    nodes_topic_pub_ = node_.advertise<visualization_msgs::MarkerArray>(nodes_topic_name_, 10);
    num_robots_ = n;
    global_frame_ = glf;
    robot_frame_ = rf;
    safety_radius_ = r;
    boundaries_topic_name_ = btn;
    boundaries_topic_pub_ = node_.advertise<visualization_msgs::MarkerArray>(boundaries_topic_name_, 10);
    listener_ = new tf::TransformListener();
    
    edges_topic_name_ = getParam<std::string>(node_, "edges_topic_name", "/edges_markers");
    edges_topic_pub_ = node_.advertise<visualization_msgs::Marker>(edges_topic_name_, 10);
    
    init();
    
    buildPatrollingNodesAsMarkers();
    buildPatrollingEdgesAsMarkers();
}


GraphViz::GraphViz(Vertex* vertex_web, int graph_dimension, std::string& ntn, int n, std::string& glf, std::string& rf, double r, std::string& btn) : node_("~")
{
    file_ = NULL;
    listener_ = NULL;
    time_zero_ = ros::Time::now();
    graph_filename_ = "";
    nodes_topic_name_ = ntn;
    nodes_topic_pub_ = node_.advertise<visualization_msgs::MarkerArray>(nodes_topic_name_, 10);
    num_robots_ = n;
    global_frame_ = glf;
    robot_frame_ = rf;
    safety_radius_ = r;
    boundaries_topic_name_ = btn;
    boundaries_topic_pub_ = node_.advertise<visualization_msgs::MarkerArray>(boundaries_topic_name_, 10);
    listener_ = new tf::TransformListener();
    
    edges_topic_name_ = getParam<std::string>(node_, "edges_topic_name", "/edges_markers");
    edges_topic_pub_ = node_.advertise<visualization_msgs::Marker>(edges_topic_name_, 10); 
    
    init(vertex_web,graph_dimension);

    buildPatrollingNodesAsMarkers();
    buildPatrollingEdgesAsMarkers();
}

GraphViz::~GraphViz()
{
}

void GraphViz::GetGraphDimensionFromFile()
{

    file_ = fopen(graph_filename_.c_str(), "r");

    if (file_ == NULL)
    {
        ROS_INFO("Can not open filename %s", graph_filename_.c_str());
        ROS_BREAK();
    }
    else
    {
        ROS_INFO("GetGraphDimension Graph File Opened. Reading Dimensions.\n");
        int r;
        r = fscanf(file_, "%d", &graph_dimension_);
        //      std::cout << "dimension: " << dimension << std::endl;

        //Initialize other dimension variables:
        r = fscanf(file_, "%d", &WIDTH_PX_);
        //     std::cout << "WIDTH_PX: " << WIDTH_PX << std::endl;
        r = fscanf(file_, "%d", &HEIGHT_PX_);
        //     std::cout << "HEIGHT_PX: " << HEIGHT_PX << std::endl;
        r = fscanf(file_, "%lf", &RESOLUTION_);
        //     std::cout << "RESOLUTION: " << RESOLUTION << std::endl;
        WIDTH_M_ = (double) WIDTH_PX_ * RESOLUTION_;
        HEIGHT_M_ = (double) HEIGHT_PX_ * RESOLUTION_;
    }
    fclose(file_);
}

void GraphViz::GetGraphInfoFromFile()
{

    file_ = fopen(graph_filename_.c_str(), "r");

    if (file_ == NULL)
    {
        ROS_INFO("Can not open filename %s", graph_filename_.c_str());
        ROS_BREAK();
    }
    else
    {
        ROS_INFO("GetGraphInfo Graph File Opened. Getting Graph Info.\n");

        int i, j;
        double temp;
        int r;

        //Start Reading the File from FIRST_VID On:
        for (i = 0; i < FIRST_VID - 1; i++)
        {
            r = fscanf(file_, "%lf", &temp);
        }

        for (i = 0; i < graph_dimension_; i++)
        {

            int k;
            int n;
            double xi;
            double yi;
            double zi;

            r = fscanf(file_, "%d", &k);
            //        std::cout << "Node id: " << k << std::endl;

            r = fscanf(file_, "%lf", &xi);
            xi *= RESOLUTION_; //convert to m	
            //        std::cout << "Node id: " << k << " x: " << xi << std::endl;

            r = fscanf(file_, "%lf", &yi);
            yi *= RESOLUTION_; //convert to m
            //        std::cout << "Node id: " << k << " y: " << yi << std::endl;

            r = fscanf(file_, "%lf", &zi);
            zi *= RESOLUTION_; //convert to m
            //        std::cout << "Node id: " << k << " y: " << yi << std::endl;

            r = fscanf(file_, "%d", &n);
            //        std::cout << "Node id: " << k << " num of neighborhood: " << n << std::endl;

            std::vector<int> idn;
            std::vector<std::string> d;
            std::vector<int> c;

            for (j = 0; j < n; j++)
            {
                int idnh;
                r = fscanf(file_, "%d", &idnh);
                //          std::cout << "Neighborhood id " << idnh << std::endl;
                idn.push_back(idnh);
                char dj[3];
                r = fscanf(file_, "%s", dj);
                //          std::cout << "Direction " << dj << std::endl;
                d.push_back(dj);
                int cj;
                r = fscanf(file_, "%d", &cj);
                //         std::cout << "Cost " << cj << std::endl;
                c.push_back(cj);
            }

            VertexViz3D v(k, n, xi, yi, zi, idn, c, d);
            vertex3D_web_.push_back(v);

        }

    }
    
#if 1 
    /// < print out the graph 
    for (int i = 0; i < graph_dimension_; i++)
    {
        printf("ID= %d\n", vertex3D_web_[i].id);
        printf("X= %f, Y= %f, Z= %f\n", vertex3D_web_[i].x, vertex3D_web_[i].y, vertex3D_web_[i].z);
        printf("priority= %f\n", vertex3D_web_[i].priority);        
        printf("#Neigh= %d\n", vertex3D_web_[i].num_neigh);

        for (int j = 0; j < vertex3D_web_[i].num_neigh; j++)
        {
            std::cout << "\tID = " << vertex3D_web_[i].id_neigh[j] << " DIR = " << vertex3D_web_[i].dir[j] << " COST = " << vertex3D_web_[i].cost[j] << std::endl;
        }

        printf("\n");
    }
#endif 
    //printf ("[v=10], x = %f (meters)\n",vertex_web[10].x); 

    fclose(file_);

}

void GraphViz::init()
{
    GetGraphDimensionFromFile();
    GetGraphInfoFromFile();
    //buildPatrollingNodesAsMarkers();

}

void GraphViz::init(Vertex* vertex_web, int graph_dimension)
{
    std::cout << "GraphViz::init() - start" << std::endl; 
    
    WIDTH_PX_  = 25;  // whatever! we don't use it here 
    HEIGHT_PX_ = 25; // whatever! we don't use it here 
    RESOLUTION_ = 1;
    WIDTH_M_  = (float) WIDTH_PX_ * RESOLUTION_;
    HEIGHT_M_ = (float) HEIGHT_PX_ * RESOLUTION_;
    
    graph_dimension_  = graph_dimension; 
    
    if(graph_dimension_ == 0) 
    {
        ROS_ERROR_STREAM("GraphViz::init() - received zero size graph");
        return;
    }
    
    vertex3D_web_.resize(graph_dimension_);

    for (int i = 0; i < graph_dimension_; i++)
    {
        std::cout << "vertex id: " << vertex_web[i].id << std::endl;
        vertex3D_web_[i].id = vertex_web[i].id;
        
        vertex3D_web_[i].priority = vertex_web[i].priority;       
        std::cout << "vertex priority: " << vertex_web[i].priority << std::endl;        

        vertex3D_web_[i].x = vertex_web[i].x;
        vertex3D_web_[i].y = vertex_web[i].y;
        vertex3D_web_[i].z = vertex_web[i].z;

        std::cout << "vertex num_neigh: " << vertex_web[i].num_neigh << std::endl;
        vertex3D_web_[i].num_neigh = vertex_web[i].num_neigh;

        vertex3D_web_[i].id_neigh.resize(vertex3D_web_[i].num_neigh);
        vertex3D_web_[i].cost.resize(vertex3D_web_[i].num_neigh);
        vertex3D_web_[i].dir.resize(vertex3D_web_[i].num_neigh);

        std::cout << "neighbours: ";
        for (int j = 0; j < vertex_web[i].num_neigh; j++)
        {
            std::cout << vertex_web[i].id_neigh[j] << ", ";
            vertex3D_web_[i].id_neigh[j] = vertex_web[i].id_neigh[j];
            vertex3D_web_[i].dir[j]      = vertex_web[i].dir[j];
            vertex3D_web_[i].cost[j]     = vertex_web[i].cost[j];
        }
        std::cout << std::endl; 
    }
    
    std::cout << "GraphViz::init() - end" << std::endl;     
}


void GraphViz::buildPatrollingNodesAsMarkers()
{
    std::cout << "GraphViz::buildPatrollingNodesAsMarkers() - start" << std::endl; 
    marker_nodes_.markers.clear();

    std_msgs::Header header;
    header.frame_id = global_frame_;
    header.stamp = ros::Time::now();

    for (int i = 0; i < graph_dimension_; i++)
    {
        /// < sphere marker 
        
        visualization_msgs::Marker marker;
        marker.header = header;
        std::stringstream ss;
        ss << "patrolling_nodes_" << vertex3D_web_[i].id;
        marker.ns = ss.str();
        marker.id = vertex3D_web_[i].id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vertex3D_web_[i].x;
        marker.pose.position.y = vertex3D_web_[i].y;
        marker.pose.position.z = vertex3D_web_[i].z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = 0.2;
        marker.scale.z = 0.1;
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //marker.lifetime = ros::Duration();
        //marker.frame_locked = false;
        marker_nodes_.markers.push_back(marker);
        
        
        /// < text marker 

        visualization_msgs::Marker marker_t;
        marker_t = marker;
        std::stringstream ss2;
        ss2 << "patrolling_nodes_text_" << vertex3D_web_[i].id;
        marker_t.ns = ss2.str();
        marker_t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_t.pose.position.x = vertex3D_web_[i].x;
        marker_t.pose.position.y = vertex3D_web_[i].y;
        marker_t.pose.position.z = vertex3D_web_[i].z + 0.5;
        marker_t.color.r = 1.0;
        marker_t.color.g = 1.0;
        marker_t.color.b = 1.0;
        marker_t.color.a = 1.0;
        marker_t.scale.x = 1;
        marker_t.scale.y = 1;
        marker_t.scale.z = kMarkerTextHeight;


        std::stringstream sss;
        sss.precision(2);
        sss.setf(std::ios::fixed, std::ios::floatfield);
        sss << "ID=" << vertex3D_web_[i].id << "\n";
#if 0        
        sss << "X= " << vertex3D_web_[i].x << " Y= " << vertex3D_web_[i].y << " Z= " << vertex3D_web_[i].z << "\n";
        sss << "#Neigh= " << vertex3D_web_[i].num_neigh << "\n";
        for (int j = 0; j < vertex3D_web_[i].num_neigh; j++)
        {
            sss << "\tID = " << vertex3D_web_[i].id_neigh[j] << " DIR = " << vertex3D_web_[i].dir[j] << " COST = " << vertex3D_web_[i].cost[j] << "\n";
        }
        double tt = header.stamp.toSec() - time_zero_.toSec();
        sss << "Current Time: " << tt << " sec." << "\n";
#endif
        //sss << "Num of visits: " << vertex_web[i].number_of_visits << "\n";
        sss << "#visits: " << vertex3D_web_[i].number_of_visits << "\n";
        //sss << "Current Idleness: " << vertex_web[i].current_idleness << "\n";
        sss << "idleness: " << vertex3D_web_[i].current_idleness << "\n";
        sss << "priority: " << vertex3D_web_[i].priority << "\n";        
#if 0
        sss << "Last Visit: " << vertex3D_web_[i].last_visit << "\n";
#endif
        marker_t.text = sss.str();

        marker_nodes_.markers.push_back(marker_t);
    }
    
    std::cout << "GraphViz::buildPatrollingNodesAsMarkers() - end" << std::endl;     
}

void GraphViz::buildPatrollingEdgesAsMarkers()
{
    std::cout << "GraphViz::buildPatrollingEdgesAsMarkers() - start" << std::endl; 
    
    boost::recursive_mutex::scoped_lock nodes_marker_array_locker(marker_edges_list_mutex_);  
    
    marker_edges_list_.points.clear();

    marker_edges_list_.header.frame_id = global_frame_;
    marker_edges_list_.header.stamp = ros::Time::now();
    
    marker_edges_list_.action = visualization_msgs::Marker::ADD;
    marker_edges_list_.pose.orientation.w = 1.0;
    marker_edges_list_.ns = "edges";
    marker_edges_list_.id = 1;
    marker_edges_list_.type = visualization_msgs::Marker::LINE_LIST;
    marker_edges_list_.scale.x = 0.05;
    marker_edges_list_.color.r = 0.85;
    marker_edges_list_.color.g = 0.85;
    marker_edges_list_.color.b =  .0;
    marker_edges_list_.color.a = 0.7;
    
    geometry_msgs::Point pointA;
    geometry_msgs::Point pointB;
    
    std::vector< std::vector<int> > mat_adj = std::vector< std::vector<int> >(graph_dimension_, std::vector<int>(graph_dimension_,0));
    
    for (int ii = 0; ii < graph_dimension_; ii++)
    {
        pointA.x = vertex3D_web_[ii].x;
        pointA.y = vertex3D_web_[ii].y;
        pointA.z = vertex3D_web_[ii].z;
        
        int num_neighbours = vertex3D_web_[ii].num_neigh;

        for (int j = 0; j < num_neighbours; j++)
        {
            int id_neighbour = vertex3D_web_[ii].id_neigh[j];
            pointB.x = vertex3D_web_[id_neighbour].x;
            pointB.y = vertex3D_web_[id_neighbour].y;
            pointB.z = vertex3D_web_[id_neighbour].z;
            
            if(mat_adj[ii][id_neighbour] == 0)
            {
                marker_edges_list_.points.push_back(pointA);
                marker_edges_list_.points.push_back(pointB);
                mat_adj[ii][id_neighbour] = 1;
                mat_adj[id_neighbour][ii] = 1;
            }
        }
        
    }
    
    edges_topic_pub_.publish(marker_edges_list_);
    
    std::cout << "GraphViz::buildPatrollingEdgesAsMarkers() - end" << std::endl;     
}

void GraphViz::buildPatrollingNodesAsMarkers(double time, double time_zero)
{

    marker_nodes_.markers.clear();


    std_msgs::Header header;
    header.frame_id = global_frame_;
    header.stamp = ros::Time::now();

    for (int i = 0; i < graph_dimension_; i++)
    {

        visualization_msgs::Marker marker;
        marker.header = header;
        std::stringstream ss;
        ss << "patrolling_nodes_" << vertex3D_web_[i].id;
        marker.ns = ss.str();
        marker.id = vertex3D_web_[i].id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vertex3D_web_[i].x;
        marker.pose.position.y = vertex3D_web_[i].y;
        marker.pose.position.z = vertex3D_web_[i].z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        double normalized_idleness = std::min(vertex3D_web_[i].current_idleness,(double)kMaxIdlenessForDrawing)/kMaxIdlenessForDrawing;
                
        double idleness_radius = 0.2 + normalized_idleness*kMaxIdlenessRadius;
        
        marker.scale.x = marker.scale.y =  idleness_radius;
        marker.scale.z = 0.1;
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0 - normalized_idleness;
        marker.color.b = 0.0;
        
        //marker.lifetime = ros::Duration();
        //marker.frame_locked = false;
        marker_nodes_.markers.push_back(marker);

        visualization_msgs::Marker marker_t;
        marker_t = marker;
        std::stringstream ss2;
        ss2 << "patrolling_nodes_text_" << vertex3D_web_[i].id;
        marker_t.ns = ss2.str();
        marker_t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_t.pose.position.x = vertex3D_web_[i].x;
        marker_t.pose.position.y = vertex3D_web_[i].y;
        marker_t.pose.position.z = vertex3D_web_[i].z + 0.5;
        marker_t.color.r = 1.0;
        marker_t.color.g = 1.0;
        marker_t.color.b = 1.0;
        marker_t.scale.x = 1;
        marker_t.scale.y = 1;
        marker_t.scale.z = kMarkerTextHeight;


        std::stringstream sss;
        sss.precision(2);
        sss.setf(std::ios::fixed, std::ios::floatfield);
        sss << "ID=" << vertex3D_web_[i].id << "\n";
#if 0
        sss << "X= " << vertex3D_web_[i].x << " Y= " << vertex3D_web_[i].y << " Z= " << vertex3D_web_[i].z << "\n";
        sss << "#Neigh= " << vertex3D_web_[i].num_neigh << "\n";
        for (int j = 0; j < vertex3D_web_[i].num_neigh; j++)
        {
            sss << "\tID = " << vertex3D_web_[i].id_neigh[j] << " DIR = " << vertex3D_web_[i].dir[j] << " COST = " << vertex3D_web_[i].cost[j] << "\n";
        }
        //double tt =  header.stamp.toSec() - time_zero.toSec();
        double tt = time - time_zero;
        sss << "Current Time: " << tt << " sec." << "\n";
#endif
        //sss << "Num of visits: " << vertex_web[i].number_of_visits << "\n";
        sss << "#visits: " << vertex3D_web_[i].number_of_visits << "\n";
        //sss << "Current Idleness: " << vertex_web[i].current_idleness << "\n";
        sss << "idleness: " << vertex3D_web_[i].current_idleness << "\n";
        sss << "priority: " << vertex3D_web_[i].priority << "\n";                
#if 0
        sss << "Last Visit: " << vertex3D_web_[i].last_visit << "\n";
#endif
        marker_t.text = sss.str();

        marker_nodes_.markers.push_back(marker_t);
        
    }
    
}

void GraphViz::publishPatrollingNodesAsMarkers()
{

    nodes_topic_pub_.publish(marker_nodes_);
    //nodes.markers.clear();
    //buildPatrollingNodesAsMarkers(); // In order to refresh the current time of each node
}

void GraphViz::publishPatrollingEdgesAsMarkers()
{

    edges_topic_pub_.publish(marker_edges_list_);
}

void GraphViz::getRobotPoses()
{

    if (listener_ == NULL)
    {
        ROS_ERROR("TF listener null");
        return;
    }

    std::string sframe = global_frame_;
    robot_poses_.clear();
    robot_boundaries_.markers.clear();
    publishRobotBoudariesAsMarkers();

    for (int i = 0; i < num_robots_; i++)
    {
        int id = i + 1;
        std::stringstream ss;
        ss << "ugv" << id; // Custom solution
        std::string robotname = ss.str();
        std::string dframe = "/" + robotname + "/" + robot_frame_;

        tf::StampedTransform transform;
        try
        {
            listener_->waitForTransform(sframe, dframe, ros::Time(0), ros::Duration(3));
            listener_->lookupTransform(sframe, dframe, ros::Time(0), transform);

            geometry_msgs::PoseStamped robot_pose;
            robot_pose.header.stamp = transform.stamp_;
            robot_pose.header.frame_id = transform.frame_id_;
            robot_pose.pose.position.x = transform.getOrigin().x();
            robot_pose.pose.position.y = transform.getOrigin().y();
            robot_pose.pose.position.z = transform.getOrigin().z();


            double roll, pitch, yaw;
            transform.getBasis().getRPY(roll, pitch, yaw);
            robot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            robot_poses_.push_back(robot_pose);

            visualization_msgs::Marker marker;
            marker.header = robot_pose.header;
            std::stringstream ss;
            ss << "boundary_" << robotname;
            marker.ns = ss.str();
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = robot_pose.pose.position.x;
            marker.pose.position.y = robot_pose.pose.position.y;
            marker.pose.position.z = robot_pose.pose.position.z;
            marker.pose.orientation = robot_pose.pose.orientation;
            marker.scale.x = 2 * safety_radius_;
            marker.scale.y = 2 * safety_radius_;
            marker.scale.z = 2 * safety_radius_;
            marker.color.a = 0.2; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            robot_boundaries_.markers.push_back(marker);

        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Cannot transform from %s to %s\n", sframe.c_str(), dframe.c_str());
            ROS_ERROR("%s", ex.what());
        }
    }
}

bool GraphViz::checkCollision()
{
    if(robot_poses_.empty()) 
    {
        std::cout << "num_robots_: " << num_robots_ << std::endl; 
        std::cout << "robot_poses_ size: " << robot_poses_.size() << std::endl; 
        return true; 
    }

    for (int i = 0; i < num_robots_; i++)
    {    
        geometry_msgs::PoseStamped center = robot_poses_[i]; 
        for (int j = 0; j < num_robots_; j++)
        {             
            if (i != j)
            {
                double dist = sqrt(pow((center.pose.position.x - robot_poses_[j].pose.position.x), 2) + pow((center.pose.position.y - robot_poses_[j].pose.position.y), 2) + pow((center.pose.position.z - robot_poses_[j].pose.position.z), 2));
                //ROS_INFO("Distance betweem the robots %lf",dist);

                if (dist <= (2 * safety_radius_ * 0.75))
                {
                    //ROS_INFO("Collision detected");
                    robot_boundaries_.markers[i].color.r = 1.0;
                    robot_boundaries_.markers[i].color.g = 0.0;
                    robot_boundaries_.markers[i].color.b = 0.0;

                    robot_boundaries_.markers[j].color.r = 1.0;
                    robot_boundaries_.markers[j].color.g = 0.0;
                    robot_boundaries_.markers[j].color.b = 0.0;

                    return false;
                }
                else if (dist <= (3 * safety_radius_ * 0.75))
                {
                    robot_boundaries_.markers[i].color.r = 1.0;
                    robot_boundaries_.markers[i].color.g = 1.0;
                    robot_boundaries_.markers[i].color.b = 0.0;

                    robot_boundaries_.markers[j].color.r = 1.0;
                    robot_boundaries_.markers[j].color.g = 1.0;
                    robot_boundaries_.markers[j].color.b = 0.0;
                }
                else
                {

                    robot_boundaries_.markers[i].color.r = 0.0;
                    robot_boundaries_.markers[i].color.g = 1.0;
                    robot_boundaries_.markers[i].color.b = 0.0;

                    robot_boundaries_.markers[j].color.r = 0.0;
                    robot_boundaries_.markers[j].color.g = 1.0;
                    robot_boundaries_.markers[j].color.b = 0.0;
                    continue;
                }

            }

        }
    }
    return true;
}

void GraphViz::publishRobotBoudariesAsMarkers()
{
    boundaries_topic_pub_.publish(robot_boundaries_);
}


//int main(int argc, char **argv) {
//    ros::init(argc, argv, "graph_viz");
//
//    GraphViz g_v;
//
//    ros::Rate r(10);
//    while (ros::ok()) {
//        g_v.publishPatrollingNodesAsMarkers(); // Handle USB events
//        g_v.getRobotPoses();
//        g_v.checkCollision();
//        g_v.publishRobotBoudariesAsMarkers();
//        ros::spinOnce(); // Handle ROS events
//        r.sleep();
//    }
//
//
//    return 0;
//}


