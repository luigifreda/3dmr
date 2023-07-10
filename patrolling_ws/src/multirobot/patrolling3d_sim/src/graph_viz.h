/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DPATROLLING. 
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

#ifndef GRAPH_VIZ_H
#define GRAPH_VIZ_H

#include <string>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "graph.h"

#define FIRST_VID 5

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("[GRAPH_VIZ] Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("[GRAPH_VIZ] Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}


///	\class VertexViz3D
///	\author Luigi Freda (2016-present) and Mario Gianni (2016)
///	\brief A class for visualizing a patrolling vertex 
///	\note
/// 	\todo 
///	\date
///	\warning
class VertexViz3D
{
public:

    int id;
    int num_neigh;
    double x, y, z; //pass these attributes in meters
    float priority; 
    std::vector<int> id_neigh;
    std::vector<int> cost;
    //std::vector<double> cost_m;
    //std::vector<bool> visited;
    std::vector<std::string> dir;
    int number_of_visits;
    double current_idleness;
    double last_visit;

    VertexViz3D()
    {
        id = 0;
        num_neigh = 0;
        x = y = z = 0; //pass these attributes in meters
        priority = 1.0; 
        number_of_visits = 0; // first visit should not be cnted for avg
        current_idleness = 0.0;
        last_visit = 0.0;
    };
    //    Vertex(int i, int n, double xi, double yi, std::vector<int>& idn, std::vector<int>& c, std::vector<double>& cm, std::vector<bool>& v, std::vector<std::string>& d){
    //        id = i;
    //        num_neigh = n;
    //        x = xi;
    //        y = yi; //pass these attributes in meters
    //        id_neigh = idn;
    //        cost= c;
    //        cost_m = cm;
    //        visited = v;
    //        dir = d;
    //    }

    VertexViz3D(int i, int n, double xi, double yi, std::vector<int>& idn, std::vector<int>& c, std::vector<std::string>& d)
    {
        id = i;
        num_neigh = n;
        x = xi;
        y = yi; //pass these attributes in meters
        z = 0;
        priority = 1.0; 
        id_neigh = idn;
        cost = c;
        dir = d;
        number_of_visits = 0; // first visit should not be cnted for avg
        current_idleness = 0.0;
        last_visit = 0.0;
    }

    VertexViz3D(int i, int n, double xi, double yi, double zi, std::vector<int>& idn, std::vector<int>& c, std::vector<std::string>& d)
    {
        id = i;
        num_neigh = n;
        x = xi;
        y = yi; //pass these attributes in meters
        z = zi;
        priority = 1.0;         
        id_neigh = idn;
        cost = c;
        dir = d;
        number_of_visits = 0; // first visit should not be cnted for avg
        current_idleness = 0.0;
        last_visit = 0.0;
    }

    ~VertexViz3D()
    {
    };

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

///	\class VertexViz3D
///	\author Luigi Freda (2016-present) and Mario Gianni (2016)
///	\brief A class for visualizing a patrolling graph 
///	\note
/// 	\todo 
///	\date
///	\warning
class GraphViz
{
    
    static const double kMaxIdlenessForDrawing;
    static const double kMaxIdlenessRadius; 
    static const double kMarkerTextHeight; 
    
public:

    GraphViz();
    GraphViz(std::string& gf, std::string& ntn, int n, std::string& glf, std::string& rf, double r, std::string& btn);
    GraphViz(Vertex* vertex_web, int graph_dimension, std::string& ntn, int n, std::string& glf, std::string& rf, double r, std::string& btn);
    
    ~GraphViz();

    void init();
    void init(Vertex* vertex_web, int graph_dimension);

    visualization_msgs::MarkerArray marker_nodes_;
    boost::recursive_mutex nodes_marker_array_mutex_;
    
    visualization_msgs::Marker marker_edges_list_;
    boost::recursive_mutex marker_edges_list_mutex_;

    void buildPatrollingNodesAsMarkers();
    void buildPatrollingNodesAsMarkers(double, double);
    
    void buildPatrollingEdgesAsMarkers(); 
    
    void publishPatrollingNodesAsMarkers();
    void publishPatrollingEdgesAsMarkers();

    std::string nodes_topic_name_;
    ros::Publisher nodes_topic_pub_;
    
    std::string edges_topic_name_;
    ros::Publisher edges_topic_pub_;

    std::string boundaries_topic_name_;
    ros::Publisher boundaries_topic_pub_;

    ros::Time time_zero_;

    int num_robots_;
    std::string global_frame_;
    std::string robot_frame_;
    std::vector<geometry_msgs::PoseStamped> robot_poses_;
    tf::TransformListener *listener_;
    void getRobotPoses();
    double safety_radius_;
    bool checkCollision();
    visualization_msgs::MarkerArray robot_boundaries_;
    void publishRobotBoudariesAsMarkers();

    void setNodeNumVisits(int goal, int n)
    {
        vertex3D_web_[goal].setNumVisits(n);
    }

    void setNodesNumVisits(int n[])
    {
        for (int i = 0; i < graph_dimension_; i++)
        {
            vertex3D_web_[i].setNumVisits(n[i]);
        }

    }

    void setNodeCurrentIdleness(int goal, double n)
    {
        vertex3D_web_[goal].setCurrentIdleness(n);
    }

    void setNodesCurrentIdleness(double n[])
    {
        for (int i = 0; i < graph_dimension_; i++)
        {
            vertex3D_web_[i].setCurrentIdleness(n[i]);
        }
    }

    void setNodeLastVisit(int goal, double n)
    {
        vertex3D_web_[goal].setLastVisit(n);
    }

    void setNodesLastVisit(double n[])
    {
        for (int i = 0; i < graph_dimension_; i++)
        {
            vertex3D_web_[i].setLastVisit(n[i]);
        }
    }

    void setNumRobots(int num)
    {
        num_robots_ = num; 
    }
    
    void setNodePriority(int id, float priority)
    {
        vertex3D_web_[id].priority = priority;
    }    
    
protected:

    void GetGraphDimensionFromFile();
    void GetGraphInfoFromFile();    
    
private:

    int WIDTH_PX_;
    int HEIGHT_PX_;
    double RESOLUTION_;
    double WIDTH_M_;
    double HEIGHT_M_;
    int graph_dimension_; // number of nodes 
    FILE *file_;
    std::vector<VertexViz3D> vertex3D_web_;

    ros::NodeHandle node_;
    std::string graph_filename_;

};



#endif /* GRAPH_VIZ_H */

