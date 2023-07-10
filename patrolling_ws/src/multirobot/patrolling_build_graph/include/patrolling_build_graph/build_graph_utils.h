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

#ifndef BUILD_GRAPH_UTILS_H
#define BUILD_GRAPH_UTILS_H

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <nav_msgs/Path.h>
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
#include <boost/foreach.hpp>
#include <robot_trajectory_saver_msgs/CheckPath.h>



namespace utils
{

    ///	\class Matrix
///	\author Luigi Freda 
///	\brief A base class for storing/managing matrix data of different types.
///	\note 
/// 	\todo 
///	\date
///	\warning
template <typename T>
class Matrix : public std::vector< std::vector<T> >
{
public:

    Matrix()
    {}

    Matrix(size_t num_rows, size_t num_cols)
    : std::vector< std::vector<T> >(num_rows, std::vector<T>(num_cols))
    {}

    Matrix(size_t num_rows, size_t num_cols, const T& val)
    : std::vector< std::vector<T> >(num_rows, std::vector<T>(num_cols, val))
    {}
};

typedef Matrix<int> MatrixInt;
typedef Matrix<float> MatrixFloat;
typedef Matrix<double> MatrixDouble;
typedef Matrix<std::string> MatrixString;

}

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("[PBG] Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("[PBG] Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}

static inline double from_degrees(double degrees)
{
    return degrees * M_PI / 180.0;
}

/*!
 * \brief Convert radians to degrees
 */
static inline double to_degrees(double radians)
{
    return radians * 180.0 / M_PI;
}

/*!
 * \brief normalize_angle_positive
 *
 *        Normalizes the angle to be 0 to 2*M_PI
 *        It takes and returns radians.
 */
static inline double normalize_angle_positive(double angle)
{
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

/*!
 * \brief normalize
 *
 * Normalizes the angle to be -M_PI circle to +M_PI circle
 * It takes and returns radians.
 *
 */
static inline double normalize_angle(double angle)
{
    double a = normalize_angle_positive(angle);
    if (a > M_PI)
        a -= 2.0 * M_PI;
    return a;
}

static inline std::string compute_direction(double angle)
{
    std::string dir = "";

    if ((0 <= angle) && (angle < M_PI / 8))
    {
        dir = "E";
    }
    else if ((M_PI / 8 <= angle) && (angle < 3 * M_PI / 8))
    {
        dir = "NE";
    }
    else if ((3 * M_PI / 8 <= angle) && (angle < 5 * M_PI / 8))
    {
        dir = "N";
    }
    else if ((5 * M_PI / 8 <= angle) && (angle < 7 * M_PI / 8))
    {
        dir = "NW";
    }
    else if ((7 * M_PI / 8 <= angle) && (angle < 9 * M_PI / 8))
    {
        dir = "W";
    }
    else if ((9 * M_PI / 8 <= angle) && (angle < 11 * M_PI / 8))
    {
        dir = "SW";
    }
    else if ((11 * M_PI / 8 <= angle) && (angle < 13 * M_PI / 8))
    {
        dir = "S";
    }
    else if ((13 * M_PI / 8 <= angle) && (angle < 15 * M_PI / 8))
    {
        dir = "SE";
    }
    else if ((15 * M_PI / 8 <= angle) && (angle < 2 * M_PI))
    {
        dir = "E";
    }
    else
    {
        std::cout << "Unknown direction" << std::endl;
    }


    return dir;
}

static inline std::string compute_inverse_direction(std::string d)
{
    std::string dir = "";

    if (d.compare("E") == 0)
    {
        dir = "W";
    }
    else if (d.compare("NE") == 0)
    {
        dir = "SW";
    }
    else if (d.compare("N") == 0)
    {
        dir = "S";
    }
    else if (d.compare("NW") == 0)
    {
        dir = "SE";
    }
    else if (d.compare("W") == 0)
    {
        dir = "E";
    }
    else if (d.compare("SW") == 0)
    {
        dir = "NE";
    }
    else if (d.compare("S") == 0)
    {
        dir = "N";
    }
    else if (d.compare("SE") == 0)
    {
        dir = "NW";
    }
    else if (d.compare("E") == 0)
    {
        dir = "W";
    }
    else
    {
        std::cout << "Unknown direction" << std::endl;
    }


    return dir;
}

static inline bool is_equal(const pcl::PointXYZ& u, const pcl::PointXYZ& v)
{
    double dist = sqrt((u.x - v.x)*(u.x - v.x) + (u.y - v.y)*(u.y - v.y) + (u.z - v.z)*(u.z - v.z));

    if (dist < 0.0001)
        return true;
    return false;
}

static inline bool is_contained(const pcl::PointXYZ& u, pcl::PointCloud<pcl::PointXYZ>* cloud_in)
{
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        if (is_equal(u, cloud_in->points[i]))
            return true;
    }
    return false;
}

static inline bool is_contained(const pcl::PointXYZ& u, const pcl::PointCloud<pcl::PointXYZ>& cloud_in)
{
    for (int i = 0; i < cloud_in.points.size(); i++)
    {
        if (is_equal(u, cloud_in.points[i]))
            return true;
    }
    return false;
}

static inline pcl::PointCloud<pcl::PointXYZ>* check_all_pts_different(pcl::PointCloud<pcl::PointXYZ>* cloud_in)
{
    ROS_INFO("Numbers of inserted patrolling nodes %lu", cloud_in->points.size());
    pcl::PointCloud<pcl::PointXYZ>* cloud_out = new pcl::PointCloud<pcl::PointXYZ>();
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        for (int j = i; j < cloud_in->points.size(); j++)
        {
            if (i != j)
            {
                if (!is_equal(cloud_in->points[i], cloud_in->points[j]))
                {
                    if (cloud_out->points.empty() || !is_contained(cloud_in->points[i], cloud_out))
                    {
                        cloud_out->points.push_back(cloud_in->points[i]);
                    }
                }
                else
                {
                    std::cout << "Attention! Node ID " << i << " is at the same position of Node ID " << j << std::endl;
                }
            }
        }
    }
    return cloud_out;
}


static void check_all_pts_different(const pcl::PointCloud<pcl::PointXYZ>& cloud_in, 
                                    const std::vector<float>& nodes_priority,
                                    const std::vector<size_t>& nodes_id,
                                    pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                                    std::vector<float>& nodes_filtered_priority,
                                    std::vector<size_t>& nodes_filtered_id                                    
                                    )
{
    ROS_INFO("Numbers of inserted patrolling nodes %lu", cloud_in.points.size());
    
    cloud_out.clear();
    nodes_filtered_priority.clear();
    nodes_filtered_id.clear();
    
    bool bDuplicated = false; 
    
    for (int i = 0; i < cloud_in.points.size(); i++)
    {
        bDuplicated = false;
        
        if(is_contained(cloud_in.points[i], cloud_out))
        {
            bDuplicated = true; 
        }    
        else
        {
            for (int j = i+1; j < cloud_in.points.size(); j++)
            {
                if(is_equal(cloud_in.points[i], cloud_in.points[j]))
                {
                    bDuplicated = true; 
                    break; 
                    std::cout << "Attention! Node ID " << i << " is at the same position of Node ID " << j << std::endl;
                }
            }
        }
        
        if(!bDuplicated)
        {
            cloud_out.points.push_back(cloud_in.points[i]);
            nodes_filtered_priority.push_back(nodes_priority[i]);
            nodes_filtered_id.push_back(nodes_id[i]);
        }
            
    }
}


static inline int compute_max_num_intersections(std::vector<int> num_intersections)
{
    int n = 0;
    for (std::vector<int>::iterator it = num_intersections.begin(); it != num_intersections.end(); ++it)
    {
        if (*it > n)
            n = *it;
    }
    return n;
}

static inline void remove_far_nodes(std::vector<int>& pointIdxNKNSearch, std::vector<float>& pointNKNSquaredDistance, const float max_squared_distance)
{
    std::vector<float>::iterator it_dist = pointNKNSquaredDistance.begin();
    std::vector<int>::iterator it_id = pointIdxNKNSearch.begin();

    while ((it_dist != pointNKNSquaredDistance.end()) && (it_id != pointIdxNKNSearch.end()))
    {
        //std::cout << "node id: " << *it_id << ", distance: " << sqrt(*it_dist) <<  std::endl; 
        if (*it_dist > max_squared_distance)
        {
            //std::cout << "removed" << std::endl; 
            it_dist = pointNKNSquaredDistance.erase(it_dist);
            it_id = pointIdxNKNSearch.erase(it_id);
        }
        else
        {
            ++it_dist;
            ++it_id;
        }
    }

    if (pointNKNSquaredDistance.size() != pointIdxNKNSearch.size())
    {
        ROS_ERROR_STREAM("remove_far_nodes() - pointNKNSquaredDistance.size() != pointIdxNKNSearch.size()");
        quick_exit(-1);
    }
}

static inline int compute_num_neighbours(const utils::MatrixInt& mat_adj, const int node_id, std::vector<int>& neighbours)   
{
    neighbours.clear();
    int res = 0;
    int num_nodes = mat_adj.size();
    for (int m = 0; m < num_nodes; m++)
    {
        if (mat_adj[node_id][m] == 1)
        {
            neighbours.push_back(m);
            res++;
        }
    }
    
    return res; 
}


#endif /* BUILD_GRAPH_H */

