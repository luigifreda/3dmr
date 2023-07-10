#pragma once 

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud2.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <limits>
#include <pcl/point_types.h>


void downsamplePointcloud(const float leaf_size,
                          const sensor_msgs::PointCloud2& cloud_in,
                          sensor_msgs::PointCloud2& cloud_out)
{ 
    // NOTE: the following commented code does not work!
    // pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    // sor.setInputCloud(cloud); // does not work even with `const sensor_msgs::PointCloud2ConstPtr& cloud`
    // sor.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
    // sor.filter(*cloud_filtered);

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
    
    pcl_conversions::toPCL(cloud_in, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*cloud_filtered);

    pcl_conversions::moveFromPCL(*cloud_filtered, cloud_out);              
}

void getObstacleMessageFromObstPointcloud(const sensor_msgs::PointCloud2& obst_point_cloud, 
                                          const float point_radius, 
                                          costmap_converter::ObstacleArrayMsg& msg)
{
    // references: 
    // http://docs.ros.org/en/noetic/api/costmap_converter/html/msg/ObstacleArrayMsg.html
    // http://docs.ros.org/en/noetic/api/costmap_converter/html/msg/ObstacleMsg.html 

    // Assumptions: the obstacle point cloud height=1 and width=#points
    // This assumption is satisfied in both LaserProximityChecker and LaserProximityChecker2

    ROS_ASSERT_MSG(obst_point_cloud.height==1,"obst_point_cloud is expected to have height==1");
    
    // init the msg out 
    msg.obstacles.clear();
    msg.header = obst_point_cloud.header;
    msg.obstacles.reserve(obst_point_cloud.width);
        

    // getting information to parse the point cloud
    unsigned int pt_step = obst_point_cloud.point_step; // size of the point structure
    // offsets of the relevent fields (init values)
    unsigned int pt_x_offset   = 0;
    unsigned int pt_y_offset   = 4;
    unsigned int pt_z_offset   = 8;
    for (unsigned int j = 0; j < obst_point_cloud.fields.size(); j++)
    {
        if (!obst_point_cloud.fields[j].name.compare("x"))
        {
            pt_x_offset = obst_point_cloud.fields[j].offset;
        }
        else if (!obst_point_cloud.fields[j].name.compare("y"))
        {
            pt_y_offset = obst_point_cloud.fields[j].offset;
        }
        else if (!obst_point_cloud.fields[j].name.compare("z"))
        {
            pt_z_offset = obst_point_cloud.fields[j].offset;
        }
    }

    // traverse point cloud
    for (size_t i = 0; i < obst_point_cloud.width; i++)
    {
        float x, y, z, intensity; // fields from the point

        const size_t i_x_pt_step_base = i * pt_step;
        size_t i_x_pt_step = i_x_pt_step_base + pt_x_offset; // i * pt_step + pt_x
        std::copy(reinterpret_cast<const char*> (&obst_point_cloud.data[i_x_pt_step]),
                  reinterpret_cast<const char*> (&obst_point_cloud.data[i_x_pt_step + 4]),
                  reinterpret_cast<char*> (&x));
        
        i_x_pt_step = i_x_pt_step_base + pt_y_offset;
        std::copy(reinterpret_cast<const char*> (&obst_point_cloud.data[i_x_pt_step]),
                  reinterpret_cast<const char*> (&obst_point_cloud.data[i_x_pt_step + 4]),
                  reinterpret_cast<char*> (&y));
        
        i_x_pt_step = i_x_pt_step_base + pt_z_offset;
        std::copy(reinterpret_cast<const char*> (&obst_point_cloud.data[i_x_pt_step]),
                  reinterpret_cast<const char*> (&obst_point_cloud.data[i_x_pt_step + 4]),
                  reinterpret_cast<char*> (&z));

        costmap_converter::ObstacleMsg om;
        om.header = msg.header;
        om.radius = point_radius;
        om.id = i; 
        geometry_msgs::Point32 point; 
        point.x = x; 
        point.y = y; 
        point.z = z; 
        om.polygon.points.push_back(point);

        msg.obstacles.push_back(om); 
    }
}