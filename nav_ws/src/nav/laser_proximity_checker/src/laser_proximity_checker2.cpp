/**
* This file is part of the ROS package laser_proximity_checker which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the MIT License.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. *
*/

#include <laser_proximity_checker/utils.h>

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <nifti_pcl_common/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <limits>
#include <pcl/point_types.h>


template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("[LPC] Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("[LPC] Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}


///	\class LaserCollisionChecker
///	\author Luigi Freda
///	\brief A class for checking potential near collisions with 3D laser scan or depth point cloud 
///	\note 
/// \todo 
///	\date
///	\warning

class LaserProximityChecker2
{
    static const float kRobotRadius;  // [m] robot radius for computing the clearance  
    static const float kProximityDistanceThreshold; // [m] distance point-to-robot 
    static const float kProximityDistanceToCheckSquared; // [m] point-to-point distance to check 
    static const float kZMax;
    static const float kZMin;
    static const float kXMin;     // forward direction control
    static const float kYmin;      // side direction control
    static const float kRadiusFilterRadius; // radius for the radius filter (for neighbor search)
    static const float kRadiusFilterMinNeighborsInRadius;  // radius filter minimum number of neighbors 

public:
    //! Constructor. ROS::init() is assumed to have been called before.
    LaserProximityChecker2();

protected: 
    
    //! Coloured scan callback function
    void scan_cb(const sensor_msgs::PointCloud2& scan);
    
    bool check_proximity(const sensor_msgs::PointCloud2& ptcld); 
    
    template <typename PointIn=pcl::PointXYZI>    
    void filter_obst_pcl_and_compute_closest_point();
    
protected:

    //! /tf listener
    tf::TransformListener tf_listener;

    //! public NodeHandle
    ros::NodeHandle n;

    //! private NodeHandle
    ros::NodeHandle n_;

    //! Name of the laser frame (default: "/laser")
    std::string laser_frame;

    //! Name of the robot frame (default: "/base_link")
    std::string robot_frame;

    //! Name of the reference world frame (default: "/odom")
    std::string world_frame;

    //! Publisher for the moving point cloud (default topic: "/obstacle_point_cloud")
    ros::Publisher obst_point_cloud_pub;
    
    //! Publisher for the proximity information (default topic: "/closest_obst_point")
    ros::Publisher closest_obst_point_pub;
    
    //! Publisher for the proximity information (default topic: "/obstacles")
    ros::Publisher obst_message_pub;

    //! Subscriber to input scans (default topic: "/scan_point_cloud")
    ros::Subscriber point_cloud_scan_sub;

    //! Subscriber to input scans (default topic: "/scan_filtered")
    ros::Subscriber laser_scan_sub;

    //! Limit the size of the point cloud in points (default: 1000000).
    int max_size;

    //! The radius of the robot (for computing the clearance) 
    float robot_radius; 

    //! Current aggregated point cloud
    sensor_msgs::PointCloud2 point_cloud;

    //! Temporary point cloud
    sensor_msgs::PointCloud2 tmp_point_cloud;
        
    //! Current aggregated point cloud
    sensor_msgs::PointCloud2 obst_point_cloud;

    //! Starting time of the new scan
    ros::Time start_time;

    //! base_link transform at first time
    tf::StampedTransform base_transform;
    
    //! Publisher for the proximity information (default topic: "laser_proximity_topic")
    ros::Publisher proximity_pub;
    
    double pcl_throttle_         = 1.0;    // [s]
    double last_pcl_time_        = NAN; 

    float min_proximity_distance = LaserProximityChecker2::kProximityDistanceThreshold+LaserProximityChecker2::kRobotRadius; // [m] point-to-point distance to check 
    float min_proximity_distance_squared = NAN; 

    bool have_laser_intensity_ = true; 
};

const float LaserProximityChecker2::kRobotRadius = 0.5;  // [m] robot radius for computing the clearance  
const float LaserProximityChecker2::kProximityDistanceThreshold = 0.5; // [m] distance point-to-robot 
const float LaserProximityChecker2::kProximityDistanceToCheckSquared = pow(LaserProximityChecker2::kProximityDistanceThreshold+LaserProximityChecker2::kRobotRadius,2); // [m] point-to-point distance to check 
const float LaserProximityChecker2::kZMax = 0.5;
const float LaserProximityChecker2::kZMin = 0.1;
const float LaserProximityChecker2::kXMin = 0.1;     // forward direction control
const float LaserProximityChecker2::kYmin = 0.1;      // side direction control
const float LaserProximityChecker2::kRadiusFilterRadius = 0.5; // radius for the radius filter 
const float LaserProximityChecker2::kRadiusFilterMinNeighborsInRadius = 8;  // radius filter minimum number of neighbors 

/*
 * Constructor
 */
LaserProximityChecker2::LaserProximityChecker2() :
tf_listener(ros::Duration(60.)),
n_("~")
{
    // frame names
    laser_frame = getParam<std::string>(n_, "laser_frame", "laser");
    robot_frame = getParam<std::string>(n_, "robot_frame", "base_link");
    world_frame = getParam<std::string>(n_, "world_frame", "odom");
    robot_radius = getParam<float>(n_, "robot_radius", kRobotRadius);    

    have_laser_intensity_ = getParam<bool>(n_, "have_laser_intensity", true);

    // max number of points
    max_size = getParam<int>(n_, "max_size", 1000000);

    // min radius filter 
    min_proximity_distance = getParam<float>(n_, "min_proximity_distance", LaserProximityChecker2::kProximityDistanceThreshold+LaserProximityChecker2::kRobotRadius);
    min_proximity_distance_squared = pow(min_proximity_distance, 2);
    
    // laser proximity publisher
    proximity_pub = n.advertise<std_msgs::Bool>("laser_proximity_topic", 50); 

    // moving
    start_time = ros::Time(0);


    if (!tf_listener.waitForTransform(laser_frame, world_frame, ros::Time(0), ros::Duration(30.)))
    {
        ROS_WARN_STREAM("[LPC] Timeout (30s) while waiting between " << laser_frame <<
                        " and " << world_frame << " at startup.");
    }
    
    // point cloud publisher
    obst_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("obst_point_cloud", 1);
    
    // closest obstacle point publisher
    closest_obst_point_pub = n.advertise<std_msgs::Float32MultiArray>("closest_obst_point", 1);

    // publisher of obstacle message 
    obst_message_pub = n.advertise<costmap_converter::ObstacleArrayMsg>("obstacles", 1);

    // laser scan subscriber
    point_cloud_scan_sub = n.subscribe("scan_point_cloud", 1, &LaserProximityChecker2::scan_cb, this);

    last_pcl_time_ = std::numeric_limits<double>::lowest();
}


double norm(const geometry_msgs::Vector3& vec3)
{
    return sqrt(vec3.x * vec3.x + vec3.y * vec3.y + vec3.z * vec3.z);
}


/*
 * laser scan callback
 */
void LaserProximityChecker2::scan_cb(const sensor_msgs::PointCloud2& point_cloud)
{
    const double pcl_curr_time = point_cloud.header.stamp.toSec();
    if((pcl_curr_time - last_pcl_time_) < pcl_throttle_) return; 

    last_pcl_time_ = pcl_curr_time;
    ROS_INFO_STREAM("received new input point cloud, time: " << last_pcl_time_ << ", size: " << point_cloud.width << " x " << point_cloud.height);

    // get current robot frame
    tf::StampedTransform current_robot_transform;
    if( tf_listener.waitForTransform(robot_frame, point_cloud.header.frame_id, point_cloud.header.stamp, ros::Duration(1.0)) )
    {
        tf_listener.lookupTransform(robot_frame, point_cloud.header.frame_id, point_cloud.header.stamp, current_robot_transform);
        
        // transform the scan into robot frame
        pcl_ros::transformPointCloud(robot_frame, current_robot_transform, point_cloud, tmp_point_cloud);
        //pcl_ros::transformPointCloud(world_frame, current_robot_transform.inverse(), tmp_point_cloud, obst_point_cloud);
        if( check_proximity(tmp_point_cloud))
        {
            // filtering the obstacle point cloud in order to get rid of outliers (mostly shadow points)
            if(have_laser_intensity_)
            {
                filter_obst_pcl_and_compute_closest_point<pcl::PointXYZI>();
            }
            else
            {
                filter_obst_pcl_and_compute_closest_point<pcl::PointXYZ>();
            }
        }
    
        if(obst_message_pub.getNumSubscribers()>0)
        {
            // generate obstacle message for TEB local planner 
            costmap_converter::ObstacleArrayMsg obstacles_msg;
            const float point_radius = 0.1;
            const float down_sample_leaf_size = 0.1; 
            sensor_msgs::PointCloud2 obst_point_cloud_world;            
            sensor_msgs::PointCloud2 obst_point_cloud_world_downsampled; 

            if( tf_listener.waitForTransform(world_frame, obst_point_cloud.header.frame_id, obst_point_cloud.header.stamp, ros::Duration(1.0)) )
            {
                tf::StampedTransform world_transform;
                tf_listener.lookupTransform(world_frame, obst_point_cloud.header.frame_id, obst_point_cloud.header.stamp, world_transform);

                // transform the scan into robot frame
                pcl_ros::transformPointCloud(world_frame, world_transform, obst_point_cloud, obst_point_cloud_world);

                downsamplePointcloud(down_sample_leaf_size, obst_point_cloud_world, obst_point_cloud_world_downsampled);
                getObstacleMessageFromObstPointcloud(obst_point_cloud_world_downsampled, point_radius, obstacles_msg);
                obst_message_pub.publish(obstacles_msg);   

                obst_point_cloud_pub.publish(obst_point_cloud_world_downsampled);    
            } 
        }
        else
        {
            obst_point_cloud_pub.publish(obst_point_cloud);
            //obst_point_cloud_pub.publish(tmp_point_cloud);                
        }
    }
    else
    {
       ROS_WARN_STREAM("[LPC] Could not get current robot position.");
    }
}

bool LaserProximityChecker2::check_proximity(const sensor_msgs::PointCloud2& ptcld)
{    
    ROS_INFO_STREAM("checking proximity on transformed point cloud of size: " << ptcld.width << " x " << ptcld.height);

    bool b_proximity = false;
    float min_dist_squared = std::numeric_limits<float>::max(); 
    float z_dist_min = 0;
    
    //obst_point_cloud = ptcld;
        
    obst_point_cloud.header = ptcld.header;
    obst_point_cloud.width = 0;
    obst_point_cloud.height = 1;// ptcld.height;     
    obst_point_cloud.fields = ptcld.fields; 
    obst_point_cloud.is_bigendian = ptcld.is_bigendian;    
    obst_point_cloud.point_step = ptcld.point_step;
    //obst_point_cloud.row_step = ptcld.row_step;  // filled below 
    obst_point_cloud.is_dense = ptcld.is_dense; // dense means that there are no invalid points    
    obst_point_cloud.data.clear();
        
    // getting information to parse the point cloud
    const unsigned int pt_step = ptcld.point_step; // size of the point structure
    // offsets of the relevent fields (init values)
    unsigned int pt_x_offset   = 0;
    unsigned int pt_y_offset   = 4;
    unsigned int pt_z_offset   = 8;
    unsigned int pt_int_offset = 12;

    const unsigned int row_step = ptcld.row_step;

    for (unsigned int j = 0; j < ptcld.fields.size(); j++)
    {
        if (!ptcld.fields[j].name.compare("x"))
        {
            pt_x_offset = ptcld.fields[j].offset;
        }
        else if (!ptcld.fields[j].name.compare("y"))
        {
            pt_y_offset = ptcld.fields[j].offset;
        }
        else if (!ptcld.fields[j].name.compare("z"))
        {
            pt_z_offset = ptcld.fields[j].offset;
        }
        else if (!ptcld.fields[j].name.compare("intensity"))
        {
            pt_int_offset = ptcld.fields[j].offset;
        }
    }
    
    // traverse point cloud
    for (size_t j = 0; j < ptcld.height; j++)
    {
        const size_t j_row_step_const = j * row_step;

        for (size_t i = 0; i < ptcld.width; i++)
        {
            float x, y, z; // fields from the point

            const size_t ij_x_pt_step_base = i * pt_step + j_row_step_const;
            
            size_t ij_x_pt_step = ij_x_pt_step_base + pt_x_offset; // i * pt_step + pt_x
            std::copy(reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step]),
                    reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step + 4]),
                    reinterpret_cast<char*> (&x));
            
            ij_x_pt_step = ij_x_pt_step_base + pt_y_offset;
            std::copy(reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step]),
                    reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step + 4]),
                    reinterpret_cast<char*> (&y));
            
            ij_x_pt_step = ij_x_pt_step_base + pt_z_offset;
            std::copy(reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step]),
                    reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step + 4]),
                    reinterpret_cast<char*> (&z));

            if( 
                (z > kZMin) && (z < kZMax) &&
                (x > kXMin) &&                         // filter out close point 
                !( (fabs(y) < kYmin) && (x < kXMin))   // filter out close point 
            )
            {
                float dist_squared = x*x + y*y;
                //std::cout << "distance : " << sqrt(dist_squared) << ", z: " << z << std::endl; 
                if (dist_squared < min_proximity_distance_squared) 
                {
                    b_proximity = true; 
                    if(min_dist_squared > dist_squared) 
                    {
                        min_dist_squared = dist_squared;
                        z_dist_min = z;
                    }
                    // ROS_WARN_STREAM("[LPC] proximity detected ");
                    // std::cout << "distance : " << sqrt(dist_squared) << ", z: " << z << std::endl; 
                    
                    //return true; /// < EXIT POINT 
                    
                    obst_point_cloud.width++; 
                    obst_point_cloud.data.insert(obst_point_cloud.data.end(), reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step_base]), reinterpret_cast<const char*> (&ptcld.data[ij_x_pt_step_base + ptcld.point_step]));
                }
            } 
        }
    }
    obst_point_cloud.row_step = obst_point_cloud.data.size()*obst_point_cloud.height;
    
#if 0
   if(b_proximity)
   {
       const float distance = std::max( sqrt(min_dist_squared) - robot_radius,0.f);
       ROS_WARN_STREAM("[LPC] proximity detected - min distance : " << distance <<", min z: " << z_dist_min << std::endl); 
    //    std_msgs::Bool msg_proximity_status;
    //    msg_proximity_status.data = true;
    //    proximity_pub.publish(msg_proximity_status);
   }
#endif 

    return b_proximity; 

}

template <typename PointIn=pcl::PointXYZI>
void LaserProximityChecker2::filter_obst_pcl_and_compute_closest_point()
{
    //ROS_INFO_STREAM("obst_pcl_filter()"); 
    
    typename pcl::PointCloud<PointIn>::Ptr pcl_cloud_in(new pcl::PointCloud<PointIn>());
    typename pcl::PointCloud<PointIn>::Ptr pcl_cloud_out(new pcl::PointCloud<PointIn>());
    pcl::fromROSMsg(obst_point_cloud, *pcl_cloud_in);

    // create the radius outlier removal filter
    pcl::RadiusOutlierRemoval<PointIn> radius_outlier_removal;

    // set input cloud
    radius_outlier_removal.setInputCloud(pcl_cloud_in);

    // set radius for neighbor search
    radius_outlier_removal.setRadiusSearch(kRadiusFilterRadius);

    // set threshold for minimum required neighbors neighbors
    radius_outlier_removal.setMinNeighborsInRadius(kRadiusFilterMinNeighborsInRadius);

    // do filtering
    radius_outlier_removal.filter(*pcl_cloud_out);
    
    if(!pcl_cloud_out->empty())
    {
        // get closest point 
        typename pcl::KdTree<PointIn>::Ptr tree(new pcl::KdTreeFLANN<PointIn>); 
        tree->setInputCloud(pcl_cloud_out); 
        std::vector<int> nn_indices(1); 
        std::vector<float> nn_dists(1); 
        if( tree->nearestKSearch(PointIn(), 1, nn_indices, nn_dists) > 0)
        {
            PointIn& closest_point = pcl_cloud_out->points[nn_indices[0]];
            const float closest_point_distance = sqrt(closest_point.x*closest_point.x + closest_point.y*closest_point.y);// + closest_point.z*closest_point.z);
            const float distance_from_robot = std::max( closest_point_distance - robot_radius,0.f);

            ROS_WARN_STREAM("[LPC] proximity detected - distance: " << distance_from_robot << std::endl); 
            std_msgs::Bool msg_proximity_status;
            msg_proximity_status.data = true;
            proximity_pub.publish(msg_proximity_status);

            std_msgs::Float32MultiArray closest_point_msg; // <x,y,z,dist>
            closest_point_msg.data.clear();
            closest_point_msg.data.push_back(closest_point.x); 
            closest_point_msg.data.push_back(closest_point.y);
            closest_point_msg.data.push_back(closest_point.z);
            closest_point_msg.data.push_back(closest_point_distance);
            closest_obst_point_pub.publish(closest_point_msg);
        }
    }

    pcl::toROSMsg(*pcl_cloud_out, obst_point_cloud);
}



/*
 * main function
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_proximity_checker");

    LaserProximityChecker2 laser_proximity_checker;

    ros::spin();

    return 0;
}

