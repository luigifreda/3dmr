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
///	\brief A class for assembling 2D laser scan in a 3D point cloud and checking potential near collisions
///	\note 
/// \todo 
///	\date
///	\warning

class LaserProximityChecker
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
    LaserProximityChecker();

protected: 
    
    //! Coloured scan callback function
    void scan_cb(const sensor_msgs::PointCloud2& scan);
    
    //! Reprojection from PointCloud to LaserScan
    void extract_laser_scan(const sensor_msgs::PointCloud2& ptcld);

    //! Get the new scan in the point cloud
    void append_scan(const sensor_msgs::PointCloud2& scan);

    //! Get laser angle from the tf
    double get_laser_angle(const ros::Time &time) const;

    //! Point cloud control callback
    // void ptcld_ctrl_cb(const std_msgs::Bool& on);
    
    bool check_proximity(const sensor_msgs::PointCloud2& ptcld); 
    
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

    //! Previous absolute value of the laser angle (to detect when to publish)
    double previous_angle;

    //! Previous absolute value of the laser angle (to detect when to publish)
    double previous_langle;

    //! Starting time of the new scan
    ros::Time start_time;

    //! base_link transform at first time
    tf::StampedTransform base_transform;

    //! Publish 2d scans when laser's horizontal (default: true)
    bool publish2d;

    //! Publisher for the horizontal scans (default topic: "/scan2d")
    ros::Publisher scan2d_pub;

    //! Relaying laser scans (default: true)
    bool relay_scans;

    //! Publisher for the relayed scans (default topic: "/scan_relay")
    ros::Publisher relay_pub;
    
    //! Publisher for the proximity information (default topic: "laser_proximity_topic")
    ros::Publisher proximity_pub;
    
    //! Last projected planar scan
    sensor_msgs::LaserScan last_scan;

    //! Subscriber to point cloud control (default topic: "/pointcloud_control")
    //ros::Subscriber ptcld_ctrl_sub;

    //! Point cloud control current state
    //bool ptcld_ctrl_on;

    float min_proximity_distance = LaserProximityChecker::kProximityDistanceThreshold+LaserProximityChecker::kRobotRadius; // [m] point-to-point distance to check 
    float min_proximity_distance_squared = NAN;     
};

const float LaserProximityChecker::kRobotRadius = 0.45;  // [m] robot radius for computing the clearance  
const float LaserProximityChecker::kProximityDistanceThreshold = 0.5; // [m] distance point-to-robot 
const float LaserProximityChecker::kProximityDistanceToCheckSquared = pow(LaserProximityChecker::kProximityDistanceThreshold+LaserProximityChecker::kRobotRadius,2); // [m] point-to-point distance to check 
const float LaserProximityChecker::kZMax = 0.5;
const float LaserProximityChecker::kZMin = 0.075;
const float LaserProximityChecker::kXMin = 0.25;     // forward direction control
const float LaserProximityChecker::kYmin = 0.3;      // side direction control
const float LaserProximityChecker::kRadiusFilterRadius = 0.05; // radius for the radius filter 
const float LaserProximityChecker::kRadiusFilterMinNeighborsInRadius = 8;  // radius filter minimum number of neighbors 

/*
 * Constructor
 */
LaserProximityChecker::LaserProximityChecker() :
tf_listener(ros::Duration(60.)),
n_("~")
{
    // frame names
    laser_frame = getParam<std::string>(n_, "laser_frame", "laser");
    robot_frame = getParam<std::string>(n_, "robot_frame", "base_link");
    world_frame = getParam<std::string>(n_, "world_frame", "odom");
    robot_radius = getParam<float>(n_, "robot_radius", kRobotRadius);

    // max number of points
    max_size = getParam<int>(n_, "max_size", 1000000);

    // min radius filter 
    min_proximity_distance = getParam<float>(n_, "min_proximity_distance", LaserProximityChecker::kProximityDistanceThreshold+LaserProximityChecker::kRobotRadius);
    min_proximity_distance_squared = pow(min_proximity_distance, 2);

    // 2d scans
    publish2d = getParam<bool>(n_, "publish2d", false);
    if (publish2d)
    {
        scan2d_pub = n.advertise<sensor_msgs::LaserScan>("scan2d", 50);
    }

    // relay
    relay_scans = getParam<bool>(n_, "relay_scans", false);
    if (relay_scans)
    {
        relay_pub = n.advertise<sensor_msgs::LaserScan>("scan_relay", 50);      
    }
    
    // laser proximity publisher
    proximity_pub = n.advertise<std_msgs::Bool>("laser_proximity_topic", 50); 

    // moving
    start_time = ros::Time(0);

    // initialized so that the first test always fails
    previous_angle = NAN;
    previous_langle = NAN;

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
    point_cloud_scan_sub = n.subscribe("scan_point_cloud", 50, &LaserProximityChecker::scan_cb, this);
    
    /* Deprecated
    if (publish2d||relay_scans) {
            laser_scan_sub = n.subscribe("/scan_filtered", 50,
                            &NiftiLaserAssembler::laserscan_cb, this);
    }*/
    // initialization of reprojected scan
    last_scan.header.frame_id = laser_frame;
    last_scan.angle_min = -2.35619449615;
    last_scan.angle_max = 2.35619449615;
    last_scan.angle_increment = 0.00872664619237;
    last_scan.time_increment = 0.000036968576751;
    last_scan.scan_time = 0.02;
    last_scan.range_min = 0.01;
    last_scan.range_max = 20.0;
    last_scan.ranges.resize(541);
    last_scan.intensities.resize(541);


    // point cloud control subscriber
    //ptcld_ctrl_sub = n.subscribe("pointcloud_control", 50, &LaserProximityChecker::ptcld_ctrl_cb, this);
    //ptcld_ctrl_on = true;
}

/* 
 * Point cloud control callback
 */
// void LaserProximityChecker::ptcld_ctrl_cb(const std_msgs::Bool& on)
// {
//     ROS_INFO_STREAM("[LPC] Point cloud " << (on.data ? "enabled." : "disabled."));
//     ptcld_ctrl_on = on.data;
// }

/*
 * Get the laser angle from the tf_listener
 */
double LaserProximityChecker::get_laser_angle(const ros::Time &time) const
{
    double angle;
    tf::StampedTransform tmp_tf;
    geometry_msgs::Quaternion rot;
    if (!tf_listener.waitForTransform(robot_frame, laser_frame, time, ros::Duration(1.)))
    {
        /*ROS_WARN_STREAM("[LPC] Timeout (1s) while waiting between "<<laser_frame<<
                        " and "<<robot_frame<<" before getting laser angle.");*/
    }
    tf_listener.lookupTransform(robot_frame, laser_frame, time, tmp_tf);


    tf::quaternionTFToMsg(tmp_tf.getRotation(), rot);
    angle = 2. * atan2(rot.x, rot.w) - M_PI;
    if (angle > M_PI)
        angle -= 2. * M_PI;
    else if (angle<-M_PI)
        angle += 2. * M_PI;

    return angle;
}

double norm(const geometry_msgs::Vector3& vec3)
{
    return sqrt(vec3.x * vec3.x + vec3.y * vec3.y + vec3.z * vec3.z);
}


/*
 * Reprojection from PointCloud to LaserScan
 */
void LaserProximityChecker::extract_laser_scan(const sensor_msgs::PointCloud2& ptcld)
{
    // update header
    last_scan.header.stamp = ptcld.header.stamp;

    // reset array values
    unsigned int i;
    for (i = 0; i < 541; i++)
    {
        last_scan.ranges[i] = last_scan.range_max + 1;
        last_scan.intensities[i] = 0;
    }

    // getting information to parse the point cloud
    unsigned int pt_step = ptcld.point_step; // size of the poitn structure
    // offsets of the relevent fields (init values)
    unsigned int pt_x = 0;
    unsigned int pt_y = 4;
    unsigned int pt_z = 8;
    unsigned int pt_int = 12;
    // getting real values
    // TODO check that the size and type are correct
    for (unsigned int j = 0; j < ptcld.fields.size(); j++)
    {
        if (!ptcld.fields[j].name.compare("x"))
        {
            pt_x = ptcld.fields[j].offset;
        }
        else if (!ptcld.fields[j].name.compare("y"))
        {
            pt_y = ptcld.fields[j].offset;
        }
        else if (!ptcld.fields[j].name.compare("z"))
        {
            pt_z = ptcld.fields[j].offset;
        }
        else if (!ptcld.fields[j].name.compare("intensity"))
        {
            pt_int = ptcld.fields[j].offset;
        }
    }

    // traverse point cloud
    for (i = 0; i < ptcld.width; i++)
    {
        float x, y, z, intensity; // fields from the point
        // unpacking values (TODO check endianness)
        std::copy(reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_x]),
                  reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_x + 4]),
                  reinterpret_cast<char*> (&x));
        std::copy(reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_y]),
                  reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_y + 4]),
                  reinterpret_cast<char*> (&y));
        std::copy(reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_z]),
                  reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_z + 4]),
                  reinterpret_cast<char*> (&z));
        std::copy(reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_int]),
                  reinterpret_cast<const char*> (&ptcld.data[i * pt_step + pt_int + 4]),
                  reinterpret_cast<char*> (&intensity));
        // computing angle, index and distance
        const float d = sqrt(x * x + y * y + z * z);
        const float angle = atan2(y, x);
        const int index = static_cast<int> (round((angle - last_scan.angle_min) / last_scan.angle_increment));
        assert((index >= 0)&&(index < 541));

        // setting distance and intensity in point cloud
        last_scan.ranges[index] = d;
        last_scan.intensities[index] = intensity;
    }
}


/*
 * laser scan callback
 */
void LaserProximityChecker::scan_cb(const sensor_msgs::PointCloud2& scan)
{
    double angle;
    try
    {
        angle = get_laser_angle(scan.header.stamp);
    }
    catch (tf::ExtrapolationException e)
    {
        ROS_WARN_STREAM("[LPC] Could not resolve rotating angle of the laser.");
        return;
    }

    // laserscan
    if (publish2d || relay_scans)
    {
        extract_laser_scan(scan);
    }
    
    /// < publish when the laser get horizontal 
    if (publish2d)
    {
        if ((angle * previous_langle <= 0.0) ||
                ((fabs(angle - previous_langle) < 0.5 * M_PI / 180.)&&
                (fabs(angle) < 10 * M_PI / 180.)))
        {
            ROS_DEBUG_STREAM("[LPC] Publishing 2d scan.");
                scan2d_pub.publish(last_scan);
        }
    }
    if (relay_scans)
    {
        relay_pub.publish(last_scan);
    }
    previous_langle = angle;

    
    /// < append a new scan 
    if (fabs(angle) <= M_PI / 2)
    {
        //ROS_INFO_STREAM("[LPC] Got scan in range.");
        if (start_time.isZero()) start_time = scan.header.stamp;
        append_scan(scan);

    }
    
    // get current robot frame
    tf::StampedTransform current_robot_transform;
    if( tf_listener.waitForTransform(robot_frame,world_frame,point_cloud.header.stamp,ros::Duration(1.0)) )
    {
        tf_listener.lookupTransform(robot_frame,world_frame,point_cloud.header.stamp,current_robot_transform);
        
        // transform the scan into robot frame
        pcl_ros::transformPointCloud(robot_frame, current_robot_transform, point_cloud, tmp_point_cloud);
        if( check_proximity(tmp_point_cloud))
        {
            // filtering the obstacle point cloud in order to get rid of outliers (mostly shadow points)
            filter_obst_pcl_and_compute_closest_point();
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
    

    /// < decide if a new point cloud has to be published 
    if ((fabs(previous_angle) < M_PI / 2) && (fabs(angle) >= M_PI / 2))
    {
        // //ROS_INFO_STREAM("[LPC] End");
        // if (ptcld_ctrl_on)
        // {
        //     // could decide to update time stamps here (but require a full
        //     // transform)
        //     /// dynamic_point_cloud_pub.publish(point_cloud);
        //     ROS_DEBUG_STREAM("[LPC] Point cloud published.");
        // }
        // else
        // {
        //     ROS_DEBUG_STREAM("[LPC] Dropping point cloud (disabled).");
        // }
        point_cloud.data.clear();
        point_cloud.width = 0;
        start_time = ros::Time(0);
    }

    /// < if point cloud is full, we publish it
    // TODO decide if relevant
    if (point_cloud.width >= (unsigned) max_size)
    {
        ROS_WARN_STREAM("[LPC] Max_size exceeded, clearing.");
        //point_cloud_pub.publish(point_cloud);
        point_cloud.data.clear();
        point_cloud.width = 0;
    }
    previous_angle = angle;
}

/*
 * Append a scan to the current point cloud
 */
void LaserProximityChecker::append_scan(const sensor_msgs::PointCloud2& scan)
{
    tf::StampedTransform current_transform;
    // Project the scan into the correct space and time reference
    if (point_cloud.width <= 0)
    {
        // transform into /base_link
    //    if( (tf_listener.waitForTransform(robot_frame, scan.header.frame_id, scan.header.stamp, ros::Duration(1)))&&
    //        (tf_listener.waitForTransform(robot_frame, world_frame, scan.header.stamp, ros::Duration(1)))
    //      )
        if( tf_listener.waitForTransform(world_frame, scan.header.frame_id, scan.header.stamp, ros::Duration(1)) )
        {
        //    tf_listener.lookupTransform(robot_frame, scan.header.frame_id, scan.header.stamp, base_transform);
           
        //    // transform the scan into robot_frame
        //    pcl_ros::transformPointCloud(robot_frame, base_transform, scan, point_cloud);
           
        //    // set base transform for next scans
        //    tf_listener.lookupTransform(robot_frame, world_frame, scan.header.stamp, base_transform);
            
            tf_listener.lookupTransform(world_frame, scan.header.frame_id, scan.header.stamp, current_transform);
            
            // transform the scan into world_frame
            pcl_ros::transformPointCloud(world_frame, current_transform, scan, point_cloud);
            
            
            ROS_INFO_STREAM("[LPC] init new scan.");
        }
        else
        {
            ROS_WARN_STREAM("[LPC] Could not initialize new point cloud with new scan.");
        }
    }
    else
    {
        // transformation in /base_link at the original time
        if (tf_listener.waitForTransform(world_frame, scan.header.frame_id, scan.header.stamp, ros::Duration(1)))
        {
        //    tf_listener.lookupTransform(world_frame, scan.header.frame_id, scan.header.stamp, current_transform);
           
        //    // transform the scan into old robot frame
        //    pcl_ros::transformPointCloud(robot_frame, base_transform*current_transform, scan, tmp_point_cloud);
           
        //    // fusion of the clouds
        //    point_cloud.width    += tmp_point_cloud.width;
        //    point_cloud.row_step += tmp_point_cloud.row_step;
        //    point_cloud.data.insert(point_cloud.data.end(), tmp_point_cloud.data.begin(), tmp_point_cloud.data.end());
           
           
            tf_listener.lookupTransform(world_frame, scan.header.frame_id, scan.header.stamp, current_transform);
            
            // transform the scan into world_frame
            pcl_ros::transformPointCloud(world_frame, current_transform, scan, tmp_point_cloud);
            
            // fusion of the clouds
            point_cloud.width    += tmp_point_cloud.width;
            point_cloud.row_step += tmp_point_cloud.row_step;
            point_cloud.data.insert(point_cloud.data.end(), tmp_point_cloud.data.begin(), tmp_point_cloud.data.end());
            point_cloud.header.stamp = scan.header.stamp;
        }
        else
        {
            ROS_WARN_STREAM("[LPC] Could not append scan to current point cloud.");
        }
    }
}


bool LaserProximityChecker::check_proximity(const sensor_msgs::PointCloud2& ptcld)
{    
    bool b_proximity = false;
    float min_dist_squared = std::numeric_limits<float>::max(); 
    float z_dist_min = 0;
    
    //obst_point_cloud = ptcld;
        
    ROS_ASSERT_MSG(ptcld.height==1,"Here, we expect a cloud with height=1");

    obst_point_cloud.header = ptcld.header;
    obst_point_cloud.fields = ptcld.fields; 
    obst_point_cloud.width = 0;
    obst_point_cloud.height = 1; //ptcld.height; 
    obst_point_cloud.is_dense = ptcld.is_dense; 
    obst_point_cloud.point_step = ptcld.point_step;
    //obst_point_cloud.row_step = ptcld.row_step;  // filled below     
    obst_point_cloud.is_bigendian = ptcld.is_bigendian;
    obst_point_cloud.data.clear();
        
    // getting information to parse the point cloud
    unsigned int pt_step = ptcld.point_step; // size of the point structure
    // offsets of the relevent fields (init values)
    unsigned int pt_x_offset   = 0;
    unsigned int pt_y_offset   = 4;
    unsigned int pt_z_offset   = 8;
    unsigned int pt_int_offset = 12;
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
    for (size_t i = 0; i < ptcld.width; i++)
    {
        float x, y, z, intensity; // fields from the point

        size_t i_x_pt_step_const = i * pt_step;
        size_t i_x_pt_step       = i_x_pt_step_const;
        
        i_x_pt_step = i_x_pt_step_const + pt_x_offset; // i * pt_step + pt_x
        std::copy(reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step]),
                  reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step + 4]),
                  reinterpret_cast<char*> (&x));
        
        i_x_pt_step = i_x_pt_step_const + pt_y_offset;
        std::copy(reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step]),
                  reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step + 4]),
                  reinterpret_cast<char*> (&y));
        
        i_x_pt_step = i_x_pt_step_const + pt_z_offset;
        std::copy(reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step]),
                  reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step + 4]),
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
                obst_point_cloud.data.insert(obst_point_cloud.data.end(), reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step_const]), reinterpret_cast<const char*> (&ptcld.data[i_x_pt_step_const + ptcld.point_step]));
            }
        } 
    }
    obst_point_cloud.row_step = obst_point_cloud.data.size()*obst_point_cloud.height;
    
#if 0
   if(b_proximity)
   {
       const float distance = std::max( sqrt(min_dist_squared) - robot_width,0.);
       ROS_WARN_STREAM("[LPC] proximity detected - distance1 : " << distance <<", z: " << z_dist_min << std::endl); 
    //    std_msgs::Bool msg_proximity_status;
    //    msg_proximity_status.data = true;
    //    proximity_pub.publish(msg_proximity_status);
   }
#endif 

    return b_proximity; 

}

void LaserProximityChecker::filter_obst_pcl_and_compute_closest_point()
{
    //ROS_INFO_STREAM("obst_pcl_filter()"); 
    
    typedef pcl::PointXYZI PointIn;
    pcl::PointCloud<PointIn>::Ptr pcl_cloud_in(new pcl::PointCloud<PointIn>());
    pcl::PointCloud<PointIn>::Ptr pcl_cloud_out(new pcl::PointCloud<PointIn>());
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
        pcl::KdTree<PointIn>::Ptr tree(new pcl::KdTreeFLANN<PointIn>); 
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

    LaserProximityChecker laser_proximity_checker;

    ros::spin();

    return 0;
}

