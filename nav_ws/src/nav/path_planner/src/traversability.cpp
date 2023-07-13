/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
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

#include <ros/ros.h>
#include <signal.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

//#include <DynamicJoinPcl.h>
#include <ClusterPcl.h>
#include <ConversionPcl.h>
#include <ColorNormalsPcl.h>
#include <MakeNormalsMarkers.h>
#include <TravAnalyzer.h>

#include "KdTreeFLANN.h"
#include "Transform.h"
#include "MultiConfig.h"


ConversionPcl<pcl::PointXYZRGBNormal> conv_pcl;
ClusterPcl<pcl::PointXYZRGBNormal> clustering_pcl;
TravAnalyzer trav_analyzer;
boost::recursive_mutex trav_analyzer_mutex;

ros::Publisher pcl_normal_pub;
ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub_dyn;
ros::Publisher pcl_pub_nowall;
ros::Publisher pcl_pub_wall;
ros::Publisher pcl_pub_borders;
ros::Publisher pcl_pub_segmented;
ros::Publisher pcl_pub_traversability;

ros::Publisher pcl_pub_clearence;
ros::Publisher pcl_pub_density;
ros::Publisher pcl_pub_label;
ros::Publisher pcl_pub_roughness;

ros::Publisher pcl_pub_path_to_avoid[kMaxNumberOfRobots];

ros::Subscriber other_multi_robot_paths_sub[kMaxNumberOfRobots];


boost::recursive_mutex map_pcl_mutex;
pcl::PointCloud<pcl::PointXYZRGBNormal> map_pcl;

pcl::PointCloud<pcl::PointXYZRGBNormal> nowall_pcl;
pcl::PointCloud<pcl::PointXYZRGBNormal> border_pcl;
pcl::PointCloud<pcl::PointXYZRGBNormal> segmented_pcl;
pcl::PointCloud<pcl::PointXYZRGBNormal> wall_pcl;

int number_of_robots = 2; 
int robot_id         = 0;

std::string robot_frame_name;

void setRobotPosition();


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

//void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
//{
//    geometry_msgs::PoseArray poseArray;
//    makeNormalsMarkers(pcl_norm, poseArray);
//    ROS_INFO("normals poseArray size: %ld", poseArray.poses.size());
//    marker_normal_pub.publish(poseArray);
//}

void pointCloudCallback(const sensor_msgs::PointCloud2& map_msg)
{
    ROS_INFO("traversability_node - got new cloud");
    
    pcl::fromROSMsg(map_msg, map_pcl);
    
    if (map_pcl.size() > 0)
    {
        setRobotPosition();
                
        std::vector<int> cluster_info;
        pcl::PointCloud<pcl::PointXYZI> traversability_pcl, clearence_pcl, density_pcl, label_pcl, roughness_pcl;

        /// < perform the clustering and basic segmentation 
        ros::Time time_start_clustering = ros::Time::now(); 
        std::cout << "- clustering -" << std::endl; 
        clustering_pcl.setInputPcl(map_pcl);
        clustering_pcl.clustering(nowall_pcl, border_pcl, segmented_pcl, wall_pcl);
        clustering_pcl.getClusterInfo(cluster_info);
        ros::Duration elapsed_time_clustering = ros::Time::now()-time_start_clustering;
        ROS_INFO_STREAM("clustering time: " << elapsed_time_clustering);

        /// < publish clusters info
        nowall_pcl.header.frame_id = map_msg.header.frame_id;
        wall_pcl.header.frame_id = map_msg.header.frame_id;
        border_pcl.header.frame_id = map_msg.header.frame_id;
        segmented_pcl.header.frame_id = map_msg.header.frame_id;

        sensor_msgs::PointCloud2 nowall_msg, wall_msg, segmented_msg;

        pcl::toROSMsg(nowall_pcl, nowall_msg);
        pcl::toROSMsg(wall_pcl, wall_msg);
        pcl::toROSMsg(segmented_pcl, segmented_msg);

        pcl_pub_nowall.publish(nowall_msg);
        pcl_pub_wall.publish(wall_msg);
        pcl_pub_segmented.publish(segmented_msg);

        /// < perform traversability analysis 
        {   
        boost::recursive_mutex::scoped_lock locker(trav_analyzer_mutex);
        ros::Time time_start_trav = ros::Time::now(); 
        trav_analyzer.setInput(cluster_info, wall_pcl, nowall_pcl);
        trav_analyzer.computeTrav(traversability_pcl);
        trav_analyzer.getPcl(clearence_pcl, density_pcl, label_pcl, roughness_pcl);
        ros::Duration elapsed_time_trav = ros::Time::now()-time_start_trav;
        ROS_INFO_STREAM("traversability time: " << elapsed_time_trav);
        }

        /// < publish traversability info 
        traversability_pcl.header.frame_id = map_msg.header.frame_id;
        density_pcl.header.frame_id = map_msg.header.frame_id;
        label_pcl.header.frame_id = map_msg.header.frame_id;
        roughness_pcl.header.frame_id = map_msg.header.frame_id;
        clearence_pcl.header.frame_id = map_msg.header.frame_id;

        sensor_msgs::PointCloud2 trav_msg_out;
        pcl::toROSMsg(traversability_pcl, trav_msg_out);
        pcl_pub_traversability.publish(trav_msg_out);

        pcl::toROSMsg(clearence_pcl, trav_msg_out);
        pcl_pub_clearence.publish(trav_msg_out);

        pcl::toROSMsg(roughness_pcl, trav_msg_out);
        pcl_pub_roughness.publish(trav_msg_out);

        pcl::toROSMsg(label_pcl, trav_msg_out);
        pcl_pub_label.publish(trav_msg_out);

        pcl::toROSMsg(density_pcl, trav_msg_out);
        pcl_pub_density.publish(trav_msg_out);
        
        for(size_t i=0; i < number_of_robots; i++)
        {
            nav_msgs::Path& path = trav_analyzer.getTeammatePath(i);
            // publish only teammates path 
            if( (i != robot_id) && ( !path.poses.empty() ))
            {
                pcl_pub_path_to_avoid[i].publish(path);
            }
        }
    }
}

void clusteringpclConfigCallback(ClusterPclConfig& config, uint32_t level)
{
    clustering_pcl.setConfig(config);
}

void travConfigCallback(TravAnalyzerConfig& config, uint32_t level)
{
    trav_analyzer.setConfig(config);
}

void laserProximityCallback(const std_msgs::Bool msg)
{
}

void obstPclCallback(const sensor_msgs::PointCloud2& obs_pcl_msg)
{
    sensor_msgs::PointCloud2 obs_pcl_in;
    conv_pcl.transform(obs_pcl_msg, obs_pcl_in); // transform into /map frame
    trav_analyzer.setObstPcl(obs_pcl_in);
}

//void robotToAvoidPathCallback(const nav_msgs::Path& path)
//{
//    trav_analyzer.setOtherRobotPathToAvoid(path);
//}
//
//void otherRobotTransformCallback(const geometry_msgs::TransformStamped& msg)
//{
//    trav_analyzer.setRobotToAvoidPosition(msg);
//}

void multiRobotPoseCallback(const trajectory_control_msgs::MultiRobotPose& msg)
{
    trav_analyzer.setMultiRobotPose(msg);
}

void multiRobotPathsCallback(const trajectory_control_msgs::MultiRobotPath& msg)
{
    boost::recursive_mutex::scoped_lock locker(trav_analyzer_mutex); 
    trav_analyzer.setMultiRobotPath(msg);
}

void setRobotPosition()
{
    /// < check if the current robot position
    Transform transform("map", robot_frame_name);
    tf::StampedTransform robot_pose;
    bool is_ok_transform = false;
    try
    {
        robot_pose = transform.get();
        is_ok_transform = transform.isOk();

    }
    catch (TransformException e)
    {
        ROS_WARN("%s", e.what());
    }
    
    trav_analyzer.setRobotPose(robot_pose,is_ok_transform);
}

void mySigintHandler(int signum)
{
    std::cout << "mySigintHandler()" << std::endl; 
    ros::shutdown(); 
    exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traversability_node");

    //override default sigint handler 
    signal(SIGINT,mySigintHandler); 
    
    tf::TransformListener tf_listener(ros::Duration(10.0));

    ros::NodeHandle n("~");

    /// < get parameters
    std::string str_robot_name   = getParam<std::string>(n, "robot_name", "ugv1");   /// < multi-robot
    std::string str_robot_prefix = "ugv"; 
    robot_id = atoi(str_robot_name.substr(3,str_robot_name.size()).c_str()) - 1;
    trav_analyzer.setRobotId(robot_id);
    trav_analyzer.setRobotName(str_robot_name);    
    
    float robot_radius = getParam<float>(n, "robot_radius", TravAnalyzer::kRobotRadiusDefault);  
    trav_analyzer.setRobotRadius(robot_radius);  
    trav_analyzer.setRobotToAvoidRadius(robot_radius);      

    robot_frame_name = getParam<std::string>(n, "robot_frame_name", "base_link");    
    
    ROS_INFO_STREAM("traversability node of robot " << robot_id << " alive");
    
    number_of_robots = getParam<int>(n, "number_of_robots", 2);   /// < multi-robot
    //std::string robot_to_avoid_frame_id = getParam<std::string>(n, "robot_to_avoid_frame_id", "");   /// < multi-robot
    bool b_enable_multi_robot_avoidance = getParam<bool>(n, "enable_multi_robot_avoidance", false);  /// < multi-robot
    
    if(b_enable_multi_robot_avoidance)  /// < multi-robot
    {
        trav_analyzer.setNumberOfRobots(number_of_robots);
        trav_analyzer.setEnableOtherRobotAvoidance(true); 
        trav_analyzer.initTransformListener();
        
        for(size_t id=0; id<kMaxNumberOfRobots; id++)
        {
            std::stringstream name;
            //name << "/" << str_robot_prefix << id+1 << "/base_link"; 
            name << str_robot_prefix << id+1 << "/base_link"; 
            trav_analyzer.setTeammateBaseFrame(name.str(),id);
        }
    }
    
    /// < dynamic reconfigure 
    dynamic_reconfigure::Server<ClusterPclConfig> clusteringpcl_config_server(ros::NodeHandle("~/ClusteringPcl"));
    clusteringpcl_config_server.setCallback(boost::bind(&clusteringpclConfigCallback, _1, _2));

    dynamic_reconfigure::Server<TravAnalyzerConfig> trav_config_server(ros::NodeHandle("~/TravAnal"));
    trav_config_server.setCallback(boost::bind(&travConfigCallback, _1, _2));

    conv_pcl.setTFListener(tf_listener);
    
    /// < Input 
    ros::Subscriber sub_pcl = n.subscribe("/dynjoinpcl", 1, pointCloudCallback);
    //ros::Subscriber robot_to_avoid_path_sub = n.subscribe("/traj_global_path_other", 1, robotToAvoidPathCallback);   /// < multi-robot
    
    ros::Subscriber laser_proximity_sub = n.subscribe("/laser_proximity_topic", 1, laserProximityCallback);
    
    ros::Subscriber obst_pcl_sub = n.subscribe("/obst_point_cloud", 1, obstPclCallback);
   
    //ros::Subscriber other_robot_transform_sub = n.subscribe("/other_robot_transform_sub", 1, otherRobotTransformCallback);  /// < multi-robot
    
    ros::Subscriber multi_robot_poses_sub = n.subscribe("/multi_robot_poses", 5, multiRobotPoseCallback); /// < multi-robot
    
    ros::Subscriber multi_robot_paths_sub = n.subscribe("/multi_robot_paths", 5, multiRobotPathsCallback); /// < multi-robot
    ros::Subscriber core_multi_robot_paths_sub = n.subscribe("/core/multi_robot_paths", 5, multiRobotPathsCallback); /// < multi-robot
    
    for(size_t id=0; id < kMaxNumberOfRobots; id++)
    {
        if(id != robot_id)
        {
            std::stringstream topic_name;
            topic_name << "/" << str_robot_prefix << id+1 << "/multi_robot_paths";  /// < /ugv"i+1"/multi_robot_paths
            std::cout << "topic name in: " << topic_name.str() << std::endl;
            other_multi_robot_paths_sub[id] = n.subscribe(topic_name.str(), 1, multiRobotPathsCallback); /// < multi-robot
        }
    }

    /// < Ouput
    pcl_pub_nowall = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/no_wall", 1, true);
    pcl_pub_wall   = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/wall", 1, true);

    pcl_pub_traversability = n.advertise<sensor_msgs::PointCloud2>("/trav/traversability", 1, true);
    pcl_pub_clearence = n.advertise<sensor_msgs::PointCloud2>("/trav/clearence", 1, true);
    pcl_pub_density = n.advertise<sensor_msgs::PointCloud2>("/trav/density", 1, true);
    pcl_pub_label = n.advertise<sensor_msgs::PointCloud2>("/trav/label", 1, true);
    pcl_pub_roughness = n.advertise<sensor_msgs::PointCloud2>("/trav/roughness", 1, true);

    pcl_pub_segmented = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/segmented", 1, true);
    pcl_normal_pub = n.advertise<sensor_msgs::PointCloud2>("/normals_pcl", 1, true);
    marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker", 1);
        
    for(size_t id=0; id < kMaxNumberOfRobots; id++)
    {
        std::stringstream topic_name;
        topic_name << "/" << str_robot_prefix << id+1 << "/path_to_avoid";  /// < /ugv"i+1"/path_to_avoid
        std::cout << "topic name out: " << topic_name.str() << std::endl;
        pcl_pub_path_to_avoid[id] = n.advertise<nav_msgs::Path>(topic_name.str(), 1);   /// < multi-robot
    }
    
    ROS_INFO_STREAM("starting the node **********************************");
     
    //ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    //spinner.spin(); // spin() will not return until the node has been shutdown

    ros::spin();
    
    return 0;
}
