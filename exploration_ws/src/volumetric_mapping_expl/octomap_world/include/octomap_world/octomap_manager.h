/*
Copyright (c) 2015, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
#define OCTOMAP_WORLD_OCTOMAP_MANAGER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "octomap_world/octomap_world.h"

#include <octomap_msgs/GetOctomap.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <volumetric_msgs_expl/LoadMap.h>
#include <volumetric_msgs_expl/SaveMap.h>
#include <volumetric_msgs_expl/SetBoxOccupancy.h>
#include <volumetric_msgs_expl/SetDisplayBounds.h>

#include <boost/noncopyable.hpp>

namespace volumetric_mapping
{

    // An inherited class from OctomapWorld, which also handles the connection to
    // ROS via publishers, subscribers, service calls, etc.
    class OctomapManager : public OctomapWorld, private boost::noncopyable
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        typedef std::shared_ptr<OctomapManager> Ptr;

        // By default, loads octomap parameters from the ROS parameter server.
        OctomapManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        void publishAll();
        void publishAllEvent(const ros::TimerEvent &e);

        // Data insertion callbacks with TF frame resolution through the listener.
        void insertDisparityImageWithTf(
            const stereo_msgs::DisparityImageConstPtr &disparity);
        void insertPointcloudWithTf(
            const sensor_msgs::PointCloud2::ConstPtr &pointcloud);

        // Camera info callbacks.
        void leftCameraInfoCallback(const sensor_msgs::CameraInfoPtr &left_info);
        void rightCameraInfoCallback(const sensor_msgs::CameraInfoPtr &right_info);

        // Service callbacks.
        bool resetMapCallback(std_srvs::Empty::Request &request,
                              std_srvs::Empty::Response &response);
        bool publishAllCallback(std_srvs::Empty::Request &request,
                                std_srvs::Empty::Response &response);
        bool getOctomapCallback(octomap_msgs::GetOctomap::Request &request,
                                octomap_msgs::GetOctomap::Response &response);

        bool loadOctomapCallback(volumetric_msgs_expl::LoadMap::Request &request,
                                 volumetric_msgs_expl::LoadMap::Response &response);
        bool saveOctomapCallback(volumetric_msgs_expl::SaveMap::Request &request,
                                 volumetric_msgs_expl::SaveMap::Response &response);

        bool setBoxOccupancyCallback(
            volumetric_msgs_expl::SetBoxOccupancy::Request &request,
            volumetric_msgs_expl::SetBoxOccupancy::Response &response);
        bool setDisplayBoundsCallback(
            volumetric_msgs_expl::SetDisplayBounds::Request &request,
            volumetric_msgs_expl::SetDisplayBounds::Response &response);

        const std::string &GetWorldFrame() const { return world_frame_; }

    public:
        boost::recursive_mutex interaction_mutex; // mutex to be locked for multi-thread interaction

        bool lookupTransform(const std::string &from_frame,
                             const std::string &to_frame, const ros::Time &timestamp,
                             Transformation *transform);

    private:
        // Sets up subscriptions based on ROS node parameters.
        void setParametersFromROS();
        void subscribe();
        void advertiseServices();
        void advertisePublishers();

        bool setQFromParams(std::vector<double> *Q_vec);
        void calculateQ();
        /*bool lookupTransform(const std::string &from_frame,
                             const std::string &to_frame, const ros::Time &timestamp,
                             Transformation *transform);*/

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        //tf::TransformListener tf_listener_;
        boost::shared_ptr<tf::TransformListener> p_tf_listener_;
        double tf_cache_length_; // [s]

        // Global/map coordinate frame. Will always look up TF transforms to this
        // frame.
        std::string world_frame_;

        // Subscriptions for input sensor data.
        ros::Subscriber disparity_sub_;
        ros::Subscriber left_info_sub_;
        ros::Subscriber right_info_sub_;
        ros::Subscriber pointcloud_sub_;

        // Publish full state of octomap.
        ros::Publisher binary_map_pub_;
        ros::Publisher full_map_pub_;

        // Publish markers for visualization.
        ros::Publisher occupied_nodes_pub_;
        ros::Publisher free_nodes_pub_;

        // Services!
        ros::ServiceServer reset_map_service_;
        ros::ServiceServer publish_all_service_;
        ros::ServiceServer get_map_service_;
        ros::ServiceServer save_octree_service_;
        ros::ServiceServer load_octree_service_;
        ros::ServiceServer set_box_occupancy_service_;
        ros::ServiceServer set_display_bounds_service_;

        // Keep state of the cameras.
        sensor_msgs::CameraInfoPtr left_info_;
        sensor_msgs::CameraInfoPtr right_info_;

        // Only calculate Q matrix for disparity once.
        bool Q_initialized_;
        Eigen::Matrix4d Q_;
        Eigen::Vector2d full_image_size_;
        double map_publish_frequency_;
        ros::Timer map_publish_timer_;

        ros::Time time0_;
    };

} // namespace volumetric_mapping

#endif // OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
