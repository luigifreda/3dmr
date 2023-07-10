/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
  
#include <ouster_gazebo_plugins/GazeboRosOusterLaser.h>

#include <algorithm>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/sensors/GpuRaySensor.hh>
#else
#include <gazebo/sensors/RaySensor.hh>
#endif
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>

#if GAZEBO_GPU_RAY
#define RaySensor GpuRaySensor
#define STR_Gpu  "Gpu"
#define STR_GPU_ "GPU "
#else
#define STR_Gpu  ""
#define STR_GPU_ ""
#endif

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosOusterLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosOusterLaser::GazeboRosOusterLaser() : nh_(NULL), min_range_(0), max_range_(0), gaussian_noise_(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosOusterLaser::~GazeboRosOusterLaser(){
    ////////////////////////////////////////////////////////////////////////////////
    // Finalize the controller / Custom Callback Queue
    laser_queue_.clear();
    laser_queue_.disable();
    if (nh_){
        nh_->shutdown();
        delete nh_;
        nh_ = NULL;
    }
    callback_laser_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosOusterLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
    // Load plugin
    RayPlugin::Load(_parent, _sdf);

    // Initialize Gazebo node
    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init();

    // Get the parent ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
    parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
    parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
    if (!parent_ray_sensor_){
        gzthrow("GazeboRosOuster" << STR_Gpu << "Laser controller requires a " << STR_Gpu << "Ray Sensor as its parent.");
    }

    ROS_INFO("Ouster laser plugin : Getting the %sRay sensor parameters.", STR_Gpu);

    robot_namespace_ = "/";
    if (_sdf->HasElement("robotNamespace")){
        robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        ROS_INFO("Ouster laser plugin : Robot namespace set to %s", robot_namespace_.c_str());
    }

    if (!_sdf->HasElement("frameName")){
        ROS_INFO("Ouster laser plugin missing <frameName>, defaults to /world");
        frame_name_ = "/world";
    }
    else{
        frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
        ROS_INFO("Ouster laser plugin : Frame name set to %s", frame_name_.c_str());
    }

    if (!_sdf->HasElement("min_range")){
        ROS_INFO("Ouster laser plugin missing <min_range>, defaults to 0");
        min_range_ = 0;
    }
    else{
        min_range_ = _sdf->GetElement("min_range")->Get<double>();
        ROS_INFO("Ouster laser plugin : Min range set to %f", min_range_);
    }

    if (!_sdf->HasElement("max_range")){
        ROS_INFO("Ouster laser plugin missing <max_range>, defaults to infinity");
        max_range_ = INFINITY;
    }
    else{
        max_range_ = _sdf->GetElement("max_range")->Get<double>();
        ROS_INFO("Ouster laser plugin : Max range set to %f", max_range_); 
    }

    if (!_sdf->HasElement("filter_radius")){
        ROS_INFO("Ouster laser plugin missing <filter_radius>, defaults to min_range");
        filter_radius_ = min_range_;
    }
    else{
        filter_radius_ = _sdf->GetElement("filter_radius")->Get<double>();
        filter_radius_ = std::max(filter_radius_, min_range_); // should be >= min_range_
        ROS_INFO("Ouster laser plugin : Filter radius set to %f", filter_radius_);
    }    

    if (!_sdf->HasElement("topicName")){
        ROS_INFO("Ouster laser plugin missing <topicName>, defaults to /points");
        topic_name_ = "/points";
    }
    else{
        topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
        ROS_INFO("Ouster laser plugin : Topic name set to %s", topic_name_.c_str());
    }

    if (!_sdf->HasElement("gaussianNoise")){
        ROS_INFO("Ouster laser plugin missing <gaussianNoise>, defaults to 0.0, using noise generation based on datasheet");
        gaussian_noise_ = 0;
    }
    else{
        gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
        if(gaussian_noise_ == 0.0) // shouldn't it be compared to epsilon ?
            ROS_INFO("Ouster laser plugin : gaussian Noise set to 0, using noise generation based on datasheet");
        else if(gaussian_noise_ == -1.0)
            ROS_INFO("Ouster laser plugin : gaussian Noise unset");
        else
            ROS_INFO("Ouster laser plugin : gaussian Noise set to %f", gaussian_noise_);
    }

    if (!_sdf->HasElement("organize_cloud")) {
        ROS_INFO("Velodyne laser plugin missing <organize_cloud>, defaults to false");
        organize_cloud_ = false;
    } else {
        organize_cloud_ = _sdf->GetElement("organize_cloud")->Get<bool>();
    }    

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    // Create node handle
    nh_ = new ros::NodeHandle(robot_namespace_);

    // Resolve tf prefix
    std::string prefix;
    nh_->getParam(std::string("tf_prefix"), prefix);
    if (robot_namespace_ != "/"){
        prefix = robot_namespace_;
    }
    boost::trim_right_if(prefix, boost::is_any_of("/"));
    frame_name_ = tf::resolve(prefix, frame_name_);

    // Advertise publisher with a custom callback queue
    if (topic_name_ != ""){
        ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            topic_name_, 1,
            boost::bind(&GazeboRosOusterLaser::ConnectCb, this),
            boost::bind(&GazeboRosOusterLaser::ConnectCb, this),
            ros::VoidPtr(), &laser_queue_);
        pub_ = nh_->advertise(ao);
    }

    // Sensor generation off by default
    parent_ray_sensor_->SetActive(false);

    // Start custom queue for laser
    callback_laser_queue_thread_ = boost::thread(boost::bind(&GazeboRosOusterLaser::laserQueueThread, this));

#if GAZEBO_MAJOR_VERSION >= 7
    ROS_INFO("Ouster %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->VerticalRangeCount());
#else
    ROS_INFO("Ouster %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->GetVerticalRangeCount());
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Subscribe on-demand
void GazeboRosOusterLaser::ConnectCb()
{
    boost::lock_guard<boost::mutex> lock(lock_);
    if (pub_.getNumSubscribers()){
        if (!sub_){
#if GAZEBO_MAJOR_VERSION >= 7
            sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRosOusterLaser::OnScan, this);
#else
            sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(), &GazeboRosOusterLaser::OnScan, this);
#endif
        }
        parent_ray_sensor_->SetActive(true);
    }
    else{
#if GAZEBO_MAJOR_VERSION >= 7
        if (sub_){
            sub_->Unsubscribe();
            sub_.reset();
        }
#endif
        parent_ray_sensor_->SetActive(false);
    }
}

void GazeboRosOusterLaser::OnScan(ConstLaserScanStampedPtr& _msg){
#if GAZEBO_MAJOR_VERSION >= 7
    const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
    const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

    const double maxRange = parent_ray_sensor_->RangeMax();
    const double minRange = parent_ray_sensor_->RangeMin();

    //const int rayCount = parent_ray_sensor_->RayCount();
    const int rangeCount = parent_ray_sensor_->RangeCount();

    const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
    const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();

    const ignition::math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
    const ignition::math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();
#else
    math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
    math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

    const double maxRange = parent_ray_sensor_->GetRangeMax();
    const double minRange = parent_ray_sensor_->GetRangeMin();

    //const int rayCount = parent_ray_sensor_->GetRayCount();
    const int rangeCount = parent_ray_sensor_->GetRangeCount();

    const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
    const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

    const math::Angle verticalMaxAngle = parent_ray_sensor_->GetVerticalAngleMax();
    const math::Angle verticalMinAngle = parent_ray_sensor_->GetVerticalAngleMin();
#endif

    const double yDiff = maxAngle.Radian() - minAngle.Radian();
    const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

    const double MIN_RANGE = std::max(min_range_, minRange);
    const double MAX_RANGE = std::min(max_range_, maxRange);

    // Populate message fields
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());

#define USE_NEW_POINT_CLOUD_VERSION 1
#if !USE_NEW_POINT_CLOUD_VERSION
    const uint32_t POINT_STEP = 32;

    msg.fields.resize(5);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;
    
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;
    
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;
    
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 16; // here there is a jump of two floats 
    msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[3].count = 1;

    msg.fields[4].name = "ring";
    msg.fields[4].offset = 20;
    msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
    msg.fields[4].count = 1;
    const int ring_offset = 20; 
    
#else
    /* from Ouster driver code:
    struct EIGEN_ALIGN16 OusterPointNew {
        PCL_ADD_POINT4D; 
        float intensity;
        uint32_t t;      
        uint16_t reflectivity;
        uint8_t ring;          
        uint16_t ambient;    
        uint32_t range;       
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointNew,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        // use std::uint32_t to avoid conflicting with pcl::uint32_t
        (std::uint32_t, t, t)
        (std::uint16_t, reflectivity, reflectivity)
        (std::uint8_t, ring, ring)
        (std::uint16_t, ambient, ambient)
        (std::uint32_t, range, range)
    ) */
    // NOTE: provide the same contained information, don't worry about matching the exact same structure
    msg.fields.resize(9);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4; // msg.fields[0].offset + "sizeof(sensor_msgs::PointField::FLOAT32)""
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8; // msg.fields[1].offset + "sizeof(sensor_msgs::PointField::FLOAT32)"
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12; // msg.fields[2].offset + "sizeof(sensor_msgs::PointField::FLOAT32)"
    msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[3].count = 1;
    const uint32_t intensity_offset = msg.fields[3].offset;     

    msg.fields[4].name = "t";
    msg.fields[4].offset = 16; // msg.fields[3].offset + "sizeof(sensor_msgs::PointField::FLOAT32)"
    msg.fields[4].datatype = sensor_msgs::PointField::UINT32;
    msg.fields[4].count = 1;    
    const uint32_t time_offset = msg.fields[4].offset; 

    // fake!
    msg.fields[5].name = "reflectivity";
    msg.fields[5].offset = 20; // msg.fields[4].offset + "sizeof(sensor_msgs::PointField::UINT32)"
    msg.fields[5].datatype = sensor_msgs::PointField::UINT16;
    msg.fields[5].count = 1; 

    msg.fields[6].name = "ring";
    msg.fields[6].offset = 22; // msg.fields[5].offset + "sizeof(sensor_msgs::PointField::UINT16)"
    msg.fields[6].datatype = sensor_msgs::PointField::UINT16; // NOTE: it's better to use uint16 that unit8 for safety and robustness 
    msg.fields[6].count = 1;
    const uint32_t ring_offset = msg.fields[6].offset; 

    // fake!
    msg.fields[7].name = "ambient";
    msg.fields[7].offset = 24; // msg.fields[6].offset + "sizeof(sensor_msgs::PointField::UINT16)"
    msg.fields[7].datatype = sensor_msgs::PointField::UINT16;
    msg.fields[7].count = 1;

    msg.fields[8].name = "range";
    msg.fields[8].offset = 26; // msg.fields[7].offset + "sizeof(sensor_msgs::PointField::UINT16)"
    msg.fields[8].datatype = sensor_msgs::PointField::UINT32;
    msg.fields[8].count = 1;

    const uint32_t POINT_STEP = 32; //  26 + "sizeof(sensor_msgs::PointField::UINT32)" + 2 for getting a multiple of 8

#endif 

    msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

    int i, j;
    uint8_t *ptr = msg.data.data();
    for (i = 0; i < rangeCount; i++){
        for (j = 0; j < verticalRangeCount; j++){

            // Range
            double r = _msg->scan().ranges(i + j * rangeCount);
   
            if ((MIN_RANGE >= r) || (r >= MAX_RANGE) ) {
                if (!organize_cloud_) continue;
            }

            // Noise
            if (gaussian_noise_ > 0.0){ // shouldn't it be compared to epsilon ?
                r += gaussianKernel(0, gaussian_noise_);
            }
            else if (gaussian_noise_ != -1.0){
                // Noise set by the datasheet
                double g_noise = gaussianKernel(0, 1.);
                if(r <= 2.){
                    r += 0.03 * g_noise;
                } else if(r <= 20.){
                    r += 0.015 * g_noise;
                } else if(r <= 60.){
                    r += 0.03 * g_noise;
                } else {
                    r += 0.1 * g_noise;
                }
            }

#if USE_NEW_POINT_CLOUD_VERSION
            // Intensity
            // hack to get the cloud coloured (since many visualizations use the intensity field to color the points)
            const double intensity = r;
#else
            // Intensity
            const double intensity = _msg->scan().intensities(i + j * rangeCount);
#endif 

            // Get angles of ray to get xyz for point
            double yAngle = minAngle.Radian();
            double pAngle = verticalMinAngle.Radian();

            if (rangeCount > 1)
            {
                yAngle = i * yDiff / (rangeCount - 1) + minAngle.Radian();
            }

            if (verticalRayCount > 1)
            {
                pAngle = j * pDiff / (verticalRangeCount - 1) + verticalMinAngle.Radian();
            }

            // pAngle is rotated by yAngle:
            //if ((MIN_RANGE < r) && (r < MAX_RANGE))
            if ((filter_radius_ < r) && (r < MAX_RANGE))
            {
                *((float*)(ptr + 0)) = (float)(r * cos(pAngle) * cos(yAngle)); // x
                *((float*)(ptr + 4)) = (float)(r * cos(pAngle) * sin(yAngle)); // y
#if GAZEBO_MAJOR_VERSION > 2
                *((float*)(ptr + 8)) = (float)(r * sin(pAngle)); // z
#else
                *((float*)(ptr + 8)) = (float)(-r * sin(pAngle)); // z
#endif
#if !USE_NEW_POINT_CLOUD_VERSION 
                *((float*)(ptr + 16)) = (float) intensity; // I
#else
                *((float*)(ptr + intensity_offset)) = (float) intensity; // I
#endif 

#if !USE_NEW_POINT_CLOUD_VERSION 
    #if GAZEBO_MAJOR_VERSION > 2
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) j; // ring
    #else
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) verticalRangeCount - 1 - j; // ring
    #endif
#else
    //NOTE: in the new ouster point cloud version we have a single byte (uint16_t) for the ring
    #if GAZEBO_MAJOR_VERSION > 2
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) j; // ring
    #else
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) verticalRangeCount - 1 - j; // ring
    #endif   
#endif 

#if USE_NEW_POINT_CLOUD_VERSION
                *((uint32_t*)(ptr + time_offset)) = 0; // time (offset w.r.t. pointcloud message timestamp)
#endif 
                ptr += POINT_STEP;
            } 
            else if(organize_cloud_)
            { 
                //if out of range, create a NAN point to keep the organization
                *((float*)(ptr + 0)) = nanf(""); //0.; // x
                *((float*)(ptr + 4)) = nanf(""); //0.; // y
#if GAZEBO_MAJOR_VERSION > 2
                *((float*)(ptr + 8)) = nanf(""); //0.; // z
#else
                *((float*)(ptr + 8)) = nanf(""); //0.; // z
#endif
#if !USE_NEW_POINT_CLOUD_VERSION 
                *((float*)(ptr + 16)) = 0. // I
#else
                *((float*)(ptr + intensity_offset)) = nanf(""); //0.; // I
#endif 

#if !USE_NEW_POINT_CLOUD_VERSION 
    #if GAZEBO_MAJOR_VERSION > 2
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) j; // ring
    #else
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) verticalRangeCount - 1 - j; // ring
    #endif
#else
    //NOTE: in the new ouster point cloud version we have a single byte (uint16_t) for the ring
    #if GAZEBO_MAJOR_VERSION > 2
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) j; // ring
    #else
                *((uint16_t*)(ptr + ring_offset)) = (uint16_t) verticalRangeCount - 1 - j; // ring
    #endif   
#endif

#if USE_NEW_POINT_CLOUD_VERSION
                *((uint32_t*)(ptr + time_offset)) = 0; // time (offset w.r.t. pointcloud message timestamp)
#endif 
                ptr += POINT_STEP;
            }         
        }
    }

#if 0
    // Populate message with number of valid points
    msg.point_step = POINT_STEP;
    msg.row_step = (uint32_t) (ptr - msg.data.data());
    msg.height = 1;
    msg.width = msg.row_step / POINT_STEP;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.data.resize(msg.row_step); // Shrink to actual size

#else 

    // Populate message with number of valid points
    msg.data.resize(ptr - msg.data.data()); // Shrink to actual size
    msg.point_step = POINT_STEP;
    msg.is_bigendian = false;
    if (organize_cloud_) {
        msg.width = verticalRangeCount;
        msg.height = msg.data.size() / POINT_STEP / msg.width;
        msg.row_step = POINT_STEP * msg.width;
        msg.is_dense = false;
    } else {
        msg.width = msg.data.size() / POINT_STEP;
        msg.height = 1;
        msg.row_step = msg.data.size();
        msg.is_dense = true;
    }
    
#endif 

    // Publish output
    pub_.publish(msg);
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// Custom callback queue thread
void GazeboRosOusterLaser::laserQueueThread(){
    while (nh_->ok()){
        laser_queue_.callAvailable(ros::WallDuration(0.01));
    }
}

} // namespace gazebo
