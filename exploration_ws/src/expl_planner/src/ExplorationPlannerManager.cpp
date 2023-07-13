
/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>
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

#include "ExplorationPlannerManager.h"

#include <pcl/filters/voxel_grid.h>


namespace explplanner{

const int ExplorationPlannerManager::kExplPlannerMaxNumAttempts = 5; // max number of times path planner try to compute the path 
    
// N.B: kMinXSizeCropBox and kMinYSizeCropBox should be bigger than the parameter "nbvp/gain/range"
const float ExplorationPlannerManager::kMinXSizeCropBox = 5.1; // [m] minimum size along X axis of the crop box for planning 
const float ExplorationPlannerManager::kMinYSizeCropBox = ExplorationPlannerManager::kMinXSizeCropBox; // [m] minimum size along Y axis of the crop box for planning 
const float ExplorationPlannerManager::kMinZSizeCropBox = ExplorationPlannerManager::kMinXSizeCropBox; // [m] minimum size along Z axis of the crop box for planning 

const float ExplorationPlannerManager::kGainCropBox = 1;
const float ExplorationPlannerManager::kGainCropBox2 = 2;
const float ExplorationPlannerManager::kGainCropBox3 = 3;
const float ExplorationPlannerManager::kGainCropBox4 = 10;

const float ExplorationPlannerManager::kGainXCropBoxPathAligned = 1; // crop box extends from (-kGainXCropBoxPathAligned* delta.norm(), -0.5*kSizeYCropBoxPathAligned, -0.5*kSizeZCropBoxPathAligned, 1) to (kGainXCropBoxPathAligned * delta.norm(), 0.5*kSizeYCropBoxPathAligned, 0.5*kSizeZCropBoxPathAligned, 1)
const float ExplorationPlannerManager::kSizeYCropBoxPathAligned = 3; // 
const float ExplorationPlannerManager::kSizeZCropBoxPathAligned = 3; // 

const double ExplorationPlannerManager::kDistanceThForRemovingPriorityPoint = 1.; // [m]

//const float ExplorationPlannerManager::kTaskCallbackPeriod = 0.5; // [s] the duration period of the task callback 

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

ExplorationPlannerManager::ExplorationPlannerManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    :nh_(nh),
     nh_private_(nh_private), 
     b_wall_info_available_(false), b_traversability_info_available_(false),
     b_utility_2d_info_available_(false),b_abort_(false)
{
    wall_pcl_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    traversability_pcl_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    utility_2d_pcl_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    p_expl_planner_.reset(new ExplorationPlanner(nh,nh_private));
    const ExplParams& expl_params = p_expl_planner_->getParams(); 
    team_model_.setConflictDistance(expl_params.conflictDistance_);
    
    cost_function_type_ = BaseCostFunction::kSimpleCost;//kOriginalCost;
    
    expl_planning_status_ = kNone; 
    
    path_cost_ = -1; 
}

ExplorationPlannerManager::~ExplorationPlannerManager()
{
}

void ExplorationPlannerManager::traversabilityCloudCallback(const sensor_msgs::PointCloud2& traversability_msg)
{
    boost::recursive_mutex::scoped_lock traversability_locker(traversability_mutex_);
    boost::recursive_mutex::scoped_lock locker(utility_2d_mutex_);

    pcl::fromROSMsg(traversability_msg, *traversability_pcl_);
    
    const ExplParams& expl_params = p_expl_planner_->getParams(); 
     
    /// < downsample the cloud 
    if(expl_params.bDownSampleTraversability_)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxelGridFilter;
        //pcl::PointCloud<pcl::PointXYZI> filtered_trav_pcl;
        ROS_INFO_STREAM("ExplorationPlannerManager::traversabilityCloudCallback() - voxelgrid filter - size before downsampling: " << traversability_pcl_->size());
        voxelGridFilter.setInputCloud(traversability_pcl_->makeShared()); // necessary deep copy
        double resolution = expl_params.downsampleTravResolution_;
        voxelGridFilter.setLeafSize(resolution,resolution,resolution); 
        voxelGridFilter.filter(*traversability_pcl_);
        ROS_INFO_STREAM("ExplorationPlannerManager::traversabilityCloudCallback() - voxelgrid filter - size after downsampling: " << traversability_pcl_->size());
    }
    
    
    std::cout << "ExplorationPlannerManager::traversabilityCloudCallback() - pcl size: " << traversability_pcl_->size() << std::endl;
    
    b_traversability_info_available_ = false; // reset
    b_utility_2d_info_available_ = false; // reset utility flag 

    if (traversability_pcl_->size() > 0)
    {
        b_traversability_info_available_ = true;
    }
}

void ExplorationPlannerManager::wallCloudCallback(const sensor_msgs::PointCloud2& wall_msg)
{
    boost::recursive_mutex::scoped_lock wall_locker(wall_mutex_);

    pcl::fromROSMsg(wall_msg, *wall_pcl_);
     
    // this cloud can be empty 
    if(wall_pcl_->size() == 0)
    {
        /// < HACK: add a single far point 
        pcl::PointXYZRGBNormal far_point;
        far_point.x = std::numeric_limits<float>::max(); 
        far_point.y = std::numeric_limits<float>::max(); 
        far_point.z = std::numeric_limits<float>::max(); 
        wall_pcl_->push_back(far_point);
    }
    
    wall_kdtree_.setInputCloud(wall_pcl_); 
    b_wall_info_available_ = true;
}

// N.B: here we assume the pointcloud contains PointXYZI: [x,y,u(x,y),var(x,y)] 
// Moreover, we assume that utility_2d_pcl_ contains info about the points in traversability_pcl_
void ExplorationPlannerManager::utility2DCloudCallback(const sensor_msgs::PointCloud2& utility_msg)
{
    boost::recursive_mutex::scoped_lock locker(utility_2d_mutex_);
    boost::recursive_mutex::scoped_lock traversability_locker(traversability_mutex_);

    pcl::fromROSMsg(utility_msg, *utility_2d_pcl_);
    
    std::cout << "ExplorationPlannerManager::utility2DCloudCallback() - pcl size: " << traversability_pcl_->size() << std::endl;
        
    b_utility_2d_info_available_ = false; // reset 
            
    /// < check if it has the same size of the traversability pcl 
    if( (utility_2d_pcl_->header.stamp != traversability_pcl_->header.stamp) || (utility_2d_pcl_->size() != traversability_pcl_->size()) )
    {
        ROS_WARN("**********************************************************************************************************");
        ROS_WARN("ExplorationPlannerManager::utility2DCloudCallback() - traversability and utility pcls have different sizes or stamp");
        ROS_WARN("**********************************************************************************************************");
        std::cout << "trav stamp: " << traversability_pcl_->header.stamp << ", utility stamp: " << utility_2d_pcl_->header.stamp << std::endl; 
        std::cout << "trav size: " << traversability_pcl_->size() << ", utility size: " << utility_2d_pcl_->size() << std::endl; 
        b_utility_2d_info_available_ = false; 
        return; /// < EXIT POINT 
    }
    
    if (utility_2d_pcl_->size() > 0)
    {
        b_utility_2d_info_available_ = true;
    }
}

//void ExplorationPlannerManager::goalSelectionCallback(geometry_msgs::PoseStamped goal_msg)
//{
//    std::cout << "ExplorationPlannerManager::goalSelectionCallback()" << std::endl;
//        
//    pcl::PointXYZI goal;
//    goal.x = goal_msg.pose.position.x;
//    goal.y = goal_msg.pose.position.y;
//    goal.z = goal_msg.pose.position.z;
//
//    setGoal(goal);
//    
//    ROS_INFO_STREAM("goal selection: ("<< goal.x << "," << goal.y << "," << goal.z << "), goal_selected: " << (int)b_goal_selected_);
//}

void ExplorationPlannerManager::abortCallback(std_msgs::Bool msg)
{
    std::cout << "ExplorationPlannerManager::abortCallback()" << std::endl;
    ROS_INFO("exploration abort");
    
    if(msg.data == 1)
    {
        setAbort(true); 
    }
}

bool ExplorationPlannerManager::isReady() const
{
    return b_traversability_info_available_ && b_wall_info_available_;
}

    
ExplorationPlannerManager::ExplorationPlannerStatus ExplorationPlannerManager::doPlanning()
{        
    boost::recursive_mutex::scoped_lock expl_planning_locker(expl_planner_mutex_);

    std::cout << "ExplorationPlannerManager::doPathPlanning()" << std::endl;
    
    expl_planning_status_ = kNotReady; 
    path_cost_ = -1; 
    
    if (!isReady())
    {
        ROS_WARN("ExplorationPlannerManager::doPathPlanning() - traversability info: %d, wall info: %d, utility info: %d", (int) b_traversability_info_available_, (int) b_wall_info_available_, (int) b_utility_2d_info_available_);
        return kNotReady; /// < EXIT POINT 
    }

    b_abort_ = false; 
    
    int num_failures = 0;
    int crop_step = 0;
    
    /// < update robot position
    pcl::PointXYZI robot_position;
    bool is_transform_ok = getRobotPosition(robot_position);
    if(!is_transform_ok) 
    {
        expl_planning_status_ = kTransformFailure; 
        return kTransformFailure; /// < EXIT POINT 
    }
    
    //ROS_INFO_STREAM("ExplorationPlannerManager::doPathPlanning() - current robot position: " << robot_position); 
        
    /// < prepare intial traversability PCL (needed for computing the starting point )
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_traversability_pcl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_utility_2d_pcl;
    ExplorationPlanner::KdTreeFLANN traversability_kdtree;

    p_traversability_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
    p_utility_2d_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
    {
        boost::recursive_mutex::scoped_lock locker_traversability(traversability_mutex_);
        boost::recursive_mutex::scoped_lock locker_utility(utility_2d_mutex_); 
        
        if(b_utility_2d_info_available_)
        {
            cropTwoPcls((CropBoxMethod)crop_step, robot_position, traversability_pcl_, utility_2d_pcl_, p_traversability_pcl, p_utility_2d_pcl);
        }
        else
        {
            cropPcl((CropBoxMethod)crop_step, robot_position, traversability_pcl_, p_traversability_pcl);
        }
    }
    
    std::cout << "traversability pcl size: " << p_traversability_pcl->size() << std::endl;
    
    // organize cropped traversability points
    traversability_kdtree.setInputCloud(p_traversability_pcl);
    
    /// < compute starting point on the segment traversability map 
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    /// < HACK : here we do not check the distance in order to avoid problems with outlier obstacle points or holes (this must be fixed)
    int found = traversability_kdtree.nearestKSearch(robot_position, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    //int found = traversability_kdtree.radiusSearch(start_position, PathPlanner::kGoalAcceptCheckThreshold, pointIdxNKNSearch, pointNKNSquaredDistance);
    if (found < 1)
    {
        ROS_WARN("ExplorationPlannerManager::doPathPlanning() - cannot find a close starting node");
        expl_planning_status_ = kInputFailure; 
        return kInputFailure; /// < EXIT POINT 
    }

    /// < check if we have reached any priority point and remove it
    geometry_msgs::Point ref_point;
    ref_point.x = robot_position.x;
    ref_point.y = robot_position.y;
    ref_point.z = robot_position.z;
    removed_priority_points_.clear();
    bool b_removed_points = priority_queue_.RemoveClosePoints(ref_point, kDistanceThForRemovingPriorityPoint, removed_priority_points_); 
    if(b_removed_points)
    {
        ROS_INFO("ExplorationPlannerManager::doPathPlanning() - reached and removed some priority points");  
    }
    
    /// < start planning 
    bool b_successful_planning = false;    

    while ((num_failures < kExplPlannerMaxNumAttempts) && (!b_successful_planning) && (!b_abort_))
    {
        /// < first set input then the bias!
        {
            boost::recursive_mutex::scoped_lock locker(traversability_mutex_);
            boost::recursive_mutex::scoped_lock wall_locker(wall_mutex_);
            boost::recursive_mutex::scoped_lock utility_locker(utility_2d_mutex_);
            p_expl_planner_->setInput(*p_traversability_pcl, *wall_pcl_, wall_kdtree_, traversability_kdtree, pointIdxNKNSearch[0]);
            if(b_utility_2d_info_available_) p_expl_planner_->set2DUtility(*p_utility_2d_pcl);
        }
        
        /// < check if we have a priority point to bias the exploration 
        pcl::PointXYZI bias_in;
        if(!priority_queue_.IsEmpty())
        {         
            //geometry_msgs::Point& priority_point = priority_queue_.TopPoint();
            PriorityPointQueue::PriorityPoint priority_point = priority_queue_.Top();
            bias_in.x = priority_point.data.x;
            bias_in.y = priority_point.data.y;
            bias_in.z = priority_point.data.z;
            ROS_INFO_STREAM("ExplorationPlannerManager::doPathPlanning() - using priority point " << priority_point.id);                   
            p_expl_planner_->setExplorationBias(bias_in, /*use exploration bias*/true);
        }
        else
        {
            // queue is empty => disable exploration bias 
            p_expl_planner_->setExplorationBias(bias_in, /*use exploration bias*/false);
            ROS_INFO_STREAM("ExplorationPlannerManager::doPathPlanning() - priority queue empty ");                         
        }        
        
        // bool bEnableBacktracking = crop_step > 0; // enable backtracking only after the first crop_step fails        
        bool bEnableBacktracking = ( crop_step == (kNumCropBoxMethod-1) ); // enable backtracking only at the last crop_step 
        
        /// < compute the path
        b_successful_planning = p_expl_planner_->planning(path_, bEnableBacktracking);
        if(!b_successful_planning)
        {
            ROS_INFO("ExplorationPlannerManager::doPathPlanning() - #failed attempts %d, crop step %d", num_failures, crop_step);

            num_failures++;
            crop_step = std::min(crop_step + 1, ((int) kNumCropBoxMethod)-1); // saturate since the number of attempts can be higher than then number of crop methods

            if (crop_step < kNumCropBoxMethod)
            {
                is_transform_ok = getRobotPosition(robot_position);
                if(!is_transform_ok) 
                {
                    expl_planning_status_ = kTransformFailure; 
                    return kTransformFailure; /// < EXIT POINT 
                }
                    
                /// < generate a bigger traversability map           
                p_traversability_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
                p_utility_2d_pcl.reset(new pcl::PointCloud<pcl::PointXYZI>);
                {
                    boost::recursive_mutex::scoped_lock locker(traversability_mutex_);
                    boost::recursive_mutex::scoped_lock locker_utility(utility_2d_mutex_); 

                    if(b_utility_2d_info_available_)
                    {
                        cropTwoPcls((CropBoxMethod)crop_step, robot_position, traversability_pcl_, utility_2d_pcl_, p_traversability_pcl, p_utility_2d_pcl);
                    }
                    else
                    {
                        cropPcl((CropBoxMethod)crop_step, robot_position, traversability_pcl_, p_traversability_pcl);
                    }
                    
                }
                // organize cropped traversability points
                traversability_kdtree.setInputCloud(p_traversability_pcl);
                
                /// compute starting point on the segment traversability map 
                int found = traversability_kdtree.nearestKSearch(robot_position, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
                if (found < 1)
                {
                    ROS_WARN("ExplorationPlannerManager::doPathPlanning() - cannot find a close starting node - aborting");
                    b_abort_ = true;  
                }
                
            }
        }

    } // end while
    
     
    explplanner::ExplorationPlanner::ExplorationStepType explorationStepType = p_expl_planner_->getExplorationStepType();
    
    if(b_successful_planning)
    {
        expl_planning_status_ = kSuccess;      
        path_cost_ = computePathLength(path_);
        
        ROS_INFO("ExplorationPlannerManager::doPathPlanning() - path successfully computed - #failed attempts %d, crop step %d", num_failures, crop_step);
    }
    else
    {
        // translate the 'exploration planner' status in an 'exploration planner manager' status 
        switch(explorationStepType)
        {
            case ExplorationPlanner::ExplorationStepType::kCompleted :
                expl_planning_status_ = ExplorationPlannerStatus::kCompleted;    
                ROS_INFO("ExplorationPlannerManager::doPathPlanning() - exploration completed");                    
                break;
                
            case ExplorationPlanner::ExplorationStepType::kNoInformation :
                expl_planning_status_ = ExplorationPlannerStatus::kNoInformation;    
                ROS_INFO("ExplorationPlannerManager::doPathPlanning() - no information");                    
                break;
                
            default: // all other cases 
                expl_planning_status_ = ExplorationPlannerStatus::kFailure;    
                ROS_INFO("ExplorationPlannerManager::doPathPlanning() - cannot find new view point - #failed attempts %d, crop step %d", num_failures, crop_step);                
                
        }
        std::string path = "3dmr_devel";        
        int ret_cmd = system(("mkdir -p ~/.ros/" + path).c_str());
        std::string filename = path + "/status_expl.txt";
        std::ofstream file;
        file.open(filename, std::ios_base::in);
        file << "STATUS=" << expl_planning_status_ << std::endl;     
        file.close();
    }
    
    if(b_abort_) 
    {
        ROS_INFO("ExplorationPlannerManager::doPathPlanning() - path aborted");
        expl_planning_status_ = kAborted;
    }

    return expl_planning_status_;
}

void ExplorationPlannerManager::insertOtherUgvPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{       
    //boost::recursive_mutex::scoped_lock expl_planning_locker(expl_planner_mutex_);
    // < N.B.: don't need to lock a mutex since ExplorationPlanner has an internal lock for the octomap manager      
    p_expl_planner_->insertOtherUgvPointcloudWithTf(pointcloud);
    
}

void ExplorationPlannerManager::insertPriorityPoint(int id, const geometry_msgs::Point& point, int priority)
{    
    priority_queue_.Push(id, point, priority);
}

void ExplorationPlannerManager::removePriorityPoint(int id)
{
    priority_queue_.Erase(id);
}

void ExplorationPlannerManager::clearPriorityPointQueue()
{
    priority_queue_.Clear();
}

bool ExplorationPlannerManager::getRobotPosition(pcl::PointXYZI& robot_position)
{
    std::cout << "ExplorationPlannerManager::getRobotPosition()" << std::endl;
    try
    {
        robot_pose_ = transform_robot_.get();
    }
    catch(TransformException e )
    {
	ROS_WARN("ExplorationPlannerManager::getRobotPosition() - %s",e.what());
    }
    
    robot_position.x = robot_pose_.getOrigin().x();
    robot_position.y = robot_pose_.getOrigin().y();
    robot_position.z = robot_pose_.getOrigin().z();
    
    {
    boost::recursive_mutex::scoped_lock expl_planning_locker(expl_planner_mutex_);        
    p_expl_planner_->setRobotPosition(robot_position.x,robot_position.y,robot_position.z);
    }
    
    ROS_INFO_STREAM("robot position: ("<< robot_position.x << "," << robot_position.y << "," << robot_position.z << ")");
    return transform_robot_.isOk(); 
}


void ExplorationPlannerManager::cropPcl(const CropBoxMethod& crop_box_method, 
                                        const pcl::PointXYZI& start, 
                                        const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in, 
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out)
{
    if( (crop_box_method == kCropBoxTakeAll) )// || (crop_box_method == kCropBoxTakeAll2) )
    {
        *pcl_out = *pcl_in;
        return; /// < EXIT POINT 
    }

    // crop box initialization: traversability
    pcl::CropBox<pcl::PointXYZI> cropbox_pcl;
    cropbox_pcl.setInputCloud(pcl_in); 

    // prepare for cropping: compute cloud translation
    Eigen::Vector3f p1(start.x, start.y, start.z);
    Eigen::Vector3f& middle_point = p1;
    Eigen::Translation<float, 3> translation(-middle_point(0), -middle_point(1), -middle_point(2)); // bring the origin in the middle of the segment 

    Eigen::Affine3f transform;

    switch (crop_box_method)
    {

    case kCropBoxMethodWorldAligned:
    case kCropBoxMethodWorldAligned2:
    case kCropBoxMethodWorldAligned3:        
    case kCropBoxMethodWorldAligned4:
    {
        // prepare for cropping: compute cloud transformation (this brings all the points in the world-aligned frame centered in middle_point)
        transform = translation;

    //    float dx = std::max(1.f * kMinXSizeCropBox, (float) fabs(delta[0])); // 1.f* is for compiling with debug flags 
    //    float dy = std::max(1.f * kMinYSizeCropBox, (float) fabs(delta[1]));
    //    float dz = std::max(1.f * kMinZSizeCropBox, (float) fabs(delta[2]));        
        float dx = 1.f * kMinXSizeCropBox;
        float dy = 1.f * kMinYSizeCropBox;
        float dz = 1.f * kMinZSizeCropBox;

        float gain = 1;
        switch(crop_box_method)
        {
            case kCropBoxMethodWorldAligned:
                gain = kGainCropBox; 
                break; 
                
            case kCropBoxMethodWorldAligned2:
                gain = kGainCropBox2; 
                break;      
                
            case kCropBoxMethodWorldAligned3:
                gain = kGainCropBox3; 
                break;       

            case kCropBoxMethodWorldAligned4:
                gain = kGainCropBox4; 
                break;  

            default:
                ROS_ERROR_STREAM("unmanaged crop_box_method");
        }        

        // apply crop box filter to the traversability cloud w.r.t. world aligned bounding box 
        cropbox_pcl.setMin(Eigen::Vector4f(-0.5 * gain*dx, -0.5 * gain*dy, -0.5 * gain*dz, 1));
        cropbox_pcl.setMax(Eigen::Vector4f(0.5 * gain*dx, 0.5 * gain*dy, 0.5 * gain*dz, 1));
        cropbox_pcl.setTranslation(Eigen::Vector3f(0, 0, 0));
        cropbox_pcl.setRotation(Eigen::Vector3f(0, 0, 0));
        cropbox_pcl.setTransform(transform); // set a transformation that should be applied to the cloud before filtering
        cropbox_pcl.filter(*pcl_out); // filter and write the output in segment->traversability_pcl
    }
        break;


    case kCropBoxTakeAll:
    //case kCropBoxTakeAll2:
    {
        /// < managed above 
    }
        break;

    default:
        ROS_ERROR("Unknown cropbox method");

    }

}


bool ExplorationPlannerManager::cropTwoPcls(const CropBoxMethod& crop_box_method, 
                                            const pcl::PointXYZI& start, 
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in, 
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in2,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out, 
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out2)
{
    bool res_ok = true; 
    if( (crop_box_method == kCropBoxTakeAll) ) //|| (crop_box_method == kCropBoxTakeAll2) )
    {
        *pcl_out  = *pcl_in;
        *pcl_out2 = *pcl_in2;
        return true; /// < EXIT POINT 
    }

    // crop box initialization: traversability
    pcl::CropBox<pcl::PointXYZI> cropbox_pcl;
    cropbox_pcl.setInputCloud(pcl_in); 
    
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(pcl_in); 
    pcl::PointIndices::Ptr indices_filtered (new  pcl::PointIndices());

    // prepare for cropping: compute cloud translation
    Eigen::Vector3f p1(start.x, start.y, start.z);
    Eigen::Vector3f& middle_point = p1;
    Eigen::Translation<float, 3> translation(-middle_point(0), -middle_point(1), -middle_point(2)); // bring the origin in the middle of the segment 

    Eigen::Affine3f transform;
    
    switch (crop_box_method)
    {

    case kCropBoxMethodWorldAligned:
    case kCropBoxMethodWorldAligned2:
    case kCropBoxMethodWorldAligned3:
    case kCropBoxMethodWorldAligned4:        
    {
        // prepare for cropping: compute cloud transformation (this brings all the points in the world-aligned frame centered in middle_point)
        transform = translation;

//        float dx = std::max(1.f * kMinXSizeCropBox, (float) fabs(delta[0])); // 1.f* is for compiling with debug flags 
//        float dy = std::max(1.f * kMinYSizeCropBox, (float) fabs(delta[1]));
//        float dz = std::max(1.f * kMinZSizeCropBox, (float) fabs(delta[2]));
        float dx = 1.f * kMinXSizeCropBox; // 1.f* is for compiling with debug flags 
        float dy = 1.f * kMinYSizeCropBox;
        float dz = 1.f * kMinZSizeCropBox;        

        float gain = 1;
        switch(crop_box_method)
        {
            case kCropBoxMethodWorldAligned:
                gain = kGainCropBox; 
                break; 
                
            case kCropBoxMethodWorldAligned2:
                gain = kGainCropBox2; 
                break;      
                
            case kCropBoxMethodWorldAligned3:
                gain = kGainCropBox3; 
                break;  

            case kCropBoxMethodWorldAligned4:
                gain = kGainCropBox4; 
                break;  

            default:
                ROS_ERROR_STREAM("unmanaged crop_box_method");
        }

        // apply crop box filter to the traversability cloud w.r.t. world aligned bounding box 
        cropbox_pcl.setMin(Eigen::Vector4f(-0.5 * gain*dx, -0.5 * gain*dy, -0.5 * gain*dz, 1));
        cropbox_pcl.setMax(Eigen::Vector4f(0.5 * gain*dx, 0.5 * gain*dy, 0.5 * gain*dz, 1));
        cropbox_pcl.setTranslation(Eigen::Vector3f(0, 0, 0));
        cropbox_pcl.setRotation(Eigen::Vector3f(0, 0, 0));
        cropbox_pcl.setTransform(transform); // set a transformation that should be applied to the cloud before filtering
        
        //cropbox_pcl.filter(*pcl_out); // filter and write the output in segment->traversability_pcl
        cropbox_pcl.filter(indices_filtered->indices); // filter and write the output in indices_filtered->indices
        extract.setIndices (indices_filtered);   
    }
    break;


    case kCropBoxTakeAll:
    {
        /// < managed above 
    }
    break;

    default:
        ROS_ERROR("Unknown cropbox method");
        res_ok = false; 
    }
    
    /// < apply the filter 
    if(res_ok)
    {
        extract.filter(*pcl_out);

        extract.setInputCloud(pcl_in2);         
        extract.setIndices (indices_filtered);
        extract.filter(*pcl_out2);        
    }
    return res_ok;
} 

void ExplorationPlannerManager::setParamExplorationBoxXY(const double minX, const double maxX, const double minY, const double maxY)
{
    p_expl_planner_->setParamExplorationBoxXY(minX, maxX, minY, maxY);
}

void ExplorationPlannerManager::setCostFunctionType(int type, double lamda_trav, double lambda_aux_utility)
{
    boost::recursive_mutex::scoped_lock path_planning_locker(expl_planner_mutex_);
    
    p_expl_planner_->setCostFunctionType(type, lamda_trav, lambda_aux_utility);
}

 void ExplorationPlannerManager::setMyRobotId(int my_robot_id) 
 { 
     team_model_.setMyRobotId(my_robot_id); 
     
     p_expl_planner_->setRobotId(my_robot_id);
 }    

double ExplorationPlannerManager::computePathLength(nav_msgs::Path& path)
{
    double d_estimated_distance_ = 0;
    int input_path_length = path.poses.size();
    for (int ii = 0; ii < (input_path_length - 1); ii++)
    {
        d_estimated_distance_ += sqrt(pow(path.poses[ii + 1].pose.position.x - path.poses[ii].pose.position.x, 2) +
                                      pow(path.poses[ii + 1].pose.position.y - path.poses[ii].pose.position.y, 2) +
                                      pow(path.poses[ii + 1].pose.position.z - path.poses[ii].pose.position.z, 2));
    }
    return d_estimated_distance_;
}


void ExplorationPlannerManager::setRobotMessage(const exploration_msgs::ExplorationRobotMessage::ConstPtr msg, bool isMsgDirect )
{
    team_model_.setRobotMessage(msg, isMsgDirect);
}

void ExplorationPlannerManager::setRobotData(int robot_id, const Eigen::Vector3d& goal, const nav_msgs::Path& path, const double path_cost, const ros::Time& timestamp)
{
    team_model_.setRobotPlanData(robot_id, goal, path, path_cost, timestamp);    
}

bool ExplorationPlannerManager::IsNodeConflict()
{
    return team_model_.IsNodeConflict();
}

void ExplorationPlannerManager::mapMessageOverlapCheck(const geometry_msgs::PoseArray& message, std::vector<sensor_msgs::PointCloud2::Ptr>& cloudsToSend)
{
    p_expl_planner_->mapMessageOverlapCheck(message, cloudsToSend);
}    

}
