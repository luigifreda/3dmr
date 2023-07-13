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

#include <NormalEstimationPcl.h>
#include <ZTimer.h>
#include <limits>
#include <queue>


#define USE_DIRECTION_PROPAGATION 1
#define USE_PSEUDONORMALS_AS_NORMALS 0
#define USE_LASER_TRAJECTORIES 1 


#define NORM3_L2(x, y, z) (sqrt(pow((x), 2) + pow((y), 2) + pow((z), 2)))
#define NORM3_L2_2(x, y, z) (x*x + y*y + z*z)
#define VALID_VECTOR(x,y,z) (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))

#define LOG_NORMALS_STATISTICS 1

#ifdef LOG_TIMES
#define LOG_TIME(obj, name, ...) (obj).measureAndLog(name, __VA_ARGS__) 
#else
#define LOG_TIME(obj, name, ...)
#endif

static const Eigen::Vector3f zAxis(0.0f,0.0f,1.0f);
    
template<typename PointT>
const float NormalEstimationPcl<PointT>::kLaserZOffset = 0.4; // [m] how much each laser view point is pushed higher for the pseudo-normals evaluation
template<typename PointT>
const float NormalEstimationPcl<PointT>::kLaserZWrtBody = 0.2; // [m] actual delta_z between laser view point and base_link
template<typename PointT>
const int NormalEstimationPcl<PointT>::kMinNumNeighboursForComputingNormal = 5; // minimum number of neighbours for computing the normal 
template<typename PointT>
const int NormalEstimationPcl<PointT>::kMaxNumNeighboursForComputingNormal = 10; // maximum number of neighbours for computing the normal 

template<typename PointT>
const int NormalEstimationPcl<PointT>::kNumInitialClosePointsForStartingPropagation = 10; // was initially just one     

template<typename PointT>
const float NormalEstimationPcl<PointT>::kMinPercentDoneWithDirectPropagation = 95; 

template<typename PointT>
const float NormalEstimationPcl<PointT>::kTrajDownsampleDistance = 0.3; // [m]
template<typename PointT>
const float NormalEstimationPcl<PointT>::kTrajDownsampleDistance2 = NormalEstimationPcl<PointT>::kTrajDownsampleDistance * NormalEstimationPcl<PointT>::kTrajDownsampleDistance;

template<typename PointT>
const float NormalEstimationPcl<PointT>::kMinCosToPropagate= cos((90-40)*M_PI/180.);

template<typename PointT>
NormalEstimationPcl<PointT>::NormalEstimationPcl(): threshold_(0.5)
{
    pcl_laser_trajectory_.reset(new PointCloudT());

    robot_id_         = 0;
    number_of_robots_ = 0;    
    
    map_frame_ = "map";     
    
    last_laser_center_.resize(kMaxNumberOfRobots);
    for(size_t ii=0; ii<kMaxNumberOfRobots; ii++)
    {
        last_laser_center_[ii].x = std::numeric_limits<float>::max();
        last_laser_center_[ii].y = std::numeric_limits<float>::max();
        last_laser_center_[ii].z = std::numeric_limits<float>::max();               
    }
}

template<typename PointT>
NormalEstimationPcl<PointT>::~NormalEstimationPcl()
{
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::kernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
#if 0    
    switch (config_.kernel_type)
    {
    case NormalEstimationPcl_Gaussian: return gaussianKernel(v1, v2);
    case NormalEstimationPcl_Cosine: return cosineKernel(v1, v2);
    default: return 0;
    }
#else
    return gaussianKernel(v1, v2);
#endif 
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::gaussianKernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    //Implement the standard gaussian kernel: k(v1,v2) = exp(-||v1-v2||^2 / h)
    return exp(-(v1 - v2).squaredNorm() / config_.smoothing);
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::cosineKernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    return pow((1 + cos(M_PI * (v1 - v2).norm() / config_.radius))*0.5, config_.smoothing);
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeCovarianceMatrix(const PointCloudT& neighbors, const pcl::PointXYZ& center, Eigen::Matrix3f& covariance_matrix)
{
    const Eigen::Vector3f c(center.x, center.y, center.z);
    covariance_matrix.setZero();
    for (size_t i = 0, iEnd = neighbors.size(); i < iEnd; i++)
    {
        const Eigen::Vector3f v(neighbors[i].x, neighbors[i].y, neighbors[i].z);
        const Eigen::Vector3f diff = v - c;
        covariance_matrix += kernel(v, c) * (diff * diff.transpose());
    }
    covariance_matrix /= neighbors.size();
}

template<typename PointT>
bool NormalEstimationPcl<PointT>::computeNormal(const size_t i, const size_t begin, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center)
{    
    if (done[i]) return true;
    
    //std::cout << "computeNormal " << i << std::endl; 
    
#if USE_LASER_TRAJECTORIES 
    const size_t center_idx = closest_laser_point_idx_[i];
    const auto& point_laser_center = (*pcl_laser_trajectory_)[center_idx];
    const Eigen::Vector3f laser_center(point_laser_center.x, point_laser_center.y, point_laser_center.z);
    Eigen::Vector3f pseudonormal( laser_center - Eigen::Vector3f(pcl[i].x, pcl[i].y, pcl[i].z));  
    pseudonormal.normalize();
#else
    Eigen::Vector3f pseudonormal(Eigen::Vector3f(center.x, center.y, center.z) - Eigen::Vector3f(pcl[i].x, pcl[i].y, pcl[i].z));    
    //Eigen::Vector3f pseudonormal(0.0f,0.0f,1.0f);
#endif 
      

#if !USE_PSEUDONORMALS_AS_NORMALS
    //Find neighbors of point
    std::vector<int> pointIdxSearch;
    std::vector<float> pointSquaredDistance;
    int found = 0;
    {
    //boost::recursive_mutex::scoped_lock locker(kdtree_mutex_);
    found = kdtree.radiusSearch(pcl[i], config_.radius, pointIdxSearch, pointSquaredDistance);
    }
    
# if 0    
    if(found < kMinNumNeighboursForComputingNormal and found > 3)  // if we have a least two neighbors then let's find the first close kMinNumNeighboursForComputingNormal
    {
        //boost::recursive_mutex::scoped_lock locker(kdtree_mutex_);        
        found = kdtree.nearestKSearch(pcl[i], kMinNumNeighboursForComputingNormal, pointIdxSearch, pointSquaredDistance);
    }
#endif     
    
    //If neighbors are more than kMinNumNeighboursForComputingNormal compute the covariance_matrix    
    if(found > kMinNumNeighboursForComputingNormal)
    {   
        
#if 0
        /* if there are too many neighbors, try to downsample at random: */
        size_t old_size = pointIdxSearch.size(), num_removed = 0;
        while (pointIdxSearch.size() > kMaxNumNeighboursForComputingNormal)
        {
            size_t index = rand() % pointIdxSearch.size();
            //std::swap(*(pointIdxRadiusSearch.begin() + index), pointIdxRadiusSearch.back());
            //pointIdxRadiusSearch.pop_back();
            //std::swap(*(pointRadiusSquaredDistance.begin() + index), pointRadiusSquaredDistance.back());
            //pointRadiusSquaredDistance.pop_back();
            pointIdxSearch.erase(pointIdxSearch.begin() + index);
            pointSquaredDistance.erase(pointSquaredDistance.begin() + index);
            //num_removed++;
        }
        /*if (num_removed)
        {
            ROS_INFO("Random downsampling removed %d points (initial neighbor count was %d)", num_removed, old_size);
        }*/
#endif

        ZTimer ztimer;

        //cout<<"neghbors size:"<<pointIdxRadiusSearch.size()<<endl;
        
        bool bCanAlignToNeighbor = false; 
        size_t indexNeighborToAlign = 0; 

        PointCloudT neighbors;
        pcl::PointXYZ baricenter(0.0, 0.0, 0.0);
        const size_t neighborhood_size = pointIdxSearch.size();
        for (size_t j = 0, jEnd = neighborhood_size; j < jEnd; j++)  
        {
            const size_t& neighbor_j = pointIdxSearch[j];            
            const PointT& point_j = pcl[neighbor_j];
            neighbors.push_back(point_j);
            baricenter.x += point_j.x;
            baricenter.y += point_j.y;
            baricenter.z += point_j.z;
            
            if(propagate[neighbor_j])
            {
                bCanAlignToNeighbor = true;
                indexNeighborToAlign = neighbor_j;
            }
        }
        baricenter.x /= neighborhood_size;
        baricenter.y /= neighborhood_size;
        baricenter.z /= neighborhood_size;

        LOG_TIME(ztimer, "build_neighbours", size);

        typedef Eigen::Matrix3f MatrixT;

        MatrixT covariance_matrix;
        computeCovarianceMatrix(neighbors, baricenter, covariance_matrix);

        LOG_TIME(ztimer, "covariance_matrix", size);

        //Eigen::JacobiSVD<MatrixT> svd(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<MatrixT> svd(covariance_matrix, Eigen::ComputeFullV);
        Eigen::Vector3f Sv = svd.singularValues();
        
        Eigen::Vector3f normal = svd.matrixV().col(2);
        
        //cout <<"normal "<<i<<": "<<"-"<<normal[0]<<"-"<<normal[1]<<"-"<<normal[2]<<endl;
#if 0
        //if (normal.norm() < 1e-3)
        if( !VALID_VECTOR(normal(0),normal(1),normal(2) )
        {
            ROS_INFO("NormalEstimationPcl::computeNormal: point %d: abnormal normal! ", i);
            //ROS_INFO("NormalEstimationPcl::computeNormal:     e0=%f, e1=%f, e2=%f", e0, e1, e2);
            ROS_INFO("NormalEstimationPcl::computeNormal:     normal: %f, %f, %f", normal(0), normal(1), normal(2));
        }
#endif

        normal.normalize();
        
        if (bCanAlignToNeighbor)
        {
            pseudonormal = Eigen::Vector3f(pcl[indexNeighborToAlign].normal[0], pcl[indexNeighborToAlign].normal[1], pcl[indexNeighborToAlign].normal[2]);  
        }

        if (normal.dot(pseudonormal) < 0)
        {
            normal = -1 * normal;
        }
        
        if (normal.dot(zAxis) > kMinCosToPropagate)
        {
            propagate[i] = true;
        }

        pcl[i].normal[0] = normal(0);
        pcl[i].normal[1] = normal(1);
        pcl[i].normal[2] = normal(2);
        done[i] = true;

        LOG_TIME(ztimer, "svd", size);

        
        if (3 * Sv[2] / (Sv[0] + Sv[1] + Sv[2]) < config_.flatness_curvature_threshold)
        {
            //std::cout<<"found flat aerea"<<std::endl;
            const double half_radius = config_.radius * 0.5;
            const double half_radius2 = half_radius*half_radius;
           
            for (size_t j = 1, jEnd = pointIdxSearch.size(); j < jEnd; j++) // we start from pointIdxRadiusSearch[1] since pointIdxRadiusSearch[0] is the index of plc[i] itself
            {
                const size_t& neighbor_j = pointIdxSearch[j];
                if (done[neighbor_j]) continue; 
                
                pcl::PointXYZ delta;
                delta.x = baricenter.x - pcl[neighbor_j].x;
                delta.y = baricenter.y - pcl[neighbor_j].y;
                delta.z = baricenter.z - pcl[neighbor_j].z;
                if (neighbor_j >= begin
                    && neighbor_j <= end
                    && NORM3_L2_2(delta.x, delta.y, delta.z) < half_radius2)
                {
                    pcl[neighbor_j].normal[0] = normal(0);
                    pcl[neighbor_j].normal[1] = normal(1);
                    pcl[neighbor_j].normal[2] = normal(2);
                    done[neighbor_j] = true;
                }
            }
        }

        LOG_TIME(ztimer, "block_assign");
        
#if USE_DIRECTION_PROPAGATION     
        for(size_t j=1, jEnd=pointIdxSearch.size(); j < jEnd; j++) // we start from pointIdxRadiusSearch[1] since pointIdxRadiusSearch[0] is the index of plc[i] itself 
        {
            const size_t& point_idx = pointIdxSearch[j];
            if(!done[point_idx]) 
            {
                boost::recursive_mutex::scoped_lock locker(queue_mutex_);                
                queue_.push_back(point_idx); 
            }
        }
#endif            

        return true;
    }
    else  // if( found > kMinNumNeighboursForComputingNormal) 
    {
        /// < assign a negative vertical direction in order to push it as non-traversable (ceiling normal)
        pcl[i].normal[0] =  0; //NAN;
        pcl[i].normal[1] =  0; //NAN;
        pcl[i].normal[2] = -1; //NAN;
        done[i] = true;

        //cout << "NormalEstimationPcl::computeNormal: point " << i << ": isolated point will have no normal " << endl;
        return false;
    }
#else
    pcl[i].normal[0] = pseudonormal(0);
    pcl[i].normal[1] = pseudonormal(1);
    pcl[i].normal[2] = pseudonormal(2);
    done[i] = true;
#endif
    
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormalsInRange(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center)
{
    ROS_INFO("NormalEstimationPcl::computeNormalsInRange: spawn thread #%ld (point range: %ld-%ld)", num_thread, start, end);

    try
    {
        size_t num_computed_points = 0, num_isolated_points = 0, num_invalid_normals = 0;
        ZTimer ztimer;

#if LOG_NORMALS_STATISTICS        
        for (size_t i = start; i < end; i++)
        {
            if (!done[i])
            {
                num_computed_points++;
            }

            if (!computeNormal(i, start, end, pcl, kdtree, done, propagate, center))
            {
                num_isolated_points++;
            }

            //if (NORM3_L2_2(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) < 0.99)
            if( !VALID_VECTOR(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) )
            {
                num_invalid_normals++;
            }
        }
#else
        for (size_t i = start; i < end; i++)
        {
           bool res = computeNormal(i, start, end, pcl, kdtree, done, propagate, center);     
        }
#endif             
        double tt = ztimer.measure();
        ROS_INFO("NormalEstimationPcl::computeNormalsInRange: finished thread #%ld: computed %ld normals in %fs (%f normal/s), #isolated: %ld, #invalid: %ld", num_thread, num_computed_points, tt, num_computed_points / tt, num_isolated_points, num_invalid_normals);
    }
    catch (boost::thread_interrupted&)
    {
        ROS_INFO("NormalEstimationPcl::computeNormalsInRange: thread #%ld has been interrupted", num_thread);
    }
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormals(PointCloudT& pcl, KdTreeT& kdtree, const pcl::PointXYZ& center)
{

    addPointToLaserTraj(center, robot_id_);
    checkTeammatePositionsFromTransform();
            
#if USE_LASER_TRAJECTORIES     
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
    
    kdtree_laser_centers_.setInputCloud(pcl_laser_trajectory_);
    ros::Time time_start = ros::Time::now();
    computeClosestLaserPoints(pcl);
    ros::Duration elapsed_time = ros::Time::now() - time_start;
    ROS_INFO_STREAM("computeClosestLaserPoints - elapsed time: " << elapsed_time); 
#endif     
    
    //N.B.: these are not allowed with atomic bools 
    //bool_vec done(pcl.size(),false); 
    //bool_vec propagate(pcl.size(),false);
    
    atom_bool_vec done(pcl.size()); 
    std::fill(done.begin(),done.end(),false);
    
    atom_bool_vec propagate(pcl.size()); 
    std::fill(propagate.begin(),propagate.end(),false);     

    queue_.clear(); // used by direct propagation approach

#if USE_DIRECTION_PROPAGATION   
    
    computeNormalsByDirectionPropagation(pcl, kdtree, done, propagate, center);
    
#else       
        
    computeNormalsStandard(pcl, kdtree, done, propagate, center);
    
#endif   // USE_DIRECTION_PROPAGATION  

}


template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormalsStandard(PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center)
{    
    const size_t n = pcl.size();
    if (n > 0)
    {
        if (config_.num_threads > 1)
        {
            std::vector<boost::shared_ptr<boost::thread> > threads;
            for (size_t num_thread = 0; num_thread < config_.num_threads; num_thread++)
            {
                size_t begin = n * num_thread / config_.num_threads;
                size_t end   = n * (num_thread + 1) / config_.num_threads;
                threads.push_back(
                                  boost::make_shared<boost::thread>(
                                  boost::bind(
                                              &NormalEstimationPcl::computeNormalsInRange,
                                              this,
                                              num_thread,
                                              begin,
                                              end,
                                              boost::ref(pcl),
                                              boost::ref(kdtree),
                                              boost::ref(done),
                                              boost::ref(propagate),                        
                                              boost::cref(center)
                                              )
                                  )
                                  );
            }

            for (size_t num_thread = 0; num_thread < threads.size(); num_thread++)
            {
                threads[num_thread]->join();
            }
        }
        else
        {
            computeNormalsInRange(0, 0, n, pcl, kdtree, done, propagate, center);
        }

#if LOG_NORMALS_STATISTICS
        size_t num_done = 0;
        for (size_t i = 0, iEnd = done.size(); i < iEnd; i++)
        {
            if (done[i]) num_done++;
        }
        double percent_done = (double) (100 * num_done) / (double) done.size();
        ROS_INFO("NormalEstimationPcl::computeNormalsStandard(): done: %f%%", percent_done);

        size_t num_invalid_normals = 0;
        for (size_t i = 0, iEnd = pcl.size(); i < iEnd; i++)
        {
            //if (NORM3_L2_2(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) < 0.99)
            if( !VALID_VECTOR(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) )    
            {
                num_invalid_normals++;
            }
        }

        ROS_INFO("NormalEstimationPcl::computeNormalsStandard(): finished; # invalid normals: %ld", num_invalid_normals);
#endif  // LOG_NORMALS_STATISTICS
        
    }
    else
    {
        ROS_WARN("NormalEstimationPcl::computeNormalsStandard(): received empty point cloud");
    }
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormalsInQueue(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center)
{
    size_t num_computed_points = 0;
    ZTimer ztimer;    
    
    bool ok = true;
    size_t point_idx = 0;    
    while(ok)
    {
        {
        boost::recursive_mutex::scoped_lock locker(queue_mutex_);          
        ok = !queue_.empty();
        if(!ok) 
        {    
            double tt = ztimer.measure();
            ROS_INFO("NormalEstimationPcl::computeNormalsInQueue: finished thread #%ld: computed %ld normals in %fs (%f normal/s)", num_thread, num_computed_points, tt, num_computed_points / tt);    
            return; /// < EXIT 
        }
        point_idx = queue_.front();
        queue_.pop_front();     
        }
        const bool res = computeNormal(point_idx, start, end, pcl, kdtree, done, propagate, center);    
        if(res) num_computed_points++;
    } 
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormalsByDirectionPropagation(PointCloudT& pcl, KdTreeT& kdtree, atom_bool_vec& done, atom_bool_vec& propagate, const pcl::PointXYZ& center)
{    
    ZTimer ztimer;
    const size_t n = pcl.size();
    if (n > 0)
    {    
        std::vector<int> pointIdxSearch;
        std::vector<float> pointSquaredDistance;        
        
        boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
        
        ROS_INFO_STREAM("NormalEstimationPcl<PointT>::computeNormalsByDirectionPropagation() - pcl_laser_trajectory size: " << pcl_laser_trajectory_->size());

        // find nearest neighbors of laser centers
        // the propagation starts from these points 
        for(size_t j=0, jEnd=pcl_laser_trajectory_->size(); j < jEnd; j++)
        {
            const PointT& center_point = (*pcl_laser_trajectory_)[j];    
            int found = 0;//kdtree.radiusSearch(center_point, config_.radius, pointIdxSearch, pointSquaredDistance);        
            if(found==0)
            {
                found = kdtree.nearestKSearch(center_point, kNumInitialClosePointsForStartingPropagation, pointIdxSearch, pointSquaredDistance);
                ROS_INFO_STREAM("NormalEstimationPcl<PointT>::computeNormalsByDirectionPropagation() - close points found: " << found);
            }
            if(found>0) 
            {
                for(size_t j=0, jEnd=pointIdxSearch.size(); j < jEnd; j++)
                    queue_.push_back(pointIdxSearch[j]); 
            }
            else 
            {
                ROS_WARN_STREAM("NormalEstimationPcl<PointT>::computeNormalsByDirectionPropagation() - could not find any close points to start the propagation!");
            }
        }
        
        if (config_.num_threads == 1)
        {
            // mono-thread version 
            while(!queue_.empty())
            {                
                const size_t point_idx = queue_.front();
                queue_.pop_front();
                bool res = computeNormal(point_idx, 0, n, pcl, kdtree, done, propagate, center); 
            }
        }    
        else
        {
            size_t begin = 0;
            size_t end   = n;                

            std::vector<boost::shared_ptr<boost::thread> > threads;
            for (size_t num_thread = 0; num_thread < config_.num_threads; num_thread++)
            {
                threads.push_back(
                                  boost::make_shared<boost::thread>(
                                  boost::bind(
                                              &NormalEstimationPcl::computeNormalsInQueue,
                                              this,
                                              num_thread,
                                              begin,
                                              end,
                                              boost::ref(pcl),
                                              boost::ref(kdtree),
                                              boost::ref(done),
                                              boost::ref(propagate),                        
                                              boost::cref(center)
                                              )
                                  )
                                  );
            }

            for (size_t num_thread = 0; num_thread < threads.size(); num_thread++)
            {
                threads[num_thread]->join();
            }
        }      

        size_t num_done = 0;
        double percent_done = 0;
        for (size_t i = 0, iEnd = done.size(); i < iEnd; i++)
        {
            if (done[i]) num_done++;
        }
        percent_done = (double) (100 * num_done) / (double) done.size();
        ROS_INFO("NormalEstimationPcl::computeNormalsByDirectionPropagation(): done: %f%%", percent_done);

#if LOG_NORMALS_STATISTICS
        size_t num_invalid_normals = 0;
        for (size_t i = 0, iEnd = pcl.size(); i < iEnd; i++)
        {
            //if (NORM3_L2_2(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) < 0.99)
            if( !VALID_VECTOR(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) )    
            {
                num_invalid_normals++;
            }
        }
        ROS_INFO("NormalEstimationPcl::computeNormalsByDirectionPropagation(): finished; # invalid normals: %ld", num_invalid_normals);
#endif         
        if(percent_done < kMinPercentDoneWithDirectPropagation)
        {
            ROS_INFO("NormalEstimationPcl::computeNormalsByDirectionPropagation(): forcing normal computation completion since only %f%% done", percent_done);
            computeNormalsStandard(pcl, kdtree, done, propagate, center);
        }
    }
    else
    {
        ROS_WARN("NormalEstimationPcl::computeNormalsByDirectionPropagation(): received empty point cloud");
    }

    double tt = ztimer.measure();
    ROS_INFO("NormalEstimationPcl::computeNormalsByDirectionPropagation(): computed %ld normals in %fs (%f normal/s)", n, tt, n / tt);   
}

template<typename PointT>
void NormalEstimationPcl<PointT>::addPointToLaserTraj(const pcl::PointXYZ& center, const int robot_id)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
     
    PointT center_point; 
    center_point.x = center.x; 
    center_point.y = center.y;
    center_point.z = center.z + kLaserZOffset;
        
    // spatially downsample the laser centers as they arrive 
    double dist2 = distSquared(center_point,last_laser_center_[robot_id]);
    if(dist2 > kTrajDownsampleDistance2)
    {
        last_laser_center_[robot_id]  = center_point;
        pcl_laser_trajectory_->push_back(center_point);
    }
}


// Compute closest laser point to each point of the cloud 
template<typename PointT>
void NormalEstimationPcl<PointT>::computeClosestLaserPoints(PointCloudT& pcl)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
        
    closest_laser_point_idx_.resize(pcl.size()); 

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    
    for(size_t jj=0, jjEnd = pcl.size(); jj < jjEnd; jj++)
    {
        // k-nearest neighborhood 
        int found = kdtree_laser_centers_.nearestKSearch(pcl[jj], 1, pointIdxNKNSearch, pointNKNSquaredDistance);   
        if(found<1)
        {
            ROS_WARN("NormalEstimationPcl<PointT>::computeClosestLaserPoints() - cannot find a close node "); 
            // get the last laser pose as reference
            closest_laser_point_idx_[jj] = pcl_laser_trajectory_->size()-1;
        }
        else
        {
            closest_laser_point_idx_[jj] = pointIdxNKNSearch[0];
        }
    }
    
}

// Set previous map base_link trajectory 
template<typename PointT>
void NormalEstimationPcl<PointT>::setPrevMapTraj(const nav_msgs::Path& base_link_traj)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
    
    // reset map trajectory 
    pcl_prev_map_trajectory_.clear();
    
    PointT center_point; 
    
    int path_length = base_link_traj.poses.size();
    for (int ii = 0; ii < path_length; ii++)
    {
        /// < approx: here we compute the laser centers by only offsetting the z coordinate
        center_point.x = base_link_traj.poses[ii].pose.position.x; 
        center_point.y = base_link_traj.poses[ii].pose.position.y; 
        center_point.z = base_link_traj.poses[ii].pose.position.z + kLaserZWrtBody; 
        
        std::cout << "NormalEstimationPcl::setPrevMapTraj() - p("<<ii<<"): " <<  base_link_traj.poses[ii].pose.position << std::endl; 
    
        pcl_prev_map_trajectory_.push_back(center_point); 
    }
    
    *pcl_laser_trajectory_ += pcl_prev_map_trajectory_; 
}


// Set laser trajectory 
template<typename PointT>
void NormalEstimationPcl<PointT>::setLaserTraj(const nav_msgs::Path& base_link_traj)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
    
    // reset laser trajectory 
    pcl_laser_trajectory_->clear();
    
    PointT center_point; 
    
    const int path_length = base_link_traj.poses.size();
    for (int ii = 0; ii < path_length; ii++)
    {
        /// < approx: here we compute the laser centers by only offsetting the z coordinate
        center_point.x = base_link_traj.poses[ii].pose.position.x; 
        center_point.y = base_link_traj.poses[ii].pose.position.y; 
        center_point.z = base_link_traj.poses[ii].pose.position.z + kLaserZWrtBody;
    
        pcl_laser_trajectory_->push_back(center_point); 
    }
    
    *pcl_laser_trajectory_ += pcl_prev_map_trajectory_; 
}
    

template<typename PointT>
void NormalEstimationPcl<PointT>::checkTeammatePositionsFromTransform()
{    
    tf::StampedTransform robot_pose;
            
    for(size_t id=0; id < number_of_robots_; id++)
    {
        if(id == robot_id_) continue; /// < CONTINUE 
        
        pcl::PointXYZ center_point;        
        try
        {
            robot_pose = p_transform_teammate_->get(map_frame_,teammate_base_link_frame_[id]);
                                                
            center_point.x = robot_pose.getOrigin().x();
            center_point.y = robot_pose.getOrigin().y();
            center_point.z = robot_pose.getOrigin().z();            
        }
        catch (TransformException e)
        {
            ROS_WARN("NormalEstimationPcl::updateTeammatePositionsFromTransform() - %s", e.what());
        }
        
        if(p_transform_teammate_->isOk())
        {
            addPointToLaserTraj(center_point, id); 
        }
    }
}
    
template class NormalEstimationPcl<pcl::PointNormal>;
template class NormalEstimationPcl<pcl::PointXYZINormal>;
template class NormalEstimationPcl<pcl::PointXYZRGBNormal>;
