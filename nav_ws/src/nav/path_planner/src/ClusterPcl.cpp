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

#include <ClusterPcl.h>
#include <NormalEstimationPcl.h>

#include <pcl/filters/extract_indices.h>


static const float DEG_2_RAD = M_PI/180; 

template<typename PointT>
double ClusterPcl<PointT>::normal_clustering_thres_ = 0.1;

template<typename PointT>
const float ClusterPcl<PointT>::kFilterWallRadiusMin = 0.2; 
    
template<typename PointT>
const int ClusterPcl<PointT>::kFilterWallRadiusMinNumNeighbors = 10; // 18

template<typename PointT>
const float ClusterPcl<PointT>::kVoxelGridFilterLeafSize = 0.07f;

template<typename PointT>
const float ClusterPcl<PointT>::kAngleGroundMax     = 15*DEG_2_RAD;
template<typename PointT>
const float ClusterPcl<PointT>::kAngleRampStairsMin = 15*DEG_2_RAD;
template<typename PointT>
const float ClusterPcl<PointT>::kAngleRampStairsMax = 60*DEG_2_RAD;
template<typename PointT>
const float ClusterPcl<PointT>::kAngleWallMin       = 60*DEG_2_RAD;
template<typename PointT>
const float ClusterPcl<PointT>::kAngleWallMax       = 120*DEG_2_RAD;;


////Used as criteria to sort a vector
//struct myclass
//{
//
//    bool operator()(int i, int j)
//    {
//        return (i < j);
//    }
//} myobject;



template<typename PointT>
ClusterPcl<PointT>::ClusterPcl()
{
}

template<typename PointT>
ClusterPcl<PointT>::~ClusterPcl()
{
}

bool enforceNormalSimilarity(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = Eigen::Vector3f::Map(point_a.normal);
    Eigen::Map<const Eigen::Vector3f> point_b_normal = Eigen::Vector3f::Map(point_b.normal);
    if (fabs(point_a_normal.dot(point_b_normal)) < 0.1)
        return (true);
    return (false);
}

template<typename PointT>
bool ClusterPcl<PointT>::inFilteredList(int p)
{
    for (int i = 0; i < point_idx_filter_out_.size(); i++)
    {
        if (i == p)
            return true;
        if (i > p)
            return false;
    }
    return false;
}

template<typename PointT>
void ClusterPcl<PointT>::clusterPcl()
{
    clusters_info_.clear();
    
    for (size_t i = 0, iEnd=pcl_in_.size(); i < iEnd ; i++)
    {
        /// < compute the angle between the normal and the z-axis
        float angle = acos(pcl_in_[i].normal[2]);
        while (angle < 0) angle += 2 * M_PI;
        //cout<<"angle: "<<angle<<endl;
        
        //uint8_t r, g, b;
        
        

        if (angle <= kAngleGroundMax && pcl_in_[i].normal[2] >= 0)
        {
            //cout<<"plane"<<endl;
            pcl_no_wall_.push_back(pcl_in_[i]);
            clusters_info_.push_back(1);
        }
        else if (angle > kAngleWallMin && angle <= kAngleWallMax)
        {
            pcl_wall_.push_back(pcl_in_[i]);
        }
        else if ((angle > kAngleRampStairsMin && angle <= kAngleRampStairsMax))
        {
            pcl_no_wall_.push_back(pcl_in_[i]);
            clusters_info_.push_back(4);
        }
        else
        {
            pcl_wall_.push_back(pcl_in_[i]);
        }

    }
}

template<typename PointT>
void ClusterPcl<PointT>::filterWall()
{
    point_idx_filter_out_.clear();
    
    /// < downsample the wall cloud 
    pcl::VoxelGrid<PointOut> sor;
    cout << "pcl_Wall size before downsampling: " << pcl_wall_.size() << endl;
    sor.setInputCloud(pcl_wall_.makeShared()); // necessary deep copy 
    sor.setLeafSize(kVoxelGridFilterLeafSize, kVoxelGridFilterLeafSize, kVoxelGridFilterLeafSize);
    sor.filter(pcl_wall_);
    cout << "pcl_Wall size after downsampling: " << pcl_wall_.size() << endl;

    if(pcl_wall_.size() == 0) 
    {
        return; /// < EXIT POINT
    }
    
    auto pcl_wall_shared = pcl_wall_.makeShared(); // necessary deep copy 

    KdTreeIn kdtree_Wall;
    kdtree_Wall.setInputCloud(pcl_wall_shared);
    double radius = kFilterWallRadiusMin;
    for (size_t i = 0, iEnd = pcl_wall_.size(); i < iEnd; i++)
    {
        PointOut p = pcl_wall_[i];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        int n = kdtree_Wall.radiusSearch(p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        
        if (n < kFilterWallRadiusMinNumNeighbors) /// < if the point has few neighbors filter out it
        {
            point_idx_filter_out_.push_back(i);
        }
#if  0
        else
        {
            double z_min = pcl_wall_[pointIdxRadiusSearch[0]].z;
            double z_max = pcl_wall_[pointIdxRadiusSearch[0]].z;

            for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            {
                if (pcl_wall_[pointIdxRadiusSearch[j]].z < z_min)
                    z_min = pcl_wall_[pointIdxRadiusSearch[j]].z;
                if (pcl_wall_[pointIdxRadiusSearch[j]].z > z_max)
                    z_max = pcl_wall_[pointIdxRadiusSearch[j]].z;
            }
            if (z_max - z_min < config_.max_superable_height) // here we should put the max heigth 
            {
                point_idx_filter_out_.push_back(i);
                /// < pcl_no_wall_.push_back(pcl_wall_[i]); /// under testing
            }
        }
#endif
    }

    ROS_INFO("pointIdxFilterOut size: %d", (int) point_idx_filter_out_.size());

    //    if (point_idx_filter_out_.size() > 0)
    //    {
    //        for (int i = point_idx_filter_out_.size(); i > 0; i--)
    //        {
    //            std::swap(pcl_wall_[point_idx_filter_out_[i - 1]], pcl_wall_.back());
    //            pcl_wall_.points.pop_back();
    //        }
    //    }
    //    pcl_wall_.width = pcl_wall_.size();
    
    if (point_idx_filter_out_.size() > 0)
    {
        pcl::ExtractIndices<PointOut> extract;
        extract.setNegative (true);
        extract.setInputCloud(pcl_wall_shared); 
        pcl::PointIndices::Ptr indices_filtered (new  pcl::PointIndices());
        indices_filtered->indices = point_idx_filter_out_;
        extract.setIndices (indices_filtered); 
        extract.filter(pcl_wall_);
    }
    
    
    ROS_INFO("pcl Wall size after filtering: %d", (int) pcl_wall_.size());

}

template<typename PointT>
void ClusterPcl<PointT>::elaborateCluster()
{

    for (size_t i = 0, iEnd=pcl_no_wall_.size(); i < iEnd; i++)
    {

        /// < compute the angle between the normal and the z-axis
        float angle = acos(pcl_no_wall_[i].normal[2]);
        while (angle < 0) angle += 2 * M_PI;
        
        //cout<<"angle: "<<angle<<endl;
        
        int classification = 0;
        uint8_t r, g, b;


        if (angle <= kAngleGroundMax && pcl_no_wall_[i].normal[2] >= 0)
        {
            //cout<<"plane"<<endl;
            r = 0;
            g = 255;
            b = 0;
            pcl_no_wall_.points[i].r = r;
            pcl_no_wall_.points[i].b = b;
            pcl_no_wall_.points[i].g = g;
        }
        //else if (angle > M_PI / 3 && angle <= 3 * M_PI / 4)
        else if (angle > kAngleWallMin && angle <= kAngleWallMax)
        {

            //cout<<"wall"<<endl;
            r = 255;
            g = 0;
            b = 0;
            pcl_no_wall_.points[i].r = r;
            pcl_no_wall_.points[i].b = b;
            pcl_no_wall_.points[i].g = g;
        }
        else if ((angle > kAngleRampStairsMin && angle <= kAngleRampStairsMax))
        {
            //cout<<"ramp-stair"<<endl;
            r = 0;
            g = 127;
            b = 255;
            pcl_no_wall_.points[i].r = r;
            pcl_no_wall_.points[i].b = b;
            pcl_no_wall_.points[i].g = g;
        }
        else
        {
            r = 255;
            g = 165;
            b = 0;
            pcl_no_wall_.points[i].r = r;
            pcl_no_wall_.points[i].b = b;
            pcl_no_wall_.points[i].g = g;
        }
    }

    /*pointIdxFilterOut.clear();
    for (int i = 0; i < clusters.size(); ++i)
    {
            double z_max = pcl_Wall.points[clusters[i].indices[0]].z;
            double z_min = z_max;
            pcl::PointCloud<PointOut> cloud_cluster;
            Eigen::Vector3f normal(0,0,0);
            bool superable = true;

            for (int j = 0; j< clusters[i].indices.size(); ++j){
                    PointOut p = pcl_Wall.points[clusters[i].indices[j]];

                    if (p.z > z_max)
                            z_max = p.z;
                    if (p.z < z_min)
                            z_min = p.z;
            }

            if (z_max - z_min <= 0.3){
                    for (int j = 0; j< clusters[i].indices.size(); ++j){
                            pointIdxFilterOut.push_back(i);
                    }
            }
    }

    cout<<"pointIdxFilterOut size:"<<pointIdxFilterOut.size()<<endl;
    cout<<"pcl_Wall before filter size:"<<pcl_Wall.size()<<endl;

    std::sort (pointIdxFilterOut.begin(), pointIdxFilterOut.end(), myobject);

    for (int i= pointIdxFilterOut.size(); i>0; i--){
                    PointOut p = pcl_Wall.points[pointIdxFilterOut[i-1]];
                    std::swap(pcl_Wall.points[pointIdxFilterOut[i-1]], pcl_Wall.points.back());
                    pcl_Wall.points.pop_back();
    }
    cout<<"pcl_Wall after filter size:"<<pcl_Wall.size()<<endl;*/

}

template<typename PointT>
void ClusterPcl<PointT>::setInputPcl(const pcl::PointCloud<PointOut>& input_pcl)
{
    pcl_in_ = input_pcl;
    cout << "input_pcl size: " << input_pcl.size() << endl;
    pcl_no_wall_.clear();
    pcl_wall_.clear();

    pcl_no_wall_.header.frame_id = pcl_in_.header.frame_id;
    pcl_wall_.header.frame_id = pcl_in_.header.frame_id;

    pcl_no_wall_.header.stamp = pcl_in_.header.stamp;
    pcl_wall_.header.stamp = pcl_in_.header.stamp;
}

template<typename PointT>
int ClusterPcl<PointT>::clustering(PointCloud& noWall, PointCloud& borders, PointCloud& segmented, PointCloud& wall)
{
    if (pcl_in_.size() > 0)
    {
        noWall.clear();
        borders.clear();
        
        cout << "Start clustering" << endl;
        clusterPcl();
        cout << "Finish clustering" << endl;
        
        filterWall();
        elaborateCluster();
        
        noWall = pcl_no_wall_;
        segmented = pcl_in_;
        wall = pcl_wall_;
        return 0;
    }
    else
    {
        return -1;
    }
}

template<typename PointT>
void ClusterPcl<PointT>::getClusterInfo(std::vector<int>& clusters_info)
{
    clusters_info = clusters_info_;
}

template class ClusterPcl<pcl::PointXYZRGBNormal>;
template class ClusterPcl<pcl::PointXYZ>;
template class ClusterPcl<pcl::PointXYZI>;

