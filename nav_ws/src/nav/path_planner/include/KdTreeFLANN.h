/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
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

#ifndef KDTREE_FLANN_PP_H
#define KDTREE_FLANN_PP_H

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/search/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>  // this solves a strange undefined reference which appear with optimizations 
// from http://www.pcl-users.org/Linking-problem-for-user-defined-point-type-td2414744.html

#include "kdtree/kdtree_flann_pp.h"  // these files comes from pcl commit bdb91a3 which fixes a double free problem in KdTreeFLANN after copy operation; 
// see here https://github.com/PointCloudLibrary/pcl/issues/335


/// < WE CANNOT USE c++11 with ros PCL see https://github.com/felixendres/rgbdslam_v2/issues/8
//#if __cplusplus > 199711L
//
//namespace pp
//{
//    template <typename PointT, typename Dist = ::flann::L2_Simple<float> >
//    //using KdTreeFLANN = pcl::KdTreeFLANN_PP<PointT,Dist>;
//    using KdTreeFLANN = pcl::KdTreeFLANN<PointT,Dist>; // for using the standard pcl::KdTreeFLANN  
//}
//
//#else

namespace pp
{
                   
//template <typename PointT, typename Dist = ::flann::L2_Simple<float> >
template <typename PointT, typename Dist = ::flann::L2_3D<float> >
class KdTreeFLANN : public pcl::KdTreeFLANN_PP<PointT, Dist>
//class KdTreeFLANN: public pcl::KdTreeFLANN<PointT,Dist>   // for using standard pcl::KdTreeFLANN
{
public:

    /** \brief Default Constructor for KdTreeFLANN.
     * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise. 
     *
     * By setting sorted to false, the \ref radiusSearch operations will be faster.
     */
    KdTreeFLANN(bool sorted = true) : pcl::KdTreeFLANN_PP<PointT, Dist>(sorted) {} 
    //KdTreeFLANN(bool sorted = true) : pcl::KdTreeFLANN<PointT, Dist>(sorted) {} 

    /** \brief Copy constructor
     * \param[in] tree the tree to copy into this
     */
    KdTreeFLANN(const KdTreeFLANN<PointT> &k) : pcl::KdTreeFLANN_PP<PointT, Dist>(k) {} 
    //KdTreeFLANN(const KdTreeFLANN<PointT> &k) : pcl::KdTreeFLANN<PointT, Dist>(k) {} 
};

}



#endif // KDTREE_FLANN_PP_H
