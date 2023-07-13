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

#ifndef PCL_FILTERS_DISTANCEPARTITION_H_
#define PCL_FILTERS_DISTANCEPARTITION_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename PointA, typename PointB>
inline double dist(const PointA& a, const PointB& b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

template<typename PointT>
void distancePartition(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT>& output_in, pcl::PointCloud<PointT>& output_out, const pcl::PointXYZ& center, double distance)
{
	for(size_t i = 0; i < input.size(); i++)
	{
		if(dist(center, input.points[i]) <= distance)
		{
			output_in.push_back(input.points[i]);
		}
		else
		{
			output_out.push_back(input.points[i]);
		}
	}
}

#endif // PCL_FILTERS_DISTANCEPARTITION_H_
