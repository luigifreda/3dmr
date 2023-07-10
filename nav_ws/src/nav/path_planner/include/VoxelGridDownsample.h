#ifndef VOXEL_GRID_DOWNSAMPLE_H
#define VOXEL_GRID_DOWNSAMPLE_H

#include <ros/ros.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <VoxelBinaryKey.h>

#include <set>
#include <map>

template<typename PointT>
void voxelGridDownsample(const pcl::PointCloud<PointT>& in, pcl::PointCloud<PointT>& out, double leaf_size)
{
	out.clear();

	std::map<uint64_t, std::set<size_t> > m;

	// partition w.r.t voxel binary key
	for(size_t i = 0; i < in.size(); i++)
	{
		uint64_t k = voxelBinaryKey(in[i], leaf_size);
		if(!m.count(k)) m[k] = std::set<size_t>();
		m[k].insert(i);
	}

	for(std::map<uint64_t, std::set<size_t> >::iterator it = m.begin(); it != m.end(); ++it)
	{
		PointT centroid;
		centroid.x = 0.0;
		centroid.y = 0.0;
		centroid.z = 0.0;

		// find centroid of each partition
		double total = 0.0;
		for(std::set<size_t>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			centroid.x += in[*it2].x;
			centroid.y += in[*it2].y;
			centroid.z += in[*it2].z;
			total += 1.0;
		}
		centroid.x /= total;
		centroid.y /= total;
		centroid.z /= total;

		// find point nearest to centroid
		size_t nearest = 0;
		double dist = INFINITY;
		for(std::set<size_t>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			double d1 = pow(centroid.x - in[*it2].x, 2) + pow(centroid.y - in[*it2].y, 2) + pow(centroid.z - in[*it2].z, 2);
			if(d1 < dist)
			{
				dist = d1;
				nearest = *it2;
			}
		}

		// add it to output
		out.push_back(in[nearest]);
	}
}

#endif // VOXEL_GRID_DOWNSAMPLE_H
