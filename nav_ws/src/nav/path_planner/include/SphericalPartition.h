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

#ifndef PCL_FILTERS_SPHERICALPARTITION_H_
#define PCL_FILTERS_SPHERICALPARTITION_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/make_shared.hpp>

#ifndef M_2PI
#define M_2PI (2 * M_PI)
#endif

inline size_t flatBucket(const size_t bucket1, const size_t bucket2, const size_t subdivisions)
{
	return bucket1 * subdivisions + bucket2;
}

inline void angleToBucket(double theta, double phi, size_t& bucket1, size_t& bucket2, const size_t subdivisions)
{
	// normalize angles between 0,2PI
	while(phi < 0) phi += 2 * M_PI;
	while(theta < 0) theta += 2 * M_PI;

	bucket1 = static_cast<size_t> (phi * subdivisions / M_2PI);
	bucket2 = static_cast<size_t> (theta * subdivisions / M_PI);
}

inline void bucketToAngle(const size_t bucket1, const size_t bucket2, double& theta, double& phi, const size_t subdivisions)
{
	phi = bucket1 * M_2PI / static_cast<double> (subdivisions);
	theta = bucket2 * M_PI / static_cast<double> (subdivisions);
}

template<typename T>
inline void coordsSphericalToEuclidean(double r, double theta, double phi, T& x, T& y, T& z, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
{
	x = offsetX + r * sin(theta) * cos(phi);
	y = offsetY + r * sin(theta) * sin(phi);
	z = offsetZ + r * cos(theta);
}

inline void coordsEuclideanToSpherical(double x, double y, double z, double& r, double& theta, double& phi, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
{
	// radius [0..infty):
	r = sqrt(pow(x - offsetX, 2) + pow(y - offsetY, 2) + pow(z - offsetZ, 2));
	// inclination [0..PI]:
	theta = acos((z - offsetZ) / r);
	//  azimuth [0..2PI):
	phi = atan2(y - offsetY, x - offsetX);
}

template<typename PointT>
void sphericalPartition(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT> *output, const pcl::PointXYZ& center, unsigned int subdivisions)
{
	// partition according to phi,theta:
	for(size_t i = 0; i < input.size(); i++)
	{
		const PointT& p = input.points[i];
		double r, theta, phi;
		coordsEuclideanToSpherical(p.x, p.y, p.z, r, theta, phi, center.x, center.y, center.z);
		size_t bucket1 = 0, bucket2 = 0;
		angleToBucket(theta, phi, bucket1, bucket2, subdivisions);
		output[flatBucket(bucket1, bucket2, subdivisions)].push_back(p);
	}
}

#endif // PCL_FILTERS_SPHERICALPARTITION_H_
