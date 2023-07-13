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

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <boost/make_shared.hpp>

#ifndef M_2PI
#define M_2PI (2 * M_PI)
#endif

class SphericalTransform
{
public: 

	SphericalTransform(){}

	void setMinMaxThetas(const double minThetaIn, const double maxThetaIn)
	{
		minTheta = minThetaIn; 
		maxTheta = maxThetaIn; 
		rangeTheta = maxTheta - minTheta;
		ROS_ASSERT_MSG(rangeTheta>0.,"Range of theta must be positive!");
	}

	inline size_t flatBucket(const size_t bucket1, const size_t bucket2) const 
	{
		return bucket1 * subdivisions2 + bucket2;
	}

	inline void angleToBucket(double phi, double theta, size_t& bucket1, size_t& bucket2) const
	{
		// normalize angles between 0,2PI
		while(phi < 0) phi += 2 * M_PI;
		while(theta < 0) theta += 2 * M_PI;

		bucket1 = static_cast<size_t> (phi * (subdivisions1) / rangePhi);   // azimuth 
		bucket2 = static_cast<size_t> ((theta-minTheta) * (subdivisions2) / rangeTheta);  // inclination
	}

	inline void bucketToAngle(const size_t bucket1, const size_t bucket2, double& phi, double& theta) const
	{
		phi = bucket1 * rangePhi / static_cast<double> (subdivisions1);
		theta = minTheta + bucket2 * rangeTheta / static_cast<double> (subdivisions2);
	}

	template<typename T>
	inline void coordsSphericalToEuclidean(const double r, const double phi, const double theta, T& x, T& y, T& z)
	{
		x = offsetX + r * sin(theta) * cos(phi);
		y = offsetY + r * sin(theta) * sin(phi);
		z = offsetZ + r * cos(theta);
	}

	inline void coordsEuclideanToSpherical(const double x, const double y, const double z, double& r, double& phi, double& theta) const
	{
		// radius [0..infinity):
		r = sqrt(pow(x - offsetX, 2) + pow(y - offsetY, 2) + pow(z - offsetZ, 2));
		//  azimuth [0..2PI):
		phi = atan2(y - offsetY, x - offsetX);	
		// inclination [0..PI]:
		theta = acos((z - offsetZ) / r);
	}

	template<typename PointCloud>
	void initCloud(PointCloud& pcl_out)
	{
		// init the output pointcloud with max readings 
		pcl_out.resize(subdivisions1*subdivisions2);
		for(size_t bucket1 = 0; bucket1 < subdivisions1; bucket1++)
		{
			for(size_t bucket2 = 0; bucket2 < subdivisions2; bucket2++)
			{
				size_t bucket = flatBucket(bucket1, bucket2);

				double phi, theta, x,y,z;
				bucketToAngle(bucket1, bucket2, phi, theta);
				coordsSphericalToEuclidean(max_reading, phi, theta, x, y, z);
				pcl_out[bucket].x = x;
				pcl_out[bucket].y = y;
				pcl_out[bucket].z = z;			
			}
		}	
	}

	template<typename PointT>
	void sphericalCloud(const pcl::PCLPointCloud2& input, pcl::PointCloud<PointT>& output) const 
	{
		ROS_ASSERT_MSG(output.size()==subdivisions1*subdivisions2, "Output cloud has not been initialized!"); 

		//ROS_INFO_STREAM("input cloud: " << input.height << "x " << input.width);

		// getting information to parse the point cloud
		const unsigned int pt_step = input.point_step; // size of the point structure
		// offsets of the relevent fields (init values)
		unsigned int pt_x_offset   = 0;
		unsigned int pt_y_offset   = 4;
		unsigned int pt_z_offset   = 8;

		for (unsigned int j = 0; j < input.fields.size(); j++)
		{
			if (!input.fields[j].name.compare("x"))
			{
				pt_x_offset = input.fields[j].offset;
			}
			else if (!input.fields[j].name.compare("y"))
			{
				pt_y_offset = input.fields[j].offset;
			}
			else if (!input.fields[j].name.compare("z"))
			{
				pt_z_offset = input.fields[j].offset;
			}
		}

		// traverse point cloud
		for (size_t j = 0; j < input.height; j++)
		{
			const size_t j_row_step_const = j * input.row_step;

			for (size_t i = 0; i < input.width; i++)
			{
				float x, y, z; // fields from the point

				const size_t ij_x_pt_step_base = i * input.point_step + j_row_step_const;
				
				size_t ij_x_pt_step = ij_x_pt_step_base + pt_x_offset; // i * pt_step + pt_x
				std::copy(reinterpret_cast<const char*> (&input.data[ij_x_pt_step]),
						reinterpret_cast<const char*> (&input.data[ij_x_pt_step + 4]),
						reinterpret_cast<char*> (&x));
				
				ij_x_pt_step = ij_x_pt_step_base + pt_y_offset;
				std::copy(reinterpret_cast<const char*> (&input.data[ij_x_pt_step]),
						reinterpret_cast<const char*> (&input.data[ij_x_pt_step + 4]),
						reinterpret_cast<char*> (&y));
				
				ij_x_pt_step = ij_x_pt_step_base + pt_z_offset;
				std::copy(reinterpret_cast<const char*> (&input.data[ij_x_pt_step]),
						reinterpret_cast<const char*> (&input.data[ij_x_pt_step + 4]),
						reinterpret_cast<char*> (&z));

				double r, theta, phi;
				coordsEuclideanToSpherical(x, y, z, r, phi, theta);
			
				size_t bucket1 = 0, bucket2 = 0;
				angleToBucket(phi, theta, bucket1, bucket2);
				if(bucket1>=subdivisions1 || bucket2>=subdivisions2) continue; 
				const size_t bucket = flatBucket(bucket1, bucket2);

				PointT p; 
				p.x = x; 
				p.y = y; 
				p.z = z; 

				PointT& po = output[bucket]; 
				const double ro2 = pow(po.x - offsetX, 2) + pow(po.y - offsetY, 2) + pow(po.z - offsetZ, 2);			
				if(ro2>r*r) po = p;
			}
		}
	} 

	size_t subdivisions1=180;
	size_t subdivisions2=90;
 	double offsetX = 0.;
	double offsetY = 0.;
	double offsetZ = 0.;	

	double minPhi=0.0;
	double maxPhi=2.0*M_PI;
	double rangePhi=2.0*M_PI; 

	double minTheta=0.0;
	double maxTheta=M_PI;
	double rangeTheta=M_PI; 

	float max_reading = 1000.0;
};
