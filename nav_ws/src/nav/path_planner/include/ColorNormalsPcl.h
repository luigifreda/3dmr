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

#ifndef COLOR_NORMALS_PCL_H_
#define COLOR_NORMALS_PCL_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void colorNormalsPCL(pcl::PointCloud<pcl::PointXYZRGBNormal>& output)
{
	for(size_t i = 0; i < output.size(); i++)
	{
#if 1
		// r should be 1, but anyways...
		double r = sqrt(pow(output[i].normal_x, 2) + pow(output[i].normal_y, 2) + pow(output[i].normal_z, 2));
		double phi = atan2(output[i].normal_y, output[i].normal_x);
		double theta = acos(output[i].normal_z / r);

		// normalize angles between 0,2PI
		while(phi < 0) phi += 2 * M_PI;
		while(theta < 0) theta += 2 * M_PI;

		if(r > 1e-6)
		{
			output[i].r = (uint8_t)((0.5 + 0.5 * sin(phi)) * 255.0);
			output[i].g = 200;
			output[i].b = (uint8_t)(theta * 255.0 / M_PI);
		}
		else
		{
			output[i].r = 255;
			output[i].g = 0;
			output[i].b = 255;
		}
#else
		double k = 1.0;
		if(output[i].normal_z < 0) k = -1.0;
		double phi = atan2(k * output[i].normal_y, k * output[i].normal_x);
		output[i].r = (uint8_t)((0.5 + 0.5 * sin(M_PI * k * output[i].normal_z)) * 255.0);
		output[i].g = (uint8_t)((0.5 + 0.5 * sin(2 * phi)) * 255.0);
		output[i].b = 255;
#endif
	}
}

#endif // COLOR_NORMALS_PCL_H_
