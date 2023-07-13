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

#ifndef COPY_POINT_H
#define COPY_POINT_H

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_types.h>

#define COPY_XY(p,q)    {q.x = p.x; q.y = p.y;}
#define COPY_XYZ(p,q)    {q.x = p.x; q.y = p.y; q.z = p.z;}
#define COPY_HSV(p,q)    {q.h = p.h; q.s = p.s; q.v = p.v;}
#define COPY_I(p,q)      {p.intensity = p.intensity;}
#define COPY_A(p,q)      {p.a = p.a;}
#define COPY_L(p,q)      {p.label = p.label;}
#define COPY_RGB(p,q)    {q.r = p.r; q.g = p.g; q.b = p.b;}
#define COPY_Normal(p,q) {q.normal_x = p.normal_x; q.normal_y = p.normal_y; q.normal_z = p.normal_z;}

template<typename PointT>
void copyPoint(const pcl::PointXY& p, PointT& q)
{
	COPY_XY(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZ& p, PointT& q)
{
	COPY_XYZ(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZHSV& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_HSV(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZI& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_I(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZINormal& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_I(p, q);
	COPY_Normal(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZL& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_L(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGBA& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
	COPY_A(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGBL& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
	COPY_L(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGB& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGBNormal& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
	COPY_Normal(p, q);
}

#endif // COPY_POINT_H
