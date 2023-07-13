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

#ifndef DYNJOINPCL_H
#define DYNJOINPCL_H

#include <ros/ros.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <DynamicJoinPclConfig.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>

using namespace path_planner;

struct Quad
{
	pcl::PointXYZ p[4];
	pcl::PointXYZRGB color;
};

template<typename Point1, typename Point2>
class BucketVolume
{
protected:
public:
	size_t bucket1;
	size_t bucket2;
	const DynamicJoinPclConfig& config;
public:
	BucketVolume(size_t bucket1, size_t bucket2, const DynamicJoinPclConfig& config);
	virtual ~BucketVolume();

	virtual int getType() = 0;

	virtual bool estimate(const pcl::PointCloud<Point1>& pcl, const pcl::PointXYZ& laser_center) = 0;

	virtual void getQuad(const pcl::PointXYZ& laser_center, Quad& quad) = 0;

	double getVolume(const pcl::PointXYZ& laser);

	virtual bool testPointForRemoval(const Point2& p, const pcl::PointXYZ& laser, double threshold) = 0;
};

template<typename Point1, typename Point2>
class BucketVolume_FromDistance : public BucketVolume<Point1, Point2>
{
protected:
public:
	double distance;
public:
	BucketVolume_FromDistance(size_t bucket1, size_t bucket2, const DynamicJoinPclConfig& config);
	virtual ~BucketVolume_FromDistance();

	void getQuad(const pcl::PointXYZ& laser_center, Quad& quad);

	bool testPointForRemoval(const Point2& p, const pcl::PointXYZ& laser, double threshold);
};

template<typename Point1, typename Point2>
class BucketVolume_MinDistance : public BucketVolume_FromDistance<Point1, Point2>
{
public:
	BucketVolume_MinDistance(size_t bucket1, size_t bucket2, const DynamicJoinPclConfig& config);
	virtual ~BucketVolume_MinDistance();

	int getType();

	bool estimate(const pcl::PointCloud<Point1>& pcl, const pcl::PointXYZ& laser_center);
};

template<typename Point1, typename Point2>
class BucketVolume_MaxDistance : public BucketVolume_FromDistance<Point1, Point2>
{
public:
	BucketVolume_MaxDistance(size_t bucket1, size_t bucket2, const DynamicJoinPclConfig& config);
	virtual ~BucketVolume_MaxDistance();

	int getType();

	bool estimate(const pcl::PointCloud<Point1>& pcl, const pcl::PointXYZ& laser_center);
};

template<typename Point1, typename Point2>
class BucketVolume_Plane : public BucketVolume<Point1, Point2>
{
protected:
public:
	pcl::ModelCoefficients plane;
public:
	BucketVolume_Plane(size_t bucket1, size_t bucket2, const DynamicJoinPclConfig& config);
	virtual ~BucketVolume_Plane();

	int getType();

	void getQuad(const pcl::PointXYZ& laser_center, Quad& quad);

	bool estimate(const pcl::PointCloud<Point1>& pcl, const pcl::PointXYZ& laser_center);

	bool testPointForRemoval(const Point2& p, const pcl::PointXYZ& laser, double threshold);
};

template<typename Point_In, typename Point_Out>
class DynamicJoinPcl
{
private:
	typedef pcl::PointCloud<Point_In> PointCloud_In;
	typedef pcl::PointCloud<Point_Out> PointCloud_Out;

	tf::StampedTransform map_laser_tf;

	DynamicJoinPclConfig config;

	void statisticOutliersFilter(const PointCloud_In& in, PointCloud_In& out);
	void deterministicOutliersFilter(const PointCloud_In& in, PointCloud_In& out);

public:
	DynamicJoinPcl();
	~DynamicJoinPcl();

	PointCloud_Out pcl_removed;

	void joinPCL(const PointCloud_In& pcl_scan, const PointCloud_Out& pcl_map_old, PointCloud_Out& pcl_map_new, const pcl::PointXYZ& laser_center);
	void joinPCL(const PointCloud_In& pcl_scan, const PointCloud_Out& pcl_map_old, PointCloud_Out& pcl_map_new, const pcl::PointXYZ& laser_center, std::vector<Quad>& quads);

	inline void setConfig(const DynamicJoinPclConfig& new_config) {config = new_config;}
	inline DynamicJoinPclConfig getConfig() {return config;}
};
#endif // DYNJOINPCL_H
