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

#include <DynamicJoinPcl.h>
#include <SphericalPartition.h>
#include <DistancePartition.h>
#include <CopyPoint.h>
#include <cmath>

static uint8_t bucketColor[2][2][3] = {{{0xFF,0x00,0x00},{0x00,0xFF,0x00}},{{0xFF,0xFF,0x00},{0x00,0x7F,0xFF}}};

/**
 * Assign color to partitions such that adjacent partitions have
 * distinct colors.
 */
template<typename PointT>
inline void colorizePointInBucket1(PointT& p, size_t bucket1, size_t bucket2, size_t subdivisions)
{
	p.r = bucketColor[bucket1 % 2][bucket2 % 2][0];
	p.g = bucketColor[bucket1 % 2][bucket2 % 2][1];
	p.b = bucketColor[bucket1 % 2][bucket2 % 2][2];
}

template<typename PointT>
inline double signedDistance(const PointT& p, const pcl::ModelCoefficients& m)
{
	return m.values[0] * p.x + m.values[1] * p.y + m.values[2] * p.z + m.values[3];
}

template<typename Point_In, typename Point_Out>
DynamicJoinPcl<Point_In, Point_Out>::DynamicJoinPcl()
{
}

template<typename Point_In, typename Point_Out>
DynamicJoinPcl<Point_In, Point_Out>::~DynamicJoinPcl()
{
}

template<typename Point_In, typename Point_Out>
void DynamicJoinPcl<Point_In, Point_Out>::joinPCL(const PointCloud_In& pcl_scan, const PointCloud_Out& pcl_map_old, PointCloud_Out& pcl_map_new, const pcl::PointXYZ& laser_center)
{
	std::vector<Quad> quads;
	joinPCL(pcl_scan, pcl_map_old, pcl_map_new, laser_center, quads);
}

template<typename Point_In, typename Point_Out>
void DynamicJoinPcl<Point_In, Point_Out>::joinPCL(const PointCloud_In& pcl_scan, const PointCloud_Out& pcl_map_old, PointCloud_Out& pcl_map_new, const pcl::PointXYZ& laser_center, std::vector<Quad>& quads)
{
	if(pcl_scan.header.frame_id != config.global_frame)
	{
		ROS_ERROR("DynamicJoinPcl::joinPCL: scan point cloud is in frame '%s', must be in global frame '%s'", pcl_scan.header.frame_id.c_str(), config.global_frame.c_str());
		return;
	}

	if(pcl_map_old.header.frame_id != config.global_frame)
	{
		ROS_ERROR("DynamicJoinPcl::joinPCL: old map point cloud is in frame '%s', must be in global frame '%s'", pcl_map_old.header.frame_id.c_str(), config.global_frame.c_str());
		return;
	}

	pcl_map_new.clear();

	pcl_removed.clear();

	// remove outliers from scan
	PointCloud_In pcl_scan_filtered;
	if(config.outlier_removal == DynamicJoinPcl_OutlierRemoval_Deterministic)
		deterministicOutliersFilter(pcl_scan, pcl_scan_filtered);
	else if(config.outlier_removal == DynamicJoinPcl_OutlierRemoval_Statistical)
		statisticOutliersFilter(pcl_scan, pcl_scan_filtered);
	else if(config.outlier_removal == DynamicJoinPcl_OutlierRemoval_None)
		pcl::copyPointCloud(pcl_scan, pcl_scan_filtered);
	else
	{
		ROS_ERROR("DynamicJoinPcl::joinPCL: bad value %d for param outlier_removal", config.outlier_removal);
		return;
	}

	// partition laser scan according to theta,phi
	PointCloud_In pcl_scan_partition[config.num_subdivisions * config.num_subdivisions];
	sphericalPartition(pcl_scan_filtered, pcl_scan_partition, laser_center, config.num_subdivisions);

	// partition map in near/far, and partition near according to theta,phi
	PointCloud_Out pcl_map_near, pcl_map_far, pcl_map_partition[config.num_subdivisions * config.num_subdivisions];
	distancePartition(pcl_map_old, pcl_map_near, pcl_map_far, laser_center, config.radius_near);
	sphericalPartition(pcl_map_near, pcl_map_partition, laser_center, config.num_subdivisions);

	for(size_t bucket1 = 0; bucket1 < config.num_subdivisions; bucket1++)
	{
		for(size_t bucket2 = 0; bucket2 < config.num_subdivisions; bucket2++)
		{
			size_t bucket = flatBucket(bucket1, bucket2, config.num_subdivisions);

			BucketVolume_MinDistance<Point_In, Point_Out> v_min(bucket1, bucket2, config);
			BucketVolume_Plane<Point_In, Point_Out> v_rsc(bucket1, bucket2, config);
			v_min.estimate(pcl_scan_partition[bucket], laser_center);
			BucketVolume<Point_In, Point_Out> *pv = &v_min;

			if(pcl_scan_partition[bucket].size() >= 4)
			{
				// we have enough points for fitting a plane...

				if(config.bucket_volume_model_type == DynamicJoinPcl_BucketVolumeModelType_SinglePlane)
				{
					if(v_rsc.estimate(pcl_scan_partition[bucket], laser_center))
					{
						BucketVolume_MaxDistance<Point_In, Point_Out> v_max(bucket1, bucket2, config);
						v_max.estimate(pcl_scan_partition[bucket], laser_center);
						double v = v_rsc.getVolume(laser_center);
						if(v >= v_min.getVolume(laser_center) && v <= v_max.getVolume(laser_center))
						{
							//ROS_INFO("plane: <%f, %f, %f, %f>", v_rsc.plane.values[0], v_rsc.plane.values[1], v_rsc.plane.values[2], v_rsc.plane.values[3]);
							pv = &v_rsc;
						}
						else
						{
							//ROS_WARN("plane(bad volume): <%f, %f, %f, %f>, dist: %f", v_rsc.plane.values[0], v_rsc.plane.values[1], v_rsc.plane.values[2], v_rsc.plane.values[3], v_min.distance);
						}
					}
					else
					{
						//ROS_INFO("plane estimation failed. using min dist with dist=%f", v_min.distance);
					}
				}
				Quad quad;
				pv->getQuad(laser_center, quad);
				colorizePointInBucket1(quad.color, bucket1, bucket2, config.num_subdivisions);
				quads.push_back(quad);
			}

			// add points from map that do not fall into deletion volume
			for(size_t j = 0; j < pcl_map_partition[bucket].size(); j++)
			{
				double d = dist(laser_center, pcl_map_partition[bucket][j]);

				if(pcl_scan_partition[bucket].size() < 4 ||
						!pv->testPointForRemoval(pcl_map_partition[bucket][j], laser_center, 0.05 + d * 0.1))
				{
					if(config.colorize_points)
						colorizePointInBucket1(pcl_map_partition[bucket][j], bucket1, bucket2, config.num_subdivisions);

					pcl_map_new.push_back(pcl_map_partition[bucket][j]);
				}
				else
				{
					pcl_removed.push_back(pcl_map_partition[bucket][j]);
				}
			}

			// add all points from scan
			for(size_t j = 0; j < pcl_scan_partition[bucket].size(); j++)
			{
				Point_Out p;
				copyPoint(pcl_scan_partition[bucket][j], p);

				if(config.colorize_points)
					colorizePointInBucket1(p, bucket1, bucket2, config.num_subdivisions);

				pcl_map_new.push_back(p);
			}
		}
	}

	// re-add far points
	for(size_t i = 0; i < pcl_map_far.size(); i++)
	{
		pcl_map_new.push_back(pcl_map_far[i]);
	}

	PointCloud_Out temp;
	temp.header = pcl_map_new.header;

	// downsample
	pcl::VoxelGrid<Point_Out> sor;
	sor.setInputCloud(pcl_map_new.makeShared());
	sor.setLeafSize(config.leaf_size, config.leaf_size, config.leaf_size);
	sor.filter(temp);

	pcl_map_new.swap(temp);
}

template<typename Point_In, typename Point_Out>
void DynamicJoinPcl<Point_In, Point_Out>::statisticOutliersFilter(const PointCloud_In& in, PointCloud_In& out)
{
	pcl::StatisticalOutlierRemoval<Point_In> sor;
	sor.setInputCloud(in.makeShared());
	sor.setMeanK(config.mean_k);
	sor.setStddevMulThresh(config.stddev_mul_thresh);
	sor.filter(out);
}

template<typename Point_In, typename Point_Out>
void DynamicJoinPcl<Point_In, Point_Out>::deterministicOutliersFilter(const PointCloud_In& in, PointCloud_In& out)
{
	pcl::RadiusOutlierRemoval<Point_In> sor;
	sor.setInputCloud(in.makeShared());
	sor.setRadiusSearch(config.radius_search);
	sor.setMinNeighborsInRadius(config.min_neighbors);
	sor.filter(out);
}

template<typename PointT>
bool plinePlaneIntersection(double pline_theta, double pline_phi, const pcl::ModelCoefficients& plane, PointT& p, double& rho, const pcl::PointXYZ& laser_center)
{
	rho = -(plane.values[0] * laser_center.x + plane.values[1] * laser_center.y +
			plane.values[2] * laser_center.z + plane.values[3])
					/
			(plane.values[0] * sin(pline_theta) * cos(pline_phi) +
			plane.values[1] * sin(pline_theta) * sin(pline_phi) +
			plane.values[2] * cos(pline_theta));
	if(rho < 0 || rho > 10.0)
		return false;
	p.x = laser_center.x + rho * sin(pline_theta) * cos(pline_phi);
	p.y = laser_center.y + rho * sin(pline_theta) * sin(pline_phi);
	p.z = laser_center.z + rho * cos(pline_theta);
	return true;
}

template<typename Point1, typename Point2>
BucketVolume<Point1, Point2>::BucketVolume(size_t bucket1_, size_t bucket2_, const DynamicJoinPclConfig& config_)
	: bucket1(bucket1_), bucket2(bucket2_), config(config_)
{
}

template<typename Point1, typename Point2>
BucketVolume<Point1, Point2>::~BucketVolume()
{
}

template<typename Point1, typename Point2>
double BucketVolume<Point1, Point2>::getVolume(const pcl::PointXYZ& laser)
{
	Quad quad;
	getQuad(laser, quad);
	Eigen::Vector3d l(laser.x, laser.y, laser.z),
			a(quad.p[0].x, quad.p[0].y, quad.p[0].z),
			b(quad.p[0].x, quad.p[0].y, quad.p[0].z),
			c(quad.p[0].x, quad.p[0].y, quad.p[0].z),
			d(quad.p[0].x, quad.p[0].y, quad.p[0].z);
	return ((a - l).dot((b - l).cross(c - l)) + (a - l).dot((c - l).cross(d - l))) / 6.0;
}

template<typename Point1, typename Point2>
BucketVolume_FromDistance<Point1, Point2>::BucketVolume_FromDistance(size_t bucket1_, size_t bucket2_, const DynamicJoinPclConfig& config_)
	: BucketVolume<Point1, Point2>(bucket1_, bucket2_, config_), distance(0.0)
{
}

template<typename Point1, typename Point2>
BucketVolume_FromDistance<Point1, Point2>::~BucketVolume_FromDistance()
{
}

template<typename Point1, typename Point2>
void BucketVolume_FromDistance<Point1, Point2>::getQuad(const pcl::PointXYZ& laser_center, Quad& q)
{
	double theta1, phi1, theta2, phi2;
	bucketToAngle(this->bucket1, this->bucket2, theta1, phi1, this->config.num_subdivisions);
	bucketToAngle(this->bucket1 + 1, this->bucket2 + 1, theta2, phi2, this->config.num_subdivisions);
	coordsSphericalToEuclidean(distance, theta1, phi1, q.p[0].x, q.p[0].y, q.p[0].z, laser_center.x, laser_center.y, laser_center.z);
	coordsSphericalToEuclidean(distance, theta2, phi1, q.p[1].x, q.p[1].y, q.p[1].z, laser_center.x, laser_center.y, laser_center.z);
	coordsSphericalToEuclidean(distance, theta2, phi2, q.p[2].x, q.p[2].y, q.p[2].z, laser_center.x, laser_center.y, laser_center.z);
	coordsSphericalToEuclidean(distance, theta1, phi2, q.p[3].x, q.p[3].y, q.p[3].z, laser_center.x, laser_center.y, laser_center.z);
}

template<typename Point1, typename Point2>
bool BucketVolume_FromDistance<Point1, Point2>::testPointForRemoval(const Point2& p, const pcl::PointXYZ& laser, double threshold)
{
	return dist(laser, p) < (distance + threshold);
}

template<typename Point1, typename Point2>
BucketVolume_MinDistance<Point1, Point2>::BucketVolume_MinDistance(size_t bucket1_, size_t bucket2_, const DynamicJoinPclConfig& config_)
	: BucketVolume_FromDistance<Point1, Point2>(bucket1_, bucket2_, config_)
{
}

template<typename Point1, typename Point2>
BucketVolume_MinDistance<Point1, Point2>::~BucketVolume_MinDistance()
{
}

template<typename Point1, typename Point2>
int BucketVolume_MinDistance<Point1, Point2>::getType()
{
	return DynamicJoinPcl_BucketVolumeModelType_MinimumDistance;
}

template<typename Point1, typename Point2>
bool BucketVolume_MinDistance<Point1, Point2>::estimate(const pcl::PointCloud<Point1>& pcl, const pcl::PointXYZ& laser_center)
{
	double d = INFINITY;
	for(size_t i = 0; i < pcl.size(); i++)
		d = std::min(d, dist(laser_center, pcl[i]));
	if(isinf(d))
		return false;
	this->distance = d;
	return true;
}

template<typename Point1, typename Point2>
BucketVolume_MaxDistance<Point1, Point2>::BucketVolume_MaxDistance(size_t bucket1_, size_t bucket2_, const DynamicJoinPclConfig& config_)
	: BucketVolume_FromDistance<Point1, Point2>(bucket1_, bucket2_, config_)
{
}

template<typename Point1, typename Point2>
BucketVolume_MaxDistance<Point1, Point2>::~BucketVolume_MaxDistance()
{
}

template<typename Point1, typename Point2>
int BucketVolume_MaxDistance<Point1, Point2>::getType()
{
	return DynamicJoinPcl_BucketVolumeModelType_MaximumDistance;
}

template<typename Point1, typename Point2>
bool BucketVolume_MaxDistance<Point1, Point2>::estimate(const pcl::PointCloud<Point1>& pcl, const pcl::PointXYZ& laser_center)
{
	double d = -INFINITY;
	for(size_t i = 0; i < pcl.size(); i++)
		d = std::max(d, dist(laser_center, pcl[i]));
	if(isinf(d))
		return false;
	this->distance = d;
	return true;
}

template<typename Point1, typename Point2>
BucketVolume_Plane<Point1, Point2>::BucketVolume_Plane(size_t bucket1_, size_t bucket2_, const DynamicJoinPclConfig& config_)
	: BucketVolume<Point1, Point2>(bucket1_, bucket2_, config_)
{
}

template<typename Point1, typename Point2>
BucketVolume_Plane<Point1, Point2>::~BucketVolume_Plane()
{
}

template<typename Point1, typename Point2>
int BucketVolume_Plane<Point1, Point2>::getType()
{
	return DynamicJoinPcl_BucketVolumeModelType_SinglePlane;
}

template<typename Point1, typename Point2>
void BucketVolume_Plane<Point1, Point2>::getQuad(const pcl::PointXYZ& laser_center, Quad& quad)
{
	double rho1, rho2, rho3, rho4;
	double theta1, phi1, theta2, phi2;
	bucketToAngle(this->bucket1, this->bucket2, theta1, phi1, this->config.num_subdivisions);
	bucketToAngle(this->bucket1 + 1, this->bucket2 + 1, theta2, phi2, this->config.num_subdivisions);
	plinePlaneIntersection(theta1, phi1, plane, quad.p[0], rho1, laser_center);
	plinePlaneIntersection(theta2, phi1, plane, quad.p[1], rho2, laser_center);
	plinePlaneIntersection(theta2, phi2, plane, quad.p[2], rho3, laser_center);
	plinePlaneIntersection(theta1, phi2, plane, quad.p[3], rho4, laser_center);
}

template<typename PointT>
bool separates(const pcl::PointCloud<PointT>& pcl, const pcl::PointXYZ& laser_center, const pcl::ModelCoefficients& plane, double tolerance)
{
	double lsd = signedDistance(laser_center, plane);
	int lsdsign = lsd < 0 ? -1 : 1;
	for(size_t i = 0; i < pcl.size(); i++)
	{
		double sd = signedDistance(pcl[i], plane);
		bool same_side = lsdsign * sd > 0;
		if(abs(sd) > tolerance && same_side)
			return false;
	}
	return true;
}

template<typename Point1, typename Point2>
bool BucketVolume_Plane<Point1, Point2>::estimate(const pcl::PointCloud<Point1>& pcl, const pcl::PointXYZ& laser_center)
{
	if(pcl.size() < 4) return false;
#ifdef ESTIMATE_PLANE_USING_RANSAC
	pcl::PointIndices inliers;
	pcl::SACSegmentation<Point1> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(this->config.ransac_distance_threshold);
	seg.setInputCloud(pcl.makeShared());
	seg.segment(inliers, plane);
	return inliers.indices.size() >= this->config.ransac_min_inliers;
#else
	double tolerance = 0.00001;

	pcl::PointCloud<Point1> chpcl;
	pcl::ConvexHull<Point1> ch;
	ch.setInputCloud(pcl.makeShared());
	std::vector<pcl::Vertices> polygons;
	ch.reconstruct(chpcl, polygons);
	//ROS_INFO("convex hull size = %d", chpcl.size());

	//ROS_INFO("first trial");
	// first trial: find point of CH which is nearest to laser
	{
		size_t mindist_point_idx = 0;
		double mindist = INFINITY, d;
		for(size_t i = 0; i < chpcl.size(); i++)
		{
			if((d = dist(laser_center, chpcl[i])) < mindist)
			{
				mindist = d;
				mindist_point_idx = i;
			}
		}
		Eigen::Vector3d laser(laser_center.x, laser_center.y, laser_center.z),
				nearest(chpcl[mindist_point_idx].x, chpcl[mindist_point_idx].y, chpcl[mindist_point_idx].z),
				norm = laser - nearest;
		norm.normalize();
		d = -norm.dot(nearest);
		pcl::ModelCoefficients c;
		c.values.resize(4);
		c.values[0] = norm(0);
		c.values[1] = norm(1);
		c.values[2] = norm(2);
		c.values[3] = d;
		if(separates(pcl, laser_center, c, tolerance))
		{
			//ROS_INFO("  ok");
			plane.values.swap(c.values);
			return true;
		}
		else
		{
			//ROS_INFO("  fails to separate");
		}
	}

	//ROS_INFO("second trial");
	// second trial: look for simplex
	{
		pcl::ModelCoefficients best;
		best.values.clear(); for(int i = 0; i < 4; i++) best.values.push_back(0);
		double bestd = 0;
		bool found = false;
		for(std::vector<pcl::Vertices>::iterator it = polygons.begin(); it != polygons.end(); ++it)
		{
			pcl::ModelCoefficients c;
			c.values.resize(4);
			Eigen::Vector3d v0(chpcl[it->vertices[0]].x, chpcl[it->vertices[0]].y, chpcl[it->vertices[0]].z),
					v1(chpcl[it->vertices[1]].x, chpcl[it->vertices[1]].y, chpcl[it->vertices[1]].z),
					v2(chpcl[it->vertices[2]].x, chpcl[it->vertices[2]].y, chpcl[it->vertices[2]].z),
					norm = (v1-v0).cross(v2-v0);
			norm.normalize();
			double d = -norm.dot(v0);
			c.values[0] = norm(0);
			c.values[1] = norm(1);
			c.values[2] = norm(2);
			c.values[3] = d;
			if(separates(pcl, laser_center, c, tolerance) && (d = pow(signedDistance(laser_center, c), 2)) > bestd)
			{
				//ROS_INFO("  new max: plane=<%f %f %f %f>, bestd=%f", c.values[0], c.values[1], c.values[2], c.values[3], d);
				bestd = d;
				best.values.swap(c.values);
				found = true;
			}
		}
		if(!found)
		{
                    ROS_WARN("  found nothing!!");
		}
		else
		{
                    best.values.swap(plane.values);
		}
		return found;
	}
#endif
}

template<typename Point1, typename Point2>
bool BucketVolume_Plane<Point1, Point2>::testPointForRemoval(const Point2& pt, const pcl::PointXYZ& laser, double threshold)
{
	// laser . plane
	double s = laser.x*plane.values[0]+laser.y*plane.values[1]+laser.z*plane.values[2]+plane.values[3];
	// pt . plane
	double p = pt.x*plane.values[0]+pt.y*plane.values[1]+pt.z*plane.values[2]+plane.values[3];
	if(s * p < 0) return false;
	return abs(p) > threshold;
}

template class DynamicJoinPcl<pcl::PointXYZ, pcl::PointXYZRGBNormal>;
template class DynamicJoinPcl<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>;
