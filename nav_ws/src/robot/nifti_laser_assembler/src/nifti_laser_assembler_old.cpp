
#include <math.h>
#include <vector>
#include <list>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <pcl_ros/transforms.h>
//#include <laser_filters/scan_shadows_filter.h>



template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}


/** \brief Class assembling the laser scans of the rolling laser
 *
 * This class manages the assembly of the rolling laser.
 * It listens to the /tf transform of the laser with respect to the robot to
 * trigger the start and end of the assembly and publishes
 * sensor_msgs/PointCloud2 messages.
 */
class NiftiLaserAssembler {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	NiftiLaserAssembler();

	virtual ~NiftiLaserAssembler();

protected:

	//! /tf listener
	tf::TransformListener tf_listener;

	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	//! Name of the laser frame (default: "laser")
	std::string laser_frame;

	//! Name of the robot frame (default: "base_link")
	std::string robot_frame;

	//! Name of the reference world frame (default: "odom")
	std::string world_frame;

	//! Publisher for the point cloud (default topic: "/static_point_cloud")
	ros::Publisher point_cloud_pub;

	//! Publisher for the moving point cloud (default topic:
	//"/dynamic_point_cloud")
	ros::Publisher dynamic_point_cloud_pub;

	//! Subscriber to laser scans (default topic: "/scan")
	ros::Subscriber laser_scan_sub;

	//! Limit the size of the point cloud in points (default: 1000000).
	int max_size;

	//! Current aggregated point cloud
	sensor_msgs::PointCloud2 point_cloud;

	//! Scan to filter and modify
	sensor_msgs::LaserScan tmp_scan;

	//! Point cloud holding the projection of one laser scan
	sensor_msgs::PointCloud2 tmp_point_cloud;

	//! Previous absolute value of the laser angle (to detect when to publish)
	double previous_angle;

	//! Laser angle offset (we publish slightly slanted when not moving
	double laser_angle_offset;

	//! Projector object from LaserScan to PointCloud2
	laser_geometry::LaserProjection projector;

	//! Starting time of the new scan
	ros::Time start_time;

	/** \brief Set of channel to add to the point cloud (default: none)
	 *
	 * This follows the laser_geometry::channel_option::ChannelOption enum.
	 * By default, we don't add any channel. Parameter ~channels can be set to
	 * change that.
	 */
	int point_cloud_channels;

	//! Publish 2d scans when laser's horizontal (default: true)
	bool publish2d;

	//! Publisher for the horizontal scans (default topic: "/scan2d")
	ros::Publisher scan2d_pub;
	
	//! Relaying laser scans (default: true)
	bool relay_scans;

	//! Publisher for the relayed scans (default topic: "/scan_relay")
	ros::Publisher relay_pub;
	
	//! Min distance to keep point (default: 0)
	double min_distance;

	//! Time offset (default: 0)
	ros::Duration time_offset;

	//! tan of min_angle to do shadow filtering (default: tan(0.1))
	const double tan_shadow_filter_min_angle;

	//! Time correction plus setting ranges
	void time_correct(sensor_msgs::LaserScan& scan);

	//! Distance distortion correction
	void undistort(sensor_msgs::LaserScan& scan);

	//! Scan filtering function
	void shadow_filter(sensor_msgs::LaserScan& scan);

	//! Filter close points for Karto
	void karto_filter(sensor_msgs::LaserScan& scan);

	bool using_gmapping;
	//! Invert scan for gmapping
	void invert_gmapping(const sensor_msgs::LaserScan& scan_in,
			sensor_msgs::LaserScan& scan_out);

	//! remove points on the robot
	void robot_filter(sensor_msgs::LaserScan& scan);

	//! Build 2D flat scan from slanted scan
	void gen2Dscan(const sensor_msgs::LaserScan& scan, double scan_angle);

	//! Publisher for the regenerated horizontal scans
	ros::Publisher scan2dsyn_pub;
	
	//! Scan callback function
	void scan_cb(const sensor_msgs::LaserScan& scan);

	//! Get the new scan in the point cloud
	void append_scan(const sensor_msgs::LaserScan& scan);
	
	//! Get laser angle from the tf
	double get_laser_angle(const ros::Time &time) const;

	//! Check if the robot moves
	bool check_no_motion(const ros::Time &time) const;

	//! Subscriber to mapping control (default topic: "/mapping_control")
	ros::Subscriber map_ctrl_sub;

	//! Map control callback
	void map_ctrl_cb(const std_msgs::Bool& on);

	//! map control current state
	bool map_ctrl_on;

	//! end of swipe publisher
	ros::Publisher end_of_swipe_pub;

	//! Subscriber to point cloud control (default topic: "/pointcloud_control")
	ros::Subscriber ptcld_ctrl_sub;

	//! Point cloud control callback
	void ptcld_ctrl_cb(const std_msgs::Bool& on);

	//! Point cloud control current state
	bool ptcld_ctrl_on;
};



/*
 * Constructor
 */
NiftiLaserAssembler::NiftiLaserAssembler():
	tf_listener(ros::Duration(60.)),
	n_("~"),
	tan_shadow_filter_min_angle(tan(M_PI/2 - getParam<double>(n_, "shadow_filter_min_angle", 0.1)))
{
	// frame names
	laser_frame = getParam<std::string>(n_, "laser_frame", "laser");
	robot_frame = getParam<std::string>(n_, "robot_frame", "base_link");
	world_frame = getParam<std::string>(n_, "world_frame", "odom");

	using_gmapping = getParam<bool>(n_, "using_gmapping", false);

	// max number of point
	max_size = getParam<int>(n_, "max_size", 1000000);

	// channels
	point_cloud_channels = getParam<int>(n_, "channels",
			laser_geometry::channel_option::None);

	// 2d scans
	laser_angle_offset = getParam<double>(n, "laser_angle_offset", 0.0);
	publish2d = getParam<bool>(n_, "publish2d", true);
	if (publish2d) {
		scan2d_pub = n.advertise<sensor_msgs::LaserScan>("/scan2d", 50);
	}

	// relay
	relay_scans = getParam<bool>(n_, "relay_scans", true);
	if (relay_scans) {
		relay_pub = n.advertise<sensor_msgs::LaserScan>("/scan_relay", 50);
	}

	// moving
	start_time = ros::Time(0);

	// filtering
	double offset;
	offset = getParam<double>(n_, "time_offset", 0.0);
	time_offset = ros::Duration(offset);
	min_distance = getParam<double>(n_, "min_distance", 0.0);
	// initialized so that the first test always fails
	previous_angle = NAN;

	if (!tf_listener.waitForTransform(laser_frame, world_frame, ros::Time(0),
			ros::Duration(10.)))
		ROS_WARN_STREAM("Timeout (10s) while waiting between "<<laser_frame<<
				" and "<<world_frame<<" at startup.");

	// point cloud publisher
	point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			("/static_point_cloud", 50);
	dynamic_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			("/dynamic_point_cloud", 50);
	
	// laser scan subscriber
	laser_scan_sub = n.subscribe("/scan", 50,
			&NiftiLaserAssembler::scan_cb, this);

	// mapping control subscriber
	map_ctrl_sub = n.subscribe("/mapping_control", 50,
			&NiftiLaserAssembler::map_ctrl_cb, this);
	map_ctrl_on = true;

	// point cloud control subscriber
	ptcld_ctrl_sub = n.subscribe("/pointcloud_control", 50,
			&NiftiLaserAssembler::ptcld_ctrl_cb, this);
	ptcld_ctrl_on = true;

	// end of swipe publisher
	end_of_swipe_pub = n.advertise<std_msgs::Bool>("/end_of_swipe", 50);

	// synthetic 2d scans
	//scan2dsyn_pub = n.advertise<sensor_msgs::LaserScan>("/scan2dsyn", 50);
}


/*
 * Destructor
 */
NiftiLaserAssembler::~NiftiLaserAssembler()
{
	// Nothing to do?
}

/*
 * Time and range correction
 */
void NiftiLaserAssembler::time_correct(sensor_msgs::LaserScan& scan){
	scan.header.stamp = scan.header.stamp + time_offset;
	scan.range_min = std::max(scan.range_min, (float)min_distance);
}

/*
 * Scan filtering
 */
void NiftiLaserAssembler::shadow_filter(sensor_msgs::LaserScan& scan)
{
	const double invalid = scan.range_max+1;
	const float sin_gamma = sin(scan.angle_increment);
	const float cos_gamma = cos(scan.angle_increment);
	float x,y;

	const sensor_msgs::LaserScan scan_ref(scan);

	//TODO: parallize with Eigen?
	for (unsigned int i=1; i<scan_ref.ranges.size(); i++) {
		x = scan_ref.ranges[i-1]*sin_gamma;
		y = fabs(scan_ref.ranges[i] - scan_ref.ranges[i-1]*cos_gamma);
		
		if (y > x*tan_shadow_filter_min_angle){
			scan.ranges[i-1] = invalid;
			scan.ranges[i] = invalid;
		}
	}
}

/* 
 * Karto filtering: putting min distance point to max_range + 1 so that they
 * don't appear in the map
 */
void NiftiLaserAssembler::karto_filter(sensor_msgs::LaserScan& scan)
{
	const double invalid = scan.range_max+1;
	const double min_range = scan.range_min;

	for (unsigned int i=0; i<scan.ranges.size(); i++) {
		if (scan.ranges[i]<min_range) {
			scan.ranges[i] = invalid;
		}
	}
}

void NiftiLaserAssembler::invert_gmapping(const sensor_msgs::LaserScan& scan_in,
			sensor_msgs::LaserScan& scan_out)
{
	unsigned int nb = scan_in.ranges.size();
	for (unsigned int i=0; i<nb; i++)
	{
		scan_out.ranges[i] = scan_in.ranges[nb-i-1];
	}
}

/* 
 * Undistort
 */
void NiftiLaserAssembler::undistort(sensor_msgs::LaserScan& scan)
{
	const double c1 = 0.03;
	const double d1 = 0.13;
	const double c2 = 0.008;
	const double d2 = 0.25;
	const double beta = (c1*d1*d1-c2*d2*d2)/(c2-c1);
	const double alpha = c1*(beta+d1*d1);
	//const double alpha = 0.000;
	//const double beta = 0.005;

	for (unsigned int i=0; i<scan.ranges.size(); i++) {
		if (scan.ranges[i]>scan.range_min) {
			scan.ranges[i] += alpha/(beta+scan.ranges[i]*scan.ranges[i]);
		}
	}
}

/*
 * Filter robot out of the current scan
 */
void NiftiLaserAssembler::robot_filter(sensor_msgs::LaserScan& scan)
{
	const double invalid = scan.range_max+1;
	std::list<double> blacklisted;
	sensor_msgs::PointCloud close_ptcld;
	sensor_msgs::PointCloud transformed_ptcld;
	close_ptcld.header = scan.header;
	sensor_msgs::ChannelFloat32 index;
	index.name = "index";
	geometry_msgs::Point32 point;
	point.z = 0;
	double angle;
	const ros::Time time = scan.header.stamp;

	double max_dist = 0.7;
	double eps = 0.015;

	// create a small point cloud with only the close points
	for (unsigned int i=0; i<scan.ranges.size(); i++) {
		if (scan.ranges[i]<max_dist) {
			angle = scan.angle_min + i*scan.angle_increment;
			point.x = scan.ranges[i]*cos(angle);
			point.y = scan.ranges[i]*sin(angle);
			close_ptcld.points.push_back(point);
			index.values.push_back(i);
		}
	}
	close_ptcld.channels.push_back(index);
	//std::cout << close_ptcld.points.size() << std::endl;

	double x, y, z;
	// remove body
	transformed_ptcld = close_ptcld;
	for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
		x = transformed_ptcld.points[i].x;
		y = transformed_ptcld.points[i].y;
		z = transformed_ptcld.points[i].z;
		if ((fabs(y)<eps+0.15)&&(fabs(z)<eps+0.10)&&(x<eps+0.05))
			blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
	}
	// remove left track
	if (tf_listener.waitForTransform(laser_frame, "/left_track", time,
			ros::Duration(1.))) {
		tf_listener.transformPointCloud("/left_track", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if
			((fabs(y)<eps+(0.097/2))&&(fabs(x)<eps+0.09+(0.4997/2))&&(z>-eps-0.0705)&&(z<eps+0.1095))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /left_track for robot_filter (ignoring this part).");
	}
	// remove left track
	if (tf_listener.waitForTransform(laser_frame, "/right_track", time,
			ros::Duration(1.))) {
		tf_listener.transformPointCloud("/right_track", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if
			((fabs(y)<eps+(0.097/2))&&(fabs(x)<eps+0.09+(0.4997/2))&&(z>-eps-0.0705)&&(z<eps+0.1095))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /right_track for robot_filter (ignoring this part).");
	}

	// remove front left flipper
	if (tf_listener.waitForTransform(laser_frame, "/front_left_flipper", time,
			ros::Duration(1.))) {
		tf_listener.transformPointCloud("/front_left_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<3*eps+0.090))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /front_left_flipper for robot_filter (ignoring this part).");
	}
	// remove front right flipper
	if (tf_listener.waitForTransform(laser_frame, "/front_right_flipper", time,
			ros::Duration(1.))) {
		tf_listener.transformPointCloud("/front_right_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<3*eps+0.090))
					// Beware 3*eps !
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /front_right_flipper for robot_filter (ignoring this part).");
	}
	// remove rear left flipper
	if (tf_listener.waitForTransform(laser_frame, "/rear_left_flipper", time,
			ros::Duration(1.))) {
		tf_listener.transformPointCloud("/rear_left_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<eps+0.090))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /rear_left_flipper for robot_filter (ignoring this part).");
	}
	// remove rear right flipper
	if (tf_listener.waitForTransform(laser_frame, "/rear_right_flipper", time,
			ros::Duration(1.))) {
		tf_listener.transformPointCloud("/rear_right_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<eps+0.090))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /rear_right_flipper for robot_filter (ignoring this part).");
	}
	
	// remove omnicam
	if (tf_listener.waitForTransform(laser_frame, "/omnicam", time,
			ros::Duration(1.))) {
		tf_listener.transformPointCloud("/omnicam", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((x*x+y*y<(eps+0.070)*(eps+0.070))&&(abs(z)<eps+(0.15/2)))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /omnicam for robot_filter (ignoring this part).");
	}

	// remove blacklisted points
	//std::cout << blacklisted.size() << " -> ";
	//blacklisted.sort();	// don't know if that's necessary
	//blacklisted.unique();
	//std::cout << blacklisted.size()<< std::endl;

	for (std::list<double>::const_iterator it=blacklisted.begin();it!=blacklisted.end();++it) {
		scan.ranges[int(*it)] = invalid;
	}

}


/*
 * Build 2D flat scan from slanted scan
 */
void NiftiLaserAssembler::gen2Dscan(const sensor_msgs::LaserScan& scan, double scan_angle)
{
	sensor_msgs::LaserScan out_scan = scan;
	const double z_min = -0.1332;
	const double z_max = 0.4868;
	const double invalid = scan.range_max+1;
	const double cos_scan = cos(scan_angle);
	const double sin_scan = sin(scan_angle);
	std::vector<double> tmp_r(scan.ranges.size());
	double z, ray_angle, old_angle;
	unsigned int i,j;

	if (cos_scan<0.1)
		return;

	// adjusting distance and filtering invalid points
	for (i=0; i<scan.ranges.size(); i++) {
		ray_angle = scan.angle_min + i*scan.angle_increment;
		z = scan.ranges[i]*sin(ray_angle)*sin_scan;
		
		if ((z>=z_min)&&(z<=z_max)) {
			tmp_r[i] = scan.ranges[i]*sqrt(cos(ray_angle)*cos(ray_angle)\
				+ sin(ray_angle)*sin(ray_angle)*cos_scan*cos_scan);
		} else {
			tmp_r[i] = invalid;
		}
	}
	// adjusting bearing (first version, should do interpolation)
	for (i=0; i<scan.ranges.size(); i++) {
		ray_angle = scan.angle_min + i*scan.angle_increment;
		old_angle = atan2(sin(ray_angle)/cos_scan, cos(ray_angle));
		if ((old_angle<scan.angle_min) || (old_angle>scan.angle_max)) {
			out_scan.ranges[i] = invalid;
		} else {
			j = int((old_angle-scan.angle_min)/scan.angle_increment);
			//ROS_INFO_STREAM(ray_angle << " " << old_angle << " " << j);
			old_angle = scan.angle_min + j*scan.angle_increment;
			out_scan.ranges[i] = tmp_r[j];
			out_scan.intensities[i] = scan.intensities[j];
		}
	}
	out_scan.header.frame_id = "/flat_scan";
	scan2dsyn_pub.publish(out_scan);
}



/* 
 * Map control callback
 */
void NiftiLaserAssembler::map_ctrl_cb(const std_msgs::Bool& on)
{
	ROS_INFO_STREAM("Mapping " << (on.data?"enabled.":"disabled."));
	map_ctrl_on = on.data;
}


/* 
 * Point cloud control callback
 */
void NiftiLaserAssembler::ptcld_ctrl_cb(const std_msgs::Bool& on)
{
	ROS_INFO_STREAM("Point cloud " << (on.data?"enabled.":"disabled."));
	ptcld_ctrl_on = on.data;
}


/*
 * laser scan callback
 */
void NiftiLaserAssembler::scan_cb(const sensor_msgs::LaserScan& scan)
{
	double angle;
	try
	{
		angle = get_laser_angle(scan.header.stamp);
	} 
	catch (tf::ExtrapolationException e) {
		ROS_ERROR_STREAM(e.what() << "Could not resolve rotating angle of the laser.");
		return;
	}
	//ROS_INFO_STREAM("Got scan");
	tmp_scan = scan;
	time_correct(tmp_scan);
	//filtering scan 
	undistort(tmp_scan); // disable due to feature freeze
	robot_filter(tmp_scan); // disabled due to feature freeze
	shadow_filter(tmp_scan);
	karto_filter(tmp_scan);

	//gen2Dscan(tmp_scan, angle);

	if (publish2d && map_ctrl_on) {
		if ((angle*previous_angle<=0.0) ||
				((fabs(angle-previous_angle)<0.5*M_PI/180.)&&
						(fabs(angle-laser_angle_offset)<10*M_PI/180.))) {
			ROS_DEBUG_STREAM("Publishing 2d scan.");
			if (using_gmapping)
			{
				sensor_msgs::LaserScan inv_scan = tmp_scan;
				invert_gmapping(tmp_scan, inv_scan);
				scan2d_pub.publish(inv_scan);
			} else {
				scan2d_pub.publish(tmp_scan);
			}
		}
	}

	if (relay_scans && map_ctrl_on) {
		relay_pub.publish(tmp_scan);
	}
	
	if (fabs(angle)<=M_PI/2) {
		//ROS_INFO_STREAM("Got scan in range.");
		if (start_time.isZero())
			start_time = tmp_scan.header.stamp;
		append_scan(tmp_scan);

	}

	if ((fabs(previous_angle-laser_angle_offset)<M_PI/2) &&
			(fabs(angle-laser_angle_offset)>=M_PI/2)) {
		std_msgs::Bool bool_msg;
		bool_msg.data = true;
		end_of_swipe_pub.publish(bool_msg);
		//ROS_INFO_STREAM("End");
		if (ptcld_ctrl_on){
			// transform point cloud into /base_link
			pcl_ros::transformPointCloud("base_link", point_cloud,
					tmp_point_cloud, tf_listener);
			dynamic_point_cloud_pub.publish(tmp_point_cloud);
			if (check_no_motion(tmp_scan.header.stamp)){
				ROS_DEBUG_STREAM("Publishing static point cloud (" << point_cloud.width << " points).");
				point_cloud_pub.publish(tmp_point_cloud);
			} else {
				ROS_DEBUG_STREAM("Point cloud in motion.");
			}
		} else {
			ROS_DEBUG_STREAM("Dropping point cloud (disabled).");
		}
		point_cloud.data.clear();
		point_cloud.width = 0;
		start_time = ros::Time(0);
	}

	// if point cloud is full, we publish it
	// TODO decide if relevant
	if (point_cloud.width>=(unsigned)max_size) {
		ROS_WARN_STREAM("max_size exceeded, clearing.");
		//point_cloud_pub.publish(point_cloud);
		point_cloud.data.clear();
		point_cloud.width = 0;
	}
	previous_angle = angle;
}


/*
 * Append a scan to the current point cloud
 */
void NiftiLaserAssembler::append_scan(const sensor_msgs::LaserScan& scan)
{
	// Project the LaserScan into a PointCloud2
	if (!tf_listener.waitForTransform(laser_frame, world_frame, scan.header.stamp
			+ ros::Duration ().fromSec (scan.scan_time)
			, ros::Duration(2.)))
		ROS_WARN_STREAM("Timeout (2s) while waiting between "<<laser_frame<<
				" and "<<world_frame<<" before transformation.");
	try
	{
		projector.transformLaserScanToPointCloud(world_frame, scan,
				tmp_point_cloud, tf_listener, point_cloud_channels);
	}
	catch(tf::ExtrapolationException& ex) 
	{
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return;
	}


	// Aggregate the new point cloud into the current stack
	if (point_cloud.width <= 0)
		point_cloud = tmp_point_cloud;
	else {
		// TODO check we don't go over max_size
		point_cloud.header = tmp_point_cloud.header;
		point_cloud.width += tmp_point_cloud.width;
		point_cloud.row_step += tmp_point_cloud.row_step;
		point_cloud.data.insert(point_cloud.data.end(),
				tmp_point_cloud.data.begin(), tmp_point_cloud.data.end());
	}
}


/*
 * Get the laser angle from the tf_listener
 */
double NiftiLaserAssembler::get_laser_angle(const ros::Time &time) const
{
	double angle;
	tf::StampedTransform tmp_tf;
	geometry_msgs::Quaternion rot;
	if (!tf_listener.waitForTransform(robot_frame, laser_frame, time,
		ros::Duration(1.)))
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and "<<robot_frame<<" before getting laser angle.");
	
	tf_listener.lookupTransform(robot_frame, laser_frame, time, tmp_tf);
	
		
	tf::quaternionTFToMsg(tmp_tf.getRotation(), rot);
	angle = 2.*atan2(rot.x, rot.w) - M_PI;
	if (angle>M_PI)
		angle -= 2.*M_PI;
	else if (angle<-M_PI)
		angle += 2.*M_PI;

	return angle;
}


double norm(const geometry_msgs::Vector3& vec3){
	return sqrt(vec3.x*vec3.x+vec3.y*vec3.y+vec3.z*vec3.z);
}
/*
 * Decide if the robot was still
 */
bool NiftiLaserAssembler::check_no_motion(const ros::Time &time) const
{
	// Checking with velocity but why not position?
	geometry_msgs::Twist mean_speed;
	ros::Duration delta = time - start_time;
	if (delta>=ros::Duration(59.))
		return false;
	if (!tf_listener.waitForTransform(robot_frame, world_frame, time,
		ros::Duration(1.)))
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<robot_frame<<
				" and "<<world_frame<<" before checking motion.");
	tf_listener.lookupTwist(robot_frame, world_frame, start_time+delta*0.5,
			delta, mean_speed);
	ROS_DEBUG_STREAM("Motion: " << norm(mean_speed.linear) << " m/s, " <<
			norm(mean_speed.angular) << " Rad/s for "<<delta.toSec()<<" s"); 
	return ((norm(mean_speed.linear)*delta.toSec()<0.02)&&
			(norm(mean_speed.angular)*delta.toSec()<3*M_PI/180.));
}


/*
 * Main function
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "nifti_laser_assembler");

	NiftiLaserAssembler nla;

	ros::spin();

	return 0;
}

