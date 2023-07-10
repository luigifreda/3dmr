Point Cloud IO
======================

Overview
---------------

These are two simple [ROS] point cloud helper nodes. **_read_** reads a point cloud from file (ply or vtk) and publishes it as a [sensor_msgs/PointCloud2] message. **_write_** subscribes to a [sensor_msgs/PointCloud2] topic and writes received messages to seperate files (ply, pcd).

For visualization, make sure to set the **Decay Time** in the **PointCloud2** tab in [rviz] to a high number to get the point cloud visible for a long time.

The point cloud io package has been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: Péter Fankhauser, Remo Diethelm<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Remo Diethelm, rdiethelm@anybotics.com<br />**

This projected was initially developed at ETH Zurich (Autonomous Systems Lab & Robotic Systems Lab).

[This work is conducted as part of ANYmal Research, a community to advance legged robotics.](https://www.anymal-research.org/)

The source code is released under a [BSD 3-Clause license](LICENSE).

Installation
------------

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the it depends on following software:

- [Point Cloud Library (PCL)](http://pointclouds.org/).


### Building

In order to install the Point Cloud IO, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd ~/catkin_workspace/src
    git clone https://github.com/anybotics/point_cloud_io.git
    cd ../
    catkin_make


Usage
------------

To create your own launch-file, you can use the examples from `point_cloud_io/launch/...`.


### Read

Load and publish a ply or vtk file with

    rosrun point_cloud_io read _file_path:=/home/user/my_point_cloud.ply _topic:=/my_topic _frame:=/sensor_frame

Optionally, you can also add `_rate:=1.0` to have the node publish your point cloud at the specified rate.


### Write

Subscribe and save point clouds to a ply file with

    rosrun point_cloud_io write _topic:=/your_topic _folder_path:=/home/user/my_point_clouds

Optionally, you can set parameters to fit the point cloud file names to your needs:

- `_file_prefix:=my_prefix` (default: "point_cloud")
- `_file_ending:=my_ending` (default: "ply", currently only format which is supported for writing)
- `_add_counter_to_path:=false` (default: `true`)
- `_add_frame_id_to_path:=true` (default: `false`)
- `_add_stamp_sec_to_path:=true` (default: `false`)
- `_add_stamp_nsec_to_path:=true` (default: `false`)


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/anybotics/point_cloud_io/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
