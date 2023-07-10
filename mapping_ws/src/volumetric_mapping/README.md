# volumetric_mapping
A repository for 3D volumetric (occupancy) maps, providing a generic interface for disparity map and pointcloud insertion, and support for custom sensor error models.

## Packages
**volumetric_map_base** - base class/package that all volumetric maps should inherit from, contains methods to handle ROS disparity maps and pointclouds.

**volumetric_msgs** - collection of messages for interacting with various maps.

**octomap_world** - an octomap-based volumetric representation, both with a library and a stand-alone ROS node.

## Dependencies
In addition to `ros-indigo-desktop-full`, please install:
```
sudo apt-get install ros-indigo-octomap-mapping
```

And the following packages:
[minkindr](https://github.com/ethz-asl/minkindr)
[minkindr_ros](https://github.com/ethz-asl/minkindr_ros)
[eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
[glog_catkin](https://github.com/ethz-asl/glog_catkin)
[gflags_catkin](https://github.com/ethz-asl/gflags_catkin)

On Mac OS X, see the [mav_tools Wiki instructions](https://github.com/ethz-asl/mav_tools/wiki/Install-the-ASL-MAV-framework#install-extra-stock-ros-packages-octomap-ompl-etc).

## Libraries
**[OctomapWorld](https://github.com/ethz-asl/volumetric_mapping/blob/master/octomap_world/include/octomap_world/octomap_world.h)** - general library for handling insertion of pointclouds, can be run outside of a ROS node, and takes parameters as a struct.

**[OctomapManager](https://github.com/ethz-asl/volumetric_mapping/blob/master/octomap_world/include/octomap_world/octomap_manager.h)** - inherits from OctomapWorld, essentially a ROS wrapper for it. Reads parameters in from the ROS parameter server.

## Nodes
### octomap_manager
Listens to disparity and pointcloud messages and adds them to an octomap.

#### Parameters
* `tf_frame` (string, default: "/world") - tf frame name to use for the world.
* `resolution` (double, default: 0.15) - resolution each grid cell in meters.
* `Q` (vector of doubles (representing 4x4 matrix, row-major)) - Q projection matrix for disparity projection, in case camera info topics are not available.
* `map_publish_frequency` (double, default: 0.0) - Frequency at which the Octomap is published for visualization purposes. If set to < 0.0, the Octomap is not regularly published (use service call instead).
* `octomap_file` (string, default: "") - Loads an octomap from this path on startup. Use `load_map` service below to load a map from file after startup.

For other parameters, see [octomap_world.h](https://github.com/ethz-asl/volumetric_mapping/blob/master/octomap_world/include/octomap_world/octomap_world.h#L16-L24).

#### Subscribed Topics
* `disparity` ([stereo_msgs/DisparityImage]) - disparity image to subscribe to.
* `pointcloud` ([sensor_msgs/PointCloud2]) - pointcloud to subscribe to.
* `cam0/camera_info` ([sensor_msgs/CameraInfo]) - left camera info.
* `cam1/camera_info` ([sensor_msgs/CameraInfo]) - right camera info.

#### Published Topics
* `octomap_occupied` ([visualization_msgs/MarkerArray]) - marker array showing occupied octomap cells, colored by z.
* `octomap_free` ([visualization_msgs/MarkerArray]) - marker array showing free octomap cells, colored by z.
* `octomap_full` ([octomap_msgs/Octomap]) - octomap with full probabilities.
* `octomap_binary` ([octomap_msgs/Octomap]) - octomap with binary occupancy - free or occupied, taken by max likelihood of each node.

#### Services
* `reset_map` ([std_srvs/Empty]) - clear the map.
* `publish_all` ([std_srvs/Empty]) - publish all the topics in the above section.
* `get_map` ([octomap_msgs/GetOctomap]) - returns binary octomap message.
* `save_map` ([volumetric_msgs/SaveMap]) - save map to the specified `file_path`.
* `load_map` ([volumetric_msgs/LoadMap]) - load map from the specified `file_path`.

## Running
Run an octomap manager, and load a map from disk, then publish it in the `map` tf frame:

```
rosrun octomap_world octomap_manager _tf_frame:=map
rosservice call /octomap_manager/load_map /home/helen/data/my_awesome_octomap.bt
rosservice call /octomap_manager/publish_all
```

[std_srvs/Empty]: http://docs.ros.org/indigo/api/std_srvs/html/srv/Empty.html
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[stereo_msgs/DisparityImage]: http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[octomap_msgs/Octomap]: http://docs.ros.org/indigo/api/octomap_msgs/html/msg/Octomap.html
[octomap_msgs/GetOctomap]: http://docs.ros.org/indigo/api/octomap_msgs/html/srv/GetOctomap.html
[visualization_msgs/MarkerArray]: http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html
[volumetric_msgs/LoadMap]: https://github.com/ethz-asl/volumetric_mapping/blob/master/volumetric_msgs/srv/LoadMap.srv
[volumetric_msgs/SaveMap]: https://github.com/ethz-asl/volumetric_mapping/blob/master/volumetric_msgs/srv/SaveMap.srv
