# New Features coming with 3DMR

This is a list of new features we added with respect to our previous framework **[3dpatrolling](https://github.com/luigifreda/3dpatrolling)**:
- An implementation of **our new exploration method**.
- Support under Ubuntu 20 and ROS noetic. We originally developed many parts under Ubuntu 18.04 and then we ported the framework under Ubuntu 18.04.   
- Support for **gazebo** with jackal robots equipped with Ouster/Velodyne LIDARs, IMUs and cameras.
- Support for pioneer 3-DX robots equipped with RGBD cameras under V-REP.  
- Integrated [TEB local planner](http://wiki.ros.org/teb_local_planner) with 3D navigation: we use as input the waypoints computed by the path planner and get dynamic obstacle descriptions from the laser proximity checker.
- Integrated [voxblox](https://github.com/ethz-asl/voxblox) as an alternative mapping system to octomap
- Improved packages (e.g. `path_planner` and `trajectory_control`), parameter management and launch scripts.
- Added new PyQt GUIs to support different robots.   
- Integrated robot-centric [elavation mapping](https://github.com/ANYbotics/elevation_mapping).  
- Included an [nbvplanner](https://github.com/ethz-asl/nbvplanner) version working under Ubuntu 20 and ROS noetic. 
  
Some of the new features are still experimental. You can contribute to the code base by pull requests, reporting bugs, leaving comments, and proposing new features through issues. Feel free to get in touch at *luigifreda(at)gmail(dot)com*. Thank you!
