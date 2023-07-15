# expl_pointcloud_filter



<p align="center">
<img src="../../../images/expl_pointcloud_filter.png" alt="expl_pointcloud_filter" height="250" border="1" />
</p>

This experimental package `expl_pointcloud_filter` allows processing an input 3D scan, downsampling it and generating a lower-resolution output 3D scan that includes information about maximum range readings. See this [discussion](../../../README.exploration.md#exploration-configuration-and-maximum-sensor-readings).  

The output scans of an `expl_pointcloud_filter` node should be carefully managed to avoid confusing the maximum readings as "obstacle" readings. Our `volumetric_mapping` and `volumetric_mapping_expl` nodes already take care of that if the values of their parameter `sensor_max_range` is kept smaller than than the max sensor reading value used by the `expl_pointcloud_filter` node (by default 1000m). 

 You can test the `expl_pointcloud_filter` with jackal by setting `ENABLE_EXPL_POINTCLOUD_FILTER=1` in the script `jackal_ws/src/jackal_3dexplorer/scripts/sim_launcher_exploration`.