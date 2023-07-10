# jackal 3D

This workspace collects a set of tools for simulating jackal UGVs under gazebo. The jackal model is equipped with camera and LIDAR (ouster of velodyne).  

### Run

To launch 2 jackals at the same time, run: 
```
rosrun jackal_3dmain jackal_ugvs.sh
```

To launch a single jackal with lidar:
```
rosrun jackal_3dmain jackal.sh
```

### Stop everything

In order to stop the simulation and running ros nodes, you can run 
```
rosrun jackal_3dmain kill_all.sh
```
