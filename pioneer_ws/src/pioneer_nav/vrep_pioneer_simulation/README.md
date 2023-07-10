# V-REP UGV Simulation

This package contains some V-REP launchers and scene descriptions.

In the folder `data`, you can find some pre-built UGV environments:

- environments with prefix `ugv1` or `ugv1n2` generates topics with prefixes `/ugv1` and `/ugv2`: here slam is faked by V-REP

- environments with prefix `ugv1` or `ugv1n2` and suffix `mapping` generates topics with prefixes `/ugv1` and `/ugv2` and must be used with laser_mapper

- environments with prefix `robot_` generates topics as the real robot 
