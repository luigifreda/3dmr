^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.6 (2021-07-18)
------------------

0.7.5 (2021-03-24)
------------------

0.7.4 (2021-03-16)
------------------
* Bumped CMake version to avoid author warning.
* Contributors: Tony Baltovski

0.7.3 (2021-03-08)
------------------

0.7.2 (2020-09-29)
------------------
* Use the JACKAL_LASER_TOPIC env var as the default for the scan topic in amcl + gmapping demos (`#74 <https://github.com/jackal/jackal/issues/74>`_)
* Contributors: Chris I-B

0.7.1 (2020-08-24)
------------------
* Remove the leading / from the gmapping default scan topic
* Expose the scan_topic and use_map_topic parameters in the demo launch files
* Contributors: Chris Iverach-Brereton

0.7.0 (2020-04-20)
------------------

0.6.4 (2020-03-04)
------------------

0.6.3 (2019-07-18)
------------------

0.6.2 (2019-04-18)
------------------

0.6.1 (2018-08-02)
------------------

0.6.0 (2018-04-12)
------------------

0.5.4 (2018-04-12)
------------------

0.5.3 (2016-06-01)
------------------

0.5.1 (2015-02-02)
------------------

0.5.0 (2015-01-20)
------------------

0.4.2 (2015-01-14)
------------------

0.4.1 (2015-01-07)
------------------

0.4.0 (2014-12-12)
------------------
* Removed redundant local and global costmaps.
* Fixed location of non-demo launch files in roslaunch-file-check in CMakeList
* Moved rviz configurations to jackal_viz.
* Changed config/ to params/. Moved non-demo launch files to launch/include.
* Added gps_transform.launch for transforming GPS data to robot's world frame. Also added gps_integration_demo.launch for fusing GPS data with odomtery data. This node generates map->odom transform.
* Modified AMCL's parameters, map and demo for Jackal.
* Modified slam_gmapping parameters for Jackal for building a map.
* Modified costmap and base planner parameters for the navigation without map with obstacle avoidance using laser scanner.
* Contributors: Mike Purvis, Shokoofeh Pourmehr

0.3.0 (2014-09-10 16:25)
------------------------

0.2.1 (2014-09-10 08:54)
------------------------

0.2.0 (2014-09-09)
------------------

0.1.1 (2014-09-06)
------------------

0.1.0 (2014-09-05)
------------------
