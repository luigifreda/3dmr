^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2020-08-24)
------------------
* Enable the joystick by default. Add yaw to the spawn_jackal launch file
* Add an additional parameter to enable teleop in the simulations
* Fix an accidental deletion of a closing tag
* Move the jackal-spawning into a separate launch file for compatibility with the new sim environments.  Add additional sim worlds: completely empty (useful for replaying bag files w/o risk of obstacle collisions!) and HRTAC since the world was in the repo, but not actually used
* Contributors: Chris I-B, Chris Iverach-Brereton, Dave Niewinski, Tony Baltovski

0.3.0 (2015-01-20)
------------------
* Add small hack to continue supporting the front_laser:=true arg, since that was prominently documented.
* Change from individual accessory args to a single "config" arg.
* Contributors: Mike Purvis

0.2.3 (2014-12-12)
------------------
* Added jackal_race world.
* Add hector_gazebo_plugins dependency.
* Contributors: Mike Purvis, spourmehr

0.2.2 (2014-09-10)
------------------
* Add author tags.
* Added launch arg to enable front-facing laser.
* Contributors: Mike Purvis

0.2.1 (2014-09-10)
------------------
* Install all directories.
* Contributors: Mike Purvis

0.2.0 (2014-09-09)
------------------
* Default world for Jackal sim is now a green one.
* Add missing dependencies on Gazebo plugin packages.
* Contributors: Mike Purvis

0.1.0 (2014-09-07)
------------------
* Initial version
* Contributors: Mike Purvis
