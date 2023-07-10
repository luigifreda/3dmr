^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package message_logger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2016-12-30)
------------------
* renamed USE_COUT to MELO_USE_COUT, added MELO_FUNCTION_PRINTS flag to prepend class::function names to the log messages
* added missing macros
* printing function names
* removed ROS implementation of *_FP loggers
* std output: removed CTRL strings, color code is now equal to ros
* Fixed cmake printout bug whether ros or cout logging will be used
* replaced USE_ROS with USE_COUT flag
* Added 'conventient' header file which includes all necessary headers
* removed exception from log level 'error'
* Copied roco logging
* Contributors: Philipp Leemann, Remo Diethelm
