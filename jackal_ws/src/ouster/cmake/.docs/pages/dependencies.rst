Dependencies
************

pkg-config dependencies
=======================

This set of macros allow to specifiy dependencies on packages that use
pkg-config to specifiy their own dependencies, compilation flags and link
flags. In particular, it allows to specify dependencies towards other projects built using the JRL CMake modules.

.. setmode:: import

.. cmake-module:: ../../pkg-config.cmake

pkg-config generation
=====================

.. setmode:: export

.. cmake-module:: ../../pkg-config.cmake

.. setmode:: user

External dependencies
=====================

Eigen
-----
.. cmake-module:: ../../eigen.cmake
Boost
-----
.. cmake-module:: ../../boost.cmake
Python
------
.. cmake-module:: ../../python.cmake

Advanced
========

pkg-config dependencies
-----------------------

.. setmode:: import-advanced

.. cmake-module:: ../../pkg-config.cmake

pkg-config generation
---------------------

.. setmode:: export-advanced

.. cmake-module:: ../../pkg-config.cmake

.. setmode:: user
