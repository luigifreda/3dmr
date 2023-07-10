CMake packages
**************

This modules allow to work with the target-based approach of CMake introduced in CMake 2.8.12.

Generating a CMake package
==========================

You should first add the line ``set(PROJECT_USE_CMAKE_EXPORT TRUE)`` in the header of your project.

Compared to the default minimal project one needs to ``EXPORT ${TARGETS_EXPORT_NAME}``
for at least one target the project should export.

Note that all exported targets will be in the ``${PROJECT_NAME}`` namespace.

Extra information
-----------------

In some situations you might want to add some extra information to the
generated config script. This can be done by manipulating the
``PROJECT_EXTRA_MACROS`` variable. The content of this variable will be
appended to the end of the config script.

Consuming packages
==================

.. cmake-module:: ../../package-config.cmake

Minimal working example with CMake packages
===========================================

.. literalinclude:: ../examples/minimal-with-packages.cmake
  :language: cmake
