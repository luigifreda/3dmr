cmake_minimum_required(VERSION 3.1)
project(ouster_os1_64)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch meshes urdf worlds)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

catkin_install_python(PROGRAMS tests/test_ouster.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    ADD_ROSTEST("launch/test_ouster.test")
endif()
