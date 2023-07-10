# Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

#.rst:
# .. command:: DISTCHECK_SETUP
#
#   .. _target-distcheck:
#
#   Add a *distcheck* target to check the generated tarball.
#
#   This step calls ``make distdir`` to generate a copy of the project without
#   the git history and with the ``.version`` file (as it will be when an user
#   will retrieve a stable version).
#   Then:
#
#   - create ``_build`` and ``_inst`` to respectively create a build and an installation
#     directory.
#   - copy the ``CMakeCache.txt`` file.
#   - run ``cmake`` with ``_inst`` as the installation prefix
#   - run ``make``, ``make check``, ``make install`` and ``make uninstall``
#   - remove ``_build`` and ``_inst``.
#
#   During the compilation phase, all files in the source tree are modified
#   to *not* be writeable to detect bad compilation steps which tries to modify
#   the source tree. Permissions are reverted at the end of the check.
#
MACRO(DISTCHECK_SETUP)
  IF(UNIX)
    FIND_PROGRAM(SED sed)
    SET(SRCDIR ${CMAKE_BINARY_DIR}/${PROJECT_NAME}${PROJECT_SUFFIX}-${PROJECT_VERSION})
    SET(BUILDDIR ${SRCDIR}/_build)
    SET(INSTDIR ${SRCDIR}/_inst)
    SET(TEST_RESULTS_DIR ${BUILDDIR}/test_results)
    SET(DISTCHECK_MAKEFLAGS "" CACHE PATH "MAKEFLAGS used for distcheck's make")
      
    # The -i argument of sed command differs according on APPLE systems
    IF(APPLE)
      SET(SED_I_OPTION "-i'.old' ")
    ELSE(APPLE)
      SET(SED_I_OPTION "-i ")
    ENDIF(APPLE)

    #Set LD_LIBRARY_PATH
    IF(APPLE)
      SET(LD_LIBRARY_PATH_VARIABLE_NAME "DYLD_LIBRARY_PATH") 
    ELSE(APPLE)
      SET(LD_LIBRARY_PATH_VARIABLE_NAME "LD_LIBRARY_PATH") 
    ENDIF(APPLE)

    ADD_CUSTOM_TARGET(distcheck
      COMMAND
         export LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}
      && export ${LD_LIBRARY_PATH_VARIABLE_NAME}=$ENV{${LD_LIBRARY_PATH_VARIABLE_NAME}}
      && export PYTHONPATH=$ENV{PYTHONPATH}
      && find . -type d -print0 | xargs -0 chmod a-w
      && chmod u+w . && rm -rf _build _inst && mkdir -p _build && mkdir -p _inst
      && chmod u+rwx _build _inst && chmod a-w .
      && cp ${CMAKE_BINARY_DIR}/CMakeCache.txt _build/
      && ${SED} ${SED_I_OPTION} -e "'s|${CMAKE_BINARY_DIR}|${BUILDDIR}|g'" 
                                _build/CMakeCache.txt # Change previous binary dir by the current _build one 
      && ${SED} ${SED_I_OPTION} -e "'s|${CMAKE_SOURCE_DIR}|${SRCDIR}|g'" 
                                _build/CMakeCache.txt # Change previous source dir to the source one 
      && ${SED} ${SED_I_OPTION} -e "'s|CMAKE_CXX_COMPILER:FILEPATH=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS:STRING=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_DEBUG:STRING=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_MINSIZEREL:STRING=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_RELEASE:STRING=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING=.\\+||g'"
                                -e "'s|CMAKE_CXX_COMPILER-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_CXX_COMPILER_WORKS:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_DEBUG-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_MINSIZEREL-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_RELEASE-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_CXX_FLAGS_RELWITHDEBINFO-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_DETERMINE_CXX_ABI_COMPILED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_C_COMPILER:FILEPATH=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS:STRING=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_DEBUG:STRING=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_MINSIZEREL:STRING=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_RELEASE:STRING=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_RELWITHDEBINFO:STRING=.\\+||g'"
                                -e "'s|CMAKE_C_COMPILER-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_DEBUG-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_MINSIZEREL-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_RELEASE-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_C_FLAGS_RELWITHDEBINFO-ADVANCED:INTERNAL=.\\+||g'"
                                -e "'s|CMAKE_DETERMINE_C_ABI_COMPILED:INTERNAL=.\\+||g'"
                                _build/CMakeCache.txt
      && cd _build
      && cmake -DCMAKE_INSTALL_PREFIX=${INSTDIR} -DCATKIN_TEST_RESULTS_DIR=${TEST_RESULTS_DIR} .. || cmake ..
         || (echo "ERROR: the cmake configuration failed." && false)
      && make ${DISTCHECK_MAKEFLAGS}
         || (echo "ERROR: the compilation failed." && false)
      && make test
         || (echo "ERROR: the test suite failed." && false)
      && make install
         || (echo "ERROR: the install target failed." && false)
      && make uninstall
         || (echo "ERROR: the uninstall target failed." && false)
      && test `find ${INSTDIR} -type f | wc -l` -eq 0
         || (echo "ERROR: the uninstall target does not work." && false)
      && make clean
         || (echo "ERROR: the clean target failed." && false)
      && cd ${CMAKE_BINARY_DIR}/${PROJECT_NAME}${PROJECT_SUFFIX}-${PROJECT_VERSION}
      && chmod u+w . _build _inst && rm -rf _build _inst
      && find . -type d -print0 | xargs -0 chmod u+w
      && echo "=============================================================="
      && echo "${PROJECT_NAME}${PROJECT_SUFFIX}-${PROJECT_VERSION}"
              "is ready for distribution."
      && echo "=============================================================="
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME}${PROJECT_SUFFIX}-${PROJECT_VERSION}
      COMMENT "Checking generated tarball..."
      )
    ADD_DEPENDENCIES(distcheck distdir)
  ELSE()
    #FIXME: what to do here?
  ENDIF()
ENDMACRO()
