#!/bin/sh
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

##########
# README #
##########
#
# Update the copyright header with the current date and make the
# copyright notice uniform.

year=`date '+%Y'`

find . -name CMakeLists.txt	\
    -or -name "*.cmake"		\
    -or -name "*.sh"		\
    -or -name "*.py"		\
    -and -not -name "git-archive-all.sh"   \
    -and -not -name "FindOpenRTM.cmake"    \
    -and -not -name "GNUInstallDirs.cmake" \
    | xargs sed -i "s|^// Copyright .*$|# Copyright (C) 2008-$year LAAS-CNRS, JRL AIST-CNRS.|"

find . -name "*.hh" -or -name "*.hxx" -or -name "*.h" -or -name "*.hpp" \
    -or -name "*.cpp" -or -name "*.cc" \
    | xargs sed -i "s|^// Copyright .*$|// Copyright (C) 2008-$year LAAS-CNRS, JRL AIST-CNRS.|"
