# Copyright (C) 2008-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA.
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
# .. ifmode:: user
#
# .. command:: CHECK_DEBIAN
#
#   Checks is the current system is Debian based
#   You can then use DEBIAN_FOUND
#
MACRO(CHECK_DEBIAN)
  FIND_FILE(DEBIAN_FOUND debian_version debconf.conf
    PATHS /etc)
ENDMACRO(CHECK_DEBIAN)


#.rst:
# .. ifmode:: user
#
# .. command:: CHECK_NETBSD
#
#   Checks is the current system is NetBSD
#   You can then use NETBSD_FOUND
#
MACRO(CHECK_NETBSD)
  FIND_FILE(NETBSD_FOUND netbsd
    PATHS /)
ENDMACRO(CHECK_NETBSD)


#.rst:
# .. ifmode:: user
#
# .. command:: CHECK_ARCHLINUX
#
#   Checks is the current system is ArchLinux
#   You can then use ARCHLINUX_FOUND
#
MACRO(CHECK_ARCHLINUX)
  FIND_FILE(ARCHLINUX_FOUND arch-release
    PATHS /etc)
ENDMACRO(CHECK_ARCHLINUX)

