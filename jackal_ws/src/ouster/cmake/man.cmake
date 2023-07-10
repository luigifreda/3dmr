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

# MANPAGE
# -------
#
# Generate a pod man page from a template, then
# generate the man page and compress it.
# This macro also adds the installation rules.
#
MACRO(MANPAGE NAME)
  IF(WIN32)
    MESSAGE(FATAL_ERROR "This macro is not supported on Microsoft Windows.")
  ENDIF(WIN32)

  FIND_PROGRAM(POD2MAN pod2man)
  FIND_PROGRAM(GZIP gzip)
  CONFIGURE_FILE(${NAME}.pod.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.pod @ONLY)

  ADD_CUSTOM_COMMAND(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.1
    COMMAND ${POD2MAN} --section=1
            --center="LOCAL USER COMMANDS"
	    --release ${PROJECT_NAME} ${NAME}.pod
	    > ${NAME}.1
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.pod)

  ADD_CUSTOM_COMMAND(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.1.gz
    COMMAND ${GZIP} -c ${NAME}.1 > ${NAME}.1.gz
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.1)

  # Trigger man page generation at install.
  INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_MAKE_PROGRAM} man)")

  # Detects if PKGMAN has been specified
  SET(DESTINATION_MAN_PAGE share/man/man1)
  IF(MANDIR)
    SET(DESTINATION_MAN_PAGE ${MANDIR}/man1)
  ENDIF(MANDIR)

  # Install man page.
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.1.gz
    DESTINATION ${DESTINATION_MAN_PAGE})

  LIST(APPEND LOGGING_WATCHED_VARIABLES
    POD2MAN
    GZIP
    )
ENDMACRO(MANPAGE)
