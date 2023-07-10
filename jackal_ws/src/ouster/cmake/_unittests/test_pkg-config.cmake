include(cmake/pkg-config.cmake)

cmake_policy(SET CMP0054 NEW)
#cmake -P
macro(EXPECT_STREQUAL _lhs _rhs)
  if(NOT "${_lhs}" STREQUAL ${_rhs})
    message(SEND_ERROR "EXPECT_STREQUAL failed: \"${_lhs}\" != \"${_rhs}\"")
  endif()
endmacro()

macro(UNSET_TEST_VARS)
  UNSET(PKG_LIB_NAME)
  UNSET(PKG_PREFIX)
  UNSET(PKG_CONFIG_STRING_NOSPACE)
endmacro()

UNSET_TEST_VARS()
_PARSE_PKG_CONFIG_STRING("my-package > 0.4" PKG_LIB_NAME PKG_PREFIX PKG_CONFIG_STRING_NOSPACE)
EXPECT_STREQUAL("my-package" "${PKG_LIB_NAME}")
EXPECT_STREQUAL("MY_PACKAGE" "${PKG_PREFIX}")
EXPECT_STREQUAL("my-package>0.4" "${PKG_CONFIG_STRING_NOSPACE}")

UNSET_TEST_VARS()
_PARSE_PKG_CONFIG_STRING("my-package >= 0.4" PKG_LIB_NAME PKG_PREFIX PKG_CONFIG_STRING_NOSPACE)
EXPECT_STREQUAL("my-package" "${PKG_LIB_NAME}")
EXPECT_STREQUAL("MY_PACKAGE" "${PKG_PREFIX}")
EXPECT_STREQUAL("my-package>=0.4" "${PKG_CONFIG_STRING_NOSPACE}")

UNSET_TEST_VARS()
_PARSE_PKG_CONFIG_STRING("my-package" PKG_LIB_NAME PKG_PREFIX PKG_CONFIG_STRING_NOSPACE)
EXPECT_STREQUAL("my-package" "${PKG_LIB_NAME}")
EXPECT_STREQUAL("MY_PACKAGE" "${PKG_PREFIX}")
EXPECT_STREQUAL("my-package" "${PKG_CONFIG_STRING_NOSPACE}")

# it the input does not have spaces around the operator,
# the operator is considered as being part of the library name.
# This is expected and consistent with pkg-config's behavior.
UNSET_TEST_VARS()
_PARSE_PKG_CONFIG_STRING("my-package>=0.4" PKG_LIB_NAME PKG_PREFIX PKG_CONFIG_STRING_NOSPACE)
EXPECT_STREQUAL("my-package>=0.4" "${PKG_LIB_NAME}")
EXPECT_STREQUAL("MY_PACKAGE__0_4" "${PKG_PREFIX}")
EXPECT_STREQUAL("my-package>=0.4" "${PKG_CONFIG_STRING_NOSPACE}")
