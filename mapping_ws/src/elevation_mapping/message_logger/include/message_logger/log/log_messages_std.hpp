/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Christian Gehring
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
* @file     log_messages_std.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once


#include "message_logger/log/log_messages_backend.hpp"
#include "message_logger/time/TimeStd.hpp"


namespace message_logger {
namespace log {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG(level, ...) \
{ \
  switch (level) { \
  case message_logger::log::levels::Fatal: \
    { \
      std::stringstream melo_assert_stringstream; \
      melo_assert_stringstream << message_logger::log::colorFatal << message_logger::common::internal::melo_string_format(__VA_ARGS__) << message_logger::log::getResetColor(); \
      message_logger::common::internal::melo_throw_exception<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__,__FILE__,__LINE__, melo_assert_stringstream.str()); \
    } \
    break; \
  default: \
    { \
      std::stringstream melo_stringstream; \
      melo_stringstream << message_logger::log::getLogColor(level) << "[" << message_logger::log::getLogLevel(level)  << "] " << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level) << message_logger::common::internal::melo_string_format(__VA_ARGS__) << message_logger::log::getResetColor(); \
      std::cout << melo_stringstream.str() << std::endl; \
    } \
    break; \
  } \
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_STREAM(level, message) \
    { \
      switch (level) { \
      case message_logger::log::levels::Fatal: \
        { \
          std::stringstream melo_assert_stringstream;             \
          melo_assert_stringstream << message_logger::log::colorFatal << message << message_logger::log::getResetColor(); \
          message_logger::common::internal::melo_throw_exception<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__,__FILE__,__LINE__, melo_assert_stringstream.str()); \
        } \
        break; \
      default: \
        { \
          std::stringstream melo_stringstream; \
          melo_stringstream << message_logger::log::getLogColor(level) << "[" << message_logger::log::getLogLevel(level)  << "] " << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level) << message << message_logger::log::getResetColor(); \
          std::cout << melo_stringstream.str() << std::endl; \
        } \
        break; \
      } \
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_FP(level, ...) \
    { \
      std::stringstream melo_stringstream; \
      message_logger::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
      melo_stringstream << message_logger::log::getLogColor(level) << "[" << message_logger::log::getLogLevel(level)  << "] " <<  sfp.toString() << " " << message_logger::common::internal::melo_string_format(__VA_ARGS__) << message_logger::log::getResetColor(); \
      std::cout << melo_stringstream.str() << std::endl; \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_STREAM_FP(level, message) \
    { \
      std::stringstream melo_stringstream; \
      message_logger::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
      melo_stringstream << message_logger::log::getLogColor(level) << "[" << message_logger::log::getLogLevel(level)  << "] " <<  sfp.toString() << " " << message << message_logger::log::getResetColor(); \
      std::cout << melo_stringstream.str() << std::endl; \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_ONCE(level, ...) \
    { \
      static bool once_hit = false; \
      if (!once_hit) \
      { \
        once_hit = true; \
        MELO_LOG(level, __VA_ARGS__) \
      } \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_STREAM_ONCE(level, message) \
  { \
    static bool once_hit = false; \
    if (!once_hit) \
    { \
      once_hit = true; \
      MELO_LOG_STREAM(level, message) \
    } \
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_THROTTLE(rate, level, ...) \
    { \
      static double last_hit = 0.0; \
      ::message_logger::time::TimeStd now = ::message_logger::time::TimeStd::now(); \
      if (last_hit + rate <= now.toSec()) \
      { \
        last_hit = now.toSec(); \
        MELO_LOG(level, __VA_ARGS__) \
      } \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_THROTTLE_STREAM(rate, level, message) \
  { \
    static double last_hit = 0.0; \
    ::message_logger::time::TimeStd now = ::message_logger::time::TimeStd::now(); \
    if (last_hit + rate <= now.toSec()) \
    { \
      last_hit = now.toSec(); \
      MELO_LOG_STREAM(level, message) \
    } \
}

} // namespace log
} // namespace message_logger
