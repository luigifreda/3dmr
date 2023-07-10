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
* @file     log_messages_backend.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once

#include "message_logger/common/preprocessor_defines.hpp"
#include "message_logger/common/colors.hpp"

#include "message_logger/log/log_messages_backend_config.hpp"

// todo: replace with std as soon as gcc 4.9.x is standard in ubuntu repo
#include <boost/regex.hpp>

namespace message_logger {
namespace log {

namespace levels
{
enum Level
{
  Debug,
  Info,
  Warn,
  Error,
  Fatal,

  Count
};
} // namespace levels

typedef levels::Level Level;

const std::string colorDebug = color::green;
const std::string colorInfo = color::def;
const std::string colorWarn = color::yellow;
const std::string colorFatal = color::red;
const std::string colorError = color::red;
const std::string colorFunction = color::cyan;

inline const std::string getResetColor() {
  return color::def;
}

inline const std::string getLogColor(const message_logger::log::levels::Level& level) {
  switch (level) {
  case message_logger::log::levels::Debug:
    return colorDebug;
  case message_logger::log::levels::Info:
    return colorInfo;
  case message_logger::log::levels::Warn:
    return colorWarn;
  case message_logger::log::levels::Error:
    return colorError;
  case message_logger::log::levels::Fatal:
    return colorFatal;
  default:
    break;
  }
  return color::def;
}

inline const std::string getLogLevel(const message_logger::log::levels::Level& level) {
  switch (level) {
  case message_logger::log::levels::Debug:
    return std::string{"DEBUG"};
  case message_logger::log::levels::Info:
    return std::string{" INFO"};
  case message_logger::log::levels::Warn:
    return std::string{" WARN"};
  case message_logger::log::levels::Error:
    return std::string{"ERROR"};
  case message_logger::log::levels::Fatal:
    return std::string{"FATAL"};
  default:
    break;
  }
  return std::string{"UNKNOWN"};
}

#ifdef MELO_FUNCTION_PRINTS
inline std::string parseMemberName(const std::string& in) {
    using namespace boost; // todo: replace with std as soon as gcc 4.9.x is standard in ubuntu repo
    regex re(".*((:{2}|\\s)([a-zA-Z0-9]*)(<.*>)?(:{2})|\\s+)([a-zA-Z0-9]+)\\s*(<.*>)?\\s*\\(.*\\).*");
    smatch match;
    if(regex_match(in, match, re)) {
        return colorFunction + "[" + match.str(3) + match.str(5) + match.str(6) + "] ";
    }
    return std::string();
}
#else
inline std::string parseMemberName(const std::string& /*in*/) {
    return std::string();
}
#endif

MELO_DEFINE_EXCEPTION(melo_fatal, std::runtime_error)


} // namespace log
} // namespace message_logger
