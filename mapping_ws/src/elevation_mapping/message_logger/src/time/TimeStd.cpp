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
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *********************************************************************/
/*!
* @file     TimeStd.cpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#include "message_logger/time/TimeStd.hpp"
#include <iomanip> // std::setw
#include <cmath> // round etc.
#include <climits>
#include <stdexcept>

#include <chrono>
#include <cstdint>

namespace message_logger {
namespace time {

TimeStd::TimeStd():
    sec_(0),
    nsec_(0)
{

}
TimeStd::TimeStd(uint32_t sec, uint32_t nsec) : sec_(sec), nsec_(nsec)
{
  normalizeSecNSec(sec_, nsec_);
}

TimeStd::TimeStd(uint64_t t) {
  fromNSec(t);
}

TimeStd::TimeStd(double t)
{
  fromSec(t);
}

TimeStd::TimeStd(const Time& time) : TimeStd(time.getSec(), time.getNSec()) {

}

TimeStd::~TimeStd()
{

}

TimeStd& TimeStd::from(uint32_t sec, uint32_t nsec) {
  normalizeSecNSec(sec_, nsec_);
  return *this;
}

TimeStd& TimeStd::fromSec(double t) {
  sec_ = (uint32_t)floor(t);
  nsec_ = (uint32_t)std::round((t-sec_) * 1e9);
  return *this;
}


TimeStd& TimeStd::fromNSec(uint64_t t) {
  sec_  = (int32_t)(t / 1000000000);
  nsec_ = (int32_t)(t % 1000000000);

  normalizeSecNSec(sec_, nsec_);

  return *this;
}

uint32_t TimeStd::getSec() const {
  return sec_;
}

uint32_t TimeStd::getNSec() const {
  return nsec_;
}


double TimeStd::toSec() const {
  return (double)sec_ + 1e-9*(double)nsec_;
}

TimeStd& TimeStd::operator=(const Time& time) {
  sec_ = time.getSec();
  nsec_ = time.getNSec();
  return *this;
}

TimeStd& TimeStd::operator=(const TimeStd& rhs) {
  sec_ = rhs.getSec();
  nsec_ = rhs.getNSec();
  return *this;
}

TimeStd TimeStd::operator+(const TimeStd& rhs) const
{
  int64_t sec_sum  = (int64_t)sec_  + (int64_t)rhs.getSec();
  int64_t nsec_sum = (int64_t)nsec_ + (int64_t)rhs.getNSec();

  // Throws an exception if we go out of 32-bit range
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);

  // now, it's safe to downcast back to uint32 bits
  return TimeStd((uint32_t)sec_sum, (uint32_t)nsec_sum);
}

TimeStd& TimeStd::operator+=(const TimeStd& rhs){
  *this = *this + rhs;
  return *this;
}

TimeStd TimeStd::operator+(double t) const {
  return *this + TimeStd(t);
}

TimeStd& TimeStd::operator+=(double t){
  *this = *this + TimeStd(t);
  return *this;
}

TimeStd& TimeStd::operator-=(const TimeStd &rhs) {
  *this += (-rhs);
  return *this;
}

TimeStd TimeStd::operator-(const TimeStd &rhs) const {
  return TimeStd((int32_t)sec_ -  (int32_t)rhs.getSec(),
                  (int32_t)nsec_ - (int32_t)rhs.getNSec());
}

TimeStd TimeStd::operator-() const {
  return TimeStd(-sec_ , -nsec_);
}

void TimeStd::normalizeSecNSec(uint64_t& sec, uint64_t& nsec) const {
  uint64_t nsec_part = nsec % 1000000000UL;
  uint64_t sec_part = nsec / 1000000000UL;

  if (sec_part > UINT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec += sec_part;
  nsec = nsec_part;
}

void TimeStd::normalizeSecNSec(uint32_t& sec, uint32_t& nsec) const {
  uint64_t sec64 = sec;
  uint64_t nsec64 = nsec;

  normalizeSecNSec(sec64, nsec64);

  sec = (uint32_t)sec64;
  nsec = (uint32_t)nsec64;
}


void TimeStd::normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec) const {
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part >= 1000000000L)
  {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0)
  {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > INT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}


Time& TimeStd::setNow() {
    *this = now();
    return *this;
}

TimeStd TimeStd::now() {
  auto time = std::chrono::high_resolution_clock::now();
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
  return TimeStd(timestamp);
}


std::ostream& operator<<(std::ostream& out, const TimeStd& rhs) {
  out << rhs.sec_ << "." << std::setw(9) << std::setfill('0') << rhs.nsec_;
  return out;
}

} /* namespace time */
} /* namespace message_logger */
