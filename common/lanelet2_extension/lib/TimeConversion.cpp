
/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include <boost/date_time/posix_time/posix_time.hpp>
#include <lanelet2_extension/time/TimeConversion.h>

namespace lanelet
{

namespace time {

double lanelet::time::toSec(const boost::posix_time::time_duration& duration) {
  if (duration.is_special()) {
    throw std::invalid_argument("Cannot convert special duration to seconds");
  }
  return duration.total_microseconds() / 1000000.0;
}

double lanelet::time::toSec(const boost::posix_time::ptime& time) {
  return lanelet::time::toSec(time - boost::posix_time::from_time_t(0));
}

boost::posix_time::ptime timeFromSec(double sec) {
  return boost::posix_time::from_time_t(0) + boost::posix_time::microseconds(static_cast<long>(sec * 1000000L));
}

boost::posix_time::time_duration durationFromSec(double sec) {
  return boost::posix_time::microseconds(static_cast<long>(sec * 1000000L));
}

} // namespace time

}  // namespace lanelet