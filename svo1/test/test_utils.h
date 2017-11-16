// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include <string.h>
#include <cstdlib> // for getenv
#ifdef SVO1USE_ROS
# include <ros/package.h>
# include <vikit1/params_helper.h>
#endif

namespace svo1 {
namespace test_utils {

std::string getDatasetDir()
{
  const char* env_dir = std::getenv("SVO1DATASET_DIR");
#ifdef SVO1USE_ROS
  std::string dataset_dir(ros::package::getPath("svo1")+"/test/data");
  if(env_dir != NULL)
    dataset_dir = std::string(env_dir);
  return dataset_dir;
#else
  return std::string(env_dir);
#endif
}

std::string getTraceDir()
{
#ifdef SVO1USE_ROS
  std::string default_dir(ros::package::getPath("svo1")+"/test/results");
  return vk::getParam<std::string>("svo1/trace_dir", default_dir);
#else
  return "/tmp";
#endif
}

} // namespace test_utils
} // namespace svo1


#endif // TEST_UTILS_H_
