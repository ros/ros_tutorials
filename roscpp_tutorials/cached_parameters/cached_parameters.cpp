/*
 * Copyright (C) 2010, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cached_parameters");
  ros::NodeHandle n;
  ros::Time begin, end;

// %Tag(NH_SETCACHEDPARAM)%
  n.setParam("my_cached_param", "r u there?");
// %EndTag(NH_SETCACHEDPARAM)%

  {
// %Tag(NH_GETCACHEDPARAM_SIMPLE)%
    std::string s;
    // 1st time to lookup parameter server to cache parameter.
    begin = ros::Time::now();
    if (n.getParamCached("my_cached_param", s)) {
      end = ros::Time::now();
      ROS_INFO("1st read: %s (%lu [nsec])\n", s.c_str(), (end.toNSec() - begin.toNSec()));
    } else {
      ROS_INFO("Failed to cache ros parameter: %s\n", "my_cached_param");
    }
// %EndTag(NH_GETCACHEDPARAM_SIMPLE)%
  }

  {
// %Tag(NH_GETCACHEDPARAM_CHECK_RETURN)%
    std::string s;
    // 2nd time to lookup parameter server to cache parameter.
    begin = ros::Time::now();
    if (n.getParamCached("my_cached_param", s)) {
      end = ros::Time::now();
      ROS_INFO("2nd read: %s (%lu [nsec])\n", s.c_str(), (end.toNSec() - begin.toNSec()));
    } else {
      ROS_INFO("Failed to cache ros parameter: %s\n", "my_cached_param");
    }
// %EndTag(NH_GETCACHEDPARAM_CHECK_RETURN)%
  }

// %Tag(NH_SETCACHEDPARAM)%
  n.setParam("my_cached_param", "hey I'm here");
// %EndTag(NH_SETCACHEDPARAM)%

  {
// %Tag(NH_GETCACHEDPARAM_CHECK_RETURN)%
    std::string s;
    // 3nd time to lookup parameter server to cache parameter.
    begin = ros::Time::now();
    if (n.getParamCached("my_cached_param", s)) {
      end = ros::Time::now();
      ROS_INFO("3rd read: %s (%lu [nsec])\n", s.c_str(), (end.toNSec() - begin.toNSec()));
    } else {
      ROS_INFO("Failed to cache ros parameter: %s\n", "my_cached_param");
    }
// %EndTag(NH_GETCACHEDPARAM_CHECK_RETURN)%
  }

// %Tag(NH_DELETEPARAM)%
  n.deleteParam("my_cached_param");
// %EndTag(NH_DELETEPARAM)%

  return EXIT_SUCCESS;
}
