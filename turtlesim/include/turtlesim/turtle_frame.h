/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <wx/wx.h>
#include <wx/event.h>
#include <wx/timer.h>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <map>

#include "turtle.h"

#define TURTLESIM_NUM_TURTLES 5

namespace turtlesim
{

class TurtleFrame : public wxFrame
{
public:
  TurtleFrame(wxWindow* parent);
  ~TurtleFrame();

  std::string spawnTurtle(const std::string& name, float x, float y, float angle);

private:
  void onUpdate(wxTimerEvent& evt);
  void onPaint(wxPaintEvent& evt);

  void updateTurtles();
  void clear();
  bool hasTurtle(const std::string& name);

  bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool spawnCallback(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
  bool killCallback(turtlesim::Kill::Request&, turtlesim::Kill::Response&);

  ros::NodeHandle nh_;
  wxTimer* update_timer_;
  wxBitmap path_bitmap_;
  wxImage path_image_;
  wxMemoryDC path_dc_;

  uint64_t frame_count_;

  ros::WallTime last_turtle_update_;

  ros::ServiceServer clear_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer spawn_srv_;
  ros::ServiceServer kill_srv_;

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_;

  wxImage turtle_images_[TURTLESIM_NUM_TURTLES];

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}
