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

#include "turtlesim/turtle_frame.h"

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtlesim
{

TurtleFrame::TurtleFrame(wxWindow* parent)
: wxFrame(parent, wxID_ANY, wxT("TurtleSim"), wxDefaultPosition, wxSize(500, 500), wxDEFAULT_FRAME_STYLE & ~wxRESIZE_BORDER)
, frame_count_(0)
, id_counter_(0)
{
  srand(time(NULL));

  update_timer_ = new wxTimer(this);
  update_timer_->Start(16);

  Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(TurtleFrame::onUpdate), NULL, this);
  Connect(wxEVT_PAINT, wxPaintEventHandler(TurtleFrame::onPaint), NULL, this);

  nh_.setParam("background_r", DEFAULT_BG_R);
  nh_.setParam("background_g", DEFAULT_BG_G);
  nh_.setParam("background_b", DEFAULT_BG_B);

  std::string turtles[5] = 
  {
    "box-turtle.png",
    "robot-turtle.png",
    "sea-turtle.png",
    "diamondback.png",
    "electric.png"
  };

  std::string images_path = ros::package::getPath("turtlesim") + "/images/";
  for (size_t i = 0; i < 5; ++i)
  {
    turtle_images_[i].LoadFile(wxString::FromAscii((images_path + turtles[i]).c_str()));
    turtle_images_[i].SetMask(true);
    turtle_images_[i].SetMaskColour(255, 255, 255);
  }

  meter_ = turtle_images_[0].GetHeight();

  path_bitmap_ = wxBitmap(GetSize().GetWidth(), GetSize().GetHeight());
  path_dc_.SelectObject(path_bitmap_);
  clear();

  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

  ROS_INFO("Starting turtlesim with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = GetSize().GetWidth() / meter_;
  height_in_meters_ = GetSize().GetHeight() / meter_;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool TurtleFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);

  return true;
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  TurtlePtr t(new Turtle(ros::NodeHandle(real_name), turtle_images_[rand() % 5], Vector2(x, y), angle));
  turtles_[real_name] = t;

  ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

  nh_.param("background_r", r, r);
  nh_.param("background_g", g, g);
  nh_.param("background_b", b, b);

  path_dc_.SetBackground(wxBrush(wxColour(r, g, b)));
  path_dc_.Clear();
}

void TurtleFrame::onUpdate(wxTimerEvent& evt)
{
  ros::spinOnce();

  updateTurtles();

  if (!ros::ok())
  {
    Close();
  }
}

void TurtleFrame::onPaint(wxPaintEvent& evt)
{
  wxPaintDC dc(this);

  dc.DrawBitmap(path_bitmap_, 0, 0, true);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(dc);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  if (frame_count_ % 3 == 0)
  {
    path_image_ = path_bitmap_.ConvertToImage();
    Refresh();
  }

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->update(0.016, path_dc_, path_image_, path_dc_.GetBackground().GetColour(), width_in_meters_, height_in_meters_);
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}
