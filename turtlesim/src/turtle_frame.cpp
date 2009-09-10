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

#define PI 3.14159265

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

namespace turtlesim
{

TurtleFrame::TurtleFrame(wxWindow* parent)
: wxFrame(parent, wxID_ANY, wxT("TurtleSim"), wxDefaultPosition, wxSize(500, 500), wxDEFAULT_FRAME_STYLE & ~wxRESIZE_BORDER)
, orient_(0.0)
, lin_vel_(0.0)
, ang_vel_(0.0)
, pen_on_(true)
, frame_count_(0)
{
  srand(time(NULL));

  update_timer_ = new wxTimer(this);
  update_timer_->Start(16);

  Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(TurtleFrame::onUpdate), NULL, this);
  Connect(GetId(), wxEVT_PAINT, wxPaintEventHandler(TurtleFrame::onPaint), NULL, this);

  nh_.setParam("background_r", DEFAULT_BG_R);
  nh_.setParam("background_g", DEFAULT_BG_G);
  nh_.setParam("background_b", DEFAULT_BG_B);

  std::string turtles[3] = 
  {
    "box-turtle.png",
    "robot-turtle.png",
    "sea-turtle.png"
  };

  std::string images_path = ros::package::getPath("turtlesim") + "/images/";
  turtle_image_.LoadFile(wxString::FromAscii((images_path + turtles[rand() % 3]).c_str()));
  turtle_image_.SetMask(true);
  turtle_image_.SetMaskColour(255, 255, 255);
  turtle_ = wxBitmap(turtle_image_);

  pos_ = Vector2(250.0f, 250.0f);

  path_bitmap_ = wxBitmap(GetSize().GetWidth(), GetSize().GetHeight());
  path_dc_.SelectObject(path_bitmap_);
  wxPen pen(wxPen(wxColour(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B)));
  pen.SetWidth(3);
  path_dc_.SetPen(pen);
  clear();

  velocity_sub_ = nh_.subscribe("command_velocity", 1, &TurtleFrame::velocityCallback, this);
  pose_pub_ = nh_.advertise<Pose>("turtle_pose", 1);
  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  set_pen_srv_ = nh_.advertiseService("set_pen", &TurtleFrame::setPenCallback, this);

  ROS_INFO("Starting turtlesim with node name %s",ros::this_node::getName().c_str()) ;
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
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

  updateTurtle();

  if (!ros::ok())
  {
    Close();
  }
}

void TurtleFrame::onPaint(wxPaintEvent& evt)
{
  wxPaintDC dc(this);

  wxSize turtle_size = wxSize(turtle_.GetWidth(), turtle_.GetHeight());
  dc.DrawBitmap(path_bitmap_, 0, 0, true);
  dc.DrawBitmap(turtle_, pos_.x - (turtle_size.GetWidth() / 2), pos_.y - (turtle_size.GetHeight() / 2), true);
}

void TurtleFrame::updateTurtle()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(1.0))
  {
    lin_vel_ = 0.0f;
    ang_vel_ = 0.0f;
  }

  Vector2 old_pos = pos_;

#if 0
  ros::WallTime now = ros::WallTime::now();
  double dt = (now - last_turtle_update_).toSec();
  last_turtle_update_ = now;
#else
  // just use a 60hz step, to avoid the randomness that using real time brings.
  double dt = 0.016;
#endif

  orient_ = fmod(orient_ + ang_vel_ * dt, 2*PI);
  pos_.x += sin(orient_ + PI) * lin_vel_ * dt;
  pos_.y += cos(orient_ + PI) * lin_vel_ * dt;

  // Clamp to screen size
  if (pos_.x < 0 || pos_.x >= GetSize().GetWidth()
      || pos_.y < 0 || pos_.y >= GetSize().GetWidth())
  {
    ROS_ERROR("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x, pos_.y);
  }

  pos_.x = std::min(std::max(pos_.x, 0.0f), (float)GetSize().GetWidth());
  pos_.y = std::min(std::max(pos_.y, 0.0f), (float)GetSize().GetHeight());

  if (pen_on_)
  {
    if (pos_ != old_pos)
    {
      path_dc_.DrawLine(pos_.x, pos_.y, old_pos.x, old_pos.y);
    }
  }

  {
    wxImage rotated_image = turtle_image_.Rotate(orient_, wxPoint(turtle_image_.GetWidth() / 2, turtle_image_.GetHeight() / 2), false);

    for (int y = 0; y < rotated_image.GetHeight(); ++y)
    {
      for (int x = 0; x < rotated_image.GetWidth(); ++x)
      {
        if (rotated_image.GetRed(x, y) == 255 && rotated_image.GetBlue(x, y) == 255 && rotated_image.GetGreen(x, y) == 255)
        {
          rotated_image.SetAlpha(x, y, 0);
        }
      }
    }

    turtle_ = wxBitmap(rotated_image);
  }

  Pose p;
  p.x = GetSize().GetWidth() - pos_.x;
  p.y = GetSize().GetHeight() - pos_.y;
  p.theta = orient_;
  p.linear_velocity = lin_vel_;
  p.angular_velocity = ang_vel_;
  pose_pub_.publish(p);

  ROS_DEBUG("%s: pos_x: %f pos_y: %f theta: %f",ros::this_node::getName().c_str(), pos_.x, pos_.y, orient_);

  if (++frame_count_ % 3 == 0)
  {
    Refresh();
  }
}

void TurtleFrame::velocityCallback(const VelocityConstPtr& vel)
{
  last_command_time_ = ros::WallTime::now();
  lin_vel_ = vel->linear;
  ang_vel_ = vel->angular;
}

bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_WARN("Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_WARN("Resetting turtlesim.");
  pos_.x = GetSize().GetWidth() / 2;
  pos_.y = GetSize().GetHeight() / 2;
  orient_ = 0.0f;
  lin_vel_ = 0.0f;
  ang_vel_ = 0.0f;
  clear();
  return true;
}

bool TurtleFrame::setPenCallback(turtlesim::SetPen::Request& req, turtlesim::SetPen::Response&)
{
  pen_on_ = !req.off;
  if (req.off)
  {
    return true;
  }

  wxPen pen(wxColour(req.r, req.g, req.b));
  if (req.width != 0)
  {
    pen.SetWidth(req.width);
  }

  path_dc_.SetPen(pen);
  return true;
}

}
