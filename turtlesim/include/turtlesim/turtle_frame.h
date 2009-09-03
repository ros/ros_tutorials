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

#include <turtlesim/Pose.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>

namespace turtlesim
{

struct Vector2
{
  Vector2()
  : x(0.0)
  , y(0.0)
  {}

  Vector2(float _x, float _y)
  : x(_x)
  , y(_y)
  {}

  bool operator==(const Vector2& rhs)
  {
    return x == rhs.x && y == rhs.y;
  }

  bool operator!=(const Vector2& rhs)
  {
    return x != rhs.x || y != rhs.y;
  }

  float x;
  float y;
};

class TurtleFrame : public wxFrame
{
public:
  TurtleFrame(wxWindow* parent);
  ~TurtleFrame();

private:
  void onUpdate(wxTimerEvent& evt);
  void onPaint(wxPaintEvent& evt);

  void updateTurtle();
  void clear();

  void velocityCallback(const VelocityConstPtr& vel);
  bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool setPenCallback(turtlesim::SetPen::Request&, turtlesim::SetPen::Response&);

  ros::NodeHandle nh_;
  wxTimer* update_timer_;
  wxImage turtle_image_;
  wxBitmap turtle_;
  wxBitmap path_bitmap_;
  wxMemoryDC path_dc_;

  Vector2 pos_;
  float orient_;

  float lin_vel_;
  float ang_vel_;
  bool pen_on_;

  ros::WallTime last_turtle_update_;
  ros::WallTime last_command_time_;

  ros::Subscriber velocity_sub_;
  ros::Publisher pose_pub_;
  ros::ServiceServer clear_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer set_pen_srv_;
};

}
