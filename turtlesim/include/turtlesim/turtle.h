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

#ifndef TURTLESIM_TURTLE_H
#define TURTLESIM_TURTLE_H

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>
# include <rclcpp_action/rclcpp_action.hpp>

# include <geometry_msgs/msg/twist.hpp>
# include <turtlesim/action/rotate_absolute.hpp>
# include <turtlesim/msg/color.hpp>
# include <turtlesim/msg/pose.hpp>
# include <turtlesim/srv/set_pen.hpp>
# include <turtlesim/srv/teleport_absolute.hpp>
# include <turtlesim/srv/teleport_relative.hpp>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

#define PI 3.14159265
#define TWO_PI 2.0 * PI

namespace turtlesim
{

class Turtle
{
public:
  using RotateAbsoluteGoalHandle = rclcpp_action::ServerGoalHandle<turtlesim::action::RotateAbsolute>;

  Turtle(rclcpp::Node::SharedPtr& nh, const std::string& real_name, const QImage& turtle_image, const QPointF& pos, float orient);

  bool update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height);
  void paint(QPainter &painter);
private:
  void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel);
  bool setPenCallback(const turtlesim::srv::SetPen::Request::SharedPtr, turtlesim::srv::SetPen::Response::SharedPtr);
  bool teleportRelativeCallback(const turtlesim::srv::TeleportRelative::Request::SharedPtr, turtlesim::srv::TeleportRelative::Response::SharedPtr);
  bool teleportAbsoluteCallback(const turtlesim::srv::TeleportAbsolute::Request::SharedPtr, turtlesim::srv::TeleportAbsolute::Response::SharedPtr);
  void rotateAbsoluteAcceptCallback(const std::shared_ptr<RotateAbsoluteGoalHandle>);

  void rotateImage();

  rclcpp::Node::SharedPtr nh_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  QPointF pos_;
  qreal orient_;

  qreal lin_vel_x_;
  qreal lin_vel_y_;
  qreal ang_vel_;
  bool pen_on_;
  QPen pen_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<turtlesim::msg::Color>::SharedPtr color_pub_;
  rclcpp::Service<turtlesim::srv::SetPen>::SharedPtr set_pen_srv_;
  rclcpp::Service<turtlesim::srv::TeleportRelative>::SharedPtr teleport_relative_srv_;
  rclcpp::Service<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_absolute_srv_;
  rclcpp_action::Server<turtlesim::action::RotateAbsolute>::SharedPtr rotate_absolute_action_server_;

  std::shared_ptr<RotateAbsoluteGoalHandle> rotate_absolute_goal_handle_;
  std::shared_ptr<turtlesim::action::RotateAbsolute::Feedback> rotate_absolute_feedback_;
  std::shared_ptr<turtlesim::action::RotateAbsolute::Result> rotate_absolute_result_;
  qreal rotate_absolute_start_orient_;

  rclcpp::Time last_command_time_;

  float meter_;

  struct TeleportRequest
  {
    TeleportRequest(float x, float y, qreal _theta, qreal _linear, bool _relative)
    : pos(x, y)
    , theta(_theta)
    , linear(_linear)
    , relative(_relative)
    {}

    QPointF pos;
    qreal theta;
    qreal linear;
    bool relative;
  };
  typedef std::vector<TeleportRequest> V_TeleportRequest;
  V_TeleportRequest teleport_requests_;
};
typedef std::shared_ptr<Turtle> TurtlePtr;

}

#endif
