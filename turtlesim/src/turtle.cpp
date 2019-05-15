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

#include "turtlesim/turtle.h"

#include <QColor>
#include <QRgb>

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

namespace turtlesim
{

Turtle::Turtle(rclcpp::Node::SharedPtr &node_handle, std::string &real_name, const QImage& turtle_image, const QPointF& pos, float orient)
: turtle_image_(turtle_image)
, pos_(pos)
, orient_(orient)
, lin_vel_(0.0)
, ang_vel_(0.0)
, pen_on_(true)
, pen_(QColor(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
{
  pen_.setWidth(3);

  nh_ = node_handle;
  last_command_time_ = nh_->now();

  pose_msg_ = std::make_shared<turtlesim::msg::Pose>();
  color_msg_ = std::make_shared<turtlesim::msg::Color>();

  auto velocity_callback = 
    [this](const geometry_msgs::msg::Twist::SharedPtr vel) -> void
    {
      last_command_time_ = nh_->now();
      lin_vel_ = vel->linear.x;
      ang_vel_ = vel->angular.z;
    };

  velocity_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(real_name + "/cmd_vel", velocity_callback);

  pose_pub_ = nh_->create_publisher<turtlesim::msg::Pose>(real_name + "/pose", rmw_qos_profile_default);
  color_pub_ = nh_->create_publisher<turtlesim::msg::Color>(real_name + "/color_sensor", rmw_qos_profile_default);
  
  auto set_pen_callback =
    [this](const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<turtlesim::srv::SetPen::Request> request,
      std::shared_ptr<turtlesim::srv::SetPen::Response> /*response*/) -> bool
  {
    pen_on_ = !request->off;
    if (request->off)
    {
      return true;
    }

    QPen pen(QColor(request->r, request->g, request->b));
    if (request->width != 0)
    {
      pen.setWidth(request->width);
    }

    pen_ = pen;
    return true;
  };

  set_pen_srv_ = nh_->create_service<turtlesim::srv::SetPen>(real_name + "/set_pen", set_pen_callback);

  auto teleport_relative_callback =
    [this](const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<turtlesim::srv::TeleportRelative::Request> request,
      std::shared_ptr<turtlesim::srv::TeleportRelative::Response> /*response*/) -> bool
  {
    teleport_requests_.push_back(TeleportRequest(0, 0, request->angular, request->linear, true));
    return true;
  };

  teleport_relative_srv_ = nh_->create_service<turtlesim::srv::TeleportRelative>(real_name + "/teleport_relative", teleport_relative_callback);

  auto teleport_absolute_callback =
    [this](const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<turtlesim::srv::TeleportAbsolute::Request> request,
      std::shared_ptr<turtlesim::srv::TeleportAbsolute::Response> /*response*/) -> bool
  {
    teleport_requests_.push_back(TeleportRequest(request->x, request->y, request->theta, 0, false));
    return true;
  };

  teleport_absolute_srv_ = nh_->create_service<turtlesim::srv::TeleportAbsolute>(real_name + "/teleport_absolute", teleport_absolute_callback);

  meter_ = turtle_image_.height();
  rotateImage();
}

void Turtle::rotateImage()
{
  QTransform transform;
  transform.rotate(-orient_ * 180.0 / PI + 90.0);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
}

bool Turtle::update(double dt, QPainter& path_painter, const QImage& path_image, float canvas_width, float canvas_height)
{
  bool modified = false;
  float old_orient = orient_;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it)
  {
    const TeleportRequest& req = *it;

    QPointF old_pos = pos_;
    if (req.relative)
    {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += - std::sin(orient_) * req.linear;
    }
    else
    {
      pos_.setX(req.pos.x());
      pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      orient_ = req.theta;
    }

    if (pen_on_)
    {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  teleport_requests_.clear();

  // TODO: It makes error
  // if ((nh_->now() - last_command_time_) > rclcpp::Duration(1.0))
  // {
    // lin_vel_ = 0.0;
    // ang_vel_ = 0.0;
  // }
  //
  QPointF old_pos = pos_;

  orient_ = orient_ + ang_vel_ * dt;
  // Keep orient_ between -pi and +pi
  orient_ -= 2*PI * std::floor((orient_ + PI)/(2*PI));
  pos_.rx() += std::cos(orient_) * lin_vel_ * dt;
  pos_.ry() += - std::sin(orient_) * lin_vel_ * dt;

  // Clamp to screen size
  if (pos_.x() < 0 || pos_.x() > canvas_width ||
      pos_.y() < 0 || pos_.y() > canvas_height)
  {
    RCLCPP_WARN(nh_->get_logger(), "Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x(), pos_.y());
  }

  pos_.setX(std::min(std::max(static_cast<double>(pos_.x()), 0.0), static_cast<double>(canvas_width)));
  pos_.setY(std::min(std::max(static_cast<double>(pos_.y()), 0.0), static_cast<double>(canvas_height)));

  // Publish pose of the turtle
  pose_msg_->x = pos_.x();
  pose_msg_->y = canvas_height - pos_.y();
  pose_msg_->theta = orient_;
  pose_msg_->linear_velocity = lin_vel_;
  pose_msg_->angular_velocity = ang_vel_;

  pose_pub_->publish(pose_msg_);

  // Figure out (and publish) the color underneath the turtle
  {
    QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
    color_msg_->r = qRed(pixel);
    color_msg_->g = qGreen(pixel);
    color_msg_->b = qBlue(pixel);

    color_pub_->publish(color_msg_);
  }

  RCLCPP_DEBUG(nh_->get_logger(), "[%s]: pos_x: %f pos_y: %f theta: %f", nh_->get_namespace(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient)
  {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos)
  {
    if (pen_on_)
    {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  return modified;
}

void Turtle::paint(QPainter& painter)
{
  QPointF p = pos_ * meter_;
  p.rx() -= 0.5 * turtle_rotated_image_.width();
  p.ry() -= 0.5 * turtle_rotated_image_.height();
  painter.drawImage(p, turtle_rotated_image_);
}

}
