#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Mimic
{
public:
  Mimic();

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr pose);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

Mimic::Mimic()
{
  auto input_nh = rclcpp::Node::make_shared("input");
  auto output_nh = rclcpp::Node::make_shared("output");
  twist_pub_ = output_nh->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  pose_sub_ = input_nh->create_subscription<turtlesim::msg::Pose>("pose", 1, std::bind(&Mimic::poseCallback, this, std::placeholders::_1));
}

void Mimic::poseCallback(const turtlesim::msg::Pose::SharedPtr pose)
{
  geometry_msgs::msg::Twist twist;
  twist.angular.z = pose->angular_velocity;
  twist.linear.x = pose->linear_velocity;
  twist_pub_->publish(twist);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("turtle_mimic");
  Mimic mimic;

  rclcpp::spin(nh);
}
