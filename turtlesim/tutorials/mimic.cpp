#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <memory>
#include "rcutils/cmdline_parser.h"

class Mimic : public rclcpp::Node
{
 public:
  explicit Mimic(std::string &input, std::string &output)
    : Node("turtle_mimic")
  {
    RCLCPP_INFO(this->get_logger(), "turtle_mimic is ON! When input turtle move, another turtle move along");
    RCLCPP_INFO(this->get_logger(), "sub : %s, pub : %s", (input + "/pose").c_str(), (output + "/cmd_vel").c_str());

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(output + "/cmd_vel", rmw_qos_profile_default);

    twist_ = std::make_shared<geometry_msgs::msg::Twist>();

    auto pose_call_back = 
      [this](const turtlesim::msg::Pose::SharedPtr pose) -> void
      {
        twist_->angular.z = pose->angular_velocity;
        twist_->linear.x = pose->linear_velocity;
        twist_pub_->publish(twist_);
      };

    pose_sub_ = create_subscription<turtlesim::msg::Pose>(input + "/pose", pose_call_back);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

  std::shared_ptr<geometry_msgs::msg::Twist> twist_;
};

void print_usage()
{
  printf("Usage for turtle_mimic :\n");
  printf("talker [-i input] [-o output]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-i input: Specify the namespace on which to pose subscriber. Defaults to input.\n");
  printf("-o output: Specify the namespace on which to cmd_vel publisher. Defaults to output.\n");
}

int main(int argc, char** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) 
  {
    print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);
  
  auto input = std::string("input");
  char * cli_option[2];
  cli_option[0] = rcutils_cli_get_option(argv, argv + argc, "-i");
  if (nullptr != cli_option[0]) 
  {
    input = std::string(cli_option[0]);
  }

  auto output = std::string("output");
  cli_option[1] = rcutils_cli_get_option(argv, argv + argc, "-o");
  if (nullptr != cli_option[1]) 
  {
    output = std::string(cli_option[1]);
  }

  auto node = std::make_shared<Mimic>(input, output);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

