#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>

#include <math.h>

using namespace std::chrono_literals;

#define PI 3.141592

enum State
{
  FORWARD = 0,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};

class DrawSquare : public rclcpp::Node
{
 public:
  explicit DrawSquare()
    : Node("draw_square"),
      g_state_(FORWARD),
      g_last_state_(FORWARD),
      g_first_goal_set_(false)
  {
    g_pose_ = std::make_shared<turtlesim::msg::Pose>();
    g_goal_ = std::make_shared<turtlesim::msg::Pose>();

    reset_client_ = create_client<std_srvs::srv::Empty>("reset");
    reset_request();

    auto pose_callback = 
      [this](const turtlesim::msg::Pose::SharedPtr pose) -> void
      {
        g_pose_ = pose;
        printPresent();
      };

    pose_sub_ = create_subscription<turtlesim::msg::Pose>("turtle1/pose", pose_callback);

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", rmw_qos_profile_default);

    auto timer_callback = 
      [this]() -> void
      {
        if (!g_pose_)
        {
          return;
        }

        if (!g_first_goal_set_)
        {
          g_first_goal_set_ = true;
          g_state_ = FORWARD;
          g_goal_->x = cos(g_pose_->theta) * 2 + g_pose_->x;
          g_goal_->y = sin(g_pose_->theta) * 2 + g_pose_->y;
          g_goal_->theta = g_pose_->theta;
          printGoal();
        }

        if (g_state_ == FORWARD)
        {
          forward();
        }
        else if (g_state_ == STOP_FORWARD)
        {
          stopForward();
        }
        else if (g_state_ == TURN)
        {
          turn();
        }
        else if (g_state_ == STOP_TURN)
        {
          stopTurn();
        }
      };

    timer_ = create_wall_timer(16ms, timer_callback);
  }

 private:
  bool hasReachedGoal()
  {
    return fabsf(g_pose_->x - g_goal_->x) < 0.1 && fabsf(g_pose_->y - g_goal_->y) < 0.1 && fabsf(g_pose_->theta - g_goal_->theta) < 0.01;
  }

  bool hasStopped()
  {
    return g_pose_->angular_velocity < 0.0001 && g_pose_->linear_velocity < 0.0001;
  }

  void printGoal()
  {
    RCLCPP_INFO(this->get_logger(), "New goal [%f %f, %f]", g_goal_->x, g_goal_->y, g_goal_->theta);
  }

  void printPresent()
  {
    RCLCPP_INFO(this->get_logger(), "Present [%f %f, %f]", g_pose_->x, g_pose_->y, g_pose_->theta);
  }

  void commandTurtle(float linear, float angular)
  {
    auto twist = std::make_shared<geometry_msgs::msg::Twist>();
    twist->linear.x = linear;
    twist->angular.z = angular;
    twist_pub_->publish(twist);
  }

  void stopForward()
  {
    if (hasStopped())
    {
      RCLCPP_INFO(this->get_logger(), "Reached goal");
      g_state_ = TURN;
      g_goal_->x = g_pose_->x;
      g_goal_->y = g_pose_->y;
      g_goal_->theta = fmod(g_pose_->theta + PI/2.0, 2*PI);

      // wrap g_goal_->theta to [-pi, pi)
      if (g_goal_->theta >= PI) g_goal_->theta -= 2 * PI;
      printGoal();
    }
    else
    {
      commandTurtle(0, 0);
    }
  }

  void stopTurn()
  {
    if (hasStopped())
    {
      RCLCPP_INFO(this->get_logger(), "Reached goal");
      g_state_ = FORWARD;
      g_goal_->x = cos(g_pose_->theta) * 2 + g_pose_->x;
      g_goal_->y = sin(g_pose_->theta) * 2 + g_pose_->y;
      g_goal_->theta = g_pose_->theta;
      printGoal();
    }
    else
    {
      commandTurtle(0, 0);
    }
  }

  void forward()
  {
    if (hasReachedGoal())
    {
      g_state_ = STOP_FORWARD;
      commandTurtle(0, 0);
    }
    else
    {
      commandTurtle(1.0, 0.0);
    }
  }

  void turn()
  {
    if (hasReachedGoal())
    {
      g_state_ = STOP_TURN;
      commandTurtle(0, 0);
    }
    else
    {
      commandTurtle(0.0, 0.4);
    }
  }

  void reset_request()
  {
    while (!reset_client_->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::Empty>::SharedFuture;
    auto response_received_callback = 
      [this](ServiceResponseFuture future)
      {
        auto result = future.get();
      };
    auto future_result = reset_client_->async_send_request(request, response_received_callback);
  }
 
 private:
  std::shared_ptr<turtlesim::msg::Pose> g_pose_;
  std::shared_ptr<turtlesim::msg::Pose> g_goal_;

  State g_state_;
  State g_last_state_;

  bool g_first_goal_set_;

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_; 

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DrawSquare>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
