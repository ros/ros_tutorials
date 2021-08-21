#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <math.h>

turtlesim::msg::Pose g_pose;
turtlesim::msg::Pose g_goal;

enum State
{
  FORWARD,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};

State g_state = FORWARD;
State g_last_state = FORWARD;
bool g_first_goal_set = false;
bool g_first_pose_set = false;

#define PI 3.141592

void poseCallback(const turtlesim::msg::Pose & pose)
{
  g_pose = pose;
  if (!g_first_pose_set)
  {
    g_first_pose_set = true;
  }
}

bool hasReachedGoal()
{
  return fabsf(g_pose.x - g_goal.x) < 0.1 && fabsf(g_pose.y - g_goal.y) < 0.1 && fabsf(g_pose.theta - g_goal.theta) < 0.01;
}

bool hasStopped()
{
  return g_pose.angular_velocity < 0.0001 && g_pose.linear_velocity < 0.0001;
}

void printGoal()
{
  RCLCPP_INFO(rclcpp::get_logger("draw_square"), "New goal [%f %f, %f]", g_goal.x, g_goal.y, g_goal.theta);
}

void commandTurtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub, float linear, float angular)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  twist_pub->publish(twist);
}

void stopForward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasStopped())
  {
    RCLCPP_INFO(rclcpp::get_logger("draw_square"), "Reached goal");
    g_state = TURN;
    g_goal.x = g_pose.x;
    g_goal.y = g_pose.y;
    g_goal.theta = fmod(g_pose.theta + static_cast<float>(PI) / 2.0f, 2.0f * static_cast<float>(PI));
    // wrap g_goal.theta to [-pi, pi)
    if (g_goal.theta >= static_cast<float>(PI)) g_goal.theta -= 2.0f * static_cast<float>(PI);
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}

void stopTurn(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasStopped())
  {
    RCLCPP_INFO(rclcpp::get_logger("draw_square"), "Reached goal");
    g_state = FORWARD;
    g_goal.x = cos(g_pose.theta) * 2 + g_pose.x;
    g_goal.y = sin(g_pose.theta) * 2 + g_pose.y;
    g_goal.theta = g_pose.theta;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}


void forward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_FORWARD;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 1.0f, 0);
  }
}

void turn(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_TURN;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 0, 0.4f);
  }
}

void timerCallback(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (g_first_pose_set)
  {
    return;
  }

  if (!g_first_goal_set)
  {
    g_first_goal_set = true;
    g_state = FORWARD;
    g_goal.x = cos(g_pose.theta) * 2 + g_pose.x;
    g_goal.y = sin(g_pose.theta) * 2 + g_pose.y;
    g_goal.theta = g_pose.theta;
    printGoal();
  }

  if (g_state == FORWARD)
  {
    forward(twist_pub);
  }
  else if (g_state == STOP_FORWARD)
  {
    stopForward(twist_pub);
  }
  else if (g_state == TURN)
  {
    turn(twist_pub);
  }
  else if (g_state == STOP_TURN)
  {
    stopTurn(twist_pub);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("draw_square");
  auto pose_sub = nh->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, std::bind(poseCallback, std::placeholders::_1));
  auto twist_pub = nh->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
  auto reset = nh->create_client<std_srvs::srv::Empty>("reset");
  auto timer = nh->create_wall_timer(std::chrono::milliseconds(16), [twist_pub](){timerCallback(twist_pub);});

  auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
  reset->async_send_request(empty);

  rclcpp::spin(nh);
}
