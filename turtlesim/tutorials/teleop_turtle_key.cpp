#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_S 0x20
#define KEYCODE_Q 0x71

class TeleopTurtle : public rclcpp::Node
{
 public:
  explicit TeleopTurtle()
    : Node("turtle_teleop"),
      linear_(0),
      angular_(0),
      l_scale_(2.0),
      a_scale_(2.0)
  {
    //TODO: Need to change to new API parameter
    this->declare_parameter("scale_angular", a_scale_);
    this->declare_parameter("scale_linear", l_scale_);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rmw_qos_profile_default);
  }
  void keyLoop();

 private:
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TeleopTurtle>();

  signal(SIGINT, quit);

  node->keyLoop();
  
  return 0;
}

void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, sizeof(c)) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_S:
        RCLCPP_DEBUG(this->get_logger(), "STOP");
        linear_ = 0.0;
        angular_ = 0.0;
        dirty = true;
        break;
    }

    auto twist = std::make_shared<geometry_msgs::msg::Twist>();
    twist->angular.z = a_scale_*angular_;
    twist->linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_->publish(twist);    
      dirty=false;
    }
  }

  return;
}



