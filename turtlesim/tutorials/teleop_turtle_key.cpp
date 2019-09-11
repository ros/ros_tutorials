#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_L;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_U;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_R;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

class TeleopTurtle
{
public:
  TeleopTurtle();
  int keyLoop();

private:

  
  rclcpp::Node::SharedPtr nh_;
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_ = rclcpp::Node::make_shared("teleop_turtle");
  nh_->declare_parameter("scale_angular", rclcpp::ParameterValue(2.0));
  nh_->declare_parameter("scale_linear", rclcpp::ParameterValue(2.0));
  nh_->get_parameter("scale_angular", a_scale_);
  nh_->get_parameter("scale_linear", l_scale_);

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  int rc = teleop_turtle.keyLoop();
  input.shutdown();
  rclcpp::shutdown();
  
  return rc;
}


int TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle. 'q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (std::runtime_error)
    {
      perror("read():");
      return -1;
    }

    linear_=angular_=0;
    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        return 0;
    }
   

    geometry_msgs::msg::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_->publish(twist);    
      dirty=false;
    }
  }


  return 0;
}



