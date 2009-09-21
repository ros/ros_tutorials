#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Velocity.h>
#include <std_srvs/Empty.h>

turtlesim::PoseConstPtr g_pose;
turtlesim::Pose g_goal;

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

#define PI 3.141592

void poseCallback(const turtlesim::PoseConstPtr& pose)
{
  g_pose = pose;
}

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal.x) < 0.1 && fabsf(g_pose->y - g_goal.y) < 0.1 && fabsf(g_pose->theta - g_goal.theta) < 0.01;
}

bool hasStopped()
{
  return g_pose->angular_velocity < 0.0001 && g_pose->linear_velocity < 0.0001;
}

void printGoal()
{
  ROS_INFO("New goal [%f %f, %f]", g_goal.x, g_goal.y, g_goal.theta);
}

void commandTurtle(ros::Publisher vel_pub, float linear, float angular)
{
  turtlesim::Velocity vel;
  vel.linear = linear;
  vel.angular = angular;
  vel_pub.publish(vel);
}

void stopForward(ros::Publisher vel_pub)
{
  if (hasStopped())
  {
    ROS_INFO("Reached goal");
    g_state = TURN;
    g_goal.x = g_pose->x;
    g_goal.y = g_pose->y;
    g_goal.theta = fmod(g_pose->theta + PI/2.0, 2*PI);
    printGoal();
  }
  else
  {
    commandTurtle(vel_pub, 0, 0);
  }
}

void stopTurn(ros::Publisher vel_pub)
{
  if (hasStopped())
  {
    ROS_INFO("Reached goal");
    g_state = FORWARD;
    g_goal.x = g_pose->x - sin(g_pose->theta) * 2;
    g_goal.y = cos(g_pose->theta) * 2 + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }
  else
  {
    commandTurtle(vel_pub, 0, 0);
  }
}


void forward(ros::Publisher vel_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_FORWARD;
    commandTurtle(vel_pub, 0, 0);
  }
  else
  {
    commandTurtle(vel_pub, 1.0, 0.0);
  }
}

void turn(ros::Publisher vel_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_TURN;
    commandTurtle(vel_pub, 0, 0);
  }
  else
  {
    commandTurtle(vel_pub, 0.0, 0.4);
  }
}

void timerCallback(const ros::TimerEvent&, ros::Publisher vel_pub)
{
  if (!g_pose)
  {
    return;
  }

  if (!g_first_goal_set)
  {
    g_first_goal_set = true;
    g_state = FORWARD;
    g_goal.x = g_pose->x - sin(g_pose->theta) * 2;
    g_goal.y = cos(g_pose->theta) * 2 + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }

  if (g_state == FORWARD)
  {
    forward(vel_pub);
  }
  else if (g_state == STOP_FORWARD)
  {
    stopForward(vel_pub);
  }
  else if (g_state == TURN)
  {
    turn(vel_pub);
  }
  else if (g_state == STOP_TURN)
  {
    stopTurn(vel_pub);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_square");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);
  ros::Publisher vel_pub = nh.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
  ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");
  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, vel_pub));

  std_srvs::Empty empty;
  reset.call(empty);

  ros::spin();
}
