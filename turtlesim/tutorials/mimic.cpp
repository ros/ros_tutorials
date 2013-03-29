#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Velocity.h>

class Mimic
{
public:
  Mimic();

private:
  void poseCallback(const turtlesim::PoseConstPtr& pose);

  ros::Publisher vel_pub_;
  ros::Subscriber pose_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  vel_pub_ = output_nh.advertise<turtlesim::Velocity>("command_velocity", 1);
  pose_sub_ = input_nh.subscribe<turtlesim::Pose>("pose", 1, &Mimic::poseCallback, this);
}

void Mimic::poseCallback(const turtlesim::PoseConstPtr& pose)
{
  turtlesim::Velocity vel;
  vel.angular = pose->angular_velocity;
  vel.linear = pose->linear_velocity;
  vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_mimic");
  Mimic mimic;

  ros::spin();
}

