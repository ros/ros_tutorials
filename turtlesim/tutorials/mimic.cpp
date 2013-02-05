#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

class Mimic
{
public:
  Mimic();

private:
  void poseCallback(const turtlesim::PoseConstPtr& pose);

  ros::Publisher twist_pub_;
  ros::Subscriber pose_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  twist_pub_ = output_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pose_sub_ = input_nh.subscribe<turtlesim::Pose>("pose", 1, &Mimic::poseCallback, this);
}

void Mimic::poseCallback(const turtlesim::PoseConstPtr& pose)
{
  geometry_msgs::Twist twist;
  twist.angular.z = pose->angular_velocity;
  twist.linear.x = pose->linear_velocity;
  twist_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_mimic");
  Mimic mimic;

  ros::spin();
}

