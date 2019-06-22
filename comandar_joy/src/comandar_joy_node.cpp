#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

bool flag = 0;
ros::Time currtime, lasttime;

class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_, l_accel_, l_max_, v0;
  std::string topic_name_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopJoy::TeleopJoy():
  linear_(1),
  angular_(0)
{

  nh_.param("/comandar_joy_node/axis_linear", linear_, 1);
  nh_.param("/comandar_joy_node/axis_angular", angular_, 0);
  nh_.param("/comandar_joy_node/scale_angular", a_scale_, 0.3);
  nh_.param("/comandar_joy_node/scale_linear", l_scale_, 0.15);
  //nh_.param("/comandar_joy_node/linear_accel", l_accel_, 0.2);
  //nh_.param("/comandar_joy_node/linear_max", l_max_, 2.0);
  nh_.param<std::string>("/comandar_joy_node/topic_name", topic_name_, "/cmd_vel");

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_name_, 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (flag == 0)
  {
    v0 = 0;
    lasttime = ros::Time::now();
    flag = 1;
  }
  currtime = ros::Time::now();
  ros::Duration diff = currtime - lasttime;
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  //twist.linear.x = v0;
  vel_pub_.publish(twist);
  //ROS_WARN("V0: [%f]", v0);
  //ROS_WARN("cmd vel: [%f]", v0*l_scale_*joy->axes[linear_]);
  //ROS_WARN("l_accel_: [%f]", l_accel_);

  //v0 = v0 + l_accel_*joy->axes[linear_]*(double)diff.toSec();
  //if (v0 > l_max_) v0 = l_max_;
  //if (v0 < -l_max_) v0 = -l_max_;
  //if (joy->axes[linear_] == 0) v0 = 0;
  lasttime = ros::Time::now();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;

  ros::spin();
}
