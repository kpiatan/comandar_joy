#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>


int status_atual;

class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_, l_accel_, l_max_;
  bool linear_pressed, angular_pressed;
  std::string topic_name_;
  ros::Publisher vel_pub_;
  ros::Publisher status_pub_;
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
  nh_.param<std::string>("/comandar_joy_node/topic_name", topic_name_, "/cmd_vel");

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_name_, 1);

  status_pub_ = nh_.advertise<std_msgs::Int16>("autonomy_level", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  geometry_msgs::Twist twist;
  
  if (joy->buttons[5] && !linear_pressed) 
  {
    l_scale_ += 0.05;
    linear_pressed = 1;
  }
  if (joy->buttons[4]&& !linear_pressed) 
  {
    l_scale_ -= 0.05;
    linear_pressed = 1;
  }
  if ((!joy->buttons[4]) && (!joy->buttons[5])) linear_pressed = 0;

  if (joy->buttons[7] && !angular_pressed) 
  {
    a_scale_ += 0.05;
    angular_pressed = 1;
  }
  if (joy->buttons[6]&& !angular_pressed) 
  {
    a_scale_ -= 0.05;
    angular_pressed = 1;
  }
  if ((!joy->buttons[6]) && (!joy->buttons[7])) angular_pressed = 0;

  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);

  std_msgs::Int16 status;

  if (joy->buttons[0]) status_atual = 1;
  if (joy->buttons[1]) status_atual = 2;
  if (joy->buttons[2]) status_atual = 3;
  if (joy->buttons[3]) status_atual = 4;

  status.data = status_atual;

  status_pub_.publish(status);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;

  ros::spin();
}
