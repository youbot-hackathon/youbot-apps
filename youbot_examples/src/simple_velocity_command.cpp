#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "simple_velocity_command");
  std::cout << "Create Node" << std::endl;

  ros::NodeHandle n;

  std::cout << "Create Publisher" << std::endl;
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {

    geometry_msgs::Twist twist;

    twist.linear.x = 1;
    twist.linear.y = 1;
    twist.linear.z = 1;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    cmd_pub.publish(twist);

     loop_rate.sleep();

  }
 std::cout << "exit" << std::endl;


  return 0;
}
