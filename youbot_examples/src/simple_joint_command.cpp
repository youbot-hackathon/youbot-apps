#include "ros/ros.h"
#include "std_msgs/String.h"
#include "brics_actuator/JointPositions.h"
#include <sstream>
#include <iostream>

#include <boost/units/systems/si.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "simple_joint_command");
  std::cout << "Create Node" << std::endl;

  ros::NodeHandle n;

  std::cout << "Create Publisher" << std::endl;
  ros::Publisher cmd_pub = n.advertise<brics_actuator::JointPositions>
    ("/arm_1/arm_controller/position_command", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {

    brics_actuator::JointPositions jp;

    std::vector<brics_actuator::JointValue> jpos(5);
    std::stringstream jointName;

    //build array and reset to zero position
    for (int i = 0; i < jpos.size(); ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i + 1);

      jpos[i].joint_uri = jointName.str();
      jpos[i].value = 0;
      jpos[i].unit = boost::units::to_string(boost::units::si::radian); 
    };

    //set the values
    jpos[0].value = 0.78;
    jpos[1].value = 0.78;

    jp.positions = jpos;

    cmd_pub.publish(jp);
    ros::spinOnce();
    std::cout << "sleep...";

    loop_rate.sleep();
     std::cout << "done" << std::endl;
  }
 std::cout << "exit" << std::endl;

  return 0;
}
