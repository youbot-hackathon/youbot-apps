#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>


// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 31.
#define MIN_TILT_ANGLE (-31.)

double pitch = 0.0;

ros::Publisher controller_pub;


void setTiltAngle(const std_msgs::Float64 angleMsg)
{
	double angle(angleMsg.data);
	angle = (angle < MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle > MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);

	pitch = angle * M_PI / 180.0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_simulated_tilt");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	ros::Duration(0.5).sleep();
	ros::Publisher tilt_pub = n.advertise<std_msgs::Float64>("cur_tilt_angle", 10);
	ros::Subscriber sub_tilt_angle = n.subscribe("tilt_angle", 10, setTiltAngle);
	ros::Rate loop_rate(30);

	while (ros::ok()) {
		// tilt angle
		std_msgs::Float64 angle_msg;
		angle_msg.data = pitch * 180.0 / M_PI;

		//send the joint state and transform
		tilt_pub.publish(angle_msg);

		ros::spinOnce();

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}

	return 0;
}
