#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>


std::string frame_id = "";
std::string child_frame_id = "";
std::string joint = "";
double pitch = 0.0;


void poseCallback(const std_msgs::Float64::ConstPtr &msg)
{
	double rad = msg->data * M_PI / 180.0;

	pitch *= 0.99;
	pitch += 0.01 * rad;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_tilt_publisher");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	ros::Duration(0.5).sleep();
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(30);

	ros::Subscriber sub = n.subscribe("cur_tilt_angle", 1000, &poseCallback);
	pn.getParam("frame_id", frame_id);
	pn.getParam("child_frame_id", child_frame_id);
	pn.getParam("joint", joint);

	ROS_INFO("frame_id: %s", frame_id.c_str());
	ROS_INFO("child_frame_id: %s", child_frame_id.c_str());
	ROS_INFO("joint: %s", joint.c_str());


	// message declarations
	geometry_msgs::TransformStamped transform;
	sensor_msgs::JointState joint_state;
	transform.header.frame_id = frame_id;
	transform.child_frame_id = child_frame_id;

	while (ros::ok()) {
		//update joint_state
		joint_state.header.stamp = ros::Time::now() - ros::Duration(0.5);
		joint_state.name.resize(1);
		joint_state.position.resize(1);
		joint_state.name[0] = joint;
		joint_state.position[0] = pitch;


		// update transform
		transform.header.stamp = ros::Time::now() - ros::Duration(0.5);
		transform.transform.translation.x = 0.0;
		transform.transform.translation.y = 0.0;
		transform.transform.translation.z = 0.0;
		transform.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, pitch, 0.0);

		//send the joint state and transform
		joint_pub.publish(joint_state);
		broadcaster.sendTransform(transform);

		ros::spinOnce();

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}

	return 0;
}
