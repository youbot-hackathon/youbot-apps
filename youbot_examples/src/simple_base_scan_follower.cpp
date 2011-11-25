/*
 * base_scan_follower.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: matthias
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
//#include "sensor_msgs/"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <iostream>


class LaserScannerDetection {

	ros::NodeHandle node;

	ros::Subscriber scan_sub;
	ros::Publisher cmd_pub;

public:

	class ScanItem {
	public:
		ScanItem() {
			start_angle = 0.0;
			end_angle = 0.0;
			mean_distance = 0.0;
			counter = 0;
		}

		double angle() {
			return (end_angle + start_angle) / 2;
		}

		double start_angle;
		double end_angle;

		double mean_distance;
		int counter;
	};

	LaserScannerDetection(ros::NodeHandle n) {
		this->node = n;

		scan_sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 10,
			boost::bind(&LaserScannerDetection::scanCB, this, _1));

		cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	}


	void scanCB(const sensor_msgs::LaserScanConstPtr& scan_in) {
		std::cout << "Scan received" << std::endl;

		std::vector<ScanItem> items;

		bool startItem = false;
		int hitcounter = 0;
		ScanItem tmpItem;

		int d = 5;

		for (int i = 0; i < scan_in->ranges.size() ; i++) {
			if (scan_in->ranges[i] > 2) {
				if(hitcounter > 0) {
					hitcounter--;
				}

				if(hitcounter == 0 && startItem==true) {
					startItem=false;
					tmpItem.end_angle = scan_in->angle_min + scan_in->angle_increment * (i-d);

					items.push_back(tmpItem);
					tmpItem = ScanItem();
				}

				continue;
			}

			hitcounter++;

			if (hitcounter > d && startItem==false) {
				startItem = true;
				tmpItem.start_angle = scan_in->angle_min + scan_in->angle_increment * (i-d);
				hitcounter = d;
			}

			if (startItem) {
				//std::cout << "distance: " << scan_in->ranges[i] << std::endl;
				tmpItem.mean_distance = (tmpItem.mean_distance * tmpItem.counter + scan_in->ranges[i]) / (tmpItem.counter + 1);
				tmpItem.counter++;
			}
			if (hitcounter > d) {
				hitcounter = d;
			}

			//std::cout << "Reading at: " << scan_in->angle_min + scan_in->angle_increment * i
			//		<< " at distance: " << scan_in->ranges[i] <<"m"<<std::endl;
		}

		std::cout << "Found " << items.size() << " items" << std::endl;

		for (int i=0; i<items.size(); i++) {
			ScanItem it = items[i];
			std::cout << "Item at yawangle: " << it.angle()
					<< " at distance: " << it.mean_distance << "m" << std::endl;
		}

		if (items.size() > 0) {
			ScanItem it = items[0];

			geometry_msgs::Twist cmd;

			if (std::abs(it.angle()) > 0.05) {
				cmd.angular.z = it.angle();

				cmd_pub.publish(cmd);
			} else if (it.mean_distance > 0.2) {
				cmd.linear.x = it.mean_distance * 3;

				if (cmd.linear.x > 0.2) {
					cmd.linear.x = 0.2;
				}
				cmd_pub.publish(cmd);
			}


		}
		//move towards 1st item


	}

};

int main(int argc, char** argv) {

	ros::init(argc, argv, "simple_base_scan_follower");
	ros::NodeHandle n;
	LaserScannerDetection lsd(n);

	ros::spin();

	return 0;
}



