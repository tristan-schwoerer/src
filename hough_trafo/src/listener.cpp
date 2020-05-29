#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

void chatterCallback(const sensor_msgs::LaserScan& msg)
	{
	ROS_INFO("LaserData: [%f]",msg.ranges[360]);
	}

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
		ros::spin();
		//test
		return 0;
	}
