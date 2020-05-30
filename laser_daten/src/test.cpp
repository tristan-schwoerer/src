
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include <math.h>
#include <sstream>




sensor_msgs::LaserScan laser;
//abgefragte hough winkel
int abtast_winkel[] = {0, 30, 60, 90, 120, 150, 180};
int abtast_winkel_size = sizeof(abtast_winkel);
int abtast_winkel_min = abtast_winkel[0];
int abtast_winkel_max = abtast_winkel[abtast_winkel_size];
int abtast_winkel_step = 30;

//lauf variablen
int i = 0;
int a = 0;

int funktion_gerade_bestimmen(double radius, double winkel)
{
	ROS_INFO("funktion");
	//von radius und winkel funktion berechnen der gerade und als marker speicern
		double winkel_gerade_rad = (90*2*M_PI/360) - winkel;
		double steigung_gerade = -tan(winkel_gerade_rad); //gehe von tan in rad berechnen aus
		double b_gerade = radius * sin(winkel) - steigung_gerade * radius *cos(winkel);

		ROS_INFO("funktion ende");
		return 0;
		//damit marker erstellen
}


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("test erfolgreich");
	laser = *msg;
	// Test Hough tresformation

	int d_min = laser.range_min;
	int d_max = laser.range_max;
	int anzahl_rang = laser.ranges.size();

	int hough [abtast_winkel_size] [d_max];
	for(i = 0; i<anzahl_rang; i++)
	{
		for(a = 0; a<abtast_winkel_size; a++)
		{
			float x = laser.ranges[i] * cos(i*laser.angle_increment+laser.angle_max);
			float y = laser.ranges[i] * sin(i*laser.angle_increment+laser.angle_max);

			int dist = x*cos(abtast_winkel[a]) + y*sin(abtast_winkel[a]);
			hough [a][dist]++;
		}
	}
	ROS_INFO("terste for.schleife");
	//als funktion  mit rückagbe von c und b

	for(int c = 0; c<anzahl_rang; i++)
	{
		for(int b = 0; b<abtast_winkel_size; b++)
		{
			if (hough[b][c]>=20){
				funktion_gerade_bestimmen(c,b); //
			}
		}
	}
	ROS_INFO("zweite for schleife");
	ROS_INFO("%d", anzahl_rang);


}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 50, laser_callback);
	//publisher
	ros::Publisher test_pub = n.advertise<std_msgs::String>("test_Topic", 10);
	ros::Rate loop_rate(10);
/*
	int count =0;
	while (ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss<<"test test"<<count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		test_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}*/
	ros::spin();
	return 0;
}

