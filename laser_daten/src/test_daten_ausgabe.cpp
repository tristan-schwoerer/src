#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include <math.h>
#include <sstream>


sensor_msgs::LaserScan laser;


/*
int funktion_gerade_bestimmen(double radius, double winkel)
{
	ROS_INFO("funktion");
	//von radius und winkel funktion berechnen der gerade und als marker speicern
		
		ROS_INFO("funktion ende");
		return 0;
		//damit marker erstellen
}
*/


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("test erfolgreich");
	laser = *msg;
	// Test onlne linien
	int anzahl_punkte = laser.ranges.size();
	
	float x[anzahl_punkte] = {0.0};
	float y[anzahl_punkte] = {0.0};
	float d[anzahl_punkte] = {0.0};
	float p_1_x[anzahl_punkte] = {0.0};
	float p_1_y[anzahl_punkte] = {0.0};
	float p_2_x[anzahl_punkte] = {0.0};
	float p_2_y[anzahl_punkte] = {0.0};
	
	float sum = 0.0;
	float ek = 0.2;
	int a = 0;
	int b = 0;
	int c = 0;
	
	x[0] = laser.ranges[0]*cos(0);
	y[0]= laser.ranges[0]*sin(0);
	for(int i=1;i<anzahl_punkte; i++){
		x[i]= laser.ranges[i]*cos((i)*laser.angle_increment);
		y[i]= laser.ranges[i]*sin(i*laser.angle_increment);
		d[i] = sqrt(pow((x[i]-x[i-1]),2)+pow((y[i]-y[i-1]),2));
		for(a=0;a<=i;a++){
			sum += d[a];
		}
		if((d[i]/sum)> 1 || (d[i]/sum)< (1-ek))	{	//punkt passt nicht zur linie
			
			if(c == 0){
				//start und endpunkte der linie
				p_1_x[b] = x[0];
				p_1_y [b] = y[0];
				p_2_x [b]= x[i];
				p_2_y [b]= y[i];
			}
			else {
				p_1_x[b] = x[c];
				p_1_y [b] = y[c];
				p_2_x [b]= x[i];
				p_2_y [b]= y[i];
				
			}
			x[anzahl_punkte] = {0.0};
			y[anzahl_punkte] = {0.0};
			c = i;
			b++;
			
			
	
		}
		else {
			ROS_INFO("Punkte gleich");
		}
		ek= ek +0.01;
		/*if(a = anzahl_punkte){
			return ;
		}*/
		ROS_INFO("i = %d", i);
		ROS_INFO("a = %d", a);
		ROS_INFO("b = %d", b);
		ROS_INFO("c = %d", c);
		
		
	}
	
	ROS_INFO("zweite for schleife");
	


}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_daten_ausgabe");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 50, laser_callback);
	/*publisher
	ros::Publisher test_pub = n.advertise<sensor_msgs::LaserScan>("test_Topic", 10);
	ros::Rate loop_rate(10);
/*
	while (ros::ok())
	{
		Sensor_msgs/LaserScan;
		
		ROS_INFO("while schleife");
		msg.angle_min = laser.angle.min;
		msg.scan_time = laser.scan_time;
		msg.range_min = laser.range_min;

		//ROS_INFO("%s", msg.data.c_str());

		test_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		
	}*/
	ros::spin();
	return 0;
}

