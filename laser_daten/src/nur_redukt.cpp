#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
//dynamic r.
#include <dynamic_reconfigure/server.h>
#include <laser_daten/myParamConfig.h>



#include <cmath>
#include <vector>
#include <math.h>
#include <sstream>
#include <string>
using namespace std;
/*
std::vector<float> x;
std::vector<float> y;
std::vector<float> d;
*/

sensor_msgs::LaserScan laser;
sensor_msgs::LaserScan filter_laser;
ros::Publisher reduktion_pub;
int anzahl_neuer_punkte = 0;


float f = 0.0;

double abstand_punkte_red;
std::string fixed_frame;


/*

void marker_points (float x_r, float y_r){

	//Marker:
	visualization_msgs::Marker points;
	points.header.frame_id = "/laser_frame";
	points.header.stamp = ros::Time::now();
	points.ns = "g_reduktionsfilter";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;



	points.id = 2;



	points.type = visualization_msgs::Marker::POINTS;




	// points markers use only the x and y component of scale, for the point width
	points.scale.x = 0.1;
	points.scale.y = 0.1;


	// Line list is red
	points.color.g = 1.0f;
	points.color.a = 1.0;



	// Create the vertices for the points and lines

	geometry_msgs::Point p;
	p.x = x_r;
	p.y = y_r;
	p.z = 0;

	points.points.push_back(p);

	//if (pub == true){
		reduktion_pub.publish(points);
	//}



}

*/
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Marker:
		visualization_msgs::Marker points;
		points.header.frame_id = fixed_frame;
		points.header.stamp = ros::Time::now();
		points.ns = "g_reduktionsfilter";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;



		points.id = 2;



		points.type = visualization_msgs::Marker::POINTS;




		// points markers use only the x and y component of scale, for the point width
		points.scale.x = 0.05;
		points.scale.y = 0.05;


		// Line list is red
		points.color.g = 1.0f;
		points.color.a = 1.0;
	//Marker init ende

	ROS_INFO("test erfolgreich");
	laser = *msg;
	// Test Reduktionsfilter

		int anzahl_punkte = laser.ranges.size();
		anzahl_neuer_punkte = 0;
		float x[anzahl_punkte] = {0.0};
		float y[anzahl_punkte] = {0.0};
		
		float d_akt_r[anzahl_punkte] = {0.0};			//für abstand erster punkt linie zu aktuellem punkt
		
		

		double abstand_punkte = abstand_punkte_red;

		
		double sum_x = 0.0;
		double sum_y = 0.0;
		
		int a = 0;
		
		int c = 0;
	
		float mittelwert_x = 0.0;
		float mittelwert_y = 0.0;
		bool letzer_punkt = false; //damit marker erst beim letzen punkt gepublished werden
		
		float reduzierte_x[1000]= {0.00};
		float reduzierte_y[1000] = {0.00};

		x[0] = laser.ranges[0]*cos(laser.angle_min);		//x koordinaten des ersten punltes
		y[0]= laser.ranges[0]*sin(laser.angle_min);		//y coordianten des ersten punktes
		for(int i=1;i<anzahl_punkte; i++){

			if (i == anzahl_punkte-1){
				letzer_punkt = true;
			}
			//in x und y koordianten umrechnen
			x[i]= laser.ranges[i]*cos(laser.angle_min + i*laser.angle_increment);
			y[i]= laser.ranges[i]*sin(laser.angle_min + i*laser.angle_increment);
			//ROS_INFO(" x: %f", x[i]);
			//ROS_INFO(" y: %f", y[i]);
			//abstand zwischen den einelnen letzten punkten in polarkoordianten
			
			//Abstand erster punkt der linie und aktueller punkt mit c als ersten punkt:
			d_akt_r[i] = sqrt(pow((x[i]-x[c]),2)+pow((y[i]-y[c]),2));
			//ROS_INFO("d_akt_r: %f", d_akt_r[i]);
			
			if(d_akt_r[i]>=abstand_punkte){
				if((i-c) >=4){
				//ROS_INFO("Punkt erkannt");
				//summe der punkte der punkte:
				for(a = c; a<i; a++){
					sum_x = sum_x + x[a];
					sum_y = sum_y + y[a];
				}
				//mittelwert
				mittelwert_x = sum_x/(i-c);
				mittelwert_y = sum_y/(i-c);
				
				//für online linien filter:
				reduzierte_x[anzahl_neuer_punkte] = mittelwert_x;
				reduzierte_y[anzahl_neuer_punkte] = mittelwert_y;
							 
				//ROS_INFO(" mittelwer x: %f", mittelwert_x);
				//ROS_INFO(" mittelwert y: %f", mittelwert_y);
				//Marker
				geometry_msgs::Point p;
					p.x = mittelwert_x;
					p.y = mittelwert_y;
					p.z = 0;

					points.points.push_back(p);
				//marker_points(mittelwert_x, mittelwert_y);
				
				sum_x = 0.0;
				sum_y = 0.0;
				anzahl_neuer_punkte++;
				c = i; //neuer punkt i fällt nicht mehr in die punkte reihe rein
				//ROS_INFO("anzahl nuer punkte %d", anzahl_neuer_punkte);
				}
				c = i;
			}		
					
		}
		reduktion_pub.publish(points);
		
}
void callback(laser_daten::myParamConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f",
             config.ek);
 
  abstand_punkte_red = config.abstand_punkte_red;
  fixed_frame = config.fixed_frame.c_str();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "g_reduktionsfilter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 50, laser_callback);
  reduktion_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_reduktion", 1000);
  
  //dynamic reconfigure
  dynamic_reconfigure::Server<laser_daten::myParamConfig> server;
  dynamic_reconfigure::Server<laser_daten::myParamConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Rate r(30);
  ros::spin();

}


