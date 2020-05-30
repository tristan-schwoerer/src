#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <cmath>
#include <vector>
#include <math.h>
#include <sstream>

/*
std::vector<float> x;
std::vector<float> y;
std::vector<float> d;
*/

sensor_msgs::LaserScan laser;
ros::Publisher marker_pub;
float f = 0.0;


void marker_funktion (float x_1, float y_1, float x_2,float y_2){
	//Marker:
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "g_marker_line_list";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;



	line_list.id = 2;



	line_list.type = visualization_msgs::Marker::LINE_LIST;




	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	line_list.scale.x = 0.01;


	// Line list is red
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;



	// Create the vertices for the points and lines

	geometry_msgs::Point p;
	p.x = x_1;
	p.y = y_1;
	p.z = 0;

	// The line list needs two points for each line
	line_list.points.push_back(p);
	p.x = x_2;
	p.y = y_2;
	p.z = 0;
	line_list.points.push_back(p);



	marker_pub.publish(line_list);

}



void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("test erfolgreich");
	laser = *msg;
	// Test onlne linien
		int anzahl_punkte = laser.ranges.size();

		float x[anzahl_punkte] = {0.0};
		float y[anzahl_punkte] = {0.0};
		float d[anzahl_punkte] = {0.0};				//fuer absatnd der einzelnen punkte zuerinander
		float d_akt[anzahl_punkte] = {0.0};			//für abstand erster punkt linie zu aktuellem punkt
		float p_1_x[anzahl_punkte] = {0.0};
		float p_1_y[anzahl_punkte] = {0.0};
		float p_2_x[anzahl_punkte] = {0.0};
		float p_2_y[anzahl_punkte] = {0.0};
		float ausgabe_d = 0;
		float vergleich = 0;


		float sum = 0.0;
		float ek = 0.08;
		float dif_ek = 1-ek;;
		int a = 0;
		int b = 0;
		int c = 0;

		x[0] = laser.ranges[0]*cos(laser.angle_min);		//x koordinaten des ersten punltes
		y[0]= laser.ranges[0]*sin(laser.angle_min);		//y coordianten des ersten punktes
		for(int i=1;i<anzahl_punkte; i++){
			//in x und y koordianten umrechnen
			x[i]= laser.ranges[i]*cos(laser.angle_min + i*laser.angle_increment);
			y[i]= laser.ranges[i]*sin(laser.angle_min + i*laser.angle_increment);


			if(b == 0){
				//abstand zwischen den einelnen letzten punkten
				d[i] = sqrt(pow((x[i]-x[i-1]),2)+pow((y[i]-y[i-1]),2));
				if(i>=(c+2)){			//damit mindest 3 punkte miteinander verglichen werden
					//summe der bisher erchneten absände
					sum = 0;
					for(a=(c+1);a<=i;a++){
						sum = sum + d[a];
					}
					//Abstand erster punkt der linie und aktueller punkt :
					d_akt[i] = sqrt(pow((x[i]-x[c]),2)+pow((y[i]-y[c]),2));
					//ROS_INFO("summe einze absände: %f", sum);
					ausgabe_d = d_akt[i];
					//ROS_INFO("absand erster zu letzer: %f", ausgabe_d);
					vergleich = d_akt[i]/sum;
					//ROS_INFO("vergleich: %f", vergleich);
					//hier muss noch d_akt[i] darf einen wert nicht überschreiten sonst wird linie abgebrochen

					//if((d_akt[i]/sum)<= 1 && (d_akt[i]/sum)> (1-ek))	{	//punkt passt nicht zur linie
					if(vergleich<= 1 && vergleich> dif_ek )	{	//punkt passt nicht zur linie
						ROS_INFO("Punkte gleich");
						//ek= ek +0.04286029934883123755;
						//marker_funktion(3.0,2.0,7.0, 5.0);


					}
					else {
						//start und endpunkte der linie

						p_1_x[b] = x[c];
						p_1_y [b] = y[c];
						p_2_x [b]= x[i];
						p_2_y [b]= y[i];


						marker_funktion(p_1_x[b], p_1_y[b],p_2_x[b], p_2_y[b]);
						//x[anzahl_punkte] = {0.0};
						//y[anzahl_punkte] = {0.0};
						c = i+1;		//damit immer erster punkt der neuen linie klar
						b = 1;			//hilfvariable für neu anfang einer linie da dort d noch nicht berechnet werden kann
						ek = 0.5;		//wert wieder zurück setzten da neue linie satrtet
						ROS_INFO("linie abgebrochen");


					}
				}
			}
			else {
				b = 0;
			}

			//ROS_INFO("eine runde");
		}


}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "g_marker_online");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 50, laser_callback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);
  ros::spin();

}


