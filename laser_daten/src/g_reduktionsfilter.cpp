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

/*
std::vector<float> x;
std::vector<float> y;
std::vector<float> d;
*/

sensor_msgs::LaserScan laser;
sensor_msgs::LaserScan filter_laser;
ros::Publisher reduktion_pub;
int anzahl_neuer_punkte = 0;

ros::Publisher marker_pub;
float f = 0.0;
double dyn_ek;



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



//comment
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



void marker_points (float x_r, float y_r, bool pub){

	//Marker:
	visualization_msgs::Marker points;
	points.header.frame_id = "/map";
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

	if (pub == true){
		reduktion_pub.publish(points);
	}



}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("test erfolgreich");
	laser = *msg;
	// Test Reduktionsfilter

		int anzahl_punkte = laser.ranges.size();
		anzahl_neuer_punkte = 0;
		float x[anzahl_punkte] = {0.0};
		float y[anzahl_punkte] = {0.0};
		float d[anzahl_punkte] = {0.0};				//fuer absatnd der einzelnen punkte zuerinander
		float d_akt[anzahl_punkte] = {0.0};			//für abstand erster punkt linie zu aktuellem punkt
		float d_akt_r[anzahl_punkte] = {0.0};			//für abstand erster punkt linie zu aktuellem punkt
		float p_1_x[anzahl_punkte] = {0.0};
		float p_1_y[anzahl_punkte] = {0.0};
		float p_2_x[anzahl_punkte] = {0.0};
		float p_2_y[anzahl_punkte] = {0.0};
		float ausgabe_d = 0;
		float vergleich = 0;


		double sum = 0;
		double sum_x = 0.0;
		double sum_y = 0.0;
		double ek = dyn_ek;
		double dif_ek = 1-ek;;
		int a = 0;
		int b = 0;
		int c = 0;
		int g = 0;		//c für online 
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
			
			if(d_akt_r[i]>=0.05){
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
				
				marker_points(mittelwert_x, mittelwert_y, letzer_punkt);
				
				sum_x = 0.0;
				sum_y = 0.0;
				anzahl_neuer_punkte++;
				c = i; //neuer punkt i fällt nicht mehr in die punkte reihe rein
				//ROS_INFO("anzahl nuer punkte %d", anzahl_neuer_punkte);
			}		
					
		}
		int gesamt_anzahl_neu = anzahl_neuer_punkte -1;
		//aschliesender online liniefilter

		
		for(int j=1;j<gesamt_anzahl_neu; j++){
			ROS_INFO("ek aktuell: %f", ek);

			if(b == 0){
				//abstand zwischen den einelnen letzten punkten
				d[j] = sqrt(pow((reduzierte_x[j]-reduzierte_x[j-1]),2)+pow((reduzierte_y[j]-reduzierte_y[j-1]),2));
				if(j>=(g+2)){			//damit mindest 3 punkte miteinander verglichen werden
					//summe der bisher erchneten absände
					sum = 0;
					for(int h=(g+1);h<=j;h++){
						sum = sum + d[h];
					}
					//Abstand erster punkt der linie und aktueller punkt :
					d_akt[j] = sqrt(pow((reduzierte_x[j]-reduzierte_x[g]),2)+pow((reduzierte_y[j]-reduzierte_y[g]),2));
					//ROS_INFO("summe einze absände: %f", sum);
					ausgabe_d = d_akt[j];
					//ROS_INFO("absand erster zu letzer: %f", ausgabe_d);
					vergleich = d_akt[j]/sum;
					//ROS_INFO("vergleich: %f", vergleich);
					//hier muss noch d_akt[i] darf einen wert nicht überschreiten sonst wird linie abgebrochen

					//if((d_akt[i]/sum)<= 1 && (d_akt[i]/sum)> (1-ek))	{	//punkt passt nicht zur linie
					if(vergleich<= 1 && vergleich> dif_ek )	{	//punkt passt nicht zur linie
						//ROS_INFO("Punkte gleich");
						//ek= ek +0.04286029934883123755;
						//marker_funktion(3.0,2.0,7.0, 5.0);


					}
					else {
						//start und endpunkte der linie

						p_1_x[b] = reduzierte_x[g];
						p_1_y [b] = reduzierte_y[g];
						p_2_x [b]= reduzierte_x[j];
						p_2_y [b]= reduzierte_y[j];


						marker_funktion(p_1_x[b], p_1_y[b],p_2_x[b], p_2_y[b]);
						//x[anzahl_punkte] = {0.0};
						//y[anzahl_punkte] = {0.0};
						g = j+1;		//damit immer erster punkt der neuen linie klar
						b = 1;			//hilfvariable für neu anfang einer linie da dort d noch nicht berechnet werden kann
						//ek = 0.5;		//wert wieder zurück setzten da neue linie satrtet
						//ROS_INFO("linie abgebrochen");


					}
				}
			}
			else {
					b = 0;
			}

			//ROS_INFO("eine runde");
		}

}
void callback(laser_daten::myParamConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f",
             config.ek);
  dyn_ek = config.ek;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "g_reduktionsfilter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 50, laser_callback);
  reduktion_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_reduktion", 1000);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  //dynamic reconfigure
  dynamic_reconfigure::Server<laser_daten::myParamConfig> server;
  dynamic_reconfigure::Server<laser_daten::myParamConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Rate r(30);
  ros::spin();

}


