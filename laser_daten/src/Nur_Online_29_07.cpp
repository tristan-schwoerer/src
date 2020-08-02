#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <laser_daten/myParamConfig.h>

#include <cmath>
#include <vector>
#include <math.h>
#include <sstream>
#include <string>
using namespace std;
//test git

sensor_msgs::LaserScan laser;
ros::Publisher reduktion_pub;
ros::Publisher marker_pub;
ros::Publisher marker_pub_zwei;
float f = 0.0;
//dynamic reconfigure paramenter
double dyn_ek;
double add_ek;
double abstand_punkte_red;
double zul_einzelabstand;

std::string fixed_frame;



void laser_callback_nur_online(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Marker  1 für linien
		visualization_msgs::Marker line_list;
		line_list.header.frame_id = fixed_frame;
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

		//Marker 2 fuer linien
		visualization_msgs::Marker line_list_zwei;
		line_list_zwei.header.frame_id = fixed_frame;
		line_list_zwei.header.stamp = ros::Time::now();
		line_list_zwei.ns = "g_marker_line_list";
		line_list_zwei.action = visualization_msgs::Marker::ADD;
		line_list_zwei.pose.orientation.w = 1.0;
		line_list_zwei.id = 2;
		line_list_zwei.type = visualization_msgs::Marker::LINE_LIST;
		// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		line_list_zwei.scale.x = 0.01;
		// Line list is blue
		line_list_zwei.color.b = 1.0;
		line_list_zwei.color.a = 1.0;


		ROS_INFO("test erfolgreich");
		laser = *msg;
		int anzahl_punkte = laser.ranges.size();
		int anzahl_neuer_punkte = 0;

		double d[anzahl_punkte] = {0.0};				//fuer absatnd der einzelnen punkte zuerinander
		double d_akt[anzahl_punkte] = {0.0};			//für abstand erster punkt linie zu aktuellem punkt
		double p_1_x[anzahl_punkte] = {0.0};
		double p_1_y[anzahl_punkte] = {0.0};
		double p_2_x[anzahl_punkte] = {0.0};
		double p_2_y[anzahl_punkte] = {0.0};
		double ausgabe_d = 0;
		double vergleich = 0;
		
		//switch case
		int akt_zustand = 0;

		double sum = 0.0;
		double ek = dyn_ek;
		double ek_plus =add_ek;
		double dif_ek = 1-ek;
		double einzelabstand = zul_einzelabstand;
		int a = 0;
		int neu_anfang_linie = 0;
		int b= 0;
		int c = 0;
		
		//lineare regession
		double summe_x = 0.0;
		double summe_y = 0.0;
		double mittelwert_x = 0.0;
		double mittelwert_y = 0.0;
		double x_abschnitt = 0.0;
		double y_abschnitt = 0.0;
		double x_y_abschnitt = 0.0;
		double steigung_x = 0.0;
		double schnittp_x = 0.0;
		double steigung_y = 0.0;
		double schnittp_y = 0.0;
		int n = 0;
		int m = 0;
		double test_x = x_array[7];
		double test_y = y_array[7];

		ROS_INFO("anzahl_punkte %d", anzahl_punkte);
		ROS_INFO("x_aaray_7 %f", test_x);
		ROS_INFO("y_aaray_7 %f", test_y);
		
		x_array[0] = laser.ranges[0]*cos(laser.angle_min);		//x koordinaten des ersten punltes
		y_array[0]= laser.ranges[0]*sin(laser.angle_min);		//y coordianten des ersten punktes
		for(int i=1;i<anzahl_punkte; i++){
			x_array[i]= laser.ranges[i]*cos(laser.angle_min + i*laser.angle_increment);
			y_array[i]= laser.ranges[i]*sin(laser.angle_min + i*laser.angle_increment);
			if(neu_anfang_linie == 0){
				//abstand zwischen den einelnen letzten punkten
				d[i] = sqrt(pow((x_array[i]-x_array[i-1]),2)+pow((y_array[i]-y_array[i-1]),2));
				
				//summe der bisher erchneten absände
				sum = 0;
				for(a=(c+1);a<=i;a++){
					sum = sum + d[a];
				}
				//Abstand erster punkt der linie und aktueller punkt :
				d_akt[i] = sqrt(pow((x_array[i]-x_array[c]),2)+pow((y_array[i]-y_array[c]),2));
			
				ausgabe_d = d_akt[i];
			
				vergleich = d_akt[i]/sum;
				ROS_INFO("abstand punkte im linienfilter %f", d[i]);
				if(d[i]<= einzelabstand){
					if(vergleich<= 1 && vergleich> dif_ek )	{	//punkt passt nicht zur linie
											
						akt_zustand = 0;
					}
					else {
						akt_zustand = 1;
						if(i<=(c+3)){
							akt_zustand = 2;
						}
					}
				}
			
				else {
					akt_zustand = 1;
					if(i<=(c+3)){
						akt_zustand = 2;
					}
				}
				switch (akt_zustand){
						
					case 0:	//Punkte gehören zu einer linie 
						{//ROS_INFO("Punkte gleich");
						ek = ek + ek_plus;
						ROS_INFO("Punkte gehoren zur linie");
						}
						break;
					case 1: // Linie Abgebrochen
					{//start und endpunkte der linie
						// Lineare regesion
						summe_x = 0;
						summe_y = 0;
						mittelwert_x = 0;
						mittelwert_y = 0;
						x_abschnitt = 0;
						y_abschnitt = 0;
						x_y_abschnitt = 0;
						steigung_x = 0;
						schnittp_x = 0.0;
						steigung_y = 0;
						schnittp_y = 0.0;
						//1. Mittelwert x und y
						for ( n = c; n<=i;n++){
							summe_x += x_array[n];
							summe_y += y_array[n];
						}
						mittelwert_x = summe_x/(i-c+1);
						mittelwert_y = summe_y/(i-c+1);
						//
						for ( m = c; m<=i;m++){
							x_abschnitt += (x_array[m]-mittelwert_x)*(y_array[m]-mittelwert_y);
							y_abschnitt += (y_array[m]-mittelwert_y);
							x_y_abschnitt += pow((x_array[m]-mittelwert_x),2);
						}
						// steigung der gerade nach x im nenenr
						steigung_x = x_abschnitt/x_y_abschnitt;
						// schnittpunkt mit y achse nach x im nenner
						schnittp_x = mittelwert_y - (steigung_x*mittelwert_x);
						// steigung der gerade nach y im nenenr
						steigung_y = x_y_abschnitt/y_abschnitt;
						// schnittpunkt mit y achse nach y im nenner
						schnittp_y = mittelwert_y - steigung_y*mittelwert_x;


						//test andere linerae regesion
						/*for ( n = c; n<=i;n++){
							summe_x = summe_x + x_array[n];
							x_abschnitt = x_abschnitt + x_array[n]*x_array[n];
							summe_y = summe_y + y_array[n];
							y_abschnitt = y_abschnitt + y_array[n]*y_array[n];
							x_y_abschnitt = x_y_abschnitt + x_array[n]*y_array[n];
						}
						steigung_x = (i*x_y_abschnitt-summe_x*summe_y)/(i*x_abschnitt-summe_x*summe_x);
						schnittp_x = (summe_y-steigung_x*summe_x)/i;
	*/

						/*//Punkte für die gerade
						p_1_x[b] = x[c];
						p_1_y [b] = y[c];
						p_2_x [b]= x[i];
						p_2_y [b]= y[i];
	*/
						//Marker

						// Create the vertices for the points and lines
						//Marker 1 mit nenner x
						geometry_msgs::Point p;
						p.x = (y_array[c+1]+(x_array[c+1]/steigung_x)-schnittp_x)/(steigung_x+(1/steigung_x));
						p.y = steigung_x*x_array[c]+schnittp_x;
						p.z = 0;

						// The line list needs two points for each line
						line_list.points.push_back(p);
						p.x = (y_array[(i)]+(x_array[(i)]/steigung_x)-schnittp_x)/(steigung_x+(1/steigung_x));
						p.y = steigung_x*x_array[i-1]+schnittp_x;
						p.z = 0;
						line_list.points.push_back(p);
						
						//Marker 2 mit y nenner
						geometry_msgs::Point p_zwei;
						p_zwei.x = x_array[c];
						p_zwei.y = y_array[c];
						p_zwei.z = 0;

						// The line list needs two points for each line
						line_list_zwei.points.push_back(p_zwei);
						p_zwei.x = x_array[i-1];
						p_zwei.y = y_array[i-1];
						p_zwei.z = 0;
						line_list_zwei.points.push_back(p_zwei);
						//marker_funktion(p_1_x[b], p_1_y[b],p_2_x[b], p_2_y[b]);
						
						c = i+1;		//damit immer erster punkt der neuen linie klar
						neu_anfang_linie = 1;			//hilfvariable für neu anfang einer linie da dort d noch nicht berechnet werden kann
						ek = dyn_ek;		//wert wieder zurück setzten da neue linie satrtet
						b++;
						ROS_INFO("linie abgebrochen");
					}//ROS_INFO("linie abgebrochen");
						break;
					case 2:		//Fals nur 3 Punkte oder weniger zusammen fallen keine linie bilden
						{c = i+1;		//damit immer erster punkt der neuen linie klar
						neu_anfang_linie = 1;			//hilfvariable für neu anfang einer linie da dort d noch nicht berechnet werden kann
						ek = dyn_ek;
						ROS_INFO("3 oder weniger punkte");}		//wert wieder zurück setzten da neue linie satrtet
						break;
					default:
					
						break;
					
				}
				//ROS_INFO("dynamic ek:  %f", dif_ek);
				
			}
			else {
				neu_anfang_linie = 0;
			}

			//ROS_INFO("eine runde");
		}

		marker_pub.publish(line_list);
		marker_pub_zwei.publish(line_list_zwei);
	
	
		
}
void callback(laser_daten::myParamConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f",
             config.ek);
 
  dyn_ek = config.ek;
  add_ek = config.add_ek;
  zul_einzelabstand = config.zul_einzelabstand;
  fixed_frame = config.fixed_frame.c_str();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "Nur_Online_29_07");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 50, laser_callback_nur_online);

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_pub_zwei = n.advertise<visualization_msgs::Marker>("visualization_marker_zwei", 10);

  
  //dynamic reconfigure
  dynamic_reconfigure::Server<laser_daten::myParamConfig> server;
  dynamic_reconfigure::Server<laser_daten::myParamConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Rate r(30);
  ros::spin();

}


