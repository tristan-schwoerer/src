#include <math.h>
#include <vector>
#include <algorithm>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <dynamic_reconfigure/server.h>
#include <parallellines/dynamic_reconfigureConfig.h>

void calculateDistance(const visualization_msgs::Marker& _lines);
void chatterCallback(const visualization_msgs::Marker& line_list);
void callback(parallellines::dynamic_reconfigureConfig &config, uint32_t level);

double gradienttolerance=0.1;

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "client");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("visualization_marker", 1000, chatterCallback);
		  dynamic_reconfigure::Server<parallellines::dynamic_reconfigureConfig> server;
		  dynamic_reconfigure::Server<parallellines::dynamic_reconfigureConfig>::CallbackType f;
		  f = boost::bind(&callback, _1, _2);
		  server.setCallback(f);
		ros::spin();
		return 0;
	}
void callback(parallellines::dynamic_reconfigureConfig &config, uint32_t level) {
	gradienttolerance = config.angletolerance;
}
void chatterCallback(const visualization_msgs::Marker& line_list)
	{
		calculateDistance(line_list);
	}

void calculateDistance(const visualization_msgs::Marker& _lines)
	{

	/*
	 * Disclaimer:
	 * Unfortunately i wasn't able to fix this.
	 * The Issue is that handling the points in a line_list doesn't work.
	 * The line_list points are all represented in extremely high or low numbers hence the point coordinates can't be extracted and the script returns infinite distance.
	 * The line_list is accurate since Rviz can handle the points.
	 *
	 * A possible solution would be to publish a separate msg of type geometry_msgs::Point [] or std::vector<geometry_msgs::Point> containing all points
	 * but that would destroy the purpose of this beeing able to handle data from any line algorithm.
	 */

	//This function is supposed to calculate the average distance of two nearly parallel lines.
	//The Tolerance can be adjusted with the Variable gradienttolerance. (1=45°)

		float gradient;
		float StepWidth;
		float deltaX;
		float newx, newy, newm, newb;
		float secx, secy;
		float Distance;
		std::vector<float> Steigungen;
		std::vector<float> Intersect;
		std::vector<geometry_msgs::Point> Point1;
		std::vector<geometry_msgs::Point> Point2;

		//split points of line list into two vectors to separate all start and endpoints
		for(int i=1; i<_lines.points.size(); i+=2)
		{
			Point1.push_back(_lines.points[i-1]);
			Point2.push_back(_lines.points[i]);
			gradient=(_lines.points[i-1].y-_lines.points[i].y)/(_lines.points[i-1].x-_lines.points[i].x);

			//calculating line coefficients
			Steigungen.push_back(gradient);
		    Intersect.push_back(_lines.points[i-1].y-gradient*_lines.points[i-1].x);
		}
		//find parallel lines
		for(int i=0;i<Steigungen.size();i++)
		{
			ROS_INFO("%f",Steigungen[i]);
			for(int j=i+1;j<Steigungen.size();j++)
			{
				//Calculate Distance between lines if gradient is similar
				if(Steigungen[i]-Steigungen[j]<gradienttolerance)
				{
					Distance=0;
					newm=-1/Steigungen[i];
					// calc new points and measure Distance
					for (int k=0; k<=10; k++)
					{
						//coefficients of 'Lotgerade'
						ROS_INFO("%f",Point1[i].x);
						newx=k*(Point1[i].x-Point2[i].x)/10;
					    newy=k*(Point1[i].y-Point2[i].y)/10;
					    newb=newy-(newm*newx);
					    secx=(newm-Steigungen[j])/(Intersect[j]-newb);
						secy=(Steigungen[j]*secx)+Intersect[j];
						//Building Average Distance for this Line
						Distance=((Distance*k)+sqrt(pow(abs(secx-newx),2.0)+pow(abs(secy-newy),2.0)))/(k+1);
					}
					ROS_INFO("The avg Distance between these parallel Lines is %f",Distance);
				}
			}
		}

	}
