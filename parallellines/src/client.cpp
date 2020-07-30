#include <math.h>
#include <vector>
#include <algorithm>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

void calculateDistance(const visualization_msgs::Marker& _lines);
void chatterCallback(const visualization_msgs::Marker& line_list);

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "client");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("visualization_marker", 1000, chatterCallback);
		ros::spin();
		return 0;
	}
void chatterCallback(const visualization_msgs::Marker& line_list)
	{
		calculateDistance(line_list);
	}

void calculateDistance(const visualization_msgs::Marker& _lines)
	{
		float gradient;
		float StepWidth;
		float deltaX;
		float gradienttolerance=0.1;
		float newx, newy, newm, newb;
		float secx, secy;
		float Distance=0;
		std::vector<float> Steigungen={};
		std::vector<float> Intersect={};
		std::vector<geometry_msgs::Point> Point1={};
		std::vector<geometry_msgs::Point> Point2={};
		//get related line points and setting up data vectors for easy access
		ROS_INFO("%d",_lines.points.size());
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
						newx=k*(Point1[i].x-Point2[i].x)/10;
					    newy=k*(Point1[i].y-Point2[i].y)/10;
					    newb=newy-(newm*newx);
					    secx=(newm-Steigungen[j])/(Intersect[j]-newb);
						secy=(Steigungen[j]*secx)+Intersect[j];
						//Building Average Distance for this Line
						Distance=((Distance*k)+sqrt(pow(abs(secx-newx),2.0)+pow(abs(secy-newy),2.0)))/(k+1);//returns naN??
						ROS_INFO("%f" , Steigungen[j]);
					}
					//ROS_INFO("The avg Distance between these parallel Lines is %f",Distance);

				}
			}
		}

	}
