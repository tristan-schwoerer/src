#include <math.h>
#include <vector>
#include <algorithm>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

void chatterCallback(const visualization_msgs::Marker& line_list);
void calculateDistance(const visualization_msgs::Marker& _lines);

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("visualization_marker", 1000, chatterCallback);

		marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		ros::spin();
		return 0;
	}
void chatterCallback(const visualization_msgs::Marker& line_list)
	{
		calculateDistance(line_list)
	}

void calculateDistance(const visualization_msgs::Marker& _lines)
	{
		float gradient;
		std::vector<float>(_lines.points.x.size()/2) Steigungen;
		for(int i=0; i<_lines.points.x.size()-1; i+2)
		{
			gradient=(_lines.points.y[i+1]-_lines.points.y[i])/(_lines.points.x[i+1]-_lines.points.x[i]);
			if(i==0)
			{
				Steigungen[i]=gradient;
			}
			else
			{
				Steigungen[i/2]=gradient;
			}
		}
	}
0=0
1=2
2=4
3=6
4=8
5=10
