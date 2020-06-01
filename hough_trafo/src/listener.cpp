#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

using namespace std;

void chatterCallback(const sensor_msgs::LaserScan& msg);
void HoughTrafo(const sensor_msgs::LaserScan& _msg);
int checkQuarter(float _Angle);
float calcDistance(float range, float scanAngle, float angle);

const ros::NodeHandle Markerhandle;
enum Precission{Angle=1000,Range=1000};



int main(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
		ros::spin();
		//test
		return 0;
	}

void chatterCallback(const sensor_msgs::LaserScan& msg)
	{
		ROS_INFO("LaserData: [%f]",msg.ranges[360]);
		HoughTrafo(msg);

	}
void HoughTrafo(const sensor_msgs::LaserScan& _msg)
	{
	const int AngleAmount=Precission::Angle*180;
	const int RangeAmount=2*Precission::Range*_msg.range_max;
	int RangeIndex;
	std::vector<std::vector<int>> HoughLayer(AngleAmount,std::vector<int>(RangeAmount));
		for(int i=0;i<_msg.ranges.size();i++)
		{
			for(int j=0;j<(Precission::Angle*180);j++)
			{
				float range=calcDistance(_msg.ranges[i],_msg.angle_increment*i,(j/Precission::Angle));
				if(range<0)
				{
					RangeIndex=abs(range/Precission::Range);
				}
				else
				{
					RangeIndex=(range/Precission::Range)+(Precission::Range*_msg.range_max);
				}
				HoughLayer[j][RangeIndex] += 1;
			}

		}
		evaluateData(HoughLayer);
	}

float calcDistance(float range, float scanAngle, float angle)
	{
	//this function calculates the y-axis intersection for a given Point in polar coordinates and an angle of a line going through that point
		float Distance;
		if((scanAngle==0)||(scanAngle==M_PI/2)||(scanAngle==M_PI)||(scanAngle==(3*M_PI)/2)||(scanAngle==2*M_PI)||(scanAngle==angle))
		{
			return 0;
		}
		else
		{
			switch (checkQuarter(scanAngle)){
				case 1:
					if(scanAngle>angle)
					{
						Distance=sin(scanAngle-angle)*range;
					}
					else
					{
						Distance=sin(angle-scanAngle)*range;
					}
					break;
				case 2:
					if(scanAngle>angle)
					{
						Distance=sin(scanAngle-M_PI-angle)*range;
					}
					else
					{
						Distance=sin(angle+M_PI-scanAngle)*range;
					}
					break;

			}
			if(angle<M_PI/2)
			{
				return Distance/cos(scanAngle);
			}
			else
			{
				return Distance/sin(M_PI-scanAngle);
			}
		}
	}
int checkQuarter(float _Angle)
	{
	//function checks if the point is in the upper or lower half of the coordinate system
		if(_Angle<M_PI)
		{
			return 1;
		}
		else if(_Angle<2*M_PI)
		{
			return 2;
		}
		else
		{
			return 0;
		}
	}



