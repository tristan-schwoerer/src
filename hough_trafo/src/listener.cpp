#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

using namespace std;

double FindAngle(double* _Range, double* _Angle, double _Precission);
void chatterCallback(const sensor_msgs::LaserScan& msg);

const int Amount=5;
const double Precission=0.1;
const ros::NodeHandle Markerhandle;



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
		for(int i=0;i<msg.ranges.size();i+4)
		{
			double _range[Amount];
			double _angle[Amount];
			double _determinedAngle=FindAngle(_range,_angle,Precission);
		}
	}

/*
 * Determines the best angle for a Line going through a given set of points
 * Input: Range and Angle of the Point and the step width of the angle
 * Output: Determined Angle
 */

double FindAngle(double* _Range, double* _Angle, double _Precission)
	{
		enum {Default=1};
		double LineAngle;
		std::vector<double> _Diff;
		if(_Precission>90)
		{
			_Precission=Default;
		}
		for(int beta=0;beta<90;beta+_Precission)
		{
			double Distance[Amount];
			for(int i=0; i<Amount;i++)
			{
				Distance[i]=abs(sqrt(pow(_Range[i],2.0)-pow(cos(beta)*_Range[i]*(sin(_Angle[i])-tan(beta)*cos(_Angle[i])),2.0)));
			}
			_Diff[beta]=std::max_element(std::begin(Distance), std::end(Distance))-std::min_element(std::begin(Distance), std::end(Distance));
		}
		LineAngle=std::distance(std::begin(_Diff), std::min_element(std::begin(_Diff),std::end(_Diff)))*_Precission;
		return LineAngle;
	}


