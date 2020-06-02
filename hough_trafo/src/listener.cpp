#include <math.h>
#include <vector>
#include <algorithm>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"


using namespace std;


void chatterCallback(const sensor_msgs::LaserScan& msg);
void HoughTrafo(const sensor_msgs::LaserScan& _msg);
int checkQuarter(float _Angle);
float calcDistance(float range, float scanAngle, float angle);
void evaluateData(std::vector<std::vector<std::vector<int>>> Data, const sensor_msgs::LaserScan& _msg);
void marker_function (float x_1, float y_1, float x_2,float y_2, ros::Publisher publisher);


enum Precission{Angle=1000,Range=10};
ros::Publisher marker_pub;

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
		marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		ros::spin();
		//test
		return 0;
	}

void chatterCallback(const sensor_msgs::LaserScan& msg)
	{

		HoughTrafo(msg);
	}
void HoughTrafo(const sensor_msgs::LaserScan& _msg)
	{
	ROS_INFO("started calculation");
	const int AngleAmount=Precission::Angle*180;
	const int RangeAmount=2*Precission::Range*_msg.range_max;
	int RangeIndex;
	std::vector<std::vector<std::vector<int> > > HoughLayer = vector<vector<vector<int> > >( AngleAmount, std::vector<std::vector<int> >(RangeAmount, std::vector<int>(1)));
		for(int i=0;i<_msg.ranges.size();i++)
		{
			for(int j=0;j<(Precission::Angle*180);j++)
			{
				float range=calcDistance(_msg.ranges[i],_msg.angle_increment*i,(j/Precission::Angle));
				if(range<0)
				{
					RangeIndex=abs(range/Precission::Range);
					ROS_INFO("%d",RangeIndex);
				}
				else
				{
					RangeIndex=(range/Precission::Range)+(Precission::Range*_msg.range_max);
				}

				//Add Index of Point to Vector of these Parameters
				HoughLayer[j][RangeIndex].push_back(i);
			}

		}
		evaluateData(HoughLayer, _msg);
	}
void evaluateData(std::vector<std::vector<std::vector<int>>> Data, const sensor_msgs::LaserScan& _msg)
	{
	ROS_INFO("started Evaluation");
	float x_1,y_1,x_2,y_2;
	int max,min=0;

		for(int i=0;i<Data.size();i++)
		{
			for(int j=0;j<Data[i].size();j++)
			{
				if(Data[i][j].size()>1)
				{
					for(int k=0;k<Data[i][j].size();k++)
					{
						min=Data[i][j][0];
						if(Data[i][j][k]<min)
						{
							min=Data[i][j][k];
						}
						if(Data[i][j][k]>max)
						{
							max=Data[i][j][k];
						}
					}
					x_1=cos(min*_msg.angle_increment)*_msg.ranges[min];
					y_1=sin(min*_msg.angle_increment)*_msg.ranges[min];
					x_2=cos(max*_msg.angle_increment)*_msg.ranges[max];
					y_2=sin(max*_msg.angle_increment)*_msg.ranges[max];
					ROS_INFO("Found Points at");
					marker_function(x_1,y_1,x_2,y_2,marker_pub);
				}
			}
		}
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

void marker_function (float x_1, float y_1, float x_2,float y_2, ros::Publisher publisher){
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



	publisher.publish(line_list);

}



