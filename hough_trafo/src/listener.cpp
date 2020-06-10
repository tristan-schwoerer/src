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
void marker_function (float x_1, float y_1, float x_2,float y_2, ros::Publisher publisher,visualization_msgs::Marker& line_list, int Trigger);
//Container Amount
enum Amount{Angle=100,Range=151};
enum Threshold{Outer=40,Inner=2};
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
	float MaxRange=*std::max_element(_msg.ranges.begin(),_msg.ranges.end());

	//Container Size
	float AngleSize=M_PI/Amount::Angle;
	float RangeSize=(2*MaxRange)/Amount::Range;

	int RangeIndex;

	//generating containers
	std::vector<std::vector<std::vector<int> > > HoughLayer = vector<vector<vector<int> > >( Amount::Angle, std::vector<std::vector<int> >(Amount::Range, std::vector<int>(1)));

	//iterating through all measured points
	for(int i=0;i<_msg.ranges.size();i++){
		//discarding all out of range msgs
		if(_msg.ranges[i]>_msg.range_min&&_msg.ranges[i]<_msg.range_max){
			//transforming to cartesian coordinates
			float X=cos(_msg.angle_min+(_msg.angle_increment*i))*_msg.ranges[i];
			float Y=sin(_msg.angle_min+(_msg.angle_increment*i))*_msg.ranges[i];


			for(int j=0;j<Amount::Angle;j++)
			{
				//calculating range
				float range=X*cos(j*AngleSize)+Y*sin(j*AngleSize);

				//calculating correct bin
				if(range<0)
				{
					RangeIndex=round(range/RangeSize)+floor(Amount::Range/2);

				}
				else
				{
					RangeIndex=round(range/RangeSize)+ceil(Amount::Range/2);
				}
				//pushing point id into bin
				HoughLayer[j][RangeIndex].push_back(i);
			}
		}

	}
	evaluateData(HoughLayer, _msg);
	}
void evaluateData(std::vector<std::vector<std::vector<int>>> Data, const sensor_msgs::LaserScan& _msg)
	{
	ROS_INFO("started Evaluation");
	float x_1,y_1,x_2,y_2;
	int max,min;

	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "hough_line_list";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.01;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

		for(int i=0;i<Data.size();i++)
		{
			for(int j=0;j<Data[i].size();j++)
			{
				if(Data[i][j].size()>Threshold::Outer)
				{
					bool arraytrigger=1;
					std::vector<int> buffer;
					//finding chunks in array
					for(int k=i;k<Data[i][j].size();k++)
					{


						//check for gaps in the point set
						if((Data[i][j][k]-Data[i][j][k-1])<=Threshold::Inner)
						{
							buffer.push_back(Data[i][j][k-1]);
						}

						if((Data[i][j][k]-Data[i][j][k-1])>=Threshold::Inner)
						{
							arraytrigger=true;
						}

						//empty the buffer array
						if(arraytrigger)
						{
							//TODO linebuild up
							buffer.clear();
							arraytrigger=false;
						}

					}
					min=*std::min_element(Data[i][j].begin()+2,Data[i][j].end()-1);
					max=*std::max_element(Data[i][j].begin()+2,Data[i][j].end()-1);
					x_1=cos(_msg.angle_min+(min*_msg.angle_increment))*_msg.ranges[min];
					y_1=sin(_msg.angle_min+(min*_msg.angle_increment))*_msg.ranges[min];
					x_2=cos(_msg.angle_min+(max*_msg.angle_increment))*_msg.ranges[max];
					y_2=sin(_msg.angle_min+(max*_msg.angle_increment))*_msg.ranges[max];


					if(i==(Data.size()-1)&&(j==Data[i].size()-1))
					{
						marker_function(x_1,y_1,x_2,y_2,marker_pub,line_list,0);
					}
					else
					{
						marker_function(x_1,y_1,x_2,y_2,marker_pub,line_list,1);
					}

				}
			}
		}
	}

void marker_function (float x_1, float y_1, float x_2,float y_2, ros::Publisher publisher,visualization_msgs::Marker& lines, int Trigger){

	geometry_msgs::Point Points;

	Points.x = x_1;
	Points.y = y_1;
	Points.z = 0;

	// The line list needs two points for each line
	lines.points.push_back(Points);
	Points.x = x_2;
	Points.y = y_2;
	Points.z = 0;
	lines.points.push_back(Points);

	if(Trigger==1)
	{
		publisher.publish(lines);
	}
}



