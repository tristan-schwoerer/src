#include <math.h>
#include <vector>
#include <algorithm>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

#include <dynamic_reconfigure/server.h>
#include <hough_trafo/dynamic_reconfigureConfig.h>

using namespace std;

void chatterCallback(const sensor_msgs::LaserScan& msg);
void HoughTrafo(const sensor_msgs::LaserScan& _msg);
int checkQuarter(float _Angle);
float calcDistance(float range, float scanAngle, float angle);
void evaluateData(std::vector<std::vector<std::vector<int>>> Data, const sensor_msgs::LaserScan& _msg);
void buildline (visualization_msgs::Marker& _lines, const sensor_msgs::LaserScan& _msg,std::vector<int> _Index);
void marker_function (float x_1, float y_1, float x_2,float y_2,visualization_msgs::Marker& line_list, int Trigger);
void callback(hough_trafo::dynamic_reconfigureConfig &config, uint32_t level);


enum Threshold{Outer=3,Inner=1};
int RangeAmount,AngleAmount,Threshold;
ros::Publisher marker_pub;

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
		marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

		  //dynamic reconfigure
		  dynamic_reconfigure::Server<hough_trafo::dynamic_reconfigureConfig> server;
		  dynamic_reconfigure::Server<hough_trafo::dynamic_reconfigureConfig>::CallbackType f;

		  f = boost::bind(&callback, _1, _2);
		  server.setCallback(f);


		ros::spin();
		return 0;
	}

void chatterCallback(const sensor_msgs::LaserScan& msg)
	{
		HoughTrafo(msg);
	}

void callback(hough_trafo::dynamic_reconfigureConfig &config, uint32_t level) {

  RangeAmount = config.rangeamount;
  AngleAmount = config.angleamount;
  Threshold=config.outerthreshold;
}

void HoughTrafo(const sensor_msgs::LaserScan& _msg)
	{
	float MaxRange=*std::max_element(_msg.ranges.begin(),_msg.ranges.end());

	//Container Size
	float AngleSize=M_PI/AngleAmount;
	float RangeSize=(2*MaxRange)/RangeAmount;

	int RangeIndex;

	//generating containers
	std::vector<std::vector<std::vector<int> > > HoughLayer = vector<vector<vector<int> > >( AngleAmount, std::vector<std::vector<int> >(RangeAmount, std::vector<int>(1)));

	//iterating through all measured points
	for(int i=0;i<_msg.ranges.size();i++){
		//discarding all out of range msgs
		if(_msg.ranges[i]>_msg.range_min&&_msg.ranges[i]<_msg.range_max){
			//transforming to cartesian coordinates
			float X=cos(_msg.angle_min+(_msg.angle_increment*i))*_msg.ranges[i];
			float Y=sin(_msg.angle_min+(_msg.angle_increment*i))*_msg.ranges[i];


			for(int j=0;j<AngleAmount;j++)
			{
				//calculating range
				float range=X*cos(j*AngleSize)+Y*sin(j*AngleSize);

				//calculating correct bin
				if(range<0)
				{

						RangeIndex=round(range/RangeSize)+floor(RangeAmount/2);

				}
				else
				{
					if((RangeAmount%2)==1)
					{
						RangeIndex=round(range/RangeSize)+ceil(RangeAmount/2);
					}
					else
					{
						RangeIndex=round(range/RangeSize)+floor(RangeAmount/2)-1;
					}
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
				bool arraytrigger=1;
				std::vector<int> buffer;

				//finding chunks in array
				for(int k=1;k<=Data[i][j].size();k++)
				{
					int Space=abs((Data[i][j][k]-Data[i][j][k-1]));
					//check for gaps in the point set
					if(Space<=Threshold::Inner)
					{
						buffer.push_back(Data[i][j][k-1]);
					}

					if(Space>Threshold::Inner)
					{
						arraytrigger=true;
					}

					//empty the buffer array
					if(arraytrigger)
					{
						buildline(line_list,_msg,buffer);
						buffer.clear();
						arraytrigger=false;
					}
				}
			}
		}
		if(line_list.points.size()!=0)
		{
			marker_pub.publish(line_list);
		}
	}
void buildline (visualization_msgs::Marker& _lines, const sensor_msgs::LaserScan& _msg,std::vector<int> _Index)
	{

		float x_1,y_1,x_2,y_2;
		int max,min;
		if(_Index.size()>Threshold){
			min=*std::min_element(_Index.begin(),_Index.end());
			max=*std::max_element(_Index.begin(),_Index.end());
			x_1=cos(_msg.angle_min+(min*_msg.angle_increment))*_msg.ranges[min];
			y_1=sin(_msg.angle_min+(min*_msg.angle_increment))*_msg.ranges[min];
			x_2=cos(_msg.angle_min+(max*_msg.angle_increment))*_msg.ranges[max];
			y_2=sin(_msg.angle_min+(max*_msg.angle_increment))*_msg.ranges[max];
			marker_function(x_1,y_1,x_2,y_2,_lines,0);
		}
	}
void marker_function (float x_1, float y_1, float x_2,float y_2, visualization_msgs::Marker& lines, int Trigger){
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
}



