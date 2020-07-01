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
void evaluateData(std::vector<std::vector<std::vector<int>>> Data, const sensor_msgs::LaserScan& _msg);
void buildline (visualization_msgs::Marker& _lines, const sensor_msgs::LaserScan& _msg,std::vector<int> _Index);
void callback(hough_trafo::dynamic_reconfigureConfig &config, uint32_t level);
bool FindPoint(int Value,const std::vector<int>& LinePoints);
void searchChunks(visualization_msgs::Marker& _lines, const sensor_msgs::LaserScan& _msg,std::vector<int> _Index);
void checkAngleIncrement(const sensor_msgs::LaserScan& _msg);
//Global Variables
int RangeAmount,AngleAmount,Threshold,Gap,Chunks;
ros::Publisher marker_pub;

double angleIncrement;



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
//Callbacks
void chatterCallback(const sensor_msgs::LaserScan& msg)
	{
	checkAngleIncrement(msg);
	HoughTrafo(msg);
	}
void checkAngleIncrement(const sensor_msgs::LaserScan& _msg)
	{
		if((_msg.angle_max-_msg.angle_min)/(p_msg.ranges.size())!=_msg.angle_Increment)
		{
			angleIncrement=(_msg.angle_max-_msg.angle_min)/(_msg.ranges.size());
			ROS_INFO("Angle Increment has been adjusted");
		}
		else
		{
			angleIncrement=_msg.angle_increment;
		}
	}
void callback(hough_trafo::dynamic_reconfigureConfig &config, uint32_t level) {
  RangeAmount = config.rangeamount;
  AngleAmount = config.angleamount;
  Threshold=config.Threshold;
  Gap=config.MaxGap;
  Chunks=config.Chunk;
}
//Calculation
void HoughTrafo(const sensor_msgs::LaserScan& _msg)
	{
	//Container Size
	float AngleSize=M_PI/AngleAmount;
	float RangeSize=(2*_msg.range_max)/RangeAmount;

	int RangeIndex;

	//generating containers
	std::vector<std::vector<std::vector<int> > > HoughLayer = vector<vector<vector<int> > >( AngleAmount, std::vector<std::vector<int> >(RangeAmount, std::vector<int>(1)));

	//iterating through all measured points
	for(int i=0;i<_msg.ranges.size();i++){
		//discarding all out of range msgs
		if(_msg.ranges[i]>_msg.range_min&&_msg.ranges[i]<_msg.range_max){
			//transforming to cartesian coordinates
			float X=cos(_msg.angle_min+(angleIncrement*i))*_msg.ranges[i];
			float Y=sin(_msg.angle_min+(angleIncrement*i))*_msg.ranges[i];


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
//Evaluation
void evaluateData(std::vector<std::vector<std::vector<int>>> Data, const sensor_msgs::LaserScan& _msg)
	{
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/laser_frame";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "hough_line_list";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.01;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;
	//function searches longest line in given Data returns this line and deletes its Points out of Data
	//returns 0 if no line > then threshold is left in array
	int MaxI,MaxJ,MaxPeak,Counter;
	std::vector<int> buffer;
	MaxPeak=Threshold+1;
	//Finding Peak
	while(MaxPeak>Threshold){
		Counter=0;
		for(int i=0;i<Data.size();i++)
		{
			for(int j=0;j<Data[i].size();j++)
			{
				if(Data[i][j].size()>Counter)
				{
					MaxI=i;
					MaxJ=j;
					Counter=Data[MaxI][MaxJ].size();
				}
			}
		}
		MaxPeak=Counter;
		if(MaxPeak>=Threshold)
		{
			buffer.clear();
			buffer=Data[MaxI][MaxJ];
			//Deleting Point in every other Line
			for(int i=0;i<Data.size();i++)
			{
				for(int j=0;j<Data[i].size();j++)
				{
					if(Data[i][j].size()>=Threshold)
					{
						for(int k=0;k<Data[i][j].size();k++)
						{
							//iterating through Line since it always has less elements
							if(FindPoint(Data[i][j][k],buffer))
							{
								Data[i][j].erase(begin(Data[i][j])+k);
							}
						}
					}
				}
			}
			if(Chunks)
			{
				searchChunks(line_list,_msg,buffer);

			}
			else
			{
				buildline(line_list,_msg,buffer);
			}
		}
	}
	if(line_list.points.size()!=0)
		{
			marker_pub.publish(line_list);
		}
	else
		{
			ROS_INFO("No Lines found for given Parameters");
		}

	}
bool FindPoint(int Value,const std::vector<int>& LinePoints)
	{
	//Checks if given Point is Contained in given vector
	for(int i=0;i<LinePoints.size();i++)
		{
		if(Value==LinePoints[i]){
			return 1;
		}
	}
	return 0;
	}
void searchChunks(visualization_msgs::Marker& _lines, const sensor_msgs::LaserScan& _msg,std::vector<int> _Index)
	{
	//function detects chunks of points and starts line build up for every chunk
		bool arraytrigger=1;
		std::vector<int> buffer;
		//finding chunks in array
		for(int k=1;k<_Index.size();k++)
		{

			int Space=abs(_Index[k]-_Index[k-1]);
			//check for gaps in the point set
			if(Space<=Gap)
			{
				buffer.push_back(_Index[k-1]);
			}

			if(Space>Gap)
			{
				arraytrigger=true;
			}

			//empty the buffer array
			if(arraytrigger)
			{
				buildline(_lines,_msg,buffer);
				buffer.clear();
				arraytrigger=false;
			}
		}
	}
void buildline (visualization_msgs::Marker& _lines, const sensor_msgs::LaserScan& _msg,std::vector<int> _Index)
	{
	geometry_msgs::Point Points;
		float x_1,y_1,x_2,y_2;
		int max,min;
		double x_avg,y_avg,beta0,beta1,denominator,counter;
		std::vector<double> XValues,YValues;
		if(_Index.size()>=Threshold){
			//calculating regression line
			for(int i=0;i<_Index.size();i++)
			{
				XValues.push_back(cos(_msg.angle_min+(_Index[i]*angleIncrement))*_msg.ranges[_Index[i]]);
				YValues.push_back(sin(_msg.angle_min+(_Index[i]*angleIncrement))*_msg.ranges[_Index[i]]);

				x_avg+=XValues[i];
				y_avg+=YValues[i];
			}

			x_avg/=_Index.size();
			y_avg/=_Index.size();

			for(int i=0;i<_Index.size();i++)
			{
				counter+=(XValues[i]-x_avg)*(YValues[i]-y_avg);
				denominator+=pow((XValues[i]-x_avg),2);
			}
			beta1=counter/denominator;
			beta0=y_avg-(beta1*x_avg);

			x_1=(YValues[0]+(XValues[0]/beta1)-beta0)/(beta1+(1/beta1));
			y_1=(beta1*XValues[0])+beta0;

			x_2=(YValues[YValues.size()-1]+(XValues[XValues.size()-1]/beta1)-beta0)/(beta1+(1/beta1));
			y_2=(beta1*XValues[XValues.size()-1])+beta0;
			XValues.clear();
			YValues.clear();
		}
		Points.x = x_1;
		Points.y = y_1;
		Points.z = 0;
		// The line list needs two points for each line
		_lines.points.push_back(Points);
		Points.x = x_2;
		Points.y = y_2;
		Points.z = 0;
		_lines.points.push_back(Points);

	}



