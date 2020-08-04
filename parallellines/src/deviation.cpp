#include <math.h>
#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include <dynamic_reconfigure/server.h>
#include <parallellines/dynamic_reconfigure_deviationConfig.h>
#include <string>
using namespace std;


void evaluation(const sensor_msgs::LaserScan& _msg);
void chatterCallback(const sensor_msgs::LaserScan& msg);
void callback(parallellines::dynamic_reconfigure_deviationConfig &config, uint32_t level);


int Amount=100;
bool Trigger=false;
int Index=100;
int Counter=0;
std::vector<double> measurements;

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);

		  //dynamic reconfigure
		  dynamic_reconfigure::Server<parallellines::dynamic_reconfigure_deviationConfig> server;
		  dynamic_reconfigure::Server<parallellines::dynamic_reconfigure_deviationConfig>::CallbackType f;

		  f = boost::bind(&callback, _1, _2);
		  server.setCallback(f);
		ros::spin();
		return 0;
	}

//Callbacks
void chatterCallback(const sensor_msgs::LaserScan& msg)
	{
		if(Trigger==true){

			evaluation(msg);
		}
	}

void evaluation(const sensor_msgs::LaserScan& _msg)
	{
		double max,min,avg,sum;
		double diff1, diff2, maxdiff, stdvariance,moin;
		avg=0;

		while(measurements.size()<Amount-1)
		{
			measurements.push_back(0);
		}
		measurements[Counter]=_msg.ranges[Index];
		Counter++;
		if(measurements[Amount-1]!=0){
			for(int i=0; i<Amount;i++)
			{
				sum+=measurements[i];
			}
			max=*max_element(measurements.begin(),measurements.end());
			min=*min_element(measurements.begin(),measurements.end());
			ROS_INFO("max-min %f",max-min);
			avg=sum/Amount;
			diff1=max-avg;
			diff2=min-avg;
			if(diff1>=diff2)
			{
				maxdiff=diff1;
			}
			else
			{
				maxdiff=diff2;
			}
			for(int j=0;j<Amount;j++)
			{
				moin+=abs(pow(measurements[j]-avg,2.0));
			}
			stdvariance=sqrt(moin/(Amount-1));
			ROS_INFO("Standardabweichung %f",stdvariance);
			ROS_INFO("Maximale Abweichung vom Mittelwert %f",maxdiff);
			fill_n(measurements.begin(),Amount,0);
			Trigger=false;
			Counter=0;
		}
	}


void callback(parallellines::dynamic_reconfigure_deviationConfig &config, uint32_t level){
  Trigger = config.Trigger;
  Amount=config.PointAmount;
  Index=config.Index;
}



