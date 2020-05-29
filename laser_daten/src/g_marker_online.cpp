#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <cmath>

sensor_msgs::LaserScan laser;
ros::Publisher marker_pub;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("test erfolgreich");
	laser = *msg;
	float f = 0.0;
	  while (ros::ok())
	  {

	    visualization_msgs::Marker line_list;
	    line_list.header.frame_id = "/my_frame";
	    line_list.header.stamp = ros::Time::now();
	    line_list.ns = "g_marker_line_list";
	    line_list.action = visualization_msgs::Marker::ADD;
	    line_list.pose.orientation.w = 1.0;



	    line_list.id = 2;



	    line_list.type = visualization_msgs::Marker::LINE_LIST;




	    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	    line_list.scale.x = 0.1;


	    // Line list is red
	    line_list.color.r = 1.0;
	    line_list.color.a = 1.0;



	    // Create the vertices for the points and lines
	    for (uint32_t i = 0; i < 100; ++i)
	    {
	      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
	      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

	      geometry_msgs::Point p;
	      p.x = (int32_t)i - 50;
	      p.y = y;
	      p.z = z;

	      // The line list needs two points for each line
	      line_list.points.push_back(p);
	      p.z += 1.0;
	      line_list.points.push_back(p);
	    }


	    marker_pub.publish(line_list);

	   // ros::sleep();

	    f += 0.04;
	  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "g_marker_line_list");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 50, laser_callback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

}


