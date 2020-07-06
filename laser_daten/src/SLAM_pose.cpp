
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
//dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <laser_daten/SLAMParamConfig.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <sstream>

//publisher
ros::Publisher pose_pub;


std::vector<double> Start_end_pose;
float f = 0.0;
//dynamic reconfigure
double Euler_uncertainty;
double max_pose_distance;
double max_pose_distance_s_e;
double Euler_angle_max;

std::string fixed_frame;

struct EulerAngles{
	double roll, pitch, yaw;
};
struct quaternion{
	double x, y, z, w;
};
struct positions{
	std::vector<double> x_position, y_position, x_rotation, y_rotation, z_rotation, w_rotation, abstand_pose, Euler_angle, x_pos_end, y_pos_end, Euler_ang_end;
	double first_point_under = 0;
	double Euler_end = 0;

};
struct help_structure{
	int counter = 0;
};
struct start_end_position{
	std::vector<double> x_position, y_position, angle;
};
/*struct dynamic_reconfigure{
	double Euler_uncertainty;
};*/

positions position;
help_structure help_structure;
start_end_position start_end_position;

quaternion convert_Euler_to_Quant(double Euler_angle){
	quaternion q;
	double cy = cos(Euler_angle*0.5);
	double sy = sin(Euler_angle*0.5);
	double cp = cos(0);
	double sp = sin(0);
	double cr = cos(0);
	double sr = sin(0);

	q.w = cr*cp*cy + sr*sp*sy;
	q.z = cr*cp*sy - sr*sp*cy;
	q.x = sr*cp*cy - cr*sp*sy;
	q.y = cr*sp*cy - sr*sp*sy;

	return q;
}

double convert_Quant_to_Euler(double x, double y, double z, double w){
	//there is only a z-axis rotation
	EulerAngles angles;
	double siny_cosp = 2*(z*w+x*y);
	double cosy_cosp = 1-2*(y*y+z*z);
	angles.yaw = std::atan2(siny_cosp, cosy_cosp);
	return angles.yaw;
}

void Pose_difference(std::vector<double> x, std::vector<double> y, std::vector<double> angle, int size_array){
	//average
	quaternion quaternion;
	//geometry_msgs

	geometry_msgs::PoseStamped Pose_dif;
	Pose_dif.header.frame_id = fixed_frame;
	Pose_dif.header.stamp = ros::Time::now();

	double avr_x = 0;
	double avr_y = 0;
	double avr_a = 0;
	double angle_dif = 0;
	for (int i = 0; i<size_array; i++){
		avr_x += x[i];
		avr_y += y[i];
		avr_a += angle[i];
	}
	start_end_position.x_position.push_back(avr_x /=(size_array));
	start_end_position.y_position.push_back(avr_y /=(size_array));
	start_end_position.angle.push_back(avr_a /=(size_array));

	if(start_end_position.x_position.size()==2){
		angle_dif = start_end_position.angle[1]-start_end_position.angle[0];
		Pose_dif.pose.position.x = start_end_position.x_position[1]-start_end_position.x_position[0];
		Pose_dif.pose.position.y = start_end_position.y_position[1]-start_end_position.y_position[0];
		quaternion = convert_Euler_to_Quant(angle_dif);
		Pose_dif.pose.orientation.x = quaternion.x;
		Pose_dif.pose.orientation.y = quaternion.y;
		Pose_dif.pose.orientation.z = quaternion.z;
		Pose_dif.pose.orientation.w = quaternion.w;
		pose_pub.publish(Pose_dif);
		avr_x = avr_y = avr_a = 0;
	}

}


void SLAM_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	int position_size;
	double distance_s_e =0;
	int current_state;
	ROS_INFO_STREAM("Received pose: " << msg);

	position.x_position.push_back(msg->pose.position.x);
	position.y_position.push_back(msg->pose.position.y);
	position.x_rotation.push_back(msg->pose.orientation.x);
	position.y_rotation.push_back(msg->pose.orientation.y);
	position.z_rotation.push_back(msg->pose.orientation.z);
	position.w_rotation.push_back(msg->pose.orientation.w);

	int array_size = position.x_position.size();

	position.Euler_angle.push_back(convert_Quant_to_Euler(position.x_rotation[position.x_position.size()-1], position.y_rotation[position.y_position.size()-1], position.z_rotation[position.z_rotation.size()-1], position.w_rotation[position.w_rotation.size()-1]));


	position.abstand_pose.push_back(sqrt(pow((position.x_position[position.x_position.size()-1]-position.x_position[position.x_position.size()-2]),2)+pow((position.y_position[position.y_position.size()-1]-position.y_position[position.y_position.size()-2]),2)));


	//State machine
	switch(current_state)
	{
	case 0: //Set the start pose
		distance_s_e = position.abstand_pose[position.abstand_pose.size()-1] - position.abstand_pose[0];
		if(position.abstand_pose[position.abstand_pose.size()-1]>=max_pose_distance){ //Abstand der aktuellen zu lestzen Punkt nicht Überschreiten
			current_state = 1;
			Pose_difference(position.x_position, position.y_position, position.Euler_angle, array_size);
		}else if(distance_s_e>max_pose_distance_s_e){//distance from the first to the last point must not be exceeded
			current_state = 1;
			Pose_difference(position.x_position, position.y_position, position.Euler_angle, array_size);
		}else if (position.Euler_angle[position.Euler_angle.size()-1]>=Euler_angle_max){
			current_state = 1;
			Pose_difference(position.x_position, position.y_position, position.Euler_angle, array_size);
		}else{
			current_state = 0;
		}
		break;
	case 1://
		if(position.abstand_pose[position.abstand_pose.size()-1]<max_pose_distance){ //Abstand der aktuellen zu lestzen Punkt nicht Überschreiten
			current_state = 2;
			if(help_structure.counter==0){
				position.first_point_under = position.abstand_pose[position.abstand_pose.size()-1];
				position.Euler_end = position.Euler_angle[position.Euler_angle.size()-1];
			}
		}
		break;
	case 2:
		distance_s_e = position.abstand_pose[position.abstand_pose.size()-1] - position.abstand_pose[position.first_point_under];
		if(distance_s_e<max_pose_distance_s_e){//distance from first to last point
			if(position.Euler_angle[position.Euler_angle.size()-1]>=(position.Euler_end+Euler_uncertainty)&&position.Euler_angle[position.Euler_angle.size()-1]>=(position.Euler_end-Euler_uncertainty)){	//neuer winkel darf bestimmten wert im bezug zum winkel davor nicht überschreiten
				position.x_pos_end[help_structure.counter] = position.x_position[position.x_position.size()-1];
				position.y_pos_end[help_structure.counter] = position.y_position[position.y_position.size()-1];
				position.Euler_ang_end[help_structure.counter] = position.Euler_ang_end[position.Euler_ang_end.size()-1];

				help_structure.counter +=1;
				if(help_structure.counter>=40){
					current_state = 3;
				}
			}
			else {
				help_structure.counter = 0;
				position.x_pos_end[help_structure.counter] = {0.00};
				position.y_pos_end[help_structure.counter] = {0.00};
				position.Euler_ang_end[help_structure.counter] = {0.00};
				current_state = 1;
			}
		}else {
			help_structure.counter = 0;
			position.x_pos_end[help_structure.counter] = {0.00};
			position.y_pos_end[help_structure.counter] = {0.00};
			position.Euler_ang_end[help_structure.counter] = {0.00};
			current_state = 1;
		}
		break;
	case 3:
		ROS_INFO("Roboter ist stehen geblieben");
		int size_array_two = help_structure.counter;
		Pose_difference(position.x_position, position.y_position, position.Euler_angle, size_array_two);
		//funktion punkte auswertung und Publishen der distanz
		break;


	}

	ROS_INFO("Schleife fertig");

}
void callback_dyn_rec(laser_daten::SLAMParamConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request");

	Euler_uncertainty = config.max_pose_distance;
	max_pose_distance = config.max_pose_distance;
	max_pose_distance_s_e = config.max_pose_distance_s_e;
	//Euler_angle_max = config.Euler_angle_max;
	//fixed_frame = config.fixed_frame.c_str();


}
int main (int argc, char **argv)
{
	ros::init(argc, argv, "SLAM_pose");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/slam_out_pose", 50, SLAM_callback);
	//publisher
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("Pose_differce", 10);
	//dynamic reconfigure
	dynamic_reconfigure::Server<laser_daten::SLAMParamConfig> server;
	dynamic_reconfigure::Server<laser_daten::SLAMParamConfig>::CallbackType f;

	f = boost::bind(&callback_dyn_rec, _1, _2);
	server.setCallback(f);
	ros::Rate loop_rate(10);

	ros::spin();
	return 0;
}
