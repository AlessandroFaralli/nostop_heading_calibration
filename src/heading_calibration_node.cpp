#include "ros/ros.h"
#include "Heading_calibration.h"
#include "time.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "heading_calibration");
	
	std::string l_robot_name;
	float l_ang_vel_z;
	int l_duration;
	ros::NodeHandle l_node("~");
	
	l_node.getParam("robot_name", l_robot_name);
	l_node.getParam("ang_vel_z", l_ang_vel_z);
	l_node.getParam("duration", l_duration);
	
	
	Heading_calibration heading_calibration(l_robot_name,l_ang_vel_z,l_duration);
	
	heading_calibration.magnetometer_calibration();
	return 0;
}