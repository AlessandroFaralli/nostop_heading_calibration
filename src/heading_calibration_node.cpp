#include "ros/ros.h"
#include "Heading_calibration.h"
#include "time.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "heading_calibration");
  
  std::string l_robot_name,l_port;
  ros::NodeHandle l_node("~");
  
  //l_node.getParam("robot_name", l_robot_name);
   l_robot_name = "thief";
float a=1.5;
int b= 10;
  
  Heading_calibration heading_calibration(l_robot_name,l_robot_name,a,b);
  
  heading_calibration.magnetometer_calibration();
  
//   while (ros::ok())
//   {
//     ros::spinOnce();
// 
// 
//     //heading_calibration.imu_reading_publish();
// 
//   }
  return 0;
}