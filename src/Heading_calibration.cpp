#include <ros/ros.h>
#include "Heading_calibration.h"
#include "time.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


void callback(const ros::TimerEvent& event)
{
  //NOTHING TODO
}


Heading_calibration::Heading_calibration(std::string& robot_name,float& ang_vel_z,int& duration):
m_robot_name(robot_name)
, m_address(IMU_ADDRESS)
, m_flag(0)
{
	ROS_INFO("HEADING_CALIBRATION  %s: ON", robot_name.c_str());
	// Publish Sensor Information:
	m_pub_calibration = m_imu_calibration_node.advertise<std_msgs::Float64MultiArray>("/"+m_robot_name+"/magnetometer_SF_B", 5);
	m_pub_command = m_imu_calibration_node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);//TODO
	m_command.angular.z = ang_vel_z;
	timer = m_imu_calibration_node.createTimer(ros::Duration(duration), callback);
 	wiringPiSetup();
	try{
	  m_reg_address = wiringPiI2CSetup(m_address);
	} catch(std::exception &e){
	  std::cerr<<"Error open IMU"<< e.what() << std::endl;
	}
// 	//disable sleep mode
         wiringPiI2CWriteReg8(m_reg_address,0x6B,00);
}



/////////////////////////////////////////////
Heading_calibration::~Heading_calibration()
{}



void Heading_calibration::magnetometer_calibration()
{

    while (ros::ok())
    {
       std_msgs::Float64MultiArray SB;
       switch(m_flag)
       {
	 case 0:
	 {
	    m_pub_command.publish<geometry_msgs::Twist>(m_command);
	    magnetometer_data_saving();
	    if(timer.hasPending())
	      m_flag = 1;
	    break;
	 }
	    
	 case 1:
	 {	   
	   SB = SB_computing();
	   m_flag=2;
	   break;
	 }
	 
	 case 2:
	 {
	   m_pub_calibration.publish<std_msgs::Float64MultiArray>(SB);
	   m_flag=3;
	   break;
	 }
	 
	 case 3:
	 {
	   ros::shutdown();
	   break;
	 }
	 
	 default:
	   
	{	
	  ROS_ERROR("Some errors. :-(");
	  break;
	}
	
      }
}
}

void Heading_calibration::magnetometer_data_saving()
{
 	short int mx,my,mz;
 	mx=wiringPiI2CReadReg8(m_reg_address,H_BYTE_X_MAG_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_X_MAG_ADDRESS);
        my=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Y_MAG_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Y_MAG_ADDRESS);
        mz=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Z_MAG_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Z_MAG_ADDRESS);
	
	int lx,ly,lz;

	lx=mx;
	ly=my;  
	lz=mz;
	
	compare_data(lx,m_x_max,m_x_min);
	compare_data(ly,m_y_max,m_y_min);
	compare_data(lz,m_z_max,m_z_min);

}


void Heading_calibration::compare_data(int& l,int& m_max,int& m_min)
{
   	if(l>m_max)
	  m_max = l;
	else if (l<m_min)
	  m_min = l;
}

std_msgs::Float64MultiArray Heading_calibration::SB_computing()
{
 double S_x,S_y,B_x,B_y;
 std_msgs::Float64MultiArray SB;
  
 S_x = max(1, (m_y_max-m_y_min)/(m_x_max-m_x_min));
 SB.data.push_back(S_x);
 S_y = max(1, (m_x_max-m_x_min)/(m_y_max-m_y_min));
 SB.data.push_back(S_y);
 B_x= ((m_x_max-m_x_min)/2-m_x_max)*S_x;
 SB.data.push_back(B_x);
 B_y= ((m_y_max-m_y_min)/2-m_x_max)*S_y;
 SB.data.push_back(B_y);

 return SB;
 
}