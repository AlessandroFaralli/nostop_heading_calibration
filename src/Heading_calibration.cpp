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


Heading_calibration::Heading_calibration(std::string& robot_name,std::string& topic_name,float& ang_vel_z,int& duration):
m_robot_name(robot_name)
, m_topic_name(topic_name)
, m_address(IMU_ADDRESS)
, m_flag(0)
{
	ROS_INFO("HEADING_CALIBRATION  %s: ON", robot_name.c_str());
	// Publish Sensor Information:
	//m_reader_imu_pub = m_imu_reader_node.advertise<sensor_msgs::Imu>("/"+m_robot_name+"/imu_data", 5);
	m_pub_command = m_imu_calibration_node.advertise<geometry_msgs::Twist>("/"+m_topic_name, 5);
	m_command.angular.z = ang_vel_z;
	timer = m_imu_calibration_node.createTimer(ros::Duration(duration), callback);
// 	wiringPiSetup();
	try{
	    //m_reg_address = wiringPiI2CSetup(m_address);
	} catch(std::exception &e){
	  std::cerr<<"Error open IMU"<< e.what() << std::endl;
	}
// 	//disable sleep mode
//         wiringPiI2CWriteReg8(m_reg_address,0x6B,00);
}



/////////////////////////////////////////////
Heading_calibration::~Heading_calibration()
{}



void Heading_calibration::magnetometer_calibration()
{

    while (ros::ok())
    {
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
	   //SB_computing();
	   //calcolo di S e B
	   m_flag=2;
	   break;
	 }
	 
	 case 2:
	 {
	   //pubblicare su topic 
	   //richiama timer e dopo 1 sec
	   m_flag=3;
	   break;
	 }
	 
	 case 3:
	 {
	   //chiudi nodo ros shutdown()
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
	
	if(lx>m_x_max)
	  m_x_max = lx;
	else if (lx<m_x_min)
	  m_x_min = lx;
	if(ly>m_x_max)
	  m_x_max = lx;
	else if (lx<m_x_min)
	  m_x_min = lx;
	if(l>m_x_max)
	  m_x_max = lx;
	else if (lx<m_x_min)
	  m_x_min = lx;
	
	
// 	if (lx > m_x_max.end())
// 	  m_x_max.push_back(lx);
// 	else if (lx < m_x_min.end())
// 	  m_x_min.push_back(lx);
// 	
// 	if (ly > m_y_max.end())
// 	  m_y_max.push_back(ly);
// 	else if (ly < m_y_min.end())
// 	  m_y_min.push_back(ly);
// 	
// 	if (lz > m_z_max.end())
// 	  m_z_max.push_back(lz);
// 	else if (lz < m_z_min.end())
// 	  m_z_min.push_back(lz);
// 	  
// 	int l_x,l_y,l_z;
// 	l_x=mx;
// 	l_y=my;
// 	l_z=mz;
	
	



}

//vector< double > Heading_calibration::SB_computing()
//{
 // std::vector<double>::iterator x_max = std::max_element<double>(m_vector_x.begin(),m_vector_x.end());
//   double x_min = std::min_element<double>(m_vector_x.begin(),m_vector_x.end());
//   double y_max = std::max_element<double>(m_vector_y.begin(),m_vector_y.end());
//   double y_min = std::min_element<double>(m_vector_y.begin(),m_vector_y.end());
//   double z_max = std::max_element<double>(m_vector_z.begin(),m_vector_z.end());
//   double z_min = std::min_element<double>(m_vector_z.begin(),m_vector_z.end());
  
 // S_x = max(1, (y_max-y_min)/(x_max-x_min));
 
  

//}



