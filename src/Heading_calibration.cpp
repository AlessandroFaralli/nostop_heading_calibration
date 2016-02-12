#include <ros/ros.h>
#include "Heading_calibration.h"
#include "time.h"



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
// 	short int ax,ay,az,wx,wy,wz;
// 	ax=wiringPiI2CReadReg8(m_reg_address,H_BYTE_X_ACC_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_X_ACC_ADDRESS);
//         ay=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Y_ACC_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Y_ACC_ADDRESS);
//         az=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Z_ACC_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Z_ACC_ADDRESS);
//         wx=wiringPiI2CReadReg8(m_reg_address,H_BYTE_X_GYRO_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_X_GYRO_ADDRESS);
//         wy=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Y_GYRO_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Y_GYRO_ADDRESS);
//         wz=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Z_GYRO_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Z_GYRO_ADDRESS);
// 
// 	
// 	// COMPOSE IMU MSG
// 	m_imu.linear_acceleration.x = ax;
// 	m_imu.linear_acceleration.y = ay;
// 	m_imu.linear_acceleration.z = az;
// 	m_imu.angular_velocity.x = wx;
// 	m_imu.angular_velocity.y = wy;
// 	m_imu.angular_velocity.z = wz;
// 	m_imu.header.frame_id = m_robot_name+"/odom";
// 	
// 	// TODO orientation
// 	float yaw = wz*dt;
// 	geometry_msgs::Quaternion l_imu_quaternion = tf::createQuaternionMsgFromYaw(yaw);
// 	m_imu.orientation.x = l_imu_quaternion.x;
// 	m_imu.orientation.y = l_imu_quaternion.y;
// 	m_imu.orientation.z = l_imu_quaternion.z;
// 	m_imu.orientation.w = l_imu_quaternion.w;
// 
// 	m_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
}
