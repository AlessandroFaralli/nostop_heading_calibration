////////////////////////////////////////////////////////////
//	Heading_calibration.h
//	Created on:	12-feb-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef HEADING_CALIBRATION_H
#define HEADING_CALIBRATION_H
#pragma once

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt8MultiArray.h"
#include <Threads.h>

//// By http://www.botched.co.uk/pic-tutorials/mpu6050-setup-data-aquisition/
#define IMU_ADDRESS 0x68 // imu address --> sudo i2cdetect -y 1
#include <wiringPi.h>
#include <wiringPiI2C.h>
#define H_BYTE_X_MAG_ADDRESS 03H
#define L_BYTE_X_MAG_ADDRESS 04H
#define H_BYTE_Y_MAG_ADDRESS 05H
#define L_BYTE_Y_MAG_ADDRESS 06H
#define H_BYTE_Z_MAG_ADDRESS 07H
#define L_BYTE_Z_MAG_ADDRESS 08H

// #define H_BYTE_X_GYRO_ADDRESS 0x43
// #define L_BYTE_X_GYRO_ADDRESS 0x44
// #define H_BYTE_Y_GYRO_ADDRESS 0x45
// #define L_BYTE_Y_GYRO_ADDRESS 0x46
// #define H_BYTE_Z_GYRO_ADDRESS 0x47
// #define L_BYTE_Z_GYRO_ADDRESS 0x48
#define message_size 10 
namespace Robotics 
{	
	namespace GameTheory
	{
		class Heading_calibration
		{
		public:

		        ros::NodeHandle m_imu_calibration_node;
			std::string m_robot_name;
			std::string m_topic_name;
			// IMU
			char m_address;
			int m_reg_address;
			int m_value;
			int m_ret;
			int m_flag;
			ros::Publisher m_pub_command;
			sensor_msgs::Imu m_imu;
			geometry_msgs::Twist m_command;
			
			ros::Timer timer;

		public:
			Heading_calibration(std::string& robot_name,std::string& topic_name,float& ang_vel_z,int& duration); 
			void magnetometer_data_saving();
			void magnetometer_calibration();
			//void Callback(const ros::TimerEvent& event);
			~Heading_calibration();
		};

	}
}


#endif // HEADING_CALIBRATION_H