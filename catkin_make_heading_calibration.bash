# bash  file to mv into main folder (/home/pi)
# use ./catkin_make.bash if there are nostop_imu_reading pkg changes



#!/bin/bash

cd
cd ros_catkin_ws
catkin_make
cd devel/lib/nostop_imu_reading  # cd to the directory with your node
sudo chown root:root imu_reading_node  # change ownship to root
sudo chmod a+rx imu_reading_node       # set as executable by all
sudo chmod u+s imu_reading_node        # set the setuid bit

cd
cd ros_catkin_ws/devel/lib/imu_calibration  # cd to the directory with your node
sudo chown root:root imu_calibration_node  # change ownship to root
sudo chmod a+rx imu_calibration_node       # set as executable by all
sudo chmod u+s imu_calibration_node        # set the setuid bit

cd 
cd ros_catkin_ws/devel/lib/nostop_heading_calibration # cd to the directory with your node
sudo chown root:root heading_calibration_node  # change ownship to root
sudo chmod a+rx heading_calibration_node       # set as executable by all
sudo chmod u+s heading_calibration_node        # set the setuid bit

