/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_ros_driver.h (header)
*********************************************************************************/
        
#ifndef I2C_SENSORS_ROS_DRIVER_H
#define I2C_SENSORS_ROS_DRIVER_H

#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>

#include <ros/ros.h>
#include <i2c_sensors/i2c_sensors_driver.h>
#include <i2c_sensors/AccGyroSensorMsg.h>
#include <i2c_sensors/GridEyeSensorMsg.h>

// Posix time ...
// --------------
// #include <boost/date_time/posix_time/posix_time.hpp>

class i2c_sensors_ros_driver: public i2c_sensors_driver{
    
public:
    i2c_sensors_ros_driver(const ros::NodeHandle &nh,std::string file_name_,std::string i2c_device_);
    void publishSensorsInfo();
    void publishAccGyroInfo(const ros::Time &now);
    void publishGridEyeInfo(const ros::Time &now);
protected:
    ros::NodeHandle pnh_;
    ros::Publisher acc_gyro_sensor_pub_;
    ros::Publisher eye_grid_sensor_pub_; 
    
    i2c_sensors::AccGyroSensorMsg acc_gyro_info_;
    i2c_sensors::GridEyeSensorMsg grid_eye_info_;
private:    
    
};
#endif