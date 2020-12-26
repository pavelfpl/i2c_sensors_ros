/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_ros_driver.cpp (c++)
*********************************************************************************/

#include <i2c_sensors/i2c_sensors_ros_driver.h>

// toString helper function / template ...
// ---------------------------------------
template <typename T> std::string toString(const T& t){
      
    std::ostringstream os;
    os << t;
    return os.str();
}

// i2c_sensors_ros_driver - constructor ...
// ----------------------------------------
i2c_sensors_ros_driver::i2c_sensors_ros_driver(const ros::NodeHandle &nh,std::string file_name_,std::string i2c_device_): i2c_sensors_driver(file_name_, i2c_device_){

pnh_ = nh;
    
acc_gyro_sensor_pub_= pnh_.advertise<i2c_sensors::AccGyroSensorMsg>(std::string("acc_gyro_sensor"), 1);
eye_grid_sensor_pub_= pnh_.advertise<i2c_sensors::GridEyeSensorMsg>(std::string("grid_eye_sensor"), 1);   

}

// publishSensorsInfo - public function ...
// ----------------------------------------
void i2c_sensors_ros_driver::publishSensorsInfo(){
  ros::Time now = ros::Time::now();
  publishAccGyroInfo(now);
  publishGridEyeInfo(now);
}

// publishAccGyroInfo - public function ...
// ----------------------------------------
void i2c_sensors_ros_driver::publishAccGyroInfo(const ros::Time &now){

    if(isAccGyroSensorReady() && acc.ok && gyro.ok){
       acc_gyro_info_.header.stamp = now; 
       acc_gyro_info_.dataAccX.resize(CONST_ACC_GYRO_INT_SIZE);  
       acc_gyro_info_.dataAccY.resize(CONST_ACC_GYRO_INT_SIZE); 
       acc_gyro_info_.dataAccZ.resize(CONST_ACC_GYRO_INT_SIZE);  
       acc_gyro_info_.dataGyroX.resize(CONST_ACC_GYRO_INT_SIZE);  
       acc_gyro_info_.dataGyroY.resize(CONST_ACC_GYRO_INT_SIZE); 
       acc_gyro_info_.dataGyroZ.resize(CONST_ACC_GYRO_INT_SIZE);
       
       std::copy(acc.x_data,acc.x_data + CONST_ACC_GYRO_INT_SIZE,acc_gyro_info_.dataAccX.begin());
       std::copy(acc.y_data,acc.y_data + CONST_ACC_GYRO_INT_SIZE,acc_gyro_info_.dataAccY.begin());
       std::copy(acc.z_data,acc.z_data + CONST_ACC_GYRO_INT_SIZE,acc_gyro_info_.dataAccZ.begin());
       
       std::copy(gyro.x_data,gyro.x_data + CONST_ACC_GYRO_INT_SIZE,acc_gyro_info_.dataGyroX.begin());
       std::copy(gyro.y_data,gyro.y_data + CONST_ACC_GYRO_INT_SIZE,acc_gyro_info_.dataGyroY.begin());
       std::copy(gyro.z_data,gyro.z_data + CONST_ACC_GYRO_INT_SIZE,acc_gyro_info_.dataGyroZ.begin());
       
       acc_gyro_sensor_pub_.publish(acc_gyro_info_);
       
       // Only for debug - disable in production ...
       // ------------------------------------------
       // std::cout << "Exporting AccGyroInfo data - OK..."<< std::endl;
       
    }
}

// publishGridEyeInfo - public function ...
// ----------------------------------------
void i2c_sensors_ros_driver::publishGridEyeInfo(const ros::Time &now){
    
    if(isGridEyeSensorReady() && gridEye.ok){
       grid_eye_info_.header.stamp = now;  
       grid_eye_info_.data.resize(CONST_GRID_EYE_DOUBLE_SIZE);
    
       grid_eye_info_.thermistor = gridEye.thermistor;
       std::copy(gridEye.data,gridEye.data + CONST_GRID_EYE_DOUBLE_SIZE,grid_eye_info_.data.begin());
    
       eye_grid_sensor_pub_.publish(grid_eye_info_);
       
       // Only for debug - disable in production ...
       // ------------------------------------------
       // std::cout << "Exporting GridEyeInfo data - OK..."<< std::endl;
    }
}
