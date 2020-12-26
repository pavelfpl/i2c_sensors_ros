/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_driver.h
*********************************************************************************/

#ifndef I2C_SENSORS_DRIVER_H
#define I2C_SENSORS_DRIVER_H

#define CONST_GRID_EYE_DOUBLE_SIZE 64
#define CONST_ACC_GYRO_INT_SIZE 10

#include <ros/ros.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

// Class AccGyroCoordinates / helper class ...
// -------------------------------------------
class AccGyroCoordinates{
public:
    bool ok;
    int16_t x_data[CONST_ACC_GYRO_INT_SIZE];
    int16_t y_data[CONST_ACC_GYRO_INT_SIZE];
    int16_t z_data[CONST_ACC_GYRO_INT_SIZE];
};

// Class GridEyeData / helper class ...
// ------------------------------------
class GridEyeData{
public:
    bool ok;
    double thermistor;
    double data[CONST_GRID_EYE_DOUBLE_SIZE];
};

// Class i2c_sensors_driver / main class ...
// -----------------------------------------
class i2c_sensors_driver{
    
public:
    i2c_sensors_driver(std::string file_name_,std::string i2c_device_);
    int32_t initGridEyeSensor();
    int32_t initAccGyroSensor();
    void acquireSensorAccGyroData();
    void acquireSensorGridEyeData();
    void stopAcquistion(){stopAcquistionI2C = true;}
    bool isGridEyeSensorReady() const {return gridEyeStatusRdy;}
    bool isAccGyroSensorReady() const {return accGyroStatusRdy;}
    GridEyeData getGridEyeSensorData() const {return gridEye;}
    AccGyroCoordinates getAccSensorData() const {return acc;}
    AccGyroCoordinates getGyroSensorData() const {return gyro;}
protected:
    bool accGyroStatusRdy;
    bool gridEyeStatusRdy;
    bool stopAcquistionI2C;
    std::string file_name;
    std::string i2c_device;
    AccGyroCoordinates acc;
    AccGyroCoordinates gyro;
    GridEyeData gridEye;
private:    
    int32_t readParamsFromFile(std::string file_name_);
    int32_t i2c_write_buf(const char *dev, uint8_t addr, uint8_t nData, uint8_t *data);
    int16_t read_output_acc_gyro(uint8_t addr);
    int32_t i2c_read_buf(const char *dev, uint8_t addr, uint8_t subaddr, uint8_t nData, uint8_t *data);
};

#endif  // I2C_SENSORS_DRIVER_H
