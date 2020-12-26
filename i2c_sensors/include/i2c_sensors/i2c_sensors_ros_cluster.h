/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_ros_cluster.h (header)
*********************************************************************************/
        
#ifndef I2C_SENSORS_ROS_CLUSTER_H
#define I2C_SENSORS_ROS_CLUSTER_H

#include <i2c_sensors/i2c_sensors_ros_driver.h>
#include <boost/thread.hpp>
#include <stdint.h>
#include <string>
#include <vector>

class i2c_sensors_ros_cluster{
    
public:
    i2c_sensors_ros_cluster(std::string file_name_);
    ~i2c_sensors_ros_cluster();
    void clusterInit();
    bool clusterStatus();
    void clusterFinish();
    void clusterAcquireAccGyro();
    void clusterAcquireGridEye();
    void clusterPublishSensorsInfo();
    void clusterPublishGridEyeInfo();
    void clusterPublishAccGyroInfo();
private:    
    int32_t accGyroSensorStat;
    int32_t gridEyeSensorStat;
    i2c_sensors_ros_driver *i2c_dev;
    boost::thread *thread_0;
    boost::thread *thread_1;
};

#endif
