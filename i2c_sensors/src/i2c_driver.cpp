/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_driver.cpp (c++, main)
*********************************************************************************/

/*
Notes: 
rostopic echo /i2c_sensors/acc_gyro_sensor 
rostopic echo /i2c_sensors/grid_eye_sensor 
*/

#include <i2c_sensors/i2c_sensors_ros_cluster.h>

#include <string>
#include <vector>
#include <iostream>

#define CONST_ROS_FRAMES 1   

int main(int argc, char ** argv){

  ros::init(argc, argv, "i2c_sensors");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
    
  bool status = false;
  
  std::vector<std::string> file_names;
  pnh.getParam("i2c_driver_param_file_path", file_names);
  
  ros::Rate loop(CONST_ROS_FRAMES);
  
  ROS_INFO_STREAM("Loading I2C configuration file: " << file_names[0]);

  i2c_sensors_ros_cluster xd(file_names[0]);
  xd.clusterInit();
  // Get cluster status ...
  // ----------------------
  status = xd.clusterStatus();
  
  if(status == false){       // No i2c drivers properly initialized ...
    return 0;
  }
  
  // Event loop ...
  // --------------
  while (ros::ok()){
    ros::spinOnce();
    xd.clusterAcquireAccGyro();
    xd.clusterAcquireGridEye();
    xd.clusterPublishSensorsInfo();
    loop.sleep();
  }
  xd.clusterFinish();
  return 1;
}
