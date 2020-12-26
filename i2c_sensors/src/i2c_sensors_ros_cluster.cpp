/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_ros_cluster.cpp (c++)
*********************************************************************************/

#include <i2c_sensors/i2c_sensors_ros_cluster.h>

// toString helper function / template ...
// ---------------------------------------
template <typename T> std::string toString(const T& t){
      
    std::ostringstream os;
    os << t;
    return os.str();
}

// i2c_sensors_ros_cluster - constructor ...
// -----------------------------------------
i2c_sensors_ros_cluster::i2c_sensors_ros_cluster(std::string file_name_){
    
    accGyroSensorStat = false;
    gridEyeSensorStat = false;
    
    ros::NodeHandle nh(std::string("/i2c_sensors"));
    
    // Create i2c_sensors_ros_driver instance (and init with NodeHandle,fileName,i2cDev) ...
    // -------------------------------------------------------------------------------------
    i2c_dev = new i2c_sensors_ros_driver(nh,file_name_,std::string("/dev/i2c-1"));
}

// i2c_sensors_ros_cluster - destructor ...
// ----------------------------------------
i2c_sensors_ros_cluster::~i2c_sensors_ros_cluster(){
    
    delete i2c_dev;
    i2c_dev = 0;
}

// clusterInit - public function ...
// ---------------------------------
void i2c_sensors_ros_cluster::clusterInit(){

    ROS_INFO_STREAM("Initializing I2C sensors - MPU-6000(ACC+GYRO) and Panasonic GridEye ..." );
    
    if (i2c_dev!=0){
        // Init acc + gyro sensors ...
        // ---------------------------
        accGyroSensorStat = i2c_dev->initAccGyroSensor();
        // Init GridEye sensor ...
        // -----------------------
        gridEyeSensorStat = i2c_dev->initGridEyeSensor();
    }
    
    if(accGyroSensorStat==1) 
       ROS_INFO_STREAM("MPU-6000(ACC+GYRO) driver properly initialized ..." );
    else
       ROS_INFO_STREAM("MPU-6000(ACC+GYRO) driver initialization error ..." ); 
    
    if(gridEyeSensorStat==1) 
       ROS_INFO_STREAM("Panasonic GridEye driver properly initialized ..." );
    else
       ROS_INFO_STREAM("Panasonic GridEye driver initialization error ..." ); 
}

// clusterStatus - public function ...
// -----------------------------------
bool i2c_sensors_ros_cluster::clusterStatus(){
    
    if(accGyroSensorStat==1 || gridEyeSensorStat==1){
       ROS_INFO_STREAM("At least one I2C sensor properly initialized ..." ); 
       return true; 
    }else{
       ROS_INFO_STREAM("No I2C sensors were initialized ..." );
       return false;
    }
}

// clusterFinish - public function ...
// -----------------------------------
void i2c_sensors_ros_cluster::clusterFinish(){
    
    i2c_dev->stopAcquistion();
}

// clusterAcquireAccGyro - public function ...
// -------------------------------------------
void i2c_sensors_ros_cluster::clusterAcquireAccGyro(){
  
   if(accGyroSensorStat == 1){  
      thread_0 = new boost::thread(&i2c_sensors_driver::acquireSensorAccGyroData, i2c_dev);
      thread_0->join();
      delete thread_0;
   }else{
      // Init acc + gyro sensors - option / reinit ...
      // ---------------------------------------------
      // accGyroSensorStat = i2c_dev->initAccGyroSensor();   
   }
}

// clusterAcquireAccGyro - public function ...
// -------------------------------------------
void i2c_sensors_ros_cluster::clusterAcquireGridEye(){
  
   if(gridEyeSensorStat==1){  
      thread_1 = new boost::thread(&i2c_sensors_driver::acquireSensorGridEyeData, i2c_dev);
      thread_1->join();
      delete thread_1;
   }else{
     // Init GridEye sensor - option / reinit ...
     // -----------------------------------------
     // gridEyeSensorStat = i2c_dev->initGridEyeSensor();  
   }  
}

// clusterPublishAccGyroInfo - public function ...
// -----------------------------------------------
void i2c_sensors_ros_cluster::clusterPublishAccGyroInfo(){
    i2c_dev->publishAccGyroInfo(ros::Time::now());
}

// clusterPublishGridEyeInfo - public function ...
// -----------------------------------------------
void i2c_sensors_ros_cluster::clusterPublishGridEyeInfo(){
    i2c_dev->publishGridEyeInfo(ros::Time::now());
}

// clusterPublishSensorsInfo - public function ...
// -----------------------------------------------
void i2c_sensors_ros_cluster::clusterPublishSensorsInfo(){
    i2c_dev->publishSensorsInfo();
}
