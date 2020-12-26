/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: sensors_consumer.cpp (c++) / main function / consumer
*********************************************************************************/

// System includes ...
// -------------------
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
// Boost & ROS & OpenCV includes ...
// ---------------------------------
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
// i2c_sensors mesages ...
// -----------------------
#include <i2c_sensors/AccGyroSensorMsg.h>
#include <i2c_sensors/GridEyeSensorMsg.h>
// ximea_sensors messages ...
// --------------------------
#include <ximea_camera/XimeaCamSensorMsg.h>

// Define cycle count variables [global] ...
// -----------------------------------------
unsigned int accGyroCount;
unsigned int gridEyeCount;
unsigned int ximeaCamCount;
// Define file variables [global] ...
// ----------------------------------
std::string accGyroFile_;
std::string gridEyeFile_;
std::string ximeCamFile_;

// toString helper function / template ...
// ---------------------------------------
template <typename T> std::string toString(const T& t){
      
    std::ostringstream os;
    os << t;
    return os.str();
}

// accGyroCallback function ...
// ----------------------------
void accGyroCallback(const i2c_sensors::AccGyroSensorMsg& msg){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = msg.header.stamp.toBoost(); 
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream accGyroFile(accGyroFile_.c_str(),std::ios::out | std::ios::app);
  
  if(accGyroFile.is_open()){
     accGyroFile << "-----------------"<< std::endl;    
     accGyroFile << "[Cycle: "<< accGyroCount <<"]"<< " timestamp: "<< iso_time_string << std::endl; accGyroCount++;    
     
     // 1] Gyro ...
     // -----------   
     accGyroFile << "dataGyroX: ";   
     for(int i=0;i<msg.dataGyroX.size();i++){
         if((msg.dataGyroX.size()-1) == i)  accGyroFile << msg.dataGyroX[i]; else accGyroFile << msg.dataGyroX[i]<< ",";              
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataGyroY: ";   
     for(int i=0;i<msg.dataGyroY.size();i++){
         if((msg.dataGyroY.size()-1) == i) accGyroFile << msg.dataGyroY[i]; else accGyroFile << msg.dataGyroY[i]<< ","; 
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataGyroZ: ";   
     for(int i=0;i<msg.dataGyroZ.size();i++){
         if((msg.dataGyroZ.size()-1) == i) accGyroFile << msg.dataGyroZ[i]; else accGyroFile << msg.dataGyroZ[i]<< ",";
     }
     accGyroFile << std::endl;
     
     // 2] Acc ...
     // ----------
     accGyroFile << "dataAccX: ";   
     for(int i=0;i<msg.dataAccX.size();i++){
         if((msg.dataAccX.size()-1) == i) accGyroFile << msg.dataAccX[i]; else accGyroFile << msg.dataAccX[i]<< ","; 
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataAccY: ";   
     for(int i=0;i<msg.dataAccY.size();i++){
          if((msg.dataAccY.size()-1) == i) accGyroFile << msg.dataAccY[i]; else accGyroFile << msg.dataAccY[i]<< ","; 
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataAccZ: ";   
     for(int i=0;i<msg.dataAccZ.size();i++){
          if((msg.dataAccZ.size()-1) == i) accGyroFile << msg.dataAccZ[i]; else accGyroFile << msg.dataAccZ[i]<< ","; 
     }
     
     accGyroFile << std::endl;
     
     accGyroFile.close();
  }
}

// gridEyeCallback function ...
// ----------------------------
void gridEyeCallback(const i2c_sensors::GridEyeSensorMsg& msg){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = msg.header.stamp.toBoost(); 
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream gridEyeFile(gridEyeFile_.c_str(),std::ios::out | std::ios::app);
  
  if(gridEyeFile.is_open()){
     gridEyeFile << "-----------------"<< std::endl;    
     gridEyeFile << "[Cycle: "<< gridEyeCount <<"]"<<" timestamp: "<< iso_time_string << std::endl; gridEyeCount++;    
     
     // GridEye ...
     // -----------   
     gridEyeFile << "GridEye thermistor: "<< msg.thermistor<< std::endl;   
     gridEyeFile << "GridEye data: ";
     for(int i=0;i<msg.data.size();i++){
         if((msg.data.size()-1) == i) gridEyeFile << msg.data[i]; else gridEyeFile << msg.data[i]<< ","; 
     }
     gridEyeFile << std::endl;
     gridEyeFile.close();
  }
}

// imageCallback function ...
// --------------------------
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
   
    // Only for debug ...
    // ------------------
    // ROS_INFO("XimeaCameraDriver image data");
    
    // Online view with opencv / 3
    // ---------------------------
    
    /*
    try{
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    */
    
    // Timestamp --> to String conversion ... 
    // --------------------------------------
    boost::posix_time::ptime my_posix_time = msg->header.stamp.toBoost(); 
    std::string header_ = boost::posix_time::to_iso_extended_string(my_posix_time); 
    
    cv_bridge::CvImagePtr cv_ptr;
    
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Tiff format / ~ 4.4 MB ...
    // --------------------------
    // cv::imwrite("/media/sd1/ximea_cam_store/"+toString(msg->header.stamp)+".tiff", cv_ptr->image);
    cv::imwrite("/media/sd1/ximea_cam_store/"+header_+".tiff", cv_ptr->image);
    
    // PNG format - compression 0 - 9 / option ...
    // -------------------------------------------
    /*
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    
    try {
        cv::imwrite("/media/sd1/ximea_cam_store/"+header_+".png", cv_ptr->image,compression_params);
    }catch (std::runtime_error& e) {
        ROS_ERROR("Exception converting image to PNG format: %s", e.what());
        return;
    }
    */
}

// imageCallbackInfo function ...
// ------------------------------
void imageCallbackInfo(const ximea_camera::XimeaCamSensorMsg& msg){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = msg.header.stamp.toBoost(); 
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream ximeaCamFile(ximeCamFile_.c_str(),std::ios::out | std::ios::app);
  
  if(ximeaCamFile.is_open()){
     ximeaCamFile << "-----------------"<< std::endl;    
     ximeaCamFile << "[Cycle: "<< ximeaCamCount <<"]"<<" timestamp: "<< iso_time_string << std::endl; ximeaCamCount++;    
     // ximea info ...
     // --------------   
     ximeaCamFile << "Exposure time: "<< msg.exposure_time<< std::endl;   
     ximeaCamFile << "Gain: "<< msg.gain << std::endl;   
     ximeaCamFile << "Chip temperature: "<< msg.temperature << std::endl;   
     ximeaCamFile << std::endl;
     ximeaCamFile.close();
  }
}

// main function ... 
// -----------------
int main(int argc, char **argv){

  ros::init(argc, argv, "sensors_consumer");
  ros::NodeHandle n;

  accGyroCount = 0;
  gridEyeCount = 0;
  ximeaCamCount = 0;
  
  ros::Time curr_time = ros::Time::now();
  boost::posix_time::ptime my_posix_time = curr_time.toBoost(); 
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  // Customize / option ...
  // ----------------------
  accGyroFile_="/media/sd1/i2c_sensors_store/accGyroLog_"+iso_time_string+".log";   // SD 1 ...
  gridEyeFile_="/media/sd1/i2c_sensors_store/gridEyeLog_"+iso_time_string+".log";   // SD 1 ...
  ximeCamFile_="/media/sd1/ximea_cam_store/ximeaInfoLog_"+iso_time_string+".log";   // SD 1 ...
  
  // Online view with opencv / 1 ...
  // -------------------------------
  // cv::namedWindow("view");
  // cv::startWindowThread();

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1_0 = it.subscribe("/camera0/image_raw", 1, imageCallback);
  ros::Subscriber sub_1_1 = n.subscribe("/camera0/ximea_info_settings",10, imageCallbackInfo);
  
  ros::Subscriber sub_2_0 = n.subscribe("/i2c_sensors/acc_gyro_sensor",10, accGyroCallback);
  ros::Subscriber sub_2_1 = n.subscribe("/i2c_sensors/grid_eye_sensor",10, gridEyeCallback);
  
  ros::spin();
  
  // Online view with opencv / 2
  // ---------------------------
  // cv::destroyWindow("view");
  return 0;
}