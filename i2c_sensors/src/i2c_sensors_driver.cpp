/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_driver.cpp (c++)
*********************************************************************************/
/* Pavel Fiala notes
 * -----------------
!!! NOTE: add selected user to i2c group [to run driver without root permission] !!!
e.g. sudo adduser odroid i2c
*
*/

/* Richard Linhart notes:
 * ----------------------
sudo modprobe i2c-dev
sudo modprobe i2c-tiny-usb
sudo usermod -a -G i2c riki
http://elinux.org/Interfacing_with_I2C_Devices
http://raspberrypi.stackexchange.com/questions/3627/is-there-an-i2c-library
*
*/

// GLOBAL constants ...
// --------------------
#define CONST_CYCLE_REPEAT 10000000
#define CONST_ACC_GYRO_SIZE 128
#define CONST_GRID_EYE_SIZE 128

#define STATUS_OK 1
#define STATUS_ERROR -1
#define STATUS_EXIT -2

#define I2C_OPEN_FAILED -1
#define I2C_IOCTL_FAILED -2
#define I2C_WRITE_FAILED -3
#define I2C_READ_FAILED -4
#define I2C_CLOSE_FAILED -5

// Use configuration file - option ...
// -----------------------------------
// #define CONST_USE_CONFIGURATION_FILE

// I2C Addresses - GRIDEYE and ACCGYRO ...
// ---------------------------------------
#define I2C_GRIDEYE 0x68
#define I2C_ACCGYRO 0x69

// MPU-6000 Settings ...
// ---------------------
#define GYRO_SENS  0x10  // Reg. 0x1B without shifting
#define ACCL_SENS  0x10  // Reg. 0x1C without shifting
#define RATE_DIV    100  // Reg. 0x19
#define DLPF_SET      6  // Reg. 0x1A only DLPF
#define FIFO_VALS  0x78  // Reg. 0x23 values to transfer via FIFO
#define CYCLES       10  // Number of measured cycles

#include <i2c_sensors/i2c_sensors_driver.h>

#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// toString helper function / template ...
// ---------------------------------------
template <typename T> std::string toString(const T& t){
      
    std::ostringstream os;
    os << t;
    return os.str();
}

// i2c_sensors_driver constructor ...
// ----------------------------------
i2c_sensors_driver::i2c_sensors_driver(std::string file_name_,std::string i2c_device_){

    file_name = file_name_;
    i2c_device = i2c_device_;

    accGyroStatusRdy = false;
    gridEyeStatusRdy = false;
    stopAcquistionI2C = false;
    
    acc.ok=false; 
    for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){acc.x_data[i]= 0;acc.y_data[i]= 0;acc.z_data[i]= 0;}
    gyro.ok=false;
    for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){gyro.x_data[i]= 0;gyro.y_data[i]= 0;gyro.z_data[i]= 0;}
    
    gridEye.ok = false; gridEye.thermistor = 0.0;
    
    for(int i=0;i<CONST_GRID_EYE_DOUBLE_SIZE;i++){ gridEye.data[i] = (double)0.0; }
    
#ifdef CONST_USE_CONFIGURATION_FILE
    readParamsFromFile(file_name);
#endif
    
}

// initGridEyeSensor - public function / return status ...
// -------------------------------------------------------
int32_t i2c_sensors_driver::initGridEyeSensor(){
    
    uint8_t buf[2];

    gridEyeStatusRdy = false;

    // Grid Eye - Normal Mode ...
    // --------------------------
    buf[0] = 0x00;
    buf[1] = 0x00;
    
    if(i2c_write_buf(i2c_device.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }

    // Grid Eye - Full Software Reset ...
    // ----------------------------------
    buf[0] = 0x01;
    buf[1] = 0x3F;
    
    if(i2c_write_buf(i2c_device.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
       return STATUS_ERROR; 
    }
    
    // Wait 500 000 micro seconds after reset ...
    // ------------------------------------------
    usleep(500000);
    
    // Grid Eye - Frame Rate - 1 FPS  or 10 FPS ...
    // --------------------------------------------
    buf[0] = 0x02;
    buf[1] = 0x01; // 0x00 -> for 10 FPS ...
    
    if (i2c_write_buf(i2c_device.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;         
    }
	
    // Grid Eye - Average Register - Double Moving Average ...
    // -------------------------------------------------------
    buf[0] = 0x07;
    buf[1] = 0x20;
    
    if (i2c_write_buf(i2c_device.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
        return STATUS_ERROR; 
    }

    gridEyeStatusRdy = true;
    
    return STATUS_OK; // configuration was properly set - OK ...
}

// initAccGyroSensor - public function / return status ...
// -------------------------------------------------------
int32_t i2c_sensors_driver::initAccGyroSensor(){
    
    uint8_t buf[3];
    
    accGyroStatusRdy = false;

    // Acc+Gyro - Reset ...
    // --------------------
    buf[0] = 0x6B;
    buf[1] = 0x80;
    
    if(i2c_write_buf(i2c_device.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;         
    }

    // Wait 500 000 micro seconds after reset ...
    // ------------------------------------------
    usleep(500000);
	
    // Acc+Gyro - Sleep Disable ...
    // ----------------------------
    buf[0] = 0x6B;
    buf[1] = 0x00;
    
    if(i2c_write_buf(i2c_device.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;    
    }
	
    // WhoAmI Readout ...
    // ------------------
    if (i2c_read_buf(i2c_device.c_str(), I2C_ACCGYRO, 0x75, 1, buf)!=STATUS_OK){
        return STATUS_ERROR;
    }
    
    // Debug - disable in production ...
    // ---------------------------------
    // std::cout << "Whoami - debug:"<< std::hex << buf[0] << std::endl;
    
    // Option - selftest ...
    // ---------------------    
    
    // Measurement Setup - rate and bW ...
    // -----------------------------------
    buf[0] = 0x19;
    buf[1] = RATE_DIV;
    buf[2] = DLPF_SET;
    
    if(i2c_write_buf(i2c_device.c_str(), I2C_ACCGYRO, 3, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }

    // Measurement Setup - sensitivity ...
    // -----------------------------------
    buf[0] = 0x1B;
    buf[1] = GYRO_SENS;
    buf[2] = ACCL_SENS;
    if(i2c_write_buf(i2c_device.c_str(), I2C_ACCGYRO, 3, buf)!=STATUS_OK){
       return STATUS_ERROR;  
    }

    // Measurement Setup - FIFO ...
    // ----------------------------
    buf[0] = 0x23;
    buf[1] = FIFO_VALS;
    if(i2c_write_buf(i2c_device.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;   
    }
	
    buf[0] = 0x6A;
    buf[1] = 0x40;
    if(i2c_write_buf(i2c_device.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;     
    }
    
    accGyroStatusRdy = true;

    return STATUS_OK; // Configuration was properly set - OK ...    
}

// acquireSensorAccGyroData - public function ...
// ----------------------------------------------
void i2c_sensors_driver::acquireSensorAccGyroData(){
  
  uint32_t pfifo = 0;  
  uint8_t buf[CONST_ACC_GYRO_SIZE];  // I2C Buffer - max size 128 / real 120...
  
  for(int i=0;i<CONST_ACC_GYRO_SIZE;i++){
      buf[i] = 0x00;
  }
    
  for(int i=0;i<CONST_CYCLE_REPEAT;i++){
      pfifo = read_output_acc_gyro(0x72); 
      if(pfifo >= (CYCLES*12)) break;
      usleep(10);   // Sleep for 10 us ...
  }
  
  // Reading the FIFO ...
  // --------------------  
  if(i2c_read_buf(i2c_device.c_str(), I2C_ACCGYRO, 0x74, CYCLES * 12, buf)!=STATUS_OK){
     acc.ok=false; 
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){acc.x_data[i]= 0;acc.y_data[i]= 0;acc.z_data[i]= 0;}
     gyro.ok=false;
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){gyro.x_data[i]= 0;gyro.y_data[i]= 0;gyro.z_data[i]= 0;}
     return;
  }
  
  acc.ok = true;
  gyro.ok = true;	

  for(int i = 0; i < CYCLES; i++) {
      acc.x_data[i] = (int16_t) buf[12 * i + 0] << 8 | buf[12 * i + 1];
      acc.y_data[i] = (int16_t) buf[12 * i + 2] << 8 | buf[12 * i + 3];
      acc.z_data[i] = (int16_t) buf[12 * i + 4] << 8 | buf[12 * i + 5];

      gyro.x_data[i] = (int16_t) buf[12 * i + 6] << 8 | buf[12 * i + 7];
      gyro.y_data[i] = (int16_t) buf[12 * i + 8] << 8 | buf[12 * i + 9];
      gyro.z_data[i] = (int16_t) buf[12 * i + 10] << 8 | buf[12 * i + 11];
  }

  // FIFO Check after reading ...
  // ----------------------------
  pfifo = read_output_acc_gyro(0x72);
  // Debug - disable in production ...
  // std::cout << "Rest in FIFO after reading:"<< pfifo << std::endl;
  
  // Stop fifo and reset ...
  // -----------------------
  buf[0] = 0x6A;
  buf[1] = 0x44;  // 0x04 - reset and stop FIFO !!! 
  
  i2c_write_buf(i2c_device.c_str(), I2C_ACCGYRO, 2, buf); // Do nothing here ...
}

// acquireSensorGridEyeData - public function ...
// ----------------------------------------------
void i2c_sensors_driver::acquireSensorGridEyeData(){
   
   int count = 0; 
   int16_t x = 0;
   uint8_t buf[CONST_GRID_EYE_SIZE];  // I2C Buffer - max size 128 ...
   
   for(int i=0;i<CONST_GRID_EYE_SIZE;i++){
       buf[i] = 0x00;
   }
   
   // Thermistor read ...
   // -------------------
   if(i2c_read_buf(i2c_device.c_str(), I2C_GRIDEYE, 0x0E, 2, buf)!=STATUS_OK){
      gridEye.ok = false; gridEye.thermistor = 0.0;
      for(int i=0;i<CONST_GRID_EYE_DOUBLE_SIZE;i++){gridEye.data[i] = (double)0.0;} 
      return;  
   }
    
   x = (int16_t) buf[1];
   x |= (x & 0x08) ? 0xF8 : 0;
   x = (x << 8) | (int16_t) buf[0];				
   gridEye.ok = true;
   gridEye.thermistor = (double)0.0625*x; // Conversion ...    
   
   // Raw data readout ...
   // --------------------
   if(i2c_read_buf(i2c_device.c_str(), I2C_GRIDEYE, 0x80, 128, buf)!=STATUS_OK){
      gridEye.ok = false; gridEye.thermistor = 0.0;
      for(int i=0;i<CONST_GRID_EYE_DOUBLE_SIZE;i++){gridEye.data[i] = (double)0.0;} 
      return;    
   }

   gridEye.ok = true;
   
   for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
             x = (int16_t) buf[1 + 2 * j + 16 * i];
             x |= (x & 0x08) ? 0xF8 : 0;
             x = (x << 8) | (int16_t) buf[2 * j + 16 * i];
             gridEye.data[count++] = (double)0.25*x; // Conversion ...
        }
    }
}

// readParamsFromFile - private function ...
// -----------------------------------------
int32_t i2c_sensors_driver::readParamsFromFile(std::string file_name_){
  
  std::ifstream fin(file_name_.c_str());
  
  if(fin.fail()){
     ROS_ERROR_STREAM("Could not open configuration file:" << file_name_.c_str() << std::endl);
     return STATUS_ERROR;
  }

  YAML::Node doc = YAML::LoadFile(file_name_);
  
  try{
    i2c_device =  doc["i2c_dev"].as<std::string>();
  } catch (std::runtime_error) {}
  
  fin.close(); 
  
  return STATUS_OK;
}

// Write  buffer content to the I2C slave - private function ...
// -------------------------------------------------------------
int32_t i2c_sensors_driver::i2c_write_buf(const char *dev, uint8_t addr, uint8_t nData, uint8_t *data){
	
        int fd = -1;
        int error = STATUS_OK;
        
	if((fd = open(dev, O_RDWR)) < 0) {
            std::cout << "Failed to open the I2C bus - status(" << fd <<")"<< std::endl;
            return I2C_OPEN_FAILED;
	}

	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}
	
	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
           std::cout << "IOCTL failed - I2C_SLAVE"<< std::endl;
           return I2C_IOCTL_FAILED;
	}
	
	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	if(write(fd, data, nData) != nData) {
           std::cout << "Failed to write to the I2C bus"<< std::endl;
           error = I2C_WRITE_FAILED;
        }
        
        if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	if(close(fd) != 0) {
           std::cout << "Failed to close the I2C bus"<< std::endl;
           error = I2C_CLOSE_FAILED;
	}

	return error;
}

// Read the word result from given register (H) and next register (L) ...
// ---------------------------------------------------------------------- 
int16_t i2c_sensors_driver::read_output_acc_gyro(uint8_t addr){
    
    uint8_t buf[2];
    
    if(i2c_read_buf(i2c_device.c_str(), I2C_ACCGYRO, addr, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }
    
    return(((int16_t) buf[0]) << 8 | buf[1]);
}

// Read  buffer content from  I2C slave - private function ...
// -----------------------------------------------------------
int32_t i2c_sensors_driver::i2c_read_buf(const char *dev, uint8_t addr, uint8_t subaddr, uint8_t nData, uint8_t *data){
        
        int fd = -1;
        int error = STATUS_OK;
        
	if((fd = open(dev, O_RDWR)) < 0) {
            std::cout << "Failed to open the I2C bus - status(" << fd <<")"<< std::endl;
            return I2C_OPEN_FAILED;
	}
	
	if(stopAcquistionI2C){ close(fd);  return STATUS_EXIT;}

	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
           std::cout << "IOCTL failed - I2C_SLAVE"<< std::endl;
           return I2C_IOCTL_FAILED;
	}
	
	if(stopAcquistionI2C) { close(fd);  return STATUS_EXIT;}

	if(write(fd, &subaddr, 1) != 1) {
           std::cout << "Failed to write to the I2C bus"<< std::endl;
           error = I2C_WRITE_FAILED;
	}
	
	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	if(read(fd, data, nData) != nData) {
           std::cout << "Failed to read from I2C bus"<< std::endl;
           error = I2C_READ_FAILED;
	}
	
	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	if(close(fd) != 0) {
           std::cout << "Failed to close the I2C bus"<< std::endl;
           error = I2C_CLOSE_FAILED;
	}

	return error;
}
