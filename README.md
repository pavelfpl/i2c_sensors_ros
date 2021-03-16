# i2c_sensors_ros 
This repo contains user space drivers for **Catkin ROS** (Robot Operating System) including:
- `Panasonic GridEye AMG8833 Infrared Array Sensor 8x8 driver`
- `MPU-6000 3-Axis Gyroscope/Accelerometer driver`

## Dependencies
- `Boost, TinyXML`
- `OpenCV `(for image format conversion - TIFF,  PNG etc. - **consumer only**)

# Building

See: http://wiki.ros.org/ROS/Tutorials/BuildingPackages

## Configuration

Configure **i2c device** number in:  `config/i2c_driver.yaml ` (e.g.  `/dev/i2c-1 `)

## Exported messages Panasonic GridEye (GridEyeSensorMsg.msg)
The **GridEye** is sampled 1/s and data field (std::vector<double>) always contains **64 values ​​(8x8)**. Thermistor value is stored too.

  `float64 thermistor`
  `float64[64] data` 

## Exported messages MPU-6000 (AccGyroSensorMsg.msg)

The **MPU-6000** contains a FIFO, 10 samples is collected every second for each of the X, Y and Z axes (total 60 values = 30 acc + 30 gyro)

`float64[10] - dataAccX`  
`float64[10] - dataAccY`  
`float64[10] - dataAccZ`  
`float64[10] - dataGyroX`  
`float64[10] - dataGyroY`  
`float64[10] - dataGyroZ` 

**sensors_consumer** sample project can be used for evaluation purpose. 

>Tested on embedded Linux platform: **ODROID-XU4**.

## Note

!! NOTE: add selected user to **i2c group** [to run driver without root permission] !!!  
e.g. `sudo adduser odroid i2c`
