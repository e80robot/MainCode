#ifndef __SENSOR_MPU_H__
#define __SENSOR_MPU_H__

#include <Arduino.h>
#include <Wire.h>
#include "SparkFunIMU.h"
#include "DataSource.h"
#include <MPU9250.h>
#include <quaternionFilters.h>

#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

typedef struct {
  float accelX;   // [mg] (g=accleration due to gravity)
  float accelY;   // [mg]
  float accelZ;   // [mg]
  float gyroX;    // [deg/sec]
  float gyroY;    // [deg/sec]
  float gyroZ;    // [deg/sec]
  float magX;     // [Gauss]
  float magY;     // [Gauss]
  float magZ;     // [Gauss]
  float roll;     // [degrees]
  float pitch;    // [degrees]
  float heading;  // [degrees] CW from magnetic north
  float deltat;   // time since last iteration
} mpu_state_t;

class SensorMPU : public DataSource {
public:
  SensorMPU(void);

  // Starts the connection to the sensor
  void init();

  // Reads data from the sensor
  void read();

  void getOrientation(); 

  // Latest reported orientation data is stored here
  mpu_state_t state; 

  // prints state to serial
  String printRollPitchHeading(void);
  String printAccels(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:

  // Create sensor instance
  MPU9250 myIMU;

  // time at which the imu is initialized [ms]
  unsigned long start_time;
  // time of last imu read
  unsigned long current_time;

  //////////////////////////////////////////////////////////////
  // these values are specific to each imu
  //////////////////////////////////////////////////////////////

  // Offsets applied to raw x/y/z accel values
  float accel_offsets[3]      = { 0.0F, 0.0F, 0.0F };

  // Offsets applied to raw x/y/z mag values
  // float mag_offsets[3]        = { 0.00,  0.00,  0.00 };
  // float mag_offsets[3]        = { -0.3806,  -0.3517,  -0.2626 }; // inbuilt
  float mag_offsets[3]        = { 512.3940, 162.3150, -317.5965 };

  // Soft iron error compensation matrix
  // float mag_ironcomp[3][3] =  { {  1.00,     0.00,     0.00   },
  //                               {  0.00,     1.00,     0.00   },
  //                               {  0.00,     0.00,     1.00   } };

  // float mag_ironcomp[3][3] =  { {  3.5493,     -0.1715,     -0.2056   },
  //                               {  0.00,     3.6553,     0.1126   },
  //                               {  0.00,     0.00,     3.7611   } }; // inbuilt
  float mag_ironcomp[3][3] =  { {  0.0024,   0.0003,   0.0002   },
                                {  0.00,     0.0021,   0.0002   },
                                {  0.00,     0.00,     0.0020   } };

};

#endif

