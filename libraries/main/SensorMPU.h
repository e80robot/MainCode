#ifndef __SENSOR_IMU_H__
#define __SENSOR_IMU_H__

#include <Arduino.h>
#include <Wire.h>
#include "LSM303CTypes.h"
#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "DataSource.h"

typedef struct {
  float accelX;   // [mg] (g=accleration due to gravity)
  float accelY;   // [mg]
  float accelZ;   // [mg]
  float magX;     // [Gauss]
  float magY;     // [Gauss]
  float magZ;     // [Gauss]
  float roll;     // [degrees]
  float pitch;    // [degrees]
  float heading;  // [degrees] CW from magnetic north
} imu_state_t;

class SensorMPU : public DataSource {
public:
  SensorMPU(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  void getOrientation(float ax, float ay, float az, float mx, float my, float mz); 

  // Latest reported orientation data is stored here
  imu_state_t state; 

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
  float mag_offsets[3]        = { 512.3940, 162.3150, -317.5965 };

  // Soft iron error compensation matrix
  // float mag_ironcomp[3][3] =  { {  1.00,     0.00,     0.00   },
  //                               {  0.00,     1.00,     0.00   },
  //                               {  0.00,     0.00,     1.00   } };
  float mag_ironcomp[3][3] =  { {  0.0024,   0.0003,   0.0002   },
                                {  0.00,     0.0021,   0.0002   },
                                {  0.00,     0.00,     0.0020   } };

};

#endif
