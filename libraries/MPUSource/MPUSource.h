#ifndef __THERMISTORSOURCE_h__
#define __THERMISTORSOURCE_h__

#include <Arduino.h>
#include "DataSource.h"
#include "Pinouts.h"
#include "MPU9250.h"

/*
 * ButtonSampler implements SD logging for the onboard pushbutton 
 */


class MPUSource : public DataSource
{
public:
  MPUSource(void);

  void init(MPU9250 * imu);

  // Managing state
  void updateState(MPU9250 * imu);
  String printState(void);

  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;

  // Write out
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;
  
};

#endif