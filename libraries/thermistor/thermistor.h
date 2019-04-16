#ifndef __THERMISTORSOURCE_h__
#define __THERMISTORSOURCE_h__

#include <Arduino.h>
#include "DataSource.h"
#include "Pinouts.h"

/*
 * ButtonSampler implements SD logging for the onboard pushbutton 
 */


class ThermistorSource : public DataSource
{
public:
  ThermistorSource(void);

  void init(void);
  
  int pin;
  String name;

  // Managing state
  float pinVoltage;
  void updateState(void);
  String printState(void);

  // Write out
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;
  
};

#endif