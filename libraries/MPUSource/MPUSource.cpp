#include "MPUSource.h"
#include "Printer.h"

extern Printer printer;

MPUSource::MPUSource(void) 
  : DataSource("MPUax,MPUay,MPUaz","float,float,float") // from DataSource
{}


void MPUSource::init(MPU9250 * imu)
{  
  imu->MPU9250SelfTest(imu->selfTest);
  imu->calibrateMPU9250(imu->gyroBias, imu->accelBias);
  imu->initMPU9250();
  imu->initAK8963(imu->factoryMagCalibration);
  imu->getAres();
  imu->getGres();
  imu->getMres();
}


void MPUSource::updateState(void)
// This function is called in the main loop of Default_Robot.ino
{
  pinVoltage = (3.3 / 1024.0) * analogRead(pin);
}


String MPUSource::printState(void)
// This function returns a string that the Printer class 
// can print to the serial monitor if desired
{
  return "Analog Voltage at " + name + ": " + String(voltage);
}


size_t MPUSource::writeDataBytes(unsigned char * buffer, size_t idx)
// This function writes data to the micro SD card
{
  bool * data_slot = (bool *) &buffer[idx];
  data_slot[0] = pinVoltage;
  return idx + sizeof(bool);
}
