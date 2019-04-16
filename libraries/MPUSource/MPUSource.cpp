#include "MPUSource.h"
#include "Printer.h"
#include "MPU9250.h"

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


void MPUSource::updateState(MPU9250 * imu)
// This function is called in the main loop of Default_Robot.ino
{
  if (imu->readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    imu->readAccelData(imu->accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    imu->ax = (float)imu->accelCount[0] * imu->aRes; // - imu->accelBias[0];
    imu->ay = (float)imu->accelCount[1] * imu->aRes; // - imu->accelBias[1];
    imu->az = (float)imu->accelCount[2] * imu->aRes; // - imu->accelBias[2];
    

    imu->readGyroData(imu->gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    imu->gx = (float)imu->gyroCount[0] * imu->gRes;
    imu->gy = (float)imu->gyroCount[1] * imu->gRes;
    imu->gz = (float)imu->gyroCount[2] * imu->gRes;

    imu->readMagData(imu->magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    imu->mx = (float)imu->magCount[0] * imu->mRes
               * imu->factoryMagCalibration[0] - imu->magBias[0];
    imu->my = (float)imu->magCount[1] * imu->mRes
               * imu->factoryMagCalibration[1] - imu->magBias[1];
    imu->mz = (float)imu->magCount[2] * imu->mRes
               * imu->factoryMagCalibration[2] - imu->magBias[2];
           }
           imu->updateTime();
    this->ax = imu->ax;
    this->ay = imu->ay;
    this->az = imu->az;

    this->gx = imu->gx;
    this->gy = imu->gy;
    this->gz = imu->gz;

    this->mx = imu->mx;
    this->my = imu->my;
    this->mz = imu->mz;

}


String MPUSource::printState(void)
// This function returns a string that the Printer class 
// can print to the serial monitor if desired
{
  return "Analog Voltage at ";// + this->name + ": " + String(this->voltage);
}


size_t MPUSource::writeDataBytes(unsigned char * buffer, size_t idx)
// This function writes data to the micro SD card
{
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = 0;
  return idx + sizeof(float);
}
