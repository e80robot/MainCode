#include "SensorMPU.h"
#include "Printer.h"
#include <MPU9250.h>
#include <quaternionFilters.h>

extern Printer printer;

SensorMPU::SensorMPU(void)
  : DataSource("rollIMU,pitchIMU,headingIMU,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,time",
               "float,float,float,float,float,float,float,float,float,float,float,float,long"), myIMU(0x68, Wire, 400000) {
}

void SensorMPU::init(void) {
  Serial.print("Initializing IMU... ");
  myIMU.MPU9250SelfTest(myIMU.selfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();
  Serial.print("IMU self test completed");
  start_time = millis();
}

void SensorMPU::read(void) {

  // Get new data samples

  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];


    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];


  }
  float ax = myIMU.ax;
  float ay = myIMU.ay;
  float az = myIMU.az;

  float gx = myIMU.gx;
  float gy = myIMU.gy;
  float gz = myIMU.gz;

  float mx = myIMU.mx;
  float my = myIMU.my;
  float mz = myIMU.mz;


  // Record current time
  current_time = millis() - start_time;

  // Remove offsets from acceleration measurements
  state.accelX = ax - accel_offsets[0];
  state.accelY = ay - accel_offsets[1];
  state.accelZ = az - accel_offsets[2];

  state.gyroX = gx;
  state.gyroY = gy;
  state.gyroZ = gz;

  // Remove offsets from magnertometer measurements (base values in uTesla)
  mx = mx - mag_offsets[0];
  my = my - mag_offsets[1];
  mz = mz - mag_offsets[2];
  
  // Apply magnetometer soft iron error compensation
  state.magX = mx * mag_ironcomp[0][0] + my * mag_ironcomp[0][1] + mz * mag_ironcomp[0][2];
  state.magY = mx * mag_ironcomp[1][0] + my * mag_ironcomp[1][1] + mz * mag_ironcomp[1][2];
  state.magZ = mx * mag_ironcomp[2][0] + my * mag_ironcomp[2][1] + mz * mag_ironcomp[2][2];

  myIMU.mx = state.magX;
  myIMU.my = state.magY;
  myIMU.mz = state.magZ;

  myIMU.updateTime();

  myIMU.count = millis();
  myIMU.sumCount = 0;
  myIMU.sum = 0;

  // populate the roll, pitch, yaw with simple orientation calcs  
  getOrientation();
  myIMU.delt_t = millis() - myIMU.count;
}

String SensorMPU::printRollPitchHeading(void) {
  String printString = "MPU:"; 
  printString += " roll: ";
  printString += String(state.roll);
  printString += "[deg],";
  printString += " pitch: "; 
  printString += String(state.pitch);
  printString += "[deg],";
  printString += " heading: ";
  printString += String(state.heading);
  printString += "[deg]";
  return printString; 
}

String SensorMPU::printAccels(void) {
  String printString = "IMU:";

  printString += " accelX: ";
  printString += String(state.accelX);
  printString += "[mg], ";
  printString += " accelY: ";
  printString += String(state.accelY);
  printString += "[mg], ";
  printString += " accelZ: ";
  printString += String(state.accelZ);
  printString += "[mg]";

  printString += " gyroX: ";
  printString += String(state.gyroX);
  printString += "[deg/sec], ";
  printString += " gyroY: ";
  printString += String(state.gyroY);
  printString += "[deg/sec], ";
  printString += " gyroZ: ";
  printString += String(state.gyroZ);
  printString += "[deg/sec]";
  return printString;

  /*
  printString += " magX: ";
  printString += String(state.magX);
  printString += "[uT], ";
  printString += " magY: ";
  printString += String(state.magY);
  printString += "[uT], ";
  printString += " magZ: ";
  printString += String(state.magZ);
  printString += "[uT]";
  return printString;
  */
}

size_t SensorMPU::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = state.roll;
  data_slot[1] = state.pitch;
  data_slot[2] = state.heading;
  data_slot[3] = state.accelX;
  data_slot[4] = state.accelY;
  data_slot[5] = state.accelZ;
  data_slot[6] = state.gyroX;
  data_slot[7] = state.gyroY;
  data_slot[8] = state.gyroZ;
  data_slot[9] = state.magX;
  data_slot[10] = state.magY;
  data_slot[11] = state.magZ;
  idx += 12*sizeof(float);
  long * long_slot = (long *) (buffer + idx);
  long_slot[0] = current_time;
  return idx + sizeof(long);
}


void SensorMPU::getOrientation() {
  // copied from Adafruit_Simple_AHRS
    MadgwickQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

        myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                                * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                        * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                        * *(getQ() + 3));
    myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                                * *(getQ() + 2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                                * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                        * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                        * *(getQ() + 3));

  state.roll = myIMU.roll;
  state.pitch = myIMU.pitch;
  state.heading = myIMU.yaw;

  float PI_F = 3.14159;
  // convert to degrees
  state.roll *= 180.0/PI_F;
  state.pitch *= 180.0/PI_F;
  state.heading *= 180.0/PI_F + 10.5;
  
}
