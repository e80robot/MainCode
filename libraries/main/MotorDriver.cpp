#include "MotorDriver.h"
#include "Printer.h"
extern Printer printer;

MotorDriver::MotorDriver()
: DataSource("motorA,motorB,motorC","int,int,int")
{
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorValues[m] = 0;
    pwmValues[m] = 0;
    pwmDir[m] = 0;
    pinMode(motorPins[m][BACKWARD_PIN],OUTPUT);
    pinMode(motorPins[m][FORWARD_PIN],OUTPUT);
  }
}

void MotorDriver::init(void) {
  for (int m = 0; m < NUM_MOTORS; m++) {
    analogWrite(motorPins[m][FORWARD_PIN], 1); // this line resets the error flag of motor drivers
    delay(10); // oone of the motor driver input pins is pulled high long enough to reset the EF pin
    analogWrite(motorPins[m][BACKWARD_PIN], 0);
    analogWrite(motorPins[m][FORWARD_PIN], 0);
  }
}

void MotorDriver::apply(void)
{
  // determine direction and magnitude of spin required:
  for (int m = 0; m < NUM_MOTORS; m++) {
    pwmDir[m] = (motorValues[m] >= 0);
    pwmValues[m] = (motorValues[m] < 0) ? -motorValues[m] : motorValues[m];
    if (motorValues[m]) { // correct for deadzone if not zero
      pwmValues[m] = pwmValues[m] - MOTOR_DEADZONE*pwmValues[m]/255 + MOTOR_DEADZONE;
    }
  }

  // write this information to motors
  for (int m = 0; m < NUM_MOTORS; m++) { // using pwmDir as 0 or 1
    analogWrite(motorPins[m][FORWARD_PIN], pwmDir[m]*pwmValues[m]);
    analogWrite(motorPins[m][BACKWARD_PIN], (!pwmDir[m])*pwmValues[m]);
  }
}

void MotorDriver::drive(int motorA_power,int motorB_power,int motorC_power) {
  motorValues[MOTOR_A_INDEX] = motorA_power; //M1
  motorValues[MOTOR_B_INDEX] = motorB_power; //M2
  motorValues[MOTOR_C_INDEX] = motorC_power; //M3
  apply();
  printState();
}

String MotorDriver::printState(void) {
  String printString =
    "Motors: PWMA: "  + String(pwmDir[MOTOR_A_INDEX] ? " " : "-") + String( pwmValues[MOTOR_A_INDEX] ) +  
           " PWMB: "  + String(pwmDir[MOTOR_B_INDEX] ? " " : "-") + String( pwmValues[MOTOR_B_INDEX] ) +  
           " PWMC: "  + String(pwmDir[MOTOR_C_INDEX] ? " " : "-") + String( pwmValues[MOTOR_C_INDEX] ) ; 
  return printString;
}

size_t MotorDriver::writeDataBytes(unsigned char * buffer, size_t idx) {
  int * data_slot = (int *) &buffer[idx];
  for (int i = 0; i < NUM_MOTORS; i++) {
    data_slot[i] = (pwmDir[i] ? 1 : -1) * pwmValues[i]; // pwmDir[i]*  
  }
  return idx + NUM_MOTORS*sizeof(int);
}


