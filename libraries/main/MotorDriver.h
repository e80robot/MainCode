#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#define NUM_MOTORS 3

#define MOTOR_A_INDEX 0
#define MOTOR_B_INDEX 1
#define MOTOR_C_INDEX 2
#define FORWARD_PIN 0
#define BACKWARD_PIN 1

// the minimum PWM amount that causes the motors to actually spin
#define MOTOR_DEADZONE 34

#include <Arduino.h>
#include "DataSource.h"
#include "Pinouts.h"

/*
 * MotorDriver handles the raw signals that are needed to drive the robot's
 * motors.
 */
class MotorDriver : public DataSource
{
public:
  MotorDriver();

  // initializes motor command state to zero
  void init(void);

  // applies the stored motor command state values to the pins
  void apply(void);

  // prints current motor command state as string
  String printState(void);

  // helper function that sets motor command state and then applies it
  void drive(int motorA_power,int motorB_power,int motorC_power);

  // Range from -255 to +255 for full reverse or full forward
  int motorValues[NUM_MOTORS];

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

private:
  // pins for the motors
  int motorPins[NUM_MOTORS][2] = {{MOTOR_L_FORWARD,MOTOR_L_REVERSE},
                                  {MOTOR_R_FORWARD,MOTOR_R_REVERSE},
                                  {MOTOR_V_FORWARD,MOTOR_V_REVERSE}};

  // pwm data
  unsigned int pwmValues[NUM_MOTORS]; // absolute value
  bool pwmDir[NUM_MOTORS]; // direction
};

#endif
