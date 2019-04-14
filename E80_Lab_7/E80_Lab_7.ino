/********
E80 Lab 7 code
Current Author:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
Previous Contributors:
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)                    
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <StateEstimator.h>
#include <Adafruit_GPS.h>
#include <ButtonSampler.h> // A template of a data source library
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <PControl.h>
#define UartSerial Serial1
#include <GPSLockLED.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
StateEstimator state_estimator;
PControl pcontrol;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ButtonSampler button_sampler;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;
 double waypoints [] = { 125, -40, 150, -40, 125, -40 };
////////////////////////* Setup *////////////////////////////////
// log35 is good
// log37
void setup() {
  
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  logger.include(&pcontrol);
  logger.include(&motor_driver);
  logger.include(&button_sampler);
  logger.init();

  printer.init();
  button_sampler.init();
  imu.init();
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();

  const int number_of_waypoints = 3;
  const int waypoint_dimensions = 2;       // waypoints are set to have two pieces of information, x then y.
    // listed as x0,y0,x1,y1, ... etc.
  pcontrol.init(number_of_waypoints, waypoint_dimensions, waypoints);
  
  state_estimator.init();

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  button_sampler.lastExecutionTime  = loopStartTime - LOOP_PERIOD + BUTTON_LOOP_OFFSET;
  state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + STATE_ESTIMATOR_LOOP_OFFSET;
  pcontrol.lastExecutionTime        = loopStartTime - LOOP_PERIOD + P_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}



//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();
  
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,logger.printState());
    printer.printValue(1,gps.printState());
    printer.printValue(2,state_estimator.printState());     
    printer.printValue(3,pcontrol.printWaypointUpdate());
    printer.printValue(4,pcontrol.printString());
    printer.printValue(5,motor_driver.printState());
    printer.printValue(6,imu.printRollPitchHeading());
    printer.printValue(7,imu.printAccels());
    printer.printToSerial();  // To stop printing, just comment this line out
  }


  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }

  if (true){//(gps.loopTime(loopStartTime)) {
    gps.lastExecutionTime = currentTime;
    gps.read(&GPS); // blocking UART calls
  }

  // uses the ButtonSampler library to read a button -- use this as a template for new libraries!
  if ( currentTime-button_sampler.lastExecutionTime > LOOP_PERIOD ) {
    button_sampler.lastExecutionTime = currentTime;
    button_sampler.updateState();
  }

    if (currentTime-led.lastExecutionTime > LOOP_PERIOD) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if ( currentTime-state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    state_estimator.lastExecutionTime = currentTime;
    state_estimator.updateState(&imu.state, &gps.state);
  }

  if ( currentTime-pcontrol.lastExecutionTime > LOOP_PERIOD ) {
    pcontrol.lastExecutionTime = currentTime;
    pcontrol.calculateControl(&state_estimator.state, &gps.state);
    // this is where the motors are actually instructed to drive
    motor_driver.drive(pcontrol.uL,pcontrol.uR,0);
  }

  if (currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}
