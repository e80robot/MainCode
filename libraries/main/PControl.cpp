
#include "PControl.h"
#include "Printer.h"

// a library to use digitalWrite
#include "Arduino.h"


extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

PControl::PControl(void)
: DataSource("u,uL,uR,yaw,yaw_des","float,float,float,float,float"){}


void PControl::init(const int totalWayPoints_in, const int stateDims_in, double * wayPoints_in) {
  totalWayPoints = totalWayPoints_in;
  stateDims = stateDims_in;
  wayPoints = wayPoints_in;
  // create a variable to calculate current time in "calculateControl"
  startTime = millis();
  // the robot starts the hard coded sequence once beginMoving has passed
  // ************************************
  // ***** CHANGE FOR DEPLOYMENT ********
  // ************************************
  beginMoving = startTime;
  // a variable to help determine when to stop moving
  condCounter = 0;
}

int PControl::getWayPoint(int dim) {
  return wayPoints[currentWayPoint*stateDims+dim];
}

void PControl::calculateControl(state_t * state, gps_state_t * gps_state_p){
  // by default push LOW and set motor PWM to be 0
  digitalWrite(A3, LOW);
  uL = 0.0;
  uR = 0.0;
  uV = 0.0;

  // figure out how long it's been since boot (mod by total cycle time)
  currentTime = (millis() - startTime) % 30000;

  // *********** HARD CODING IN MOVEMENT FOR THE ROBOT ***********
  // check if we've run the move forward/take video/dive more than 10 times
  if (condCounter >= 1100000){
    Serial.println("hit the cond limit");
    uL = 0.0;
    uR = 0.0;
    uV = 0.0;
  }
  // move forward for 20 seconds
  else if((currentTime >= beginMoving) && (currentTime <= beginMoving + 10000)) {
    uL = 0.0;
    uR = 0.0;
    uV = 200.0;
    condCounter++;
  }
  // trigger the camera to take video at surface level
  else if((currentTime >= beginMoving + 15000) && (currentTime <= beginMoving + 30000)){
    uL = 0.0;
    uR = 0.0;
    uV = -200.0;
    digitalWrite(A3, HIGH);
    condCounter++;
  }
  // dive down for 5 seconds
  else if((currentTime >= beginMoving + 60000) && (currentTime <= beginMoving + 65000)){
    uL = 0.0;
    uR = 0.0;
    uV = -200.0;
    condCounter++;
  }
  // trigger camera to take video at the depth we've arrived at
  else if((currentTime >= beginMoving + 65001) && (currentTime <= beginMoving + 65200)){
    uL = 0.0;
    uR = 0.0;
    uV = 0.0; // this should be some value which holds us stationart (0.0 now)
    digitalWrite(A3, HIGH);
    condCounter++;
  }
  // return to the surface (estimated 40 seconds required)
  else if((currentTime >= beginMoving + 100000) && (currentTime <= beginMoving + 140000)){
    uL = 0.0;
    uR = 0.0;
    uV = -255.0;
    condCounter++;
  }
}

String PControl::printString(void) {
  String printString = "";
  if(false){
    printString += "PControl: Waiting to acquire more satellites...";
  }
  else{
    printString += "PControl: ";
    printString += "Yaw_Des: ";
    printString += String(yaw_des*180.0/PI);
    printString += "[deg], ";
    printString += "Yaw: ";
    printString += String(yaw*180.0/PI);
    printString += "[deg], ";
    printString += "u: ";
    printString += String(u);
    printString += ", u_L: ";
    printString += String(uL);
    printString += ", u_R: ";
    printString += String(uR);
  }
  return printString;
}

String PControl::printWaypointUpdate(void) {
  String wayPointUpdate = "";
  if(false){
    wayPointUpdate += "PControl: Waiting to acquire more satellites...";
  }
  else{
    wayPointUpdate += "PControl: ";
    wayPointUpdate += "Current Waypoint: ";
    wayPointUpdate += String(currentWayPoint);
    wayPointUpdate += " Distance from Waypoint: ";
    wayPointUpdate += String(dist);
    wayPointUpdate += "[m]";
  }
  return wayPointUpdate;
}

void PControl::updatePoint(float x, float y) {
  // note that this means we will not take data on the last waypoint-- by convention sould be pick up point
  if (currentWayPoint == totalWayPoints) return; // don't check if finished

  // get the next waypoint
  int x_des = getWayPoint(0);
  int y_des = getWayPoint(1);
  dist = sqrt(pow(x-x_des,2) + pow(y-y_des,2));

  if (dist < SUCCESS_RADIUS && currentWayPoint < totalWayPoints) {
    // trigger the pi to take a video
    piTriggerTime = millis()+1000;
    String changingWPMessage = "Got to waypoint " + String(currentWayPoint)
      + ", now directing to next point";
    int cwpmTime = 20;
    currentWayPoint++;
    if (currentWayPoint == totalWayPoints) {
      changingWPMessage = "Congratulations! You completed the path! Stopping motors.";
      uR=0;
      uL=0;
      cwpmTime = 0;
    }
    printer.printMessage(changingWPMessage,cwpmTime);
  }
}

size_t PControl::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = u;
  data_slot[1] = uL;
  data_slot[2] = uR;
  data_slot[3] = yaw;
  data_slot[4] = yaw_des;
  return idx + 5*sizeof(float);
}
