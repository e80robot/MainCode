#include "PControl.h"
#include "Printer.h"
#include
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
}

int PControl::getWayPoint(int dim) {
  return wayPoints[currentWayPoint*stateDims+dim];
}

void PControl::calculateControl(state_t * state, gps_state_t * gps_state_p) {
  if (gps_state_p->num_sat >= N_SATS_THRESHOLD){
    gpsAcquired = 1;

    updatePoint(state->x, state->y);
    if (currentWayPoint == totalWayPoints) return; // stops motors at final point

    // set up variables
    int x_des = getWayPoint(0);
    int y_des = getWayPoint(1);

    // Set the values of yaw_des, yaw, control effort (u), uL, and uR appropriately
    // You can use trig functions (atan2 might be useful)
    // You can access the x and y coordinates calculated in StateEstimator.cpp using state->x and state->y respectively
    // You can access the heading calculated in StateEstimator.cpp using state->heading

    //////////////////////////////////////////////////////////////////////
    // write code here
    //////////////////////////////////////////////////////////////////////

    yaw_des = atan2(y_des - state->y, x_des - state->x);
    yaw = state->yaw;
    u = Kp*angleDiff(yaw_des - yaw);

    uL = max(0.0,min(255.0,(avgPower - u)*Kl));
    uR = max(0.0,min(255.0,(avgPower + u)*Kr));

    ///////////////////////////////////////////////////////////////////////
    // don't change code past this point
    ///////////////////////////////////////////////////////////////////////
  }
  else{
    gpsAcquired = 0;
  }

}

String PControl::printString(void) {
  String printString = "";
  if(!gpsAcquired){
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
  if(!gpsAcquired){
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
  
  
  // push a high out of the desired pin to trigger the pi sequence to take video 
  
  
  
  // get the next waypoint
  int x_des = getWayPoint(0);
  int y_des = getWayPoint(1);
  dist = sqrt(pow(x-x_des,2) + pow(y-y_des,2));

  if (dist < SUCCESS_RADIUS && currentWayPoint < totalWayPoints) {
    
    
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
