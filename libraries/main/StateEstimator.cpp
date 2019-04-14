#include "StateEstimator.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

StateEstimator::StateEstimator(void)
  : DataSource("x,y","float,float") // from DataSource
{}

void StateEstimator::init(void) {
 	state.x = 0;
  state.y = 0;
  state.yaw = 0;
}

void StateEstimator::updateState(imu_state_t * imu_state_p, gps_state_t * gps_state_p) {
  if (gps_state_p->num_sat >= N_SATS_THRESHOLD){
    gpsAcquired = 1;

    // get x and y
    float cosOrigLat = cos(origin_lat*PI/180.0);
    state.x = (gps_state_p->lon-origin_lon)*PI/180.0*RADIUS_OF_EARTH_M*cosOrigLat;
    state.y = (gps_state_p->lat-origin_lat)*PI/180.0*RADIUS_OF_EARTH_M;

    // get heading
    float heading_rad = imu_state_p->heading*PI/180.0; // convert to radians
    float yaw_rad = -heading_rad + PI/2.0; // adjust from 0=North, CWW=(+) to 0=East, CCW=(+)
    state.yaw = angleDiff(yaw_rad);
    
  }
  else{
    gpsAcquired = 0;
  }
}

String StateEstimator::printState(void) {
  String currentState = "";
  if (!gpsAcquired){
    currentState += "State: Waiting to acquire more satellites...";
  }
  else{
    int decimals = 2;
    currentState += "State: x: ";
    currentState += String(state.x,decimals);
    currentState += "[m], ";
    currentState += "y: ";
    currentState += String(state.y,decimals);
    currentState += "[m], ";
    currentState += "yaw: ";
    currentState += String(state.yaw,decimals);
    currentState += "[rad]";
  }
  return currentState;
}

size_t StateEstimator::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = state.x;
  data_slot[1] = state.y;
  return idx + 2*sizeof(float);
}
