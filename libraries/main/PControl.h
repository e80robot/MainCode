#ifndef __PCONTROL_H__
#define __PCONTROL_H__

#define SUCCESS_RADIUS 2.0 // success radius in meters

#include <Arduino.h>
#include "MotorDriver.h"
#include "StateEstimator.h"
extern MotorDriver motorDriver;

class PControl : public DataSource
{
public:
  PControl(void);

  // defines the waypoints used for pControl
  void init(const int totalWayPoints_in, const int stateDims_in, double * wayPoints_in);

  // sets the motor speeds using P-Control
  void calculateControl(state_t * state, gps_state_t * gps_state_p);

  String printString(void);

  String printWaypointUpdate(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

  // control fields
  float yaw_des;         // desired yaw
  float yaw;             // current yaw
  float dist;            // distance to waypoint
  float u;               // control effort
  float Kp=10.0;         // proportional control gain
  float Kr=1.0;          // right motor gain correction
  float Kl=1.0;          // left motor gain correction
  float avgPower = 20.0; // average forward thrust
  float uR;             // right motor effort
  float uL;             // left motor effort


private:

  // updates the current waypoint if necessary
  void updatePoint(float x, float y);

  int getWayPoint(int dim);

  int totalWayPoints, stateDims;
  double * wayPoints;
  int currentWayPoint = 0;
  bool gpsAcquired;
};

#endif
