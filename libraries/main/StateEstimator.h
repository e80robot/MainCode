#ifndef __STATE_ESTIMATOR_H__
#define __STATE_ESTIMATOR_H__

#include <Arduino.h>

#include <SensorGPS.h>
#include <SensorIMU.h>
#include "DataSource.h"

#define RADIUS_OF_EARTH_M 6371000 // [m]

typedef struct {
  float x = 0; // x position in global frame [m]
  float y = 0; // y position in global frame [m]
  float yaw = 0; // yaw in global frame [rad] CCW from magnetic east
} state_t;

/*
 * StateEstimator class keeps track of the robot's state, incorporating
 * measurements of the system outputs from the various sensors like IMU or
 * GPS as well as the control inputs to the system.
 */
class StateEstimator : public DataSource
{
public:
  StateEstimator(void);

  // init
  void init(void);

  // State Access
  state_t state;
  void updateState(imu_state_t * imu_state_p, gps_state_t * gps_state_p);
  String printState(void);

  void latlonToXY(double lat, double lon, float* x_p, float* y_p);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  // set coordinates of chosen origin below (origin is now center of pHake Lake
  // BFS
  // const float origin_lat = 34.1091;
  // const float origin_lon = -117.712567;

  // Parsons
  const float origin_lat = 34.106465;
  const float origin_lon = -117.712488;

  bool gpsAcquired;


};

#endif
