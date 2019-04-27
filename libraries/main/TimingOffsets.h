#ifndef __TIMINGOFFSETS_H__
#define __TIMINGOFFSETS_H__

// all times are recorded in units of ms
// the offsets define when tasks happen within a loop period
#define LOOP_PERIOD 50
#define PRINTER_LOOP_OFFSET 0
#define IMU_LOOP_OFFSET 1
#define MPU_LOOP_OFFSET 1
#define GPS_LOOP_OFFSET 2
#define ADC_LOOP_OFFSET 3	
#define ERROR_FLAG_LOOP_OFFSET 4
#define BUTTON_LOOP_OFFSET 5
#define STATE_ESTIMATOR_LOOP_OFFSET 6
#define P_CONTROL_LOOP_OFFSET 7
#define LED_LOOP_OFFSET 8
#define LOGGER_LOOP_OFFSET 9

#endif