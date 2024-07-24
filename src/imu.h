#ifndef _IMU_H_
#define _IMU_H_

#include <Wire.h>

/* global variables and function */
typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float yaw_gyro;
    float pitch_gyro;
    float roll_gyro;
} st_imu;
extern st_imu stg_imu;

extern void imu_setup(void);
extern void get_imu(void);

#endif // _IMU_H_