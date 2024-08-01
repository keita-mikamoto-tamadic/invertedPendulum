/*******************************************************************************
 * File Name    : imu.h
 * Description  : IMU GY521の制御
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2024.04.04 1.00 K.Mikamoto 新設
 *****************************************************************************/

#ifndef _IMU_H_
#define _IMU_H_

#include <Wire.h>

/******************************************************************************
 * Typedef definitions
 *****************************************************************************/
typedef struct {
  float yaw;
  float pitch;
  float roll;
  float yaw_gyro;
  float pitch_gyro;
  float roll_gyro;
} st_imu;

/******************************************************************************
 * Global variables
 *****************************************************************************/
extern st_imu stg_imu;

/******************************************************************************
 * Global functions
 *****************************************************************************/
extern void imu_setup(void);
extern void get_imu(void);

#endif  // _IMU_H_