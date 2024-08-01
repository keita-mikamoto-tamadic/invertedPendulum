/*******************************************************************************
 * File Name    : imu.cpp
 * Description  : IMU GY521の制御
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2024.04.04 1.00 K.Mikamoto 新設
 *****************************************************************************/

#include <MPU6050_6Axis_MotionApps20.h>
#include <imu.h>
#include <userdefine.h>
MPU6050 mpu;

/******************************************************************************
 * Macro definitions
 *****************************************************************************/

/*******************************************************************************
 * Static variables and functions
 *****************************************************************************/
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
VectorInt16 rpy_gyro;
float rpy[3];
typedef struct {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} st_Prv;
static st_Prv sts_Prv;

/*******************************************************************************
 * Global variables and functions
 *****************************************************************************/
void imu_setup(void);
void get_imu(void);

// structure defined in header file to give global scope
// typedef struct {
//   float yaw;
//   float pitch;
//   float roll;
//   float yaw_gyro;
//   float pitch_gyro;
//   float roll_gyro;
// } st_imu;
st_imu stg_imu;

/*****************************************************************************
 * Function Name: imu_setup
 * Description  : IMUのセットアップ
 * Arguments    : none
 * Return Value : none
 *****************************************************************************/
void imu_setup() {
  Wire.begin();
  Wire.setClock(115200);
  mpu.initialize();
  delay(300);

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-3500);
  mpu.setYAccelOffset(-2505);
  mpu.setZAccelOffset(649);
  mpu.setXGyroOffset(145);
  mpu.setYGyroOffset(-46);
  mpu.setZGyroOffset(-1);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed.");
  }
}

/*****************************************************************************
 * Function Name: get_imu
 * Description  : IMUデータの取得
 * Arguments    : none
 * Return Value : none
 *****************************************************************************/
void get_imu() {
  st_imu *stp_imu = &stg_imu;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(rpy, &q, &gravity);

    // 角度取得[rad]
    stp_imu->yaw = rpy[0];
    stp_imu->roll = rpy[2];
    // ANGOFFSの計算はここでしないとloopに入れるとバグる
    stp_imu->pitch = rpy[1] - ANGOFFS;

    // 角速度取得[rad/s]
    mpu.dmpGetGyro(&rpy_gyro, fifoBuffer);
    stp_imu->pitch_gyro = (float)rpy_gyro.y * 2.0 * PI / 360.0;
  }
}