/*******************************************************************************
 * File Name    : lqrctrl.cpp
 * Description  : LQR制御の実装
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2024.07.19 1.00 K.Mikamoto 新設
 *****************************************************************************/

#include <imu.h>
#include <lqrctrl.h>
#include <motctrl.h>
#include <userdefine.h>

/*******************************************************************************
 * Global variables and functions
 *****************************************************************************/
void LQRcontrol(float pitch, float pitch_gyro, float pos, float vel);

// structure defined in header file to give global scope
// typedef struct {
//   float refTorq;
// } st_lqr;
st_lqr stg_lqr;

/******************************************************************************
 * Function Name: LQRcontrol
 * Description  : LQR制御を実行する
 * Arguments    : pitch - ピッチ角度
 *                pitch_gyro - ピッチ角速度
 *                pos - 位置
 *                vel - 速度
 * Return Value : none
 *****************************************************************************/
void LQRcontrol(float pitch, float pitch_gyro, float pos, float vel) {
  st_lqr *stp_lqr = &stg_lqr;

  stp_lqr->refTorq =
      -((F1LQR * pitch) + (F2LQR * pitch_gyro) + (F3LQR * pos) + (F4LQR * vel));
}