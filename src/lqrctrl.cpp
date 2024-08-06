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
void LQRcontrol(MotID_t id);
void LQRCtrlTrqRef(MotID_t id);

// structure defined in header file to give global scope
// typedef struct {
//   float ref_trq;
// } st_lqr;
st_lqr stg_lqr[kMotNum];

/******************************************************************************
 * Function Name: LQRcontrol
 * Description  : LQR制御を実行する
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
void LQRcontrol(MotID_t id) {
  st_lqr *stp_lqr = &stg_lqr[id];
  st_imu *stp_imu = &stg_imu;
  st_motctrl *stp_motctrl = &stg_motctrl[id];

  stp_lqr->ref_trq =
      -((kF1Lqr * stp_imu->pitch) + (kF2Lqr * stp_imu->pitch_gyro) +
        (kF3Lqr * stp_motctrl->act_pos) + (kF4Lqr * stp_motctrl->act_vel));

  // IMU用の不感帯
  if ((-0.02 < stp_imu->pitch) && (stp_imu->pitch < 0.02)) stp_lqr->ref_trq = 0;
}

/******************************************************************************
 * Function Name: LQRCtrlTrqRef
 * Description  : 指令値制御
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
void LQRCtrlTrqRef(MotID_t id) {
  switch (id) {
    case MOTID_1:
      // do nothing
      break;

    case MOTID_2:
      stg_lqr[MOTID_2].ref_trq = -stg_lqr[MOTID_1].ref_trq;  // 符号反転
      break;

    default:
      break;
  }
}