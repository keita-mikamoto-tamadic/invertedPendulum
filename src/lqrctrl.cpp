#include <imu.h>
#include <lqrctrl.h>
#include <motctrl.h>
#include <userdefine.h>

st_lqr stg_lqr;

void LQRcontrol(float pitch, float pitch_gyro, float pos, float vel);

void LQRcontrol(float pitch, float pitch_gyro, float pos, float vel) {
  st_lqr *stp_lqr = &stg_lqr;

  stp_lqr->refTorq =
      -((F1LQR * pitch) + (F2LQR * pitch_gyro) + (F3LQR * pos) + (F4LQR * vel));
}