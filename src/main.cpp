/*******************************************************************************
 * File Name    : main.cpp
 * Description  : メインプログラムの実装
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2024.04.04 1.00 K.Mikamoto 新設
 *****************************************************************************/

#include <Arduino.h>
#include <FspTimer.h>
#include <imu.h>
#include <lqrctrl.h>
#include <motctrl.h>
#include <userdefine.h>

static FspTimer fsp_timer;

/*******************************************************************************
 * Global variables and functions
 *****************************************************************************/
void timer_callback([[maybe_unused]] timer_callback_args_t *arg);

/******************************************************************************
 * Function Name: setup
 * Description  : システムのセットアップ
 * Arguments    : none
 * Return Value : none
 *****************************************************************************/
void setup() {
  Serial.begin(9600);

  // 割り込み用タイマーの設定(現在未使用)
  uint8_t timer_type;
  int8_t timer_ch = FspTimer::get_available_timer(timer_type);
  if (timer_ch < 0) {
    Serial.println("timer setup failed");
    return;
  }
  fsp_timer.begin(TIMER_MODE_PERIODIC, timer_type,
                  static_cast<uint8_t>(timer_ch), 100.0, 0.0, timer_callback,
                  nullptr);
  fsp_timer.setup_overflow_irq();
  fsp_timer.open();
  fsp_timer.start();

  // Motor setup function
  MotSetup(MOTID_1);
  MotSetup(MOTID_2);
  StartSerial1();
  imu_setup();

  delay(2000);  // いきなりサーボ入るので、少し長めに待つ(2000 [ms])
  Serial.println("setup complete");
}

/******************************************************************************
 * Function Name: timer_callback
 * Description  : タイマー割り込みコールバック関数
 * Arguments    : arg - タイマーコールバック引数
 * Return Value : none
 *****************************************************************************/
void timer_callback([[maybe_unused]] timer_callback_args_t *arg) {}

/******************************************************************************
 * Function Name: loop
 * Description  : メインループ処理
 * Arguments    : none
 * Return Value : none
 *****************************************************************************/
void loop() {
  st_imu *stp_imu = &stg_imu;

  // USBシリアル通信用
  static bool debug = true;

  // 時間計測用
  float start, end;
  // 計測したい区間を下記処理で挟み、end -start の結果が実行時間
  // start = micros();
  // end = micros();

  start = micros();

  // IMU:角度・角速度取得
  get_imu();

  // STS3032:位置・角速度取得
  MotPosVelRead(MOTID_1);
  MotPosVelRead(MOTID_2);

  // トルク計算
  LQRcontrol(MOTID_1);
  LQRCtrlTrqRef(MOTID_2);

  // モータートルク印可
  MotTorqWrite(MOTID_1);
  MotTorqWrite(MOTID_2);

  end = micros();

  // MotAllTest(500,2);

  if (debug == true) {
    Serial.print("act_pos_1: ");
    Serial.print(stg_motctrl[MOTID_1].act_pos);
    Serial.print(" [rad], act_pos_2: ");
    Serial.print(stg_motctrl[MOTID_2].act_pos);
    Serial.print(" [rad], act_vel_1: ");
    Serial.print(stg_motctrl[MOTID_1].act_vel);
    Serial.print(" [rad/s], act_vel_2: ");
    Serial.print(stg_motctrl[MOTID_2].act_vel);
    Serial.print(" [rad/s], imu_pitch: ");
    Serial.print(stg_imu.pitch);
    Serial.print(" [rad], imu_pitch_gyro: ");
    Serial.print(stg_imu.pitch_gyro);
    Serial.print(" [rad/s], 処理時間: ");
    Serial.print(end - start);
    Serial.println(" [us]");
  }
}
