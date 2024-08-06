/*******************************************************************************
 * File Name    : motctrl.cpp
 * Description  : モータ制御の実装
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2024.07.14 1.00 K.Mikamoto 新設
 *****************************************************************************/

#include <Wire.h>
#include <lqrctrl.h>
#include <motctrl.h>
#include <userdefine.h>

/******************************************************************************
 * Macro definitions
 *****************************************************************************/

/*******************************************************************************
 * Global variables and functions
 *****************************************************************************/
void MotSetup(MotID_t id);
void StartSerial1(void);
void MotAllTest(float torq, MotID_t id);
void MotTorqWrite(MotID_t id);
void MotPosVelRead(MotID_t id);

// structure defined in header file to give global scope
// typedef struct {
//   float act_pos;
//   float act_trq;
//   float act_vel;
// } st_motctrl;
st_motctrl stg_motctrl[kMotNum];

/******************************************************************************
 * File Scope variables and functions
 *****************************************************************************/
static void STSWriteTorq(MotID_t id, int torq);
static bool STSSendData(byte arr[], int len);
static bool STSReciveData(HardwareSerial *serial_port, byte *buffer,
                          int byteCount, unsigned long timeout, MotID_t id);
static byte STSCheckSum(byte arr[], int len);
static void STSReverseDir(MotID_t id);
static void STSReqPos(MotID_t id);
static void STSCalcVel(MotID_t id);

byte buffer[10];  // 10Byteバッファ

typedef struct {
  float delta_time;
  float past_pos;
  unsigned long past_time;
} st_Prv;
static st_Prv sts_Prv[kMotNum];

/******************************************************************************
 * Function Name: MotSetup
 * Description  : モータ制御システムのセットアップ
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
void MotSetup(MotID_t id) {
  st_Prv *stp_Prv = &sts_Prv[id];

  stp_Prv->delta_time = 0.0f;
  stp_Prv->past_pos = 0.0f;
  stp_Prv->past_time = 0;
}

/******************************************************************************
 * Function Name: MotSetup
 * Description  : シリアル通信開始
 * Arguments    : none
 * Return Value : none
 *****************************************************************************/
void StartSerial1(void) { Serial1.begin(1000000); }

/******************************************************************************
 * Function Name: MotAllTest
 * Description  : 指定したトルクで全モータをテストする
 * Arguments    : torq - トルク値
 *                id - モータID
 * Return Value : none
 *****************************************************************************/
void MotAllTest(float torq, MotID_t id) {
  STSReqPos(id);

  STSWriteTorq(id, torq);
}

/******************************************************************************
 * Function Name: MotTorqWrite
 * Description  : 指定したモータにトルクを書き込む
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
// リミットトルクは0.441Nm
// 0.0515Nm以下はモータ動作不可なのでトルク0とする
void MotTorqWrite(MotID_t id) {
  st_lqr *stp_lqr = &stg_lqr[id];
  static int conv_trq = 0;

  // トルクをSTS3032の指令値へ変換
  // トルク = 2439 * トルク [Nm]
  conv_trq = (int)(kMotCurGain * stp_lqr->ref_trq);

  // 飽和
  if (conv_trq >= 1000) conv_trq = 1000;
  if (conv_trq <= -1000) conv_trq = -1000;

  // モータへ書き込み
  STSWriteTorq(id, conv_trq);
}

/******************************************************************************
 * Function Name: MotPosVelRead
 * Description  : 位置・速度の算出
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
void MotPosVelRead(MotID_t id) {
  st_Prv *stp_Prv = &sts_Prv[id];
  st_motctrl *stp_motctrl = &stg_motctrl[id];

  // 位置データ要求
  STSReqPos(id);

  // 後退差分のための時間算出
  stp_Prv->delta_time = (float)((micros() - stp_Prv->past_time) * 0.000001);

  // 角速度算出
  STSCalcVel(id);

  // 時間保存
  stp_Prv->past_time = micros();
}

/******************************************************************************
 * Function Name: STSWriteTorq
 * Description  : モータにトルクを書き込む
 * Arguments    : id - モータID
 *                torq - トルク値
 * Return Value : none
 *****************************************************************************/
void STSWriteTorq(MotID_t id, int torq) {
  byte message[13];

  // トルクが負の時は、方向ビットを設定
  if (torq < 0) {
    torq = -torq;
    torq |= (1 << 10);
  }

  // コマンドパケットを作成
  // https://akizukidenshi.com/goodsaffix/feetech_digital_servo_20220729.pdf
  message[0] = 0xFF;                // ヘッダ
  message[1] = 0xFF;                // ヘッダ
  message[2] = (byte)id;            // サーボID
  message[3] = 9;                   // パケットデータ長
  message[4] = 3;                   // コマンド（3は書き込み命令）
  message[5] = 42;                  // レジスタ先頭番号
  message[6] = 0x00;                // 位置情報バイト下位
  message[7] = 0x00;                // 位置情報バイト上位
  message[8] = torq & 0xFF;         // 時間情報バイト下位
  message[9] = (torq >> 8) & 0xFF;  // 時間情報バイト上位
  message[10] = 0x00;               // 速度情報バイト下位
  message[11] = 0x00;               // 速度情報バイト上位
  message[12] = STSCheckSum(message, 13);

  STSSendData(message, 13);
}

/******************************************************************************
 * Function Name: STSReqPos
 * Description  : モータから位置データを要求する
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
void STSReqPos(MotID_t id) {
  const int kStsTimeOut = 500;
  byte message[8];  // コマンドパケットを作成

  message[0] = 0xFF;                     // ヘッダ
  message[1] = 0xFF;                     // ヘッダ
  message[2] = (byte)id;                 // サーボID
  message[3] = 4;                        // パケットデータ長
  message[4] = 2;                        // コマンド
  message[5] = 56;                       // レジスタ先頭番号
  message[6] = 2;                        // 読み込みバイト数
  message[7] = STSCheckSum(message, 8);  // チェックサム

  // コマンドパケットを送信
  STSSendData(message, 8);

  // リクエストデータの受信完了まで待つ
  STSReciveData(&Serial1, buffer, 8, kStsTimeOut, id);
  // delay入れないとバグる
  // delayMicroseconds(50);
}

/******************************************************************************
 * Function Name: STSCalcVel
 * Description  : 速度の算出
 *                コマンド送受信は遅いので速度は計算
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
void STSCalcVel(MotID_t id) {
  st_Prv *stp_Prv = &sts_Prv[id];
  st_motctrl *stp_motctrl = &stg_motctrl[id];
  float delta_pos = 0.0f;

  delta_pos = stp_motctrl->act_pos - stp_Prv->past_pos;

  // 0またぎの処理
  if (delta_pos > USER_2PI) {
    delta_pos -= USER_2PI;
  } else if (delta_pos < -USER_2PI) {
    delta_pos += USER_2PI;
  }

  // 速度の計算
  stp_motctrl->act_vel = delta_pos / stp_Prv->delta_time;

  // 前回値の保存
  stp_Prv->past_pos = stp_motctrl->act_pos;
}

/******************************************************************************
 * Function Name: STSSendData
 * Description  : モータへデータを送信する
 * Arguments    : arr - 送信バッファ
 *                len - 送信データ長さ
 * Return Value : true - 送信成功
 *****************************************************************************/
bool STSSendData(byte arr[], int len) {
  for (int i = 0; i < len; i++) {
    Serial1.write(arr[i]);
  }

  Serial1.flush();  // 送信完了待ち

  return true;
}

/******************************************************************************
 * Function Name: STSReciveData
 * Description  : モータからデータを受信する
 * Arguments    : serial_port - シリアルポート
 *                buffer - 受信バッファ
 *                byteCount - 受信するバイト数
 *                timeout - 受信のタイムアウト
 *                id - モータID
 * Return Value : true - データ受信成功
 *                false - データ受信失敗
 *****************************************************************************/
bool STSReciveData(HardwareSerial *serial_port, byte *buffer, int byteCount,
                   unsigned long timeout, MotID_t id) {
  st_motctrl *stp_motctrl = &stg_motctrl[id];
  static int temp_ang = 0;
  unsigned long startTime = micros();
  int receivedBytes = 0;

  // 指定されたバイト数が来るまで待つ
  while (receivedBytes < byteCount + 2 - 2) {
    if (serial_port->available()) {
      buffer[receivedBytes] =
          (byte)serial_port->read();  // 指定されたシリアルポートを使用
      receivedBytes++;
    }
    // タイムアウトチェック
    if (micros() - startTime > timeout) {
      stg_motctrl[MOTID_2].act_pos = 0;
      return false;  // タイムアウト
    }
  }

  temp_ang = int(buffer[6]) * 256 + int(buffer[5]);

  // 4095分解能をradへ変換
  switch (id) {
    case MOTID_1:
      stp_motctrl->act_pos = USER_2PI * (float(temp_ang) / kMotResol);
      break;

    case MOTID_2:
      // 方向反転
      stp_motctrl->act_pos =
          USER_2PI * (float(~(temp_ang) + kMotResol) / kMotResol);
      break;
    default:
      break;
  }

  return true;
}

/******************************************************************************
 * Function Name: STSCheckSum
 * Description  : データのチェックサムを計算する
 * Arguments    : arr - データ配列
 *                len - データ長さ
 * Return Value : 計算されたチェックサム
 *****************************************************************************/
byte STSCheckSum(byte arr[], int len) {
  int checksum = 0;

  for (int i = 2; i < len - 1; i++) {
    checksum += arr[i];
  }

  return ~((byte)(checksum & 0xFF));  // チェックサム
}

/******************************************************************************
 * Function Name: STSReverseDir
 * Description  : モータの方向を反転する
 * Arguments    : id - モータID
 * Return Value : none
 *****************************************************************************/
void STSReverseDir(MotID_t id) {
  byte message[13];

  // コマンドパケットを作成
  // https://akizukidenshi.com/goodsaffix/feetech_digital_servo_20220729.pdf
  message[0] = 0xFF;      // ヘッダ
  message[1] = 0xFF;      // ヘッダ
  message[2] = (byte)id;  // サーボID
  message[3] = 9;         // パケットデータ長
  message[4] = 3;         // コマンド（3は書き込み命令）
  message[5] = 31;        // レジスタ先頭番号
  message[6] = 0x00;      // 位置情報バイト下位
  message[7] = 0x08;      // 位置情報バイト上位
  message[8] = 0x00;      // 時間情報バイト下位
  message[9] = 0x00;      // 時間情報バイト上位
  message[10] = 0x00;     // 速度情報バイト下位
  message[11] = 0x00;     // 速度情報バイト上位
  message[12] = STSCheckSum(message, 13);

  STSSendData(message, 13);
}