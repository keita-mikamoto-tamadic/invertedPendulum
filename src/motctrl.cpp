#include <Wire.h>
#include <motctrl.h>
#include <userdefine.h>

#define STS_TIMEOUT 500

byte buffer[10];  // 10Byteバッファ

/* global */
st_motctrl stg_motctrl;
void MotSetup(void);
void MotAllTest(float torq, int id);
void MotTorqWrite(float torq, int motch);
void MotPosVelRead(int id);

/* static */
typedef struct {
  float past_pos1;
  float past_pos2;
  unsigned long past_time1;
  unsigned long past_time2;
  float delta_time;
} st_Prv;
static st_Prv sts_Prv;

static void STSWriteTorq(byte id, int torq);
static bool STSSendData(byte arr[], int len);
static bool STSReciveData(HardwareSerial *serial_port, byte *buffer,
                          int byteCount, unsigned long timeout, int id);
static byte STSCheckSum(byte arr[], int len);
static void STSReverseDir(int id);
static void STSReqPos(int id);
static void STSCalcVel(int id);

void MotSetup() {
  st_Prv *stp_Prv = &sts_Prv;

  stp_Prv->past_pos1 = 0;
  stp_Prv->past_pos2 = 0;
  stp_Prv->past_time1 = 0;
  stp_Prv->past_time2 = 0;
  stp_Prv->delta_time = 0;

  Serial1.begin(1000000);
}

void MotAllTest(float torq, int id) {
  st_Prv *stp_Prv = &sts_Prv;
  st_motctrl *stp_motctrl = &stg_motctrl;

  STSReqPos(id);

  STSWriteTorq(id, torq);
}

/* リミットトルクは0.441Nm */
/* 0.0515Nm以下はモータ動作不可なのでトルク0とする */
void MotTorqWrite(float torq, int id) {
  static int conv_torq = 0;

  /* トルクをSTS3032の指令値へ変換 */
  /* 2439 * トルク （Nm）+ 50 */
  /*     if (torq > 0)
      {
        conv_torq = (int)(MOTCURGAIN * torq + 50);
      }
      else if(torq < 0)
      {
        conv_torq = (int)(MOTCURGAIN * torq - 50);
      }
      else conv_torq = 0; */

  conv_torq = (int)(MOTCURGAIN * torq);

  /* 飽和 */
  if (conv_torq >= 1000) conv_torq = 1000;
  if (conv_torq <= -1000) conv_torq = -1000;
  /* モータへ書き込み */
  STSWriteTorq((byte)id, conv_torq);
}

void MotPosVelRead(int id) {
  st_Prv *stp_Prv = &sts_Prv;
  st_motctrl *stp_motctrl = &stg_motctrl;

  /* 位置データ要求 */
  STSReqPos(id);

  /* 後退差分のための時間算出 */
  if (id == 1) {
    stp_Prv->delta_time = (float)((micros() - stp_Prv->past_time1) * 0.000001);
  } else if (id == 2) {
    stp_Prv->delta_time = (float)((micros() - stp_Prv->past_time2) * 0.000001);
  }

  stp_motctrl->test = stp_Prv->delta_time;
  /* 角速度算出 */
  STSCalcVel(id);

  /* 時間保存 */
  if (id == 1) {
    stp_Prv->past_time1 = micros();
  } else if (id == 2) {
    stp_Prv->past_time2 = micros();
  }
}

void STSWriteTorq(byte id, int torq) {
  // トルクが負の時は、
  if (torq < 0) {
    torq = -torq;
    torq |= (1 << 10);
  }
  // コマンドパケットを作成
  // https://akizukidenshi.com/goodsaffix/feetech_digital_servo_20220729.pdf
  byte message[13];
  message[0] = 0xFF;                // ヘッダ
  message[1] = 0xFF;                // ヘッダ
  message[2] = id;                  // サーボID
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

/* リクエストデータの生成と送信 */
void STSReqPos(int id) {
  byte message[8];                       // コマンドパケットを作成
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
  STSReciveData(&Serial1, buffer, 8, STS_TIMEOUT, id);
  /* delay入れないとバグる */
  // delayMicroseconds(50);
}

/* コマンド送受信は遅いので速度は計算 */
void STSCalcVel(int id) {
  st_Prv *stp_Prv = &sts_Prv;
  st_motctrl *stp_motctrl = &stg_motctrl;

  if (id == MOT1) {
    stp_motctrl->actVel_1 =
        (stp_motctrl->actPos_1 - stp_Prv->past_pos1) / stp_Prv->delta_time;
    stp_Prv->past_pos1 = stp_motctrl->actPos_1;
  } else if (id == MOT2) {
    stp_motctrl->actVel_2 =
        (stp_motctrl->actPos_2 - stp_Prv->past_pos2) / stp_Prv->delta_time;
    stp_Prv->past_pos2 = stp_motctrl->actPos_2;
  }
}

bool STSSendData(byte arr[], int len) {
  for (int i = 0; i < len; i++) {
    Serial1.write(arr[i]);
  }
  Serial1.flush();  // 送信完了待ち
}

bool STSReciveData(HardwareSerial *serial_port, byte *buffer, int byteCount,
                   unsigned long timeout, int id) {
  static int temp_ang = 0;
  st_motctrl *stp_motctrl = &stg_motctrl;

  int receivedBytes = 0;
  unsigned long startTime = micros();

  // 指定されたバイト数が来るまで待つ
  while (receivedBytes < byteCount + 2 - 2) {
    if (serial_port->available()) {
      buffer[receivedBytes] =
          (byte)serial_port->read();  // 指定されたシリアルポートを使用
      receivedBytes++;
    }
    // タイムアウトチェック
    if (micros() - startTime > timeout) {
      stp_motctrl->actPos_2 = 0;
      return false;  // タイムアウト
    }
  }

  temp_ang = int(buffer[6]) * 256 + int(buffer[5]);
  // 4095分解能をradへ変換
  if (id == 1) {
    stp_motctrl->actPos_1 = 2 * PI * (float(temp_ang) / MOTRESOL);
  } else if (id == 2) {
    // 方向反転
    stp_motctrl->actPos_2 = 2 * PI * (float(~(temp_ang) + MOTRESOL) / MOTRESOL);
  }

  return true;
}

byte STSCheckSum(byte arr[], int len) {
  int checksum = 0;
  for (int i = 2; i < len - 1; i++) {
    checksum += arr[i];
  }
  return ~((byte)(checksum & 0xFF));  // チェックサム
}

void STSReverseDir(int id) {
  // コマンドパケットを作成
  // https://akizukidenshi.com/goodsaffix/feetech_digital_servo_20220729.pdf
  byte message[13];
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