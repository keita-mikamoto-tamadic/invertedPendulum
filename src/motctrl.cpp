#include <motctrl.h>
#include <SCServo.h>
#include <Wire.h>
#include <userdefine.h>

#define STS_TIMEOUT 1000

byte buffer[10]; //10Byteバッファ

/* global */
st_motctrl stg_motctrl;
void MotSetup(void);
void MotTorqWrite(float torq, int motch);
void STSReqPos(int id);

/* static */
static void STSWriteTorq(byte id, int torq);
static bool STSSendData(byte arr[], int len);
static bool STSReciveData(HardwareSerial *serial_port, byte * buffer, int byteCount, unsigned long timeout, int id);
static byte STSCheckSum(byte arr[], int len);
static void STSReverseDir(int id);


void MotSetup()
{
    Serial1.begin(1000000);
}

/* リミットトルクは0.441Nm */
/* 0.0515Nm以下はモータ動作不可なのでトルク0とする */
void MotTorqWrite(float torq, int id)
{
    static int conv_torq = 0;

    /* トルクをSTS3032の指令値へ変換 */
    /* 2439 * トルク （Nm）+ 50 */
    conv_torq = (int)(MOTCURGAIN * torq);
    
    /* 飽和 */
    if (conv_torq >= 1000) conv_torq = 1000;
    if (conv_torq <= -1000) conv_torq = -1000;
    /* モータへ書き込み */
    STSWriteTorq((byte)id, conv_torq);
    
}

void STSWriteTorq(byte id, int torq)
{
    // トルクが負の時は、
    if(torq<0){
      torq = -torq;
      torq |= (1<<10);
    }
    // コマンドパケットを作成 https://akizukidenshi.com/goodsaffix/feetech_digital_servo_20220729.pdf
    byte message[13];
    message[0] = 0xFF;  // ヘッダ
    message[1] = 0xFF;  // ヘッダ
    message[2] = id;    // サーボID
    message[3] = 9;     // パケットデータ長
    message[4] = 3;     // コマンド（3は書き込み命令）
    message[5] = 42;    // レジスタ先頭番号
    message[6] = 0x00; // 位置情報バイト下位
    message[7] = 0x00; // 位置情報バイト上位
    message[8] = torq & 0xFF;  // 時間情報バイト下位
    message[9] = (torq >> 8) & 0xFF;  // 時間情報バイト上位
    message[10] = 0x00; // 速度情報バイト下位
    message[11] = 0x00; // 速度情報バイト上位
    message[12] = STSCheckSum(message, 13);

    STSSendData(message, 13);
}

/* リクエストデータの生成と送信 */
void STSReqPos(int id)
{
    byte message[8];   // コマンドパケットを作成
    message[0] = 0xFF; // ヘッダ
    message[1] = 0xFF; // ヘッダ
    message[2] = (byte)id;   // サーボID
    message[3] = 4;    // パケットデータ長
    message[4] = 2;    // コマンド
    message[5] = 56;   // レジスタ先頭番号
    message[6] = 2;    // 読み込みバイト数
    message[7] = STSCheckSum(message, 8) ; // チェックサム
  
    // コマンドパケットを送信
    STSSendData(message, 8);
    
    // リクエストデータの受信完了まで
    STSReciveData(&Serial1, buffer, 8, STS_TIMEOUT, id);
  
}

bool STSSendData(byte arr[], int len)
{
    for (int i = 0; i < len; i++) 
    {
      Serial1.write(arr[i]);
    }
    Serial1.flush(); //送信完了待ち
}

bool STSReciveData(HardwareSerial *serial_port, byte * buffer, int byteCount, unsigned long timeout, int id)
{
    static int temp_ang = 0;
    st_motctrl *stp_motctrl = &stg_motctrl;

    int receivedBytes = 0;
    unsigned long startTime = micros();

    // 指定されたバイト数が来るまで待つ
    while (receivedBytes < byteCount + 2 - 2) {
      if (serial_port->available()) {
        buffer[receivedBytes] = (byte)serial_port->read(); // 指定されたシリアルポートを使用
        receivedBytes++;
      }
      // タイムアウトチェック
      if (micros() - startTime > timeout) {
        return false; // タイムアウト
      }
    }

    temp_ang = int(buffer[6]) * 256 + int(buffer[5]);
    // 4095分解能をradへ変換
    if (id == 1)
    {
      stp_motctrl->actPos_1 = 2 * PI *(float(temp_ang) / MOTRESOL);
    }
    else if (id == 2)
    { 
      // 方向反転
      stp_motctrl->actPos_2 = 2 * PI *(float(~(temp_ang) + MOTRESOL) / MOTRESOL);
    }

    return true;
}

byte STSCheckSum(byte arr[], int len)
{
    int checksum = 0;
    for (int i = 2; i < len - 1; i++) {
      checksum += arr[i];
    }
    return ~((byte)(checksum & 0xFF)); // チェックサム
}

void STSReverseDir(int id)
{
    // コマンドパケットを作成 https://akizukidenshi.com/goodsaffix/feetech_digital_servo_20220729.pdf
    byte message[13];
    message[0] = 0xFF;  // ヘッダ
    message[1] = 0xFF;  // ヘッダ
    message[2] = (byte)id;    // サーボID
    message[3] = 9;     // パケットデータ長
    message[4] = 3;     // コマンド（3は書き込み命令）
    message[5] = 31;    // レジスタ先頭番号
    message[6] = 0x00; // 位置情報バイト下位
    message[7] = 0x08; // 位置情報バイト上位
    message[8] = 0x00;  // 時間情報バイト下位
    message[9] = 0x00;  // 時間情報バイト上位
    message[10] = 0x00; // 速度情報バイト下位
    message[11] = 0x00; // 速度情報バイト上位
    message[12] = STSCheckSum(message, 13);

    STSSendData(message, 13);
}