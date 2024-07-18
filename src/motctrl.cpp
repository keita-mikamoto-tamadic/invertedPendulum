#include <motctrl.h>
#include <SCServo.h>
#include <Wire.h>
#include <userdefine.h>

//SMS_STS st_stsmotor;

void MotSetup(void);
void MotTorqWrite(float torq, int motch);
void STSPackCom(byte id, int torq);

void MotSetup()
{
    Serial1.begin(1000000);
}

/* リミットトルクは0.441Nm */
/* 0.0515Nm以下はモータ動作不可なのでトルク0とする */
void MotTorqWrite(float torq, int motch)
{
    static int conv_torq = 0;

    /* トルクをSTS3032の指令値へ変換 */
    /* 2439 * トルク （Nm）+ 50 */
    conv_torq = (int)(MOTCURGAIN * torq);
    
    /* 飽和 */
    if (conv_torq >= 1000) conv_torq = 1000;
    if (conv_torq <= -1000) conv_torq = -1000;
    /* モータへ書き込み */
    STSPackCom((byte)motch, conv_torq);
    
}

void STSPackCom(byte id, int torq)
{
	if(torq<0){
		torq = -torq;
		torq |= (1<<10);
	}
      // コマンドパケットを作成
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

      // チェックサムの計算
      byte checksum = 0;
      for (int i = 2; i < 12; i++) {
        checksum += message[i];
      }
      message[12] = ~checksum; // チェックサム

      // コマンドパケットを送信
      for (int i = 0; i < 13; i++) {
        Serial1.write(message[i]);
    }
}