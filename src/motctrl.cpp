#include <motctrl.h>
#include <SCServo.h>
#include <Wire.h>
#include <userdefine.h>

SMS_STS st_stsmotor;

void MotSetup(void);
void MotTorqWrite(float torq, int motch);

void MotSetup()
{
    Serial1.begin(1000000);
    st_stsmotor.pSerial = &Serial1;
}

/* リミットトルクは0.441Nm */
/* 0.0515Nm以下はモータ動作不可なのでトルク0とする */
void MotTorqWrite(float torq, int motch)
{
    static int conv_torq = 0;
    long begin, end;

    /* トルクをSTS3032の指令値へ変換 */
    /* 2439 * トルク （Nm）+ 50 */
    conv_torq = (int)(MOTCURGAIN * torq);
    
    /* 飽和 */
    if (conv_torq >= 1000) conv_torq = 1000;
    if (conv_torq <= -1000) conv_torq = -1000;
    /* モータへ書き込み */
    begin = millis();
    st_stsmotor.WriteTorq(motch, conv_torq);
    end = millis();
    Serial.println(end-begin);
    
}