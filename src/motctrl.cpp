#include <motctrl.h>
#include <SCServo.h>
#include <Wire.h>

SMS_STS st_stsmotor;

void MotSetup(void);
void WriteTorq(float torq, int motch);

void MotSetup()
{
    Serial1.begin(1000000);
    st_stsmotor.pSerial = &Serial1;
}

/* リミットトルクは0.441Nm */
/* 0.0515Nm以下はモータ動作不可なのでトルク0とする */
void WriteTorq(float torq, int motch)
{
    static int conv_torq = 0;
    
    /* トルクをSTS3032の指令値へ変換 */
    /* 2439 * トルク （Nm）+ 50 */
    conv_torq = (int)(2439.0 * torq + 50);
    
    /* モータへ書き込み */
    st_stsmotor.WriteTorq(motch, conv_torq);
    
}