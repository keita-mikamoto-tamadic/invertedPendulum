#include <Arduino.h>
#include <FspTimer.h>
#include <imu.h>
#include <userdefine.h>
#include <motctrl.h>
#include <lqrctrl.h>

static FspTimer fsp_timer;

void timer_callback([[maybe_unused]]timer_callback_args_t *arg);


void setup()
{
    Serial.begin(9600);

    /* 割り込み用タイマーの設定(現在未使用) */
    uint8_t timer_type;
    int8_t timer_ch = FspTimer::get_available_timer(timer_type);
    if (timer_ch < 0)
    {
        Serial.println("timer setup failed");
        return;
    }
    fsp_timer.begin(TIMER_MODE_PERIODIC, timer_type, static_cast<uint8_t>(timer_ch), 100.0, 0.0, timer_callback, nullptr);
    fsp_timer.setup_overflow_irq();
    fsp_timer.open();
    fsp_timer.start();
    /* ********************************* */

    /* Motor setup function */
    MotSetup();
    imu_setup();
    

    delay(2000);                            //いきなりサーボ入るので、少し長めに待つ
    Serial.println("setup complete");
}

void timer_callback([[maybe_unused]]timer_callback_args_t *arg)
{ 

}

void loop()
{
    // 時間計測用
    float start, end;
    // start = micros();
    // end = micros();
    static bool debug = true;

    
    st_imu *stp_imu = &stg_imu;
    st_lqr *stp_lqr = &stg_lqr;
    st_motctrl * stp_motctrl = &stg_motctrl;

    // IMU:角度・角速度取得
    get_imu();
    start = micros();
    
    // STS3032:位置・角速度取得
    MotPosVelRead(MOT1);
    MotPosVelRead(MOT2);
    
    // トルク計算
    LQRcontrol(stp_imu->pitch, stp_imu->pitch_gyro, stp_motctrl->actPos_1, stp_motctrl->actVel_1);
    
    // IMU用の不感帯
    if (stp_imu->pitch > -0.02 && stp_imu->pitch < 0.02) stp_lqr->refTorq = 0;
    
    // モータートルク印可
    MotTorqWrite(stp_lqr->refTorq,1);
    MotTorqWrite(-(stp_lqr->refTorq),2);
    
    //MotAllTest(500,2);
    end = micros();

    if (debug == true)
    {
        Serial.print(stp_motctrl->actPos_1);
        Serial.print(" ; ");
        Serial.print(String(stp_motctrl->actVel_2, 6));       
        Serial.print(" ; ");
        Serial.print(stp_imu->pitch);
        Serial.print(" ; ");
        Serial.print(String(stp_imu->pitch_gyro, 6));       
        Serial.print(" ; ");
        Serial.println(end-start);
    }
}
