#include <Arduino.h>
#include <FspTimer.h>
#include <SCServo.h>
#include <imu.h>
#include <userdefine.h>
#include <motctrl.h>
#include <lqrctrl.h>

static FspTimer fsp_timer;

void timer_callback([[maybe_unused]]timer_callback_args_t *arg);

SMS_STS st;

void setup()
{
    Serial.begin(9600);

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

    /* Motor setup function */
    MotSetup();
    imu_setup();

    delay(3000);
    Serial.println("setup complete");
}

void timer_callback([[maybe_unused]]timer_callback_args_t *arg)
{ 

}

void loop()
{

    static float pitch_ang = 0;
    

    st_imu *stp_imu = &stg_imu;
    st_lqr *stp_lqr = &stg_lqr;

    get_imu();
    
    LQRcontrol(stp_imu->pitch);   
    
    //if (stp_imu->pitch > -0.01 && stp_imu->pitch < 0.01) stp_lqr->refTorq = 0;
    
    MotTorqWrite(stp_lqr->refTorq, 1);
    MotTorqWrite(-(stp_lqr->refTorq), 2);
    
    


/* 
    if (temp_torq > 1) temp_torq = 1;
    if (temp_torq < 1) temp_torq = -1;
    
    if (pitch_ang > 0)
    {
        WriteTorq(500, 1);
        WriteTorq(-500, 2);
    }else if(pitch_ang < 0)
    {
        WriteTorq(-500, 1);
        WriteTorq(500, 2);
    }
  */   
    
}
