#include <Arduino.h>
#include <SCServo.h>
#include <imu.h>
#include <userdefine.h>

#define Gain 10

SMS_STS st;

float dt;
float pretime;

void setup()
{
    Serial1.begin(1000000);
    Serial.begin(115200);
    st.pSerial = &Serial1;
    delay(5000);
}

void loop()
{
    static float diffAng;
    st_imu *stp_imu = &stg_imu;
  
    static float refAng = 0;
    static float integral, div, preP= 0;
    static const float kp = 900;
    static const float ki = 0;
    static const float kd = 2;
    static int actPos, actCur;
    
    static float pretime_serial;
    
    static int counter = 0;
  
/*     if (micros() - pretime_serial > 10000)
    {
        Serial.print(input);
        pretime_serial = micros();
    } */

    if (counter < 1000){

        st.WriteTorq(1, 1000);
        actCur = st.ReadPos(1);
        
        Serial.print(actCur);
        Serial.print(" | ");
        Serial.print(int(micros()-pretime));
        Serial.print("\n");       

        pretime = micros();
        
        counter += 1;

    }else if (counter >= 1000 && counter < 2000){
        st.WriteTorq(1, -1000);
        actCur = st.ReadPos(1);
        
        Serial.print(actCur);
        Serial.print(" | ");
        Serial.print(int(micros()-pretime));
        Serial.print("\n");       

        pretime = micros();
        
        counter += 1;
    }else{
        Serial.end();
        st.WriteTorq(1, 0);
    }

}
