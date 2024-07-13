#include <Arduino.h>
#include <SCServo.h>
#include <imu.h>
#include <userdefine.h>
#include <motctrl.h>

SMS_STS st;

void setup()
{
    Serial.begin(9600);
    /* Motor setup function */
    //MotSetup();
    imu_setup();

    delay(5000);
}

void loop()
{
    static float pitch = 0;

    st_imu *stp_imu = &stg_imu;

    get_imu();
    
    Serial.println(stp_imu->pitch);
    delay(300);
    

/*
    st_imu *stp_imu = &stg_imu;

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
*/
}
