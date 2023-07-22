#ifndef CAPTEURS_SM_H
#define CAPTEURS_SM_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <ArduinoJson.h>

#include "HygrometerSensor.h"
#include "JY901.h"
#include "MS5837.h"
#include "TinyGPS.h"
#include "Ultrasonic.h"

class capteurs_sm
{
    public:
        capteurs_sm();
        capteurs_sm(int hygrometer_pin_analog,
                    int ultrason_trigger_pin_digital,
                    int ultrason_echo_pin_digital,
                    int tension_pin_analog);
        virtual ~capteurs_sm();

        void setup_init();
        
        void get_and_send_data();
        void send_data_global();
        void send_data(String key, float value);

        void get_data_GPS();
        void get_data_hygrometre();
        void get_data_debitmetre();
        void get_data_tension();
        void get_data_IMU();
        void get_data_pression();
        void get_data_ultrason();
        void get_data_temperature();

    protected:

    private:

        //SoftwareSerial *ss;
        eHygrometerKind kind;

        HygrometerSensor *Hygrometer;
        //JY901 *Imu;
        TinyGPS *Gps;
        MS5837 *Pressure;
        Ultrasonic *Ultrason;

        //GPS
        float GPS_array[1];

        //Hygrometre
        float hygro_array[1];

        //Debitmetre
        float debit_array[1];
        
        //Tension
        int voltageSensorPin;
        float vOUT = 0.0;
        float vIN = 0.0;
        float R1 = 30000.0;
        float R2 = 7500.0;

        int value;

        float ten_array[1];
        
        //IMU
        float imu_array[17];

        //Pression
        float press_array[3];

        //Ultrason
        float ultra_array[1];

        //Temperature
        float temp_array[1];

        //Sensors
        //float *sensors_array[8];

        //Disponibilites des capteurs : 1 : dispo ; 0 : non dispo
        int disp_imu;
        int disp_press;
        int disp_gps;
        int disp_hygro;
        int disp_debit;
        int disp_tension;
        int disp_ultra;
        int disp_temp;

        //Variables for JSON
        /*static constexpr const int capacity = JSON_OBJECT_SIZE(8) 
                                    + JSON_ARRAY_SIZE(17) + 2 * JSON_OBJECT_SIZE(2)
                                    + JSON_ARRAY_SIZE(3) + 2 * JSON_OBJECT_SIZE(2)
                                    + JSON_ARRAY_SIZE(1) + 2 * JSON_OBJECT_SIZE(2)
                                    + JSON_ARRAY_SIZE(1) + 2 * JSON_OBJECT_SIZE(2)
                                    + JSON_ARRAY_SIZE(1) + 2 * JSON_OBJECT_SIZE(2)
                                    + JSON_ARRAY_SIZE(1) + 2 * JSON_OBJECT_SIZE(2)
                                    + JSON_ARRAY_SIZE(1) + 2 * JSON_OBJECT_SIZE(2)
                                    + JSON_ARRAY_SIZE(1) + 2 * JSON_OBJECT_SIZE(2);*/

        static constexpr const int capacity = JSON_OBJECT_SIZE(4); 

};

#endif // CAPTEURS_SM_H