#include "capteurs_sm.h"

//Arrays of key

//IMU
/*char a_x[] = "a_x";
char a_y[] = "a_y";
char a_z[] = "a_z";

char p[] = "p";
char q[] = "q";
char r[] = "r";

char roll[] = "roll";
char pitch[] = "pitch";
char yaw[] = "yaw";

char mag_x[] = "mag_x";
char mag_y[] = "mag_y";
char mag_z[] = "mag_z";

char press[] = "press";

char q1[] = "q1";
char q2[] = "q2";
char q3[] = "q3";
char q4[] = "q4";

char temp[] = "temp";
char depth[] = "depth";

char tension[] = "tension";
char hygro[] = "hygro";
char gps[] = "gps";
char debit[] = "debit";
char ultra[] = "ultra";

char imu_[] = {a_x, a_y, a_z, p, q, r, roll, pitch, yaw, mag_x, mag_y, mag_z, press, q1, q2, q3, q4};
char pressure_[] = {press, temp, depth};
char tension_[] = {tension};
char hygro_[] = {hygro};
char gps_[] = {gps};
char debit_[] = {debit};
char ultrason_[] = {ultra};
char temp_[] = {temp};
    
char sensors[] = {imu_, pressure_, tension_, hygro_, gps_, debit_, ultrason_, temp_};

char imu_c[] = "imu";
char pressure_c[] = "pressure";
char tension_c[] = "tension";
char hygro_c[] = "hygro";
char gps_c[] = "gps";
char debit_c[] = "debit";
char ultrason_c[] = "ultra";
char temp_c[] = "temp";

char sensors_headers[] = {imu_c, pressure_c, tension_c, hygro_c, gps_c, debit_c, ultrason_c, temp_c};*/

capteurs_sm::capteurs_sm()
{
    //ctor
}

capteurs_sm::capteurs_sm(int hygrometer_pin_analog,
                         int ultrason_trigger_pin_digital,
                         int ultrason_echo_pin_digital,
                         int tension_pin_analog)
{

    voltageSensorPin = tension_pin_analog;
    vOUT = 0.0;
    vIN = 0.0;
    R1 = 30000.0;
    R2 = 7500.0;

    this->kind = ANALOG;

    //this->ss = new SoftwareSerial(gps_pin_RX, gps_pin_TX); //pinRX, pinTX (serial)

    this->Hygrometer = new HygrometerSensor(kind, hygrometer_pin_analog); //entree analogique
    //this->Imu = new JY901(); //I2C : SDA, SCL
    this->Gps = new TinyGPS();
    this->Pressure = new MS5837(); //I2C : SDA, SCL
    this->Ultrason = new Ultrasonic(ultrason_trigger_pin_digital, ultrason_echo_pin_digital);

    //ss->begin(baudrate_ss);

    //Imu->StartIIC();
    //JY901.StartIIC();

    //initialisation des tableaux
    for(int i=0; i<sizeof(imu_array); i++)
        imu_array[i] = 0;
    for(int i=0; i<sizeof(press_array); i++)
        press_array[i] = 0;
    ten_array[0] = 0;
    ultra_array[0] = 0;
    temp_array[0] = 0;
    GPS_array[0] = 0;
    hygro_array[0] = 0;
    debit_array[0] = 0;

    /*sensors_array[0] = &imu_array;
    sensors_array[1] = &press_array;
    sensors_array[2] = &ten_array;
    sensors_array[3] = &hygro_array;
    sensors_array[4] = &GPS_array;
    sensors_array[5] = &debit_array;
    sensors_array[6] = &ultra_array;
    sensors_array[7] = &temp_array;*/

    disp_imu = 0;
    disp_press = 1;
    disp_gps = 0;
    disp_hygro = 0;
    disp_debit = 0;
    disp_tension = 0;
    disp_ultra = 0;
    disp_temp = 0;
}

capteurs_sm::~capteurs_sm()
{
    //dtor
    //delete ss;
    //ss = NULL;
    delete Hygrometer;
    Hygrometer = NULL;
    //delete Imu;
    //Imu = NULL;
    delete Gps;
    Gps = NULL;
    delete Pressure;
    Pressure = NULL;
    delete Ultrason;
    Ultrason = NULL;
    //delete sensors_array;
    //sensors_array = NULL;
}

//Fonction a mettre dans le setup de l'Arduino
void capteurs_sm::setup_init()
{
    //Serial.begin(230400);
    //Serial1.begin(9600);
    
    //Wire.begin();
    
    if (!Pressure->init())
    {
        Serial.println("Init failed!");
        Serial.println("Are SDA/SCL connected correctly?");
        Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
        Serial.println("\n\n\n");
        //delay(5000);
        disp_press = 0;
    }

    Pressure->setFluidDensity(1029); // kg/m^3 (997 for freshwater, 1029 for seawater)
}

void capteurs_sm::get_and_send_data()
{    
    
    if(disp_imu == 1)
        get_data_IMU(); //17
    if(disp_press == 1)
        get_data_pression();//3
    if(disp_ultra == 1) 
        get_data_ultrason(); //1
    if(disp_tension == 1)
        get_data_tension(); //1
    if(disp_hygro == 1)
        get_data_hygrometre(); //1
    if(disp_temp == 1)
        get_data_temperature(); //0
    if(disp_gps == 1)
        get_data_GPS(); //0
    if(disp_debit == 1)
        get_data_debitmetre(); //0

    send_data_global();

    /*StaticJsonDocument<capacity> doc;

    //IMU
    JsonObject obj_imu = doc.createNestedObject("imu");
    obj_imu["a_x"] = imu_array[0];
    obj_imu["a_y"] = imu_array[1];
    obj_imu["a_z"] = imu_array[2];
    obj_imu["p"] = imu_array[3];
    obj_imu["q"] = imu_array[4];
    obj_imu["r"] = imu_array[5];
    obj_imu["roll"] = imu_array[6];
    obj_imu["pitch"] = imu_array[7];
    obj_imu["yaw"] = imu_array[8];
    obj_imu["mag_x"] = imu_array[9];
    obj_imu["mag_y"] = imu_array[10];
    obj_imu["mag_z"] = imu_array[11];
    /*obj_imu["press"] = imu_array[12];
    obj_imu["q1"] = imu_array[13];
    obj_imu["q2"] = imu_array[14];
    obj_imu["q3"] = imu_array[15];
    obj_imu["q4"] = imu_array[16];*/

    //Pressure
    /*JsonObject obj_press = doc.createNestedObject("press");
    obj_press["press"] = press_array[0];
    obj_press["temp"] = press_array[1];
    obj_press["depth"] = press_array[2];*/

    //Tension
    /*JsonObject obj_ten = doc.createNestedObject("ten");
    obj_ten["tension"] = ten_array[0];*/
    
    //Hygrometre
    /*JsonObject obj_hygro = doc.createNestedObject("hygro");
    obj_hygro["hygro"] = hygro_array[0];*/
    
    //GPS
    /*JsonObject obj_GPS = doc.createNestedObject("gps");
    obj_GPS["gps"] = press_array[0];*/

    //Debit
    /*JsonObject obj_deb = doc.createNesensors_headers[indice] + "_" + sensors[indice][i]stedObject("debit");
    obj_deb["debit"] = debit_array[0];*/

    //Ultrason
    /*JsonObject obj_ultra = doc.createNestedObject("ultra");
    obj_ultra["ultra"] = ultra_array[0];*/

    //Temperature
    /*JsonObject obj_temp = doc.createNestedObject("temp");
    obj_temp["temp"] = temp_array[0];*/

    //serializeJson(doc, Serial);

    //Serial.println("");
    
}

void capteurs_sm::send_data_global()
{
    if(disp_imu == 1)
    {
        send_data("imu_a_x", imu_array[0]);
        send_data("imu_a_y", imu_array[1]);
        send_data("imu_a_z", imu_array[2]);
        send_data("imu_p", imu_array[3]);
        send_data("imu_q", imu_array[4]);
        send_data("imu_r", imu_array[5]);
        send_data("imu_roll", imu_array[6]);
        send_data("imu_pitch", imu_array[7]);
        send_data("imu_yaw", imu_array[8]);
        send_data("imu_mag_x", imu_array[9]);
        send_data("imu_mag_y", imu_array[10]);
        send_data("imu_mag_z", imu_array[11]);
    }

    if(disp_press == 1)
    {
        send_data("press_press", press_array[0]);
        send_data("press_temp", press_array[1]);
        send_data("press_depth", press_array[2]);
    }
}

void capteurs_sm::send_data(String key, float value)
{
    StaticJsonDocument<capacity> doc;

    doc[key] = value;
    
    /*for(int i=0; i<sizeof(sensors[indice]); i++)
    {
        float ptr = *sensors_array[indice];
        String key = (String)sensors_headers[indice] + "_" + (String)sensors[indice][i];
        doc[key] = *ptr[i];
    }*/

    serializeJson(doc, Serial);

    //Serial.println("");
}

void capteurs_sm::get_data_GPS()
{

}

void capteurs_sm::get_data_hygrometre() //longueur : 1
{
    hygro_array[0] = Hygrometer->readHumidityValue();
}

void capteurs_sm::get_data_debitmetre() // longueur : 0
{

}

void capteurs_sm::get_data_tension() // longueur : 1
{

    value = analogRead(voltageSensorPin);
    vOUT = (value * 5.0) / 1024.0;
    vIN = vOUT / (R2/(R1+R2));

    ten_array[0] = vIN;
}

//Mode I2C
/*void capteurs_sm::get_data_IMU() // longueur : 17
{

    JY901.GetAcc();
    imu_array[0] = (float)JY901.stcAcc.a[0]/32768*16;
    imu_array[1] = (float)JY901.stcAcc.a[1]/32768*16;
    imu_array[2] = (float)JY901.stcAcc.a[2]/32768*16;

    JY901.GetGyro();
    imu_array[3] = (float)JY901.stcGyro.w[0]/32768*2000;
    imu_array[4] = (float)JY901.stcGyro.w[1]/32768*2000;
    imu_array[5] = (float)JY901.stcGyro.w[2]/32768*2000;

    JY901.GetAngle();
    imu_array[6] = (float)JY901.stcAngle.Angle[0]/32768*180;
    imu_array[7] = (float)JY901.stcAngle.Angle[1]/32768*180;
    imu_array[8] = (float)JY901.stcAngle.Angle[2]/32768*180;

    JY901.GetMag();
    imu_array[9] = (float)JY901.stcMag.h[0];
    imu_array[10] = (float)JY901.stcMag.h[1];
    imu_array[11] = (float)JY901.stcMag.h[2];

    JY901.GetPress();
    imu_array[12] = (float)JY901.stcPress.lPressure;

    JY901.GetQuaternion();
    imu_array[13] = (float)JY901.stcQuater.q0;
    imu_array[14] = (float)JY901.stcQuater.q1;
    imu_array[15] = (float)JY901.stcQuater.q2;
    imu_array[16] = (float)JY901.stcQuater.q3;
}*/

//Mode Serial
void capteurs_sm::get_data_IMU() // longueur : 17
{   
    imu_array[0] = (float)JY901.stcAcc.a[0]/32768*16;
    imu_array[1] = (float)JY901.stcAcc.a[1]/32768*16;
    imu_array[2] = (float)JY901.stcAcc.a[2]/32768*16;

    imu_array[3] = (float)JY901.stcGyro.w[0]/32768*2000;
    imu_array[4] = (float)JY901.stcGyro.w[1]/32768*2000;
    imu_array[5] = (float)JY901.stcGyro.w[2]/32768*2000;

    imu_array[6] = (float)JY901.stcAngle.Angle[0]/32768*180;
    imu_array[7] = (float)JY901.stcAngle.Angle[1]/32768*180;
    imu_array[8] = (float)JY901.stcAngle.Angle[2]/32768*180;

    imu_array[9] = (float)JY901.stcMag.h[0];
    imu_array[10] = (float)JY901.stcMag.h[1];
    imu_array[11] = (float)JY901.stcMag.h[2];

    imu_array[12] = (float)JY901.stcPress.lPressure;

    imu_array[13] = (float)JY901.stcQuater.q0;
    imu_array[14] = (float)JY901.stcQuater.q1;
    imu_array[15] = (float)JY901.stcQuater.q2;
    imu_array[16] = (float)JY901.stcQuater.q3;
}

void capteurs_sm::get_data_pression() //longueur : 3
{
    Pressure->read();
    press_array[0] = Pressure->pressure();
    press_array[1] = Pressure->temperature();
    press_array[2] = Pressure->depth();
}

void capteurs_sm::get_data_ultrason() // longueur : 1
{
    ultra_array[0] = Ultrason->read() * 0.01;//sortie en cm par defaut
}

void capteurs_sm::get_data_temperature() // longueur : 0
{

}

//Serial event sur le port serial 1 de l'Arduino Mega pour recuperer les donnees de l'IMU
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
/*void serialEvent1() 
{
  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
}*/
