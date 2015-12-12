/*************************************************************************
 * Dragonfly Example program 
 *
 * The following hardware is required to successfully run this program:
 *   - MultiTech UDK2 (4" square white PCB with Arduino headers, antenna
 *     connector, micro USB ports, and 40-pin connector for Dragonfly)
 *   - MultiTech Dragonfly (1"x2" green PCB with Telit radio)
 *   - Seeed Studio Base Shield
 *   - Grove moisture sensor (to connect to Base Shield)
 *   - Grove button (to connect to Base Shield)
 *   - MEMs Inertial and Environmental Nucleo Expansion board (LSM6DS0
 *     3-axis accelerometer + 3-axis gyroscope, LIS3MDL 3-axis
 *     magnetometer, HTS221 humidity and temperature sensor and LPS25HB
 *     pressure sensor)
 *
 * What this program does:
 *   - reads data from all sensors on MEMs board and moisture sensor on a
 *     periodic basis
 *   - prints all sensor data to debug port on a periodic basis
 *   - optionally send a SMS containing sensor data when the Grove Button
 *     is pushed
 *       - you need to set the "phone_number" field
 *   - optionally sends sensor data to AT&T M2X cloud platform (user must
 *     create own M2X account and configure a device)
 *       - you need to set the "m2x_api_key" field and the "m2x_device_id"
 *         field based on your M2X account for this to work
 *       - you need to set the "do_cloud_post" flag to true for this to
 *         work
 *
 * Setup:
 *   - Correctly insert SIM card into Dragonfly
 *   - Seat the Dragonfly on the UDK2 board
 *   - Connect an antenna to the connector on the Dragonfly labled "M"
 *   - Stack the Base Shield on the UDK2 Arduino headers
 *   - Connect the Grove button to the D8 socket on the Base Shield
 *   - Connect the Grove moisture sensor to the A0 socket on the Base
 *     Shield
 *   - Make sure the reference voltage selector switch (next to the A0
 *     socket) is switched to 5V so you get accurate analog readings
 *   - Stack the MEMs board on top of the Base Shield
 *   - Plug in the power cable
 *   - Plug a micro USB cable into the port below and slightly to the
 *     left of the Dragonfly (NOT the port on the Dragonfly)
 *
 * Go have fun and make something cool!
 *
 ************************************************************************/
/*
Sample Program Description:
   This Program will enable to Multi-Tech Dragonfly platform to utilize ROHM's Multi-sensor Shield Board.
   This program will initialize all sensors on the shield and then read back the sensor data.
   Data will then be output to the UART Debug Terminal every 1 second.

Sample Program Author:
   ROHM USDC

Additional Resources:
   ROHM Sensor Shield GitHub Repository: https://github.com/ROHMUSDC/ROHM_SensorPlatform_Multi-Sensor-Shield
*/



#include "mbed.h"
#include "mtsas.h"
#include "MbedJSONValue.h"
#include "HTTPJson.h"
#include <string>

// Debug serial port
static Serial debug(USBTX, USBRX);

// MTSSerialFlowControl - serial link between processor and radio
static MTSSerialFlowControl* io;

// Cellular - radio object for cellular operations (SMS, TCP, etc)
Cellular* radio;

// APN associated with SIM card
// this APN should work for the AT&T SIM that came with your Dragonfly
//static const std::string apn = "";
//static const std::string apn = "m2m.com.attz";
//static const std::string apn = "broadband";  // for send sms with ATT sim card

// Phone number to send SMS messages to
// just change the x digits - the 1 needs to stay!
static const std::string phone_number = "1**********";

// see https://m2x.att.com/developer/documentation/v2/overview for M2X API documentation
// M2X device ID
static const std::string m2x_device_id = "";

// M2X primary API key
static const std::string m2x_api_key = "";

// set to true if you want to post to the cloud
// you need to have you M2X account set up properly for this to work?
//bool do_cloud_post = false;
bool do_cloud_post = true;

std::string url = "http://api-m2x.att.com/v2/devices/" + m2x_device_id + "/update";


// variables for sensor data
float temp_celsius;
float humidity_percent;
float pressure_mbar;
float moisture_percent;
int32_t mag_mgauss[3];
int32_t acc_mg[3];
int32_t gyro_mdps[3];

// misc variables
static char wall_of_dash[] = "--------------------------------------------------";
bool radio_ok = false;
static int thpm_interval_ms = 5000;
static int motion_interval_ms = 5000;
static int print_interval_ms = 5000;
static int sms_interval_ms = 30000;
static int post_interval_ms = 10000;
int debug_baud = 115200;




/****************************************************************************************************

 ****************************************************************************************************/

//Macros for checking each of the different Sensor Devices
#define AnalogTemp  //BDE0600
#define AnalogUV    //ML8511
#define HallSensor  //BU52011
#define RPR0521     //RPR0521
#define KMX62       //KMX61, Accel/Mag         
#define COLOR       //BH1745
#define KX022       //KX022, Accel Only
#define Pressure    //BM1383
//#define SMS         //allow SMS messaging
#define Web         //allow M2X communication


//Define Pins for I2C Interface
I2C i2c(I2C_SDA, I2C_SCL);
bool        RepStart = true;
bool        NoRepStart = false;

//Define Sensor Variables
#ifdef AnalogTemp
AnalogIn    BDE0600_Temp(PC_4); //Mapped to A2
uint16_t    BDE0600_Temp_value;
float       BDE0600_output;
#endif

#ifdef AnalogUV
AnalogIn    ML8511_UV(PC_1);    //Mapped to A4
uint16_t    ML8511_UV_value;
float       ML8511_output;
#endif

#ifdef HallSensor
DigitalIn   Hall_GPIO0(PC_8);
DigitalIn   Hall_GPIO1(PB_5);
int         Hall_Return1;
int         Hall_Return0;
int32_t     Hall_Return[2];
#endif

#ifdef RPR0521
int         RPR0521_addr_w = 0x70;          //7bit addr = 0x38, with write bit 0
int         RPR0521_addr_r = 0x71;          //7bit addr = 0x38, with read bit 1
char        RPR0521_ModeControl[2] = {0x41, 0xE6};
char        RPR0521_ALSPSControl[2] = {0x42, 0x03};
char        RPR0521_Persist[2] = {0x43, 0x20};
char        RPR0521_Addr_ReadData = 0x44;
char        RPR0521_Content_ReadData[6];
int         RPR0521_PS_RAWOUT = 0;                  //this is an output
float       RPR0521_PS_OUT = 0;
int         RPR0521_ALS_D0_RAWOUT = 0;
int         RPR0521_ALS_D1_RAWOUT = 0;
float       RPR0521_ALS_DataRatio = 0;
float       RPR0521_ALS_OUT = 0;                    //this is an output
float       RPR0521_ALS[2];                         // is this ok taking an int to the [0] value and float to [1]???????????
#endif

#ifdef KMX62
int         KMX62_addr_w = 0x1C;          //7bit addr = 0x38, with write bit 0
int         KMX62_addr_r = 0x1D;          //7bit addr = 0x38, with read bit 1
char        KMX62_CNTL2[2] = {0x3A, 0x5F};
char        KMX62_Addr_Accel_ReadData = 0x0A;
char        KMX62_Content_Accel_ReadData[6];
char        KMX62_Addr_Mag_ReadData = 0x10;
char        KMX62_Content_Mag_ReadData[6];
short int   MEMS_Accel_Xout = 0;
short int   MEMS_Accel_Yout = 0;
short int   MEMS_Accel_Zout = 0;
double      MEMS_Accel_Conv_Xout = 0;
double      MEMS_Accel_Conv_Yout = 0;
double      MEMS_Accel_Conv_Zout = 0;

short int   MEMS_Mag_Xout = 0;
short int   MEMS_Mag_Yout = 0;
short int   MEMS_Mag_Zout = 0;
float       MEMS_Mag_Conv_Xout = 0;
float       MEMS_Mag_Conv_Yout = 0;
float       MEMS_Mag_Conv_Zout = 0;

double      MEMS_Accel[3];
float       MEMS_Mag[3];
#endif

#ifdef COLOR
int         BH1745_addr_w = 0x72;   //write
int         BH1745_addr_r = 0x73;   //read
char        BH1745_persistence[2] = {0x61, 0x03};
char        BH1745_mode1[2] = {0x41, 0x00};
char        BH1745_mode2[2] = {0x42, 0x92};
char        BH1745_mode3[2] = {0x43, 0x02};
char        BH1745_Content_ReadData[6];
char        BH1745_Addr_color_ReadData = 0x50;
int         BH1745_Red;
int         BH1745_Blue;
int         BH1745_Green;
int32_t     BH1745[3];  //Red, Blue Green matrix
#endif

#ifdef KX022
int         KX022_addr_w = 0x3C;   //write
int         KX022_addr_r = 0x3D;   //read
char        KX022_Accel_CNTL1[2] = {0x18, 0x41};
char        KX022_Accel_ODCNTL[2] = {0x1B, 0x02};
char        KX022_Accel_CNTL3[2] = {0x1A, 0xD8};
char        KX022_Accel_TILT_TIMER[2] = {0x22, 0x01};
char        KX022_Accel_CNTL2[2] = {0x18, 0xC1};
char        KX022_Content_ReadData[6];
char        KX022_Addr_Accel_ReadData = 0x06;
float       KX022_Accel_X;
float       KX022_Accel_Y;
float       KX022_Accel_Z;
short int   KX022_Accel_X_RawOUT = 0;
short int   KX022_Accel_Y_RawOUT = 0;
short int   KX022_Accel_Z_RawOUT = 0;
int         KX022_Accel_X_LB = 0;
int         KX022_Accel_X_HB = 0;
int         KX022_Accel_Y_LB = 0;
int         KX022_Accel_Y_HB = 0;
int         KX022_Accel_Z_LB = 0;
int         KX022_Accel_Z_HB = 0;
float       KX022_Accel[3];
#endif

#ifdef Pressure
int         Press_addr_w = 0xBA;   //write
int         Press_addr_r = 0xBB;   //read
char        PWR_DOWN[2] = {0x12, 0x01};
char        SLEEP[2] = {0x13, 0x01};
char        Mode_Control[2] = {0x14, 0xC4};
char        Press_Content_ReadData[6];
char        Press_Addr_ReadData =0x1A;
int         BM1383_Temp_highByte;
int         BM1383_Temp_lowByte;
int         BM1383_Pres_highByte;
int         BM1383_Pres_lowByte;
int         BM1383_Pres_leastByte;
short int   BM1383_Temp_Out;
float       BM1383_Temp_Conv_Out;
float       BM1383_Pres_Conv_Out;
float_t       BM1383[2];   // Temp is 0 and Pressure is 1
float       BM1383_Var;
float       BM1383_Deci;
#endif

/****************************************************************************************************
// function prototypes
 ****************************************************************************************************/
bool init_mtsas();
void ReadAnalogTemp();
void ReadAnalogUV ();
void ReadHallSensor ();
void ReadCOLOR ();
void ReadRPR0521_ALS ();
void ReadKMX62_Accel ();
void ReadKMX62_Mag ();
void ReadPressure ();
void ReadKX022();

/****************************************************************************************************
// main
 ****************************************************************************************************/
int main()
{
    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);
    debug.baud(debug_baud);
    logInfo("starting...");


    /****************************************************************************************************
          Initialize I2C Devices ************
     ****************************************************************************************************/

#ifdef RPR0521
    i2c.write(RPR0521_addr_w, &RPR0521_ModeControl[0], 2, false);
    i2c.write(RPR0521_addr_w, &RPR0521_ALSPSControl[0], 2, false);
    i2c.write(RPR0521_addr_w, &RPR0521_Persist[0], 2, false);
#endif

#ifdef KMX62
    i2c.write(KMX62_addr_w, &KMX62_CNTL2[0], 2, false);
#endif

#ifdef COLOR
    i2c.write(BH1745_addr_w, &BH1745_persistence[0], 2, false);
    i2c.write(BH1745_addr_w, &BH1745_mode1[0], 2, false);
    i2c.write(BH1745_addr_w, &BH1745_mode2[0], 2, false);
    i2c.write(BH1745_addr_w, &BH1745_mode3[0], 2, false);
#endif

#ifdef KX022
    i2c.write(KX022_addr_w, &KX022_Accel_CNTL1[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_ODCNTL[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_CNTL3[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_TILT_TIMER[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_CNTL2[0], 2, false);
#endif

#ifdef Pressure
    i2c.write(Press_addr_w, &PWR_DOWN[0], 2, false);
    i2c.write(Press_addr_w, &SLEEP[0], 2, false);
    i2c.write(Press_addr_w, &Mode_Control[0], 2, false);
#endif
//End I2C Initialization Section **********************************************************


// Initialization Radio Section **********************************************************

    radio_ok = init_mtsas();
    if (! radio_ok)
        logError("MTSAS init failed");
    else
        logInfo("MTSAS is ok");

//End Radio Initialization Section **********************************************************

//    button.fall(&button_irq);


    Timer thpm_timer;
    thpm_timer.start();         // Timer data is set in the Variable seciton see misc variables    Timer motion_timer;
    Timer print_timer;
    print_timer.start();
    Timer motion_timer;
    motion_timer.start();

#ifdef SMS
    Timer sms_timer;
    sms_timer.start();
#endif
#ifdef Web
    Timer post_timer;
    post_timer.start();
#endif
    
    while (true) {
        if (thpm_timer.read_ms() > thpm_interval_ms) {
#ifdef AnalogTemp
            ReadAnalogTemp ();
#endif

#ifdef AnalogUV
            ReadAnalogUV ();
#endif

#ifdef HallSensor
            ReadHallSensor ();
#endif

#ifdef COLOR
            ReadCOLOR ();
#endif

#ifdef RPR0521       //als digital
            ReadRPR0521_ALS ();
#endif

#ifdef Pressure
            ReadPressure();
#endif
            thpm_timer.reset();
        }

        if (motion_timer.read_ms() > motion_interval_ms) {
#ifdef KMX62
            ReadKMX62_Accel ();
            ReadKMX62_Mag ();
#endif

#ifdef KX022
            ReadKX022 ();
#endif
            motion_timer.reset();
        }

        if (print_timer.read_ms() > print_interval_ms) {
            logDebug("%s", wall_of_dash);
            logDebug("SENSOR DATA");
            logDebug("temperature: %0.2f C", BM1383[0]);
            logDebug("analog uv: %.1f mW/cm2", ML8511_output);
            logDebug("ambient Light  %0.3f", RPR0521_ALS[0]);
            logDebug("proximity count  %0.3f", RPR0521_ALS[1]);
            logDebug("hall effect: South %d\t North %d",  Hall_Return[0],Hall_Return[1]);
            logDebug("pressure: %0.2f hPa", BM1383[1]);
            logDebug("magnetometer:\r\n\tx: %0.3f\ty: %0.3f\tz: %0.3f\tuT", MEMS_Mag[0], MEMS_Mag[1], MEMS_Mag[2]);
            logDebug("accelerometer:\r\n\tx: %0.3f\ty: %0.3f\tz: %0.3f\tg", MEMS_Accel[0], MEMS_Accel[1], MEMS_Accel[2]);
            logDebug("color:\r\n\tred: %ld\tgrn: %ld\tblu: %ld\t", BH1745[0], BH1745[1], BH1745[2]);
            logDebug("%s", wall_of_dash);
            print_timer.reset();
        }



#ifdef SMS
        if (sms_timer.read_ms() > sms_interval_ms) {
            sms_timer.reset();
            logInfo("SMS Send Routine");
printf("  In sms routine \r\n");
            if (radio_ok) {
                MbedJSONValue sms_json;
                string sms_str;

//                sms_json["temp_C"] = BDE0600_output;
//                sms_json["UV"] = ML8511_output;
                sms_json["Ambient Light"] = RPR0521_ALS[0];
                sms_json["Prox"]      = RPR0521_ALS[1];
//                sms_json["pressure_hPa"] = BM1383[1];
//                sms_json["mag_mgauss"]["x"] = MEMS_Mag[0];
//                sms_json["mag_mgauss"]["y"] = MEMS_Mag[1];
//                sms_json["mag_mgauss"]["z"] = MEMS_Mag[2];
//                sms_json["acc_mg"]["x"] = MEMS_Accel[0];
//                sms_json["acc_mg"]["y"] = MEMS_Accel[1];
//                sms_json["acc_mg"]["z"] = MEMS_Accel[2];
//                sms_json["Red"]   = BH1745[0];
//                sms_json["Green"] = BH1745[1];
//                sms_json["Blue"]  = BH1745[2];

                sms_str = "SENSOR DATA:\n";
                sms_str += sms_json.serialize();

                logDebug("sending SMS to %s:\r\n%s", phone_number.c_str(), sms_str.c_str());
                Code ret = radio->sendSMS(phone_number, sms_str);
                if (ret != MTS_SUCCESS)
                    logError("sending SMS failed");
            }
        }
#endif
#ifdef Web
        if (post_timer.read_ms() > post_interval_ms && do_cloud_post) {
    printf("in web\n\r");
            if (radio->connect()) {
                logDebug("posting sensor data");

                HTTPClient http;
                MbedJSONValue http_json_data;
                std::string http_json_str;
                std::string m2x_header = "X-M2X-KEY: " + m2x_api_key + "\r\n";
                int ret;
                char http_response_buf[256];
                HTTPText http_response(http_response_buf, sizeof(http_response_buf));

                // temp_c, temp_f, humidity, pressure, and moisture are all stream IDs for my device in M2X
                // modify these to match your streams or give your streams the same name
                http_json_data["values"]["temp_c"] = BDE0600_output;
                http_json_data["values"]["uv"] = ML8511_output;
                http_json_data["values"]["amb_light"] = RPR0521_ALS[0];
                http_json_data["values"]["prox"] = RPR0521_ALS[1];
                http_json_str = http_json_data.serialize();

                // add extra header with M2X API key
                http.setHeader(m2x_header.c_str());

                HTTPJson http_json((char*)  http_json_str.c_str());
                ret = http.post(url.c_str(), http_json, &http_response);
                if (ret != HTTP_OK)
                    logError("posting data to cloud failed: [%d][%s]", ret, http_response_buf);
                else
                    logDebug("post result [%d][%s]", http.getHTTPResponseCode(), http_response_buf);

                radio->disconnect();
            } else {
                logError("establishing PPP link failed");
            }

            post_timer.reset();
        }
#endif
        wait_ms(10);
    }
}

// init functions
bool init_mtsas()
{
    io = new MTSSerialFlowControl(RADIO_TX, RADIO_RX, RADIO_RTS, RADIO_CTS);
    if (! io)
        return false;

    io->baud(115200);
    radio = CellularFactory::create(io);
    if (! radio)
        return false;
////////////////////////////////-------------------/////////////////
//    Code ret = radio->setApn(apn);
//    if (ret != MTS_SUCCESS)
//        return false;

//    Transport::setTransport(radio);

//    return true;
}


// Sensor data acquisition functions
/************************************************************************************************/
#ifdef AnalogTemp
void ReadAnalogTemp ()
{
    BDE0600_Temp_value = BDE0600_Temp.read_u16();

    BDE0600_output = (float)BDE0600_Temp_value * (float)0.000050354; //(value * (3.3V/65535))
    BDE0600_output = (BDE0600_output-(float)1.753)/((float)-0.01068) + (float)30;

//    printf("BDE0600 Analog Temp Sensor Data:\r\n");
//    printf(" Temp = %.2f C\r\n", BDE0600_output);
}
#endif

#ifdef AnalogUV
void ReadAnalogUV ()
{
    ML8511_UV_value = ML8511_UV.read_u16();
    ML8511_output = (float)ML8511_UV_value * (float)0.000050354; //(value * (3.3V/65535))   //Note to self: when playing with this, a negative value is seen... Honestly, I think this has to do with my ADC converstion...
    ML8511_output = (ML8511_output-(float)2.2)/((float)0.129) + 10;                           // Added +5 to the offset so when inside (aka, no UV, readings show 0)... this is the wrong approach... and the readings don't make sense... Fix this.

//    printf("ML8511 Analog UV Sensor Data:\r\n");
//    printf(" UV = %.1f mW/cm2\r\n", ML8511_output);

}
#endif


#ifdef HallSensor
void ReadHallSensor ()
{

    Hall_Return[0] = Hall_GPIO0;
    Hall_Return[1] = Hall_GPIO1;

//    printf("BU52011 Hall Switch Sensor Data:\r\n");
//    printf(" South Detect = %d\r\n", Hall_Return[0]);
//    printf(" North Detect = %d\r\n", Hall_Return[1]);

    
}
#endif

#ifdef COLOR
void ReadCOLOR ()
{

    //Read color data from the IC
    i2c.write(BH1745_addr_w, &BH1745_Addr_color_ReadData, 1, RepStart);
    i2c.read(BH1745_addr_r, &BH1745_Content_ReadData[0], 6, NoRepStart);

    //separate all data read into colors
    BH1745[0] = (BH1745_Content_ReadData[1]<<8) | (BH1745_Content_ReadData[0]);
    BH1745[1] = (BH1745_Content_ReadData[3]<<8) | (BH1745_Content_ReadData[2]);
    BH1745[2] = (BH1745_Content_ReadData[5]<<8) | (BH1745_Content_ReadData[4]);

    //Output Data into UART
//    printf("BH1745 COLOR Sensor Data:\r\n");
//    printf(" Red   = %d ADC Counts\r\n",BH1745[0]);
//    printf(" Green = %d ADC Counts\r\n",BH1745[1]);
//    printf(" Blue  = %d ADC Counts\r\n",BH1745[2]);

}
#endif

#ifdef RPR0521       //als digital
void ReadRPR0521_ALS ()
{
    i2c.write(RPR0521_addr_w, &RPR0521_Addr_ReadData, 1, RepStart);
    i2c.read(RPR0521_addr_r, &RPR0521_Content_ReadData[0], 6, NoRepStart);

    RPR0521_ALS[1] = (RPR0521_Content_ReadData[1]<<8) | (RPR0521_Content_ReadData[0]);
    RPR0521_ALS_D0_RAWOUT = (RPR0521_Content_ReadData[3]<<8) | (RPR0521_Content_ReadData[2]);
    RPR0521_ALS_D1_RAWOUT = (RPR0521_Content_ReadData[5]<<8) | (RPR0521_Content_ReadData[4]);
    RPR0521_ALS_DataRatio = (float)RPR0521_ALS_D1_RAWOUT / (float)RPR0521_ALS_D0_RAWOUT;

    if(RPR0521_ALS_DataRatio < (float)0.595) {
        RPR0521_ALS[0] = ((float)1.682*(float)RPR0521_ALS_D0_RAWOUT - (float)1.877*(float)RPR0521_ALS_D1_RAWOUT);
    } else if(RPR0521_ALS_DataRatio < (float)1.015) {
        RPR0521_ALS[0] = ((float)0.644*(float)RPR0521_ALS_D0_RAWOUT - (float)0.132*(float)RPR0521_ALS_D1_RAWOUT);
    } else if(RPR0521_ALS_DataRatio < (float)1.352) {
        RPR0521_ALS[0] = ((float)0.756*(float)RPR0521_ALS_D0_RAWOUT - (float)0.243*(float)RPR0521_ALS_D1_RAWOUT);
    } else if(RPR0521_ALS_DataRatio < (float)3.053) {
        RPR0521_ALS[0] = ((float)0.766*(float)RPR0521_ALS_D0_RAWOUT - (float)0.25*(float)RPR0521_ALS_D1_RAWOUT);
    } else {
        RPR0521_ALS[0] = 0;
    }
//    printf("RPR-0521 ALS/PROX Sensor Data:\r\n");
//    printf(" ALS = %0.2f lx\r\n", RPR0521_ALS[0]);
//    printf(" PROX= %0.2f ADC Counts\r\n", RPR0521_ALS[1]);     //defined as a float but is an unsigned.

}
#endif

#ifdef KMX62
void ReadKMX62_Accel ()
{
    //Read Accel Portion from the IC
    i2c.write(KMX62_addr_w, &KMX62_Addr_Accel_ReadData, 1, RepStart);
    i2c.read(KMX62_addr_r, &KMX62_Content_Accel_ReadData[0], 6, NoRepStart);

    //Note: The highbyte and low byte return a 14bit value, dropping the two LSB in the Low byte.
    //      However, because we need the signed value, we will adjust the value when converting to "g"
    MEMS_Accel_Xout = (KMX62_Content_Accel_ReadData[1]<<8) | (KMX62_Content_Accel_ReadData[0]);
    MEMS_Accel_Yout = (KMX62_Content_Accel_ReadData[3]<<8) | (KMX62_Content_Accel_ReadData[2]);
    MEMS_Accel_Zout = (KMX62_Content_Accel_ReadData[5]<<8) | (KMX62_Content_Accel_ReadData[4]);

    //Note: Conversion to G is as follows:
    //      Axis_ValueInG = MEMS_Accel_axis / 1024
    //      However, since we did not remove the LSB previously, we need to divide by 4 again
    //      Thus, we will divide the output by 4096 (1024*4) to convert and cancel out the LSB
    MEMS_Accel[0] = ((float)MEMS_Accel_Xout/4096/2);
    MEMS_Accel[1] = ((float)MEMS_Accel_Yout/4096/2);
    MEMS_Accel[2] = ((float)MEMS_Accel_Zout/4096/2);

    // Return Data to UART
//    printf("KMX62 Accel+Mag Sensor Data:\r\n");
//    printf(" AccX= %0.2f g\r\n", MEMS_Accel[0]);
//    printf(" AccY= %0.2f g\r\n", MEMS_Accel[1]);
//    printf(" AccZ= %0.2f g\r\n", MEMS_Accel[2]);

}

void ReadKMX62_Mag ()
{

    //Read Mag portion from the IC
    i2c.write(KMX62_addr_w, &KMX62_Addr_Mag_ReadData, 1, RepStart);
    i2c.read(KMX62_addr_r, &KMX62_Content_Mag_ReadData[0], 6, NoRepStart);

    //Note: The highbyte and low byte return a 14bit value, dropping the two LSB in the Low byte.
    //      However, because we need the signed value, we will adjust the value when converting to "g"
    MEMS_Mag_Xout = (KMX62_Content_Mag_ReadData[1]<<8) | (KMX62_Content_Mag_ReadData[0]);
    MEMS_Mag_Yout = (KMX62_Content_Mag_ReadData[3]<<8) | (KMX62_Content_Mag_ReadData[2]);
    MEMS_Mag_Zout = (KMX62_Content_Mag_ReadData[5]<<8) | (KMX62_Content_Mag_ReadData[4]);

    //Note: Conversion to G is as follows:
    //      Axis_ValueInG = MEMS_Accel_axis / 1024
    //      However, since we did not remove the LSB previously, we need to divide by 4 again
    //      Thus, we will divide the output by 4095 (1024*4) to convert and cancel out the LSB
    MEMS_Mag[0] = (float)MEMS_Mag_Xout/4096*(float)0.146;
    MEMS_Mag[1] = (float)MEMS_Mag_Yout/4096*(float)0.146;
    MEMS_Mag[2] = (float)MEMS_Mag_Zout/4096*(float)0.146;

    // Return Data to UART
//    printf(" MagX= %0.2f uT\r\n", MEMS_Mag[0]);
//    printf(" MagY= %0.2f uT\r\n", MEMS_Mag[1]);
//    printf(" MagZ= %0.2f uT\r\n", MEMS_Mag[2]);

}
#endif

#ifdef KX022
void ReadKX022 ()
{
    
    //Read KX022 Portion from the IC
    i2c.write(KX022_addr_w, &KX022_Addr_Accel_ReadData, 1, RepStart);
    i2c.read(KX022_addr_r, &KX022_Content_ReadData[0], 6, NoRepStart);

    //Format Data
    KX022_Accel_X_RawOUT = (KX022_Content_ReadData[1]<<8) | (KX022_Content_ReadData[0]);
    KX022_Accel_Y_RawOUT = (KX022_Content_ReadData[3]<<8) | (KX022_Content_ReadData[2]);
    KX022_Accel_Z_RawOUT = (KX022_Content_ReadData[5]<<8) | (KX022_Content_ReadData[4]);

    //Scale Data
    KX022_Accel[0] = (float)KX022_Accel_X_RawOUT / 16384;
    KX022_Accel[1] = (float)KX022_Accel_Y_RawOUT / 16384;
    KX022_Accel[2] = (float)KX022_Accel_Z_RawOUT / 16384;

    //Return Data through UART
//    printf("KX022 Accelerometer Sensor Data: \r\n");
//    printf(" AccX= %0.2f g\r\n", KX022_Accel[0]);
//    printf(" AccY= %0.2f g\r\n", KX022_Accel[1]);
//    printf(" AccZ= %0.2f g\r\n", KX022_Accel[2]);

}
#endif


#ifdef Pressure
void ReadPressure ()
{

    i2c.write(Press_addr_w, &Press_Addr_ReadData, 1, RepStart);
    i2c.read(Press_addr_r, &Press_Content_ReadData[0], 6, NoRepStart);

    BM1383_Temp_Out = (Press_Content_ReadData[0]<<8) | (Press_Content_ReadData[1]);
    BM1383[0] = (float)BM1383_Temp_Out/32;

    BM1383_Var  = (Press_Content_ReadData[2]<<3) | (Press_Content_ReadData[3] >> 5);
    BM1383_Deci = ((Press_Content_ReadData[3] & 0x1f) << 6 | ((Press_Content_ReadData[4] >> 2)));
    BM1383_Deci = (float)BM1383_Deci* (float)0.00048828125;  //0.00048828125 = 2^-11
    BM1383[1] = (BM1383_Var + BM1383_Deci);   //question pending here...

//    printf("BM1383 Pressure Sensor Data:\r\n");
//    printf(" Temperature= %0.2f C\r\n", BM1383[0]);
//    printf(" Pressure   = %0.2f hPa\r\n", BM1383[1]);

}
#endif


/************************************************************************************
//  reference only to remember what the names and fuctions are without finding them above.
 ************************************************************************************
    (" Temp = %.2f C\r\n", BDE0600_output);
    printf(" UV = %.1f mW/cm2\r\n", ML8511_output);

    printf("BH1745 COLOR Sensor Data:\r\n");
    printf(" Red   = %d ADC Counts\r\n",BH1745[0]);
    printf(" Green = %d ADC Counts\r\n",BH1745[1]);
    printf(" Blue  = %d ADC Counts\r\n",BH1745[2]);

    printf(" ALS = %0.2f lx\r\n", RPR0521_ALS[0]);
    printf(" PROX= %u ADC Counts\r\n", RPR0521_ALS[1]);     //defined as a float but is an unsigned, bad coding on my part.

    printf("KMX62 Accel+Mag Sensor Data:\r\n");
    printf(" AccX= %0.2f g\r\n", MEMS_Accel[0]);
    printf(" AccY= %0.2f g\r\n", MEMS_Accel[1]);
    printf(" AccZ= %0.2f g\r\n", MEMS_Accel[2]);

    printf(" MagX= %0.2f uT\r\n", MEMS_Mag[0]);
    printf(" MagY= %0.2f uT\r\n", MEMS_Mag[1]);
    printf(" MagZ= %0.2f uT\r\n", MEMS_Mag[2]);

    printf("KX022 Accelerometer Sensor Data: \r\n");
    printf(" AccX= %0.2f g\r\n", KX022_Accel[0]);
    printf(" AccY= %0.2f g\r\n", KX022_Accel[1]);
    printf(" AccZ= %0.2f g\r\n", KX022_Accel[2]);

    printf("BM1383 Pressure Sensor Data:\r\n");
    printf(" Temperature= %0.2f C\r\n", BM1383[0]);
    printf(" Pressure   = %0.2f hPa\r\n", BM1383[1]);

 **********************************************************************************/







