TaskHandle_t Core1;
TaskHandle_t Core2;
// ESP32 Ntrip Client by Coffeetrac
// Release: V1.26
// 01.01.2019 W.Eder
// Enhanced by Matthias Hammer 12.01.2019
//##########################################################################################################
//### Setup Zone ###########################################################################################
//### Just Default values ##################################################################################
struct Storage{
  char ssid[24]        = "0C8FFFE98F71-2G";          // WiFi network Client name
  char password[24]    = "5530119582832";      // WiFi network password
//  char ssid[24]        = "HUAWEI P20 lite";          // WiFi network Client name
//  char password[24]    = "94f8af0770b0";      // WiFi network password
  unsigned long timeoutRouter = 65;           // Time (seconds) to wait for WIFI access, after that own Access Point starts 

  // Ntrip Caster Data
  char host[40]        = "52.194.74.83";    // Server IP
  int  port            = 2101;                // Server Port
  char mountpoint[40]  = "HIGASHIURA";   // Mountpoint
  char ntripUser[40]   = "guest";     // Username
  char ntripPassword[40]= "guest";    // Password

  byte sendGGAsentence = 0  ; // 0 = No Sentence will be sended
                            // 1 = fixed Sentence from GGAsentence below will be sended
                            // 2 = GGA from GPS will be sended
  
  byte GGAfreq =5;         // time in seconds between GGA Packets

  char GGAsentence[100] = "$GPGGA,121532.502,3457.224,N,13656.113,E,1,12,1.0,0.0,M,0.0,M,,*6A"; //hc create via www.nmeagen.org
  
  long baudOut = 38400;     // Baudrate of RTCM Port

  byte send_UDP_AOG  = 1;   // 0 = Transmission of NMEA Off
                            // 1 = Transmission of NMEA Sentences to AOG via Ethernet-UDP
                            // 2 = Bluetooth attention: not possible if line useBluetooth = false

  byte enableNtrip   = 1;   // 0 = NTRIP disabled
                            // 1 = ESP NTRIP Client enabled
                            // 2 = AOG NTRIP Client enabled (Port=2233)
  
  byte AHRSbyte      = 4;   // 0 = No IMU, No Inclinometer
                            // 1 = BNO055 IMU installed
                            // 2 = MMA8452 Inclinometer installed
                            // 3 = BNO055 + MMA 8452 installed
                            // 4 =  LSM9DS1 installed
}; Storage NtripSettings;

//##########################################################################################################
//### End of Setup Zone ####################################################################################
//##########################################################################################################

boolean debugmode = false;
#define useBluetooth  1  // 1= possibility to use bluetooth to transfer data to AOG later on, but needs lots of memory.

// IO pins --------------------------------
#define RX0      3//3
#define TX0      1//1

#define RX1     14  //17 simpleRTK TX(xbee) = RX(f9p)
#define TX1     13  //27simpleRTK RX(xbee) = TX(f9p)

#define RX2     16  
#define TX2     15 

#define SDA     32  //I2C Pins
#define SCL     33

#define LED_PIN_WIFI   2   // WiFi Status LED

//########## BNO055 adress 0x28 ADO = 0 set in BNO_ESP.h means ADO -> GND
//########## MMA8451 adress 0x1D SAO = 0 set in MMA8452_AOG.h means SAO open (pullup!!)

#define restoreDefault_PIN 35  // set to 1 during boot, to restore the default values


//libraries -------------------------------
#include <Wire.h>
#include <WiFi.h>
#include <base64.h>
#include "Network_AOG.h"
#include "EEPROM.h"
#include "BNO_ESP.h"
#include "MMA8452_AOG.h"
#include "BluetoothSerial.h"
#include "LSM9DS1_Registers.h"
#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Types.h"

// Declarations
void DBG(String out, byte nl = 0);

//Accesspoint name and password:
const char* ssid_ap     = "NTRIP_Client_ESP_Net";
const char* password_ap = "";

//static IP
IPAddress myip(192, 168, 3,79 );  // Roofcontrol module
IPAddress gwip(192, 168, 3, 1);   // Gateway & Accesspoint IP
IPAddress mask(255, 255, 255, 0);
IPAddress myDNS(8, 8, 8, 8);      //optional

unsigned int portMy = 5544;       //this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
unsigned int portAOG = 8888;      // port to listen for AOG
unsigned int portMyNtrip = 2233;

//IP address to send UDP data to:
IPAddress ipDestination(192, 168, 3, 255);
unsigned int portDestination = 9999;  // Port of AOG that listens

// Variables ------------------------------
// WiFistatus LED 
// blink times: searching WIFI: blinking 4x faster; connected: blinking as times set; data available: light on; no data for 2 seconds: blinking
unsigned int LED_WIFI_time = 0;
unsigned int LED_WIFI_pulse = 700;   //light on in ms 
unsigned int LED_WIFI_pause = 700;   //light off in ms
boolean LED_WIFI_ON = false;
unsigned long Ntrip_data_time = 0;

// program flow
bool AP_running=0, EE_done = 0, restart=0;
int value = 0; 
unsigned long repeat_ser;   
//int error = 0;
unsigned long repeatGGA, lifesign, aogntriplife;

//loop time variables in microseconds
const unsigned int LOOP_TIME = 100; //10hz 
unsigned int lastTime = LOOP_TIME;
unsigned int currentTime = LOOP_TIME;
unsigned int dT = 50000;

//Kalman variables
float rollK = 0, Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
float XeRoll = 0;
const float varRoll = 0.1; // variance,
const float varProcess = 0.0055; //0,00025 smaller is more filtering

// GPS-Bridge
int cnt=0;
int i=0;  
byte gpsBuffer[100], c;
char imuBuffer[20];
bool newSentence = false;
bool newIMUSentence = false;
char lastSentence[100]="";

char strmBuf[512];         // rtcm Message Buffer

 //IMU, inclinometer variables
  bool imu_initialized = 0;
  int16_t roll = 0, roll_corr = 0;
  uint16_t x_ , y_ , z_;

//Array to send data back to AgOpenGPS
byte GPStoSend[100]; 
byte IMUtoSend[] = {0x7F,0xEE,0,0,0,0,0,0,0,0};
byte IMUtoSendLenght = 10; //lenght of array to AOG

// Instances ------------------------------
// instances
LSM9DS1 imu;
MMA8452 accelerometer;
WiFiServer server(80);
WiFiClient ntripCl;
WiFiClient client_page;
AsyncUDP udpRoof;
AsyncUDP udpNtrip;
#if (useBluetooth)
BluetoothSerial SerialBT;
#endif


// Setup procedure ------------------------
void setup() {
  pinMode(restoreDefault_PIN, INPUT);  //
  
  restoreEEprom();
  Wire.begin(SDA, SCL, 400000);

  //  Serial1.begin (NtripSettings.baudOut, SERIAL_8N1, RX1, TX1); 
  if (debugmode) { Serial1.begin(115200, SERIAL_8N1, RX1, TX1); } //set new Baudrate
  else { Serial1.begin(NtripSettings.baudOut, SERIAL_8N1, RX1, TX1); } //set new Baudrate
  Serial2.begin(115200,SERIAL_8N1,RX2,TX2); 

  Serial.begin(115200);
  //imu setting
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = 0x1C;
  imu.settings.device.agAddress = 0x6A;
  imu.settings.mag.scale = 4; // Set mag scale to +/-12 Gs
        // [sampleRate] sets the output data rate (ODR) of the
        // magnetometer.
        // mag data rate can be 0-7:
        // 0 = 0.625 Hz  4 = 10 Hz
        // 1 = 1.25 Hz   5 = 20 Hz
        // 2 = 2.5 Hz    6 = 40 Hz
        // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 5; // Set OD rate to 20Hz
        // [tempCompensationEnable] enables or disables
        // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = true;
        // [XYPerformance] sets the x and y-axis performance of the
        // magnetometer to either:
        // 0 = Low power mode      2 = high performance
        // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
        // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
        // [lowPowerEnable] enables or disables low power mode in
        // the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
  if (!imu.begin()) {
    Serial.println("IMU Failed");
  } else {
  Serial.println("IMU Ok");
  }
 #if (useBluetooth)
     if(!SerialBT.begin("BT_GPS_ESP")){
      DBG("\nAn error occurred initializing Bluetooth\n");
     }
 #endif

 pinMode(LED_PIN_WIFI, OUTPUT);
     
  //------------------------------------------------------------------------------------------------------------  
  //create a task that will be executed in the Core1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(Core1code, "Core1", 10000, NULL, 1, &Core1, 0);
  delay(500); 
  //create a task that will be executed in the Core2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(Core2code, "Core2", 10000, NULL, 1, &Core2, 1); 
  delay(500); 
  //------------------------------------------------------------------------------------------------------------
 
}

void loop() {
}

//--------------------------------------------------------------
//  Debug Messaging
//--------------------------------------------------------------
bool debug = 1;  // Print Debug Messages to Serial0

void DBG(String out, byte nl){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(int out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(long out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(char out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(char out, byte type, byte nl = 0){ // type = HEX,BIN,DEZ..
  if (debug == 1) {
    if (nl) Serial.println(out,type);
    else Serial.print(out,type);
  }
}

void DBG(IPAddress out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}
