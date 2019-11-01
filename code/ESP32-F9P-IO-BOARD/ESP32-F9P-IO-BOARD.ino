TaskHandle_t Core1;
TaskHandle_t Core2;
// ESP32 Ntrip Client by Coffeetrac
// Release: V1.26
// 01.01.2019 W.Eder
// Enhanced by Matthias Hammer 12.01.2019
// Enhanced by minolu takeuchi 1.11.2019 (for esp32-f9p-io-board designed doppelgrau )
// https://github.com/doppelgrau/esp32-f9p-io-board
//##########################################################################################################
//### Setup Zone ###########################################################################################
//### Just Default values ##################################################################################
struct Storage{
  
  char ssid[24]        = "your SSID";          // WiFi network Client name
  char password[24]    = "wifi password";      // WiFi network password
  unsigned long timeoutRouter = 65;           // Time (seconds) to wait for WIFI access, after that own Access Point starts 

  // Ntrip Caster Data
  char host[40]        = "ntripcasterIP";    // Server IP
  int  port            = 2101;                // Server Port
  char mountpoint[40]  = "MOUNTPOINT";   // Mountpoint
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
                            // 4 = LSM9DS1 Inclinometer installed **
  byte output_type = 5;       //set to 1  if you want to use Stering Motor + Cytron MD30C Driver
                              //set to 2  if you want to use Stering Motor + IBT 2  Driver
                              //set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve 
                              //set to 4  if you want to use  IBT 2  Driver + Danfoss Valve PVE A/H/M
                              //set to 5  if you want to use  VNH7070 

  byte input_type  = 3;       //0 = No ADS installed, Wheel Angle Sensor connected directly to ESP at GPIO 4 (attention 3,3V only)
                              //1 = Single Mode of ADS1115 - Sensor Signal at A0 (ADS)
                              //2 = Differential Mode - Connect Sensor GND to A1, Signal to A0
                              //3 = Differential Mode - Connect Sensor 5V to A3, Signal to A2 **                            //1 = Single Mode of ADS1115 - Sensor Signal at A0 (ADS) **
                          

  byte IMU_type     = 0;      // set to 1 to enable BNO055 IMU
  
  byte Inclino_type = 2;      // set to 1 if MMA8452 is installed
                              // set to 2 if LSM9DS1 is installed

  bool Invert_WAS   = 1;      // set to 1 to Change Direction of Wheel Angle Sensor - to + 
         
  byte SWEncoder    = 0;      // Steering Wheel ENCODER Installed
  byte pulseCountMax= 3;      // Switch off Autosteer after x Pulses from Steering wheel encoder 
  
  int SteerPosZero  = 512;

  byte SteerSwitchType = 1;   //0 = enable = switch high (3,3V) //1 = enable = switch low(GND) //2 = toggle = button to low(GND)
                              //3 = enable = button to high (3,3V), disable = button to low (GND), neutral = 1,65V
  
  byte WorkSW_mode = 1;       // 0 = disabled   // 1 = digital ON/OFF // 2 = analog Value 0...4095 (0 - 3,3V)
  
  byte Invert_WorkSW = 0;     // 0 = Hitch raised -> High    // 1 = Hitch raised -> Low
  
  unsigned int WorkSW_Threshold = 1200;    // Value for analog hitch level to switch workswitch  
  //##########################################################################################################
  //### End of Setup Zone ####################################################################################
  //##########################################################################################################
  float Ko = 0.05f;  //overall gain  
  float Kp = 50.0f;  //proportional gain  
  float Ki = 0.001f;//integral gain
  float Kd = 1.0f;  //derivative gain 
  float steeringPositionZero = 0;  byte minPWMValue=10;
  int maxIntegralValue=20;//max PWM value for integral PID component
  float steerSensorCounts=100;  int roll_corr = 0;
}; Storage IOSettings;



//##########################################################################################################
//### End of Setup Zone ####################################################################################
//##########################################################################################################

boolean debugmode = 0;
#define useBluetooth  0  // 1= possibility to use bluetooth to transfer data to AOG later on, but needs lots of memory.
#define timeoutRouter  65 
// IO pins --------------------------------

//#define ANALOG_INPUT1 36
//#define ANALOG_INPUT2 39
//#define ANALOG_INPUT3 34                  
#define I2C_SDA 32
#define I2C_SCL 33
#define VNH_A_PWM 4
#define VNH_B_PWM 12
#define F9P_RX 14
#define F9P_TX 13
#define RS232_RX 16
#define RS232_TX 15
#define UART_RX 2
#define UART_TX 0
#define Autosteer_Led  2
#define LED_PIN_WIFI   2
#define led1           2
#define led2           2

#define W_A_S           34  //ANALOG_INPUT3 34
#define WORKSW_PIN     39  //ANALOG_INPUT2 39
#define STEERSW_PIN    36  //ANALOG_INPUT1036
//#define encAPin        34
//#define encBPin        36
// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -7.73 // Declination (degrees) 

//libraries -------------------------------
#include <Wire.h>
#include <WiFi.h>
#include <base64.h>
#include "Network_AOG.h"
#include "EEPROM.h"
#include "Adafruit_ADS1015.h"
#include "driver/gpio.h"
#include "LSM9DS1_Registers.h"
#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Types.h"

#include "BNO_ESP.h"
#include "MMA8452_AOG.h"
//#include "BluetoothSerial.h"

// Declarations
void DBG(String out, byte nl = 0);

//Accesspoint name and password:
const char* ssid_ap     = "ESP-F9P-IO-BORAD_Net";
const char* password_ap = "";

//static IP
IPAddress myip(192, 168, 3,79);  // Roofcontrol module
IPAddress gwip(192, 168, 3, 1);   // Gateway & Accesspoint IP
IPAddress mask(255, 255, 255, 0);
IPAddress myDNS(8, 8, 8, 8);      //optional

unsigned int portMy = 5577;       //5544this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
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
unsigned long UDP_data_time = 0;
// program flow
bool AP_running=0, EE_done = 0, restart=0;
int value = 0; 
unsigned long repeat_ser;   

//program flow
bool iI2C_SDAtaFound = false, isSettingFound = false;
int header = 0, tempHeader = 0, temp;
int AnalogValue = 0;
volatile bool steerEnable = false, toggleSteerEnable=false;
byte relay = 0, uTurn = 0, speeed = 0, workSwitch = 0, workSwitchOld = 0, steerSwitch = 1, switchByte = 0;
float distanceFromLine = 0, corr = 0; // not used
int16_t idistanceFromLine = 0;
float olddist=0;
unsigned long oldmillis;  

//int error = 0;
unsigned long repeatGGA, lifesign, aogntriplife;
bool isDataFound = false;

//loop time variables in microseconds
const unsigned int LOOP_TIME = 100; //10hz 
unsigned int lastTime = LOOP_TIME;
unsigned int currentTime = LOOP_TIME;
unsigned int dT = 50000;
byte count = 0;
byte watchdogTimer = 0;

//Kalman variables
float rollK = 0, Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
float XeRoll = 0;
const float varRoll = 0.1; // variance,
const float varProcess = 0.0055; //0,00025 smaller is more filtering



//steering variables
float steerAngleActual = 0;
int steerPrevSign = 0, steerCurrentSign = 0; // the steering wheels angle currently and previous one
int16_t isteerAngleSetPoint = 0; //the desired angle from AgOpen
float steerAngleSetPoint = 0;
long steeringPosition = 0, steeringPosition_corr = 0,actualSteerPos=0; //from steering sensor
float steerAngleError = 0; //setpoint - actual
float distanceError = 0; //
volatile int  pulseACount = 0, pulseBCount = 0;; // Steering Wheel Encoder

//for use 1k ohm Potentiometer.  you need change 
int steeringPositionRawMim = -20800;
int steeringPositionRawMax = 0 ;
  
//IMU, inclinometer variables
bool imu_initialized=0;
int16_t roll = 0;
uint16_t x_ , y_ , z_;

//pwm variables
int pwmDrive = 0, drive = 0, pwmDisplay = 0, pwmOut = 0;
float pValue = 0, iValue = 0, dValue = 0;
    
//integral values - **** change as required *****
int maxIntErr = 200; //anti windup max
int maxIntegralValue = 20; //max PWM value for integral PID component 


// GPS-Bridge
int cnt=0;
int i=0;  
byte gpsBuffer[100], c;
char imuBuffer[20];
bool newSentence = false;
bool newIMUSentence = false;
char lastSentence[100]="";

char strmBuf[512];         // rtcm Message Buffer



//Array to send data back to AgOpenGPS
byte GPStoSend[100]; 
//byte toSend[] = {0x7F,0xEE,0,0,0,0,0,0,0,0};
byte toSend[] = {0x7F,0xFD,0,0,0,0,0,0,0,0};

byte toSendLenght = 10; //lenght of array to AOG
//data that will be received from server
uint8_t data[10];
// Debug ----------------------------------
byte state_after=0, state_previous=0, breakreason=0;


// Instances ------------------------------
MMA8452 accelerometer;
WiFiServer server(80);
WiFiClient ntripCl;
WiFiClient client_page;
AsyncUDP udpAOG;
AsyncUDP udpNtrip;
LSM9DS1 imu;
Adafruit_ADS1115 ads;     


// Setup procedure ------------------------
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  //GPIO expander FXL6408 setting
  // direction (Input/Output)
  setByteI2C(0x43, 0x03, 0b11111110);
  // disable High-Z on outputs
  setByteI2C(0x43, 0x07, 0b00000001);
  // en-/disable Pullup/downs
  setByteI2C(0x43, 0x0B, 0b00000001);
  // set direction of the pull
  setByteI2C(0x43, 0x0D, 0b00000001);

  Serial.begin(115200);
  //  Serial1.begin (IOSettings.baudOut, SERIAL_8N1, F9P_RX, F9P_TX); 
  if (debugmode) { Serial1.begin(115200, SERIAL_8N1, F9P_RX, F9P_TX); } //set new Baudrate
  else { Serial1.begin(IOSettings.baudOut, SERIAL_8N1, F9P_RX, F9P_TX); } //set new Baudrate
  //Serial2.begin(115200,SERIAL_8N1,RS232_RX,RS232_TX); 
  
  pinMode(Autosteer_Led, OUTPUT);
  pinMode(WORKSW_PIN, INPUT);
  pinMode(STEERSW_PIN, INPUT);
  pinMode(W_A_S, INPUT);
  //pinMode(LED_PIN_WIFI, OUTPUT);
  //pinMode(led1, OUTPUT);
  //pinMode(led2, OUTPUT);
  // Setup PINs for VNHs Motor driver
  pinMode(VNH_A_PWM, OUTPUT);
  pinMode(VNH_B_PWM, OUTPUT);
  ledcSetup(0,1000,8);  // PWM Output with channel 0, 1kHz, 8-bit resolution (0-255)
  ledcSetup(1,1000,8);  // PWM Output with channel 1, 1kHz, 8-bit resolution (0-255)
  ledcAttachPin(VNH_A_PWM,0);  // attach VNH_A_PWM PIN to Channel 0
  ledcAttachPin(VNH_B_PWM,1);  // attach VNH_B_PWM PIN to Channel 1
  // analog inputs - set input to explicit disable any pullups
  // pinMode(ANALOG_INPUT1, INPUT_PULLUP);
  // pinMode(ANALOG_INPUT2, INPUT);
  //pinMode(ANALOG_INPUT3, INPUT);
  //analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  //analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. But needed for full scale range  Recommended to use 2.5 or 6.
  if (IOSettings.input_type==0)  IOSettings.SteerPosZero =2048;                //Starting Point with ESP ADC 2048 
  if (IOSettings.input_type >0 && IOSettings.input_type < 3 )  IOSettings.SteerPosZero =13000;  //with ADS start with 13000  
  if (IOSettings.input_type == 3 )  IOSettings.SteerPosZero =0;  //with ADS start with 0  forLSM9DS1
  //IMU setting
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
  //imu.begin();

  

  
  //Setup PINs for FXL6408 GPIO Expander
  // direction (Input/Output)
  setByteI2C(0x43, 0x03, 0b11111110);
  // disable High-Z on outputs
  setByteI2C(0x43, 0x07, 0b00000001);
  // en-/disable Pullup/downs
  setByteI2C(0x43, 0x0B, 0b00000001);
  // set direction of the pull
  setByteI2C(0x43, 0x0D, 0b00000001);

  restoreEEprom();

 //pinMode(LED_PIN_WIFI, OUTPUT);
   
  //------------------------------------------------------------------------------------------------------------  
  //create a task that will be executed in the Core1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(Core1code, "Core1", 8000, NULL, 1, &Core1, 0);
  delay(500); 
  //create a task that will be executed in the Core2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(Core2code, "Core2", 12000, NULL, 1, &Core2, 1); 
  delay(500); 
  //------------------------------------------------------------------------------------------------------------
  while ((my_WiFi_Mode == 0)) {   // Waiting for WiFi Access
    delay(1000);
  }
  
  //Setup Interrupt -Steering Wheel encoder + SteerSwitchbutton
  //if (IOSettings.SteerSwitchType > 0 ){ pinMode(STEERSW_PIN, INPUT); }
  //if (IOSettings.SteerSwitchType == 0) { pinMode(STEERSW_PIN, INPUT); }
  //pinMode(encAPin, INPUT);
  //pinMode(encBPin, INPUT);
  //Setup Interrupt -Steering Wheel encoder + SteerSwitchbutton
  //attachInterrupt(digitalPinToInterrupt(encAPin), EncoderA_ISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(encBPin), EncoderB_ISR, FALLING);
  if (IOSettings.SteerSwitchType >= 2) { attachInterrupt(digitalPinToInterrupt(STEERSW_PIN), Steersw_ISR, FALLING); }

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
//ISR SteerSwitch Interrupt
void Steersw_ISR() // handle pin change interrupt for Steersw Pin 
  {
   static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
   // If interrupts come faster than 300ms, assume it's a bounce and ignore
   if (interrupt_time - last_interrupt_time > 300 ) 
     {
      //steerEnable = !steerEnable;
      toggleSteerEnable=1;
     }
   last_interrupt_time = interrupt_time;
  }

 //ISR Steering Wheel Encoder
  void EncoderA_ISR()
  {       
    #if (SWEncoder >=0)      
         pulseACount++; 
         //digitalWrite(led1, !digitalRead(led1));
      
    #endif     
} 
 //ISR Steering Wheel Encoder
  void EncoderB_ISR()
  {       
    #if (SWEncoder >=0)      
         pulseBCount++; 
         //digitalWrite(led2, !digitalRead(led2));
    #endif     
} 
