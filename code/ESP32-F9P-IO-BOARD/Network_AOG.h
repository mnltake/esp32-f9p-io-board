// all about Network

#include <WiFi.h>
#include <AsyncUDP.h>

#include <WiFiAP.h>
#include <WiFiClient.h>


// Wifi variables & definitions

#define MAX_PACKAGE_SIZE 2048
char HTML_String[9300];
char HTTP_Header[160];

byte my_WiFi_Mode = 0;  // WIFI_STA = 1 = Workstation  WIFI_AP = 2  = Accesspoint
//---------------------------------------------------------------------
String _userAgent = "NTRIP CoffeetracNTRIPClient";

// Allgemeine Variablen
String _base64Authorization;
String _accept = "*/*";

#define ACTION_SET_SSID        1  
#define ACTION_SET_NTRIPCAST   2
#define ACTION_SET_SENDPOS     3
#define ACTION_SET_RESTART     4
#define ACTION_SET_GGA         5
#define ACTION_SET_NMEAOUT     6
#define ACTION_SET_AHRS        7
#define ACTION_SET_OUTPUT_TYPE 8  // also adress at EEPROM
#define ACTION_SET_WAS_TYPE    9
#define ACTION_SET_WAS_ZERO    10
#define ACTION_SET_WAS_INVERT  11
#define ACTION_SET_IMU_TYPE    12
#define ACTION_SET_INCLINO     13
#define ACTION_SET_INCL_ZERO   14
#define ACTION_SET_ENCODER     15
#define ACTION_SET_SWITCHES    16
#define ACTION_SET_THRESHOLD   17
int action;

// Radiobutton Select your Position type
char position_type[3][26] = {"Position Off", "Position Fixed via String", "GGA Position from GPS"};

// Radiobutton Select the time between repetitions.
char repeatPos[3][8] = {"1 sec.", "5 sec.", "10 sec."};

// Radiobutton Baudrate of serial1
char baud_output[6][7] = {"  9600", " 14400", " 19200", " 38400", " 57600", "115200"};

// Radiobutton Select if NMEA are transmitted via UDP.
char sendNmea[3][10] = {"OFF", "AOG-UDP", "Bluetooth"};

// Radiobutton Select if NTRIP Client is enabled. (Off to use only NMEA Transmission to AOG)
char ntripOn_type[3][9] = {"OFF", "ESP-WiFi", "AOG-UDP"};

//Inclinometer
char AHRS_tab[5][21] = {"No IMU","IMU BNO055", "MMA8452 Inclinometer","BNO055+MMA 8452","LSM9DS1 Inclinometer"};

// Radiobutton output
char output_driver_tab[6][22] = {"None", "Cytron MD30 + SWM", "IBT_2 +SWM", "IBT_2 +PWM Valve ", "IBT_2 +Danfoss Valve", "VNH7070 "};

// Radiobutton analog input
char was_input_tab[4][39] = {"Arduino/ESP direct", "ADS 1115 single", "ADSdiff  GND to A1, Signal to A0"," ADSdiff 5V to A3, Signal to A2"};

// Radiobutton WAS Invert
char was_invert_tab[2][15] = {"not inverted", "inverted"};

// Radiobutton IMU Heading Unit
char imu_type_tab[2][10] = {"None", "BNO 055"};

// Radiobutton Inclinometer
char inclino_type_tab[3][15] = {"None", "MMA ","LSM9DS1"};

// Radiobutton Steerswitch
char steersw_type_tab[5][15] = {"Switch High", "Switch Low", "Toggle Button", "Analog Buttons",""};

// Radiobutton Workswitch
char worksw_type_tab[4][8] = {"None", "Digital", "Analog", ""};

// Radiobutton WorkSwitch Invert
char worksw_invert_tab[2][15] = {"not inverted", "inverted"};

// Radiobutton Encoder
char encoder_type_tab[2][11] = {"None", "Installed"};

char tmp_string[20];
//---------------------------------------------------------------------
