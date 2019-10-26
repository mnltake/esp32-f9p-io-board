/*
 * External connections during the test: 
 *  - 14V external power (J4)
 *  - USB connected and terminal open
 *  - Jumper on J2 (after powerup)
 *  - short RX/TX on J9 (after powerup)
 *  - RS232 plug that shortens RX/TX (pin 2 and 3)
 *  - M1 - I1, I2:
 *    J4-M1A / J1-I1 <-> 20 Ohm <-> J4-M1B / J1-I2
 *    + J4-M1A connected with J1-I1
 *    + J4-M1B connected with J1-I2
 *    + J4-M1A connected to a 20Ohm resistor to J4-M1B
*  - M2 - I3:
 *    J3-M2A <-> 10 Ohm <-> J1-I3 <-> 10 Ohm <-> J3-MB 
 *    + J3-M2A connected with 10 Ohm resistor
 *    + J3-M2B connected with 10 Ohm resistor
 *    + J4-M1B is connected with both resistors
 *  - Relay K2 / ADS1115
 *    + J5-A0 <-> J5-GND
 *    + J5-A1 <-> J3-NC
 *    + J5-5V <-> J3-COM
 *    + J5-A2 <-> J3-NO
 *  - Ethernet connected to a network with a DHCP server
 * 
 * 
 *  Uses
 *    + arduino-can: https://github.com/sandeepmistry/arduino-CAN
 */

// IO pins --------------------------------
#define ANALOG_INPUT1 36
#define ANALOG_INPUT2 39
#define ANALOG_INPUT3 34
#define I2C_SDA 32
#define I2C_SCL 33
#define VNH_A_PWM 4
#define VNH_B_PWM 12
#define F9P_RX 13
#define F9P_TX 14
#define RS232_RX 16
#define RS232_TX 15
#define UART_RX 2
#define UART_TX 0
#define CAN_TX 5
#define CAN_RX 35
// Ethernet
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN   -1
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        0
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18

// includes
#include "Adafruit_ADS1015.h"
#include <Wire.h>
#include "driver/gpio.h"
#include <LSM9DS1_Registers.h>
#include <SparkFunLSM9DS1.h>
#include <LSM9DS1_Types.h>
#include <ETH.h>
#include <CAN.h>

// global variables
int check = 1;
byte byteRead;
byte sentByte = 'U';
byte ethernetStatus = -1;


// instances
LSM9DS1 imu;

<<<<<<< Updated upstream
uint8_t getByteI2C(int address, int i2cregister) {
  Wire.beginTransmission(address);
  Wire.write(i2cregister);
  Wire.endTransmission(false);
  uint8_t state = Wire.requestFrom(address, 1, (int)true);
  return Wire.read();
}


uint8_t setByteI2C(int address, byte i2cregister, byte value) {
  Wire.beginTransmission(address);
  Wire.write(i2cregister);
  Wire.write(value);
  return Wire.endTransmission();
}

=======
>>>>>>> Stashed changes
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial.println("-------------------------------------");
  Serial.println("ESP32 - F9P - IO-Board Tester");
  Serial.println("-------------------------------------");
  Serial.println("");
  // Setup IO
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  // initialize PINs for VNHs
  pinMode(VNH_A_PWM, OUTPUT);
  pinMode(VNH_B_PWM, OUTPUT);
<<<<<<< Updated upstream
  ledcSetup(0, 1500, 8);
  ledcAttachPin(VNH_A_PWM, 0);
=======

>>>>>>> Stashed changes
  // analog inputs - set input to explicit disable any pullups
  pinMode(ANALOG_INPUT1, INPUT);
  pinMode(ANALOG_INPUT2, INPUT);
  pinMode(ANALOG_INPUT3, INPUT);
  analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. But needed for full scale range  Recommended to use 2.5 or 6.

  // Serial for F9P, RS232, Light
  gpio_pad_select_gpio(GPIO_NUM_13);
  gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
  
  gpio_pad_select_gpio(GPIO_NUM_15);
  gpio_pad_select_gpio(GPIO_NUM_16);
  gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_16, GPIO_MODE_INPUT);
  
  gpio_pad_select_gpio(GPIO_NUM_2);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

  // PINs for CAN
  pinMode(CAN_RX, INPUT);
  gpio_pad_select_gpio(GPIO_NUM_35);
  gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
  pinMode(CAN_TX, OUTPUT);
<<<<<<< Updated upstream
  Serial.println("");
  Serial.println("Configure the FXL6408");
        // direction (Input/Output)
        setByteI2C(0x43, 0x03, 0b11111110);
        // disable High-Z on outputs
        setByteI2C(0x43, 0x07, 0b00000001);
        // en-/disable Pullup/downs
        setByteI2C(0x43, 0x0B, 0b00000001);
        // set direction of the pull
        setByteI2C(0x43, 0x0D, 0b00000001);
        // enable gpio 2 (LED)
        setByteI2C(0x43, 0x05, 0b00000100);
        Serial.println("LED on, press SW1 to disable LED and continue");
        
        // input state
        int value = getByteI2C(0x43, 0x0F);
        // only intrested in gpio0
        value = value & 0b00000001;
        while (value == 1) {
          delay(25);
          value = getByteI2C(0x43, 0x0F);
          // only intrested in gpio0
          value = value & 0b00000001;
        }
        // LED off
        setByteI2C(0x43, 0x05, 0b00000000);
}







void loop() {
        Adafruit_ADS1115 ads = Adafruit_ADS1115(0x48);
        int a0,a1,a2,a3,angle,pwm;


        // read data
        //a0 = ads.readADC_SingleEnded(0);
        //a1 = ads.readADC_SingleEnded(1);
        a2 = ads.readADC_SingleEnded(2);
        //a3 = ads.readADC_SingleEnded(3); 
        angle = map(a2,0,21600, -45, 45);
        pwm = map(a2,0,21600,-255,255);
        Serial.print(" - a2: ");
        Serial.println(a2);
        Serial.print(" - angle: ");
        Serial.println(angle);
        Serial.print(" - pwm: ");
        Serial.println(pwm);
        if (pwm > 0){
          ledcWrite(0, pwm);
          setByteI2C(0x43, 0x05, 0b01000000);
        }else{
          ledcWrite(0, -pwm);
          setByteI2C(0x43, 0x05, 0b10000000);
        }


=======

  Serial.println("Short J2 and pull up RS232");
  Serial.println("Press to continue");


}

void i2cTestAdress(byte address) {
  byte error;

    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.println("Found");
    } else if (error == 4) {
      Serial.println("Unknown error");
    } else {
      Serial.println("Not found");
    }
  }


uint8_t getByteI2C(int address, int i2cregister) {
  Wire.beginTransmission(address);
  Wire.write(i2cregister);
  Wire.endTransmission(false);
  uint8_t state = Wire.requestFrom(address, 1, (int)true);
  return Wire.read();
}


uint8_t setByteI2C(int address, byte i2cregister, byte value) {
  Wire.beginTransmission(address);
  Wire.write(i2cregister);
  Wire.write(value);
  return Wire.endTransmission();
}

void loop() {
  


        Serial.println("");
        Serial.println("Test ADS1115");

        // "init"
        Adafruit_ADS1115 ads = Adafruit_ADS1115(0x48);
        int a0,a1,a2,a3,diff;


        // read data
        a0 = ads.readADC_SingleEnded(0);
        a1 = ads.readADC_SingleEnded(1);
        a2 = ads.readADC_SingleEnded(2);
        a3 = ads.readADC_SingleEnded(3); 
        diff = ads.readADC_Differential_2_3();
          Serial.print(" a0: ");
          Serial.print(a0);
          Serial.print(" - a1: ");
          Serial.print(a1);
          Serial.print(" - a2: ");
          Serial.print(a2);
          Serial.print(" - a3: ");
          Serial.println(a3);
          Serial.print(" - diff2_3: ");
          Serial.println(diff);
>>>>>>> Stashed changes
        delay(500);
}
