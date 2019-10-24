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
////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

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

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8N1, F9P_RX, F9P_TX);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  Serial.println("-------------------------------------");
  Serial.println("ESP32 - F9P - IO-Board Tester-IMU");
  Serial.println("-------------------------------------");
  Serial.println("");
  // Setup IO

  // initialize PINs for VNHs
  pinMode(VNH_A_PWM, OUTPUT);
  pinMode(VNH_B_PWM, OUTPUT);

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

  Serial.println("Press to continue");
  while (Serial.read() == -1 ) {
    delay(1);
  }

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
void loop()
{
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    //printGyro();  // Print "G: gx, gy, gz"
    //printAccel(); // Print "A: ax, ay, az"
    //printMag();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    printAttitude(imu.ax, imu.ay, imu.az, 
                 -imu.my, -imu.mx, imu.mz);
    Serial.println();
    
    lastPrint = millis(); // Update lastPrint time
  }
}


void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();

  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();

  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(
  float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  //  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  //Serial.print("Pitch, Roll: ");
  //Serial.print(pitch, 2);
  //Serial.print(", ");
  Serial.println(roll, 2);
 // Serial.print("Heading: "); Serial.println(heading, 2);
}

void setupDevice()
{
  // [commInterface] determines whether we'll use I2C or SPI
  // to communicate with the LSM9DS1.
  // Use either IMU_MODE_I2C or IMU_MODE_SPI
  imu.settings.device.commInterface = IMU_MODE_I2C;
  // [mAddress] sets the I2C address or SPI CS pin of the
  // LSM9DS1's magnetometer.
  imu.settings.device.mAddress = 0x1C; // Use I2C addres 0x1E
  // [agAddress] sets the I2C address or SPI CS pin of the
  // LSM9DS1's accelerometer/gyroscope.
  imu.settings.device.agAddress = 0x6A; // I2C address 0x6B
}

void setupGyro()
{
  // [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;  // Enable the gyro
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245; // Set scale to +/-245dps
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 2; // 59.5Hz ODR
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  // [flipX], [flipY], and [flipZ] are booleans that can
  // automatically switch the positive/negative orientation
  // of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel()
{
  // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 2; // Set accel scale to +/-8g.
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 1; // Set accel to 10Hz.
  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3.
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 0; // BW = 408Hz
  // [highResEnable] enables or disables high resolution
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = true; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;
}

void setupMag()
{
  // [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
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
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode
}

void setupTemperature()
{
  // [enabled] turns the temperature sensor on or off.
  imu.settings.temp.enabled = true;
}

uint16_t initLSM9DS1()
{
  setupDevice(); // Setup general device parameters
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  setupTemperature(); // Set up temp sensor parameter

  return imu.begin();
}
