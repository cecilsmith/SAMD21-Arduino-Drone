///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

//HardWire /*H*/Wire(2, I2C_FAST_MODE);
//TwoWire /*H*/Wire (2, I2C_FAST_MODE);


#include <Arduino.h>
#include <Wire.h>

#include "wiring_private.h"
#include <DimmerZero.h>

#include <TFMPlus.h>  // Include TFMini Plus Library v1.4.2
TFMPlus tfmP;         // Create a TFMini Plus object


// OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BNO055 9 axis imu
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//UART assignment to pins 0 and 1
Uart Serial10 (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 20; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 1000; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// GPS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//Let's declare some variables so we can use them in the complete program.
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
uint8_t disable_throttle;
uint8_t error;
uint32_t loop_timer;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;
int16_t loop_counter;
uint8_t data, start, warning;
int16_t acc_axis[4], gyro_axis[4] , temperature;
int32_t gyro_axis_cal[4], acc_axis_cal[4];
int32_t orientation[3];
int32_t cal_int;
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;


//Barometer variables.
uint16_t C[7];
uint8_t barometer_counter, temperature_counter;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;

//Compass_variables.
int16_t compass_x, compass_y, compass_z;

/*uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
uint8_t MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
uint8_t compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.
*/

//Create an instance for pwm_1 on pin PA20 in inverted mode
DimmerZero pwm_1(4,false);
//Create an instance for pwm_2 on pin PA21 in inverted mode
DimmerZero pwm_2(2,false);
//Create an instance for pwm_1 on pin PA16 in inverted mode
DimmerZero pwm_3(6,false);
//Create an instance for pwm_2 on pin PA17 in inverted mode
DimmerZero pwm_4(7,false);

//variables for reciever
byte input[32];
int cnt=0;
char str[32];
int pos[4];
bool rfFlag=false;
bool rfFirstChar=false;
int count=0;

int periodCounts;
int periodTimeMs;
int minPulseValue;
int maxPulseValue;

sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;

// Initialize variables
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
long latitude;
long longitude;
long altitude;
byte SIV;

void setup() {
  //NEEDED
  //pinMode(4, INPUT_ANALOG); //Battery Voltage

  //DISPLAY
  /*
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                     //Connects PB3 and PB4 to output function.

  //On the Flip32 the LEDs are connected differently. A check is needed for controlling the LEDs.
  pinMode(PB3, INPUT);                                         //Set PB3 as input.
  pinMode(PB4, INPUT);                                         //Set PB4 as input.
  if (digitalRead(PB3) && digitalRead(PB4))flip32 = 1;         //Input PB3 and PB4 are high on the Flip32
  else flip32 = 0;
  flip32 = 0;

  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output.

  green_led(LOW);                                               //Set output PB3 low.
  red_led(LOW);                                                 //Set output PB4 low.
*/
  Serial.begin(115200);                                          //Set the serial output to 57600 kbps.
  delay(100);                                                    //Give the serial port some time to start to prevent data loss.
  timer_setup();      //Make sure throttle is setup correctly                                      //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to start.

  Wire.begin();                                                //Start the I2C as master

  tfmP.begin(&Serial1);   // Initialize device library object and pass device serial port to the object.

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
#if HAS_SERIAL
    Serial.println(F("SSD1306 allocation failed"));
#endif
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();

  /* Initialise the IMU */
  if (!bno.begin())
  {
#if HAS_SERIAL
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
#endif
    while (1);
  }

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
#if HAS_SERIAL
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
#endif
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  // Initialize serial port for remote control
  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0

  Serial10.begin(115200); //Start secondary serial (for UART)
  Serial1.begin(112500);


  // Initialize PWM
  //change frequnecy to 250Hz for all channels
  pwm_1.setFrequency(50);
  pwm_2.setFrequency(50);
  pwm_3.setFrequency(50);
  pwm_4.setFrequency(50);

  //initialize channels
  pwm_1.init();
  pwm_2.init();
  pwm_3.init();
  pwm_4.init();

  periodCounts = pwm_1.getMaxValue();
  periodTimeMs = 20;
  maxPulseValue = periodCounts/20*2;  //1500
  minPulseValue = periodCounts/20;    //750
  pwm_1.setValue(minPulseValue);
  pwm_2.setValue(minPulseValue);
  pwm_3.setValue(minPulseValue);
  pwm_4.setValue(minPulseValue);
/*
  //Config for GYRO
  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  Wire.endTransmission();                                      //End the transmission with the gyro.
*/

  print_intro();                                                //Print the intro on the serial monitor.
}

void loop() {
  //delay(10);
  //get reciever data
  int max = pwm_1.getMaxValue();
  if (getStickPositions()) {
    pwm_1.setValue(pos[0]*3/4);
    pwm_2.setValue(pos[1]*3/4);
    pwm_3.setValue(pos[2]*3/4);
    pwm_4.setValue(pos[3]*3/4);
  }
  
  if (Serial.available() > 0) {
    data = Serial.read();                                       //Read the incomming byte.
    delay(100);                                                 //Wait for any other bytes to come in.
    while (Serial.available() > 0)loop_counter = Serial.read(); //Empty the Serial buffer.
    disable_throttle = 1;                                       //Set the throttle to 1000us to disable the motors.
  }
  
  /*
  if (!disable_throttle) {                                      //If the throttle is not disabled.
    TIMER4_BASE->CCR1 = channel_3;                              //Set the throttle receiver input pulse to the ESC 1 output pulse.
    TIMER4_BASE->CCR2 = channel_3;                              //Set the throttle receiver input pulse to the ESC 2 output pulse.
    TIMER4_BASE->CCR3 = channel_3;                              //Set the throttle receiver input pulse to the ESC 3 output pulse.
    TIMER4_BASE->CCR4 = channel_3;                              //Set the throttle receiver input pulse to the ESC 4 output pulse.
  }
  else {                                                        //If the throttle is disabled
    TIMER4_BASE->CCR1 = 1000;                                   //Set the ESC 1 output to 1000us to disable the motor.
    TIMER4_BASE->CCR2 = 1000;                                   //Set the ESC 2 output to 1000us to disable the motor.
    TIMER4_BASE->CCR3 = 1000;                                   //Set the ESC 3 output to 1000us to disable the motor.
    TIMER4_BASE->CCR4 = 1000;                                   //Set the ESC 4 output to 1000us to disable the motor.
  }*/

    if (data == 'a') {
    Serial.println(F("Reading receiver input pulses."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    reading_receiver_signals();
  }

  if (data == 'b') {
    Serial.println(F("Starting the I2C scanner."));
    i2c_scanner();
  }

  if (data == 'c') {
    Serial.println(F("Reading raw gyro data."));
    Serial.println(F("You can exit by sending a q (quit)."));
    read_gyro_values();
  }

  if (data == 'd') {
    Serial.println(F("Reading the raw accelerometer data."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    read_gyro_values();
  }

  if (data == 'e') {
    Serial.println(F("Reading the IMU angles."));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_imu_angles();
  }

  if (data == 'f') {
    Serial.println(F("Test the LEDs."));
    test_leds();
  }

  if (data == 'g') {
    Serial.println(F("Reading the battery voltage."));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_battery_voltage();
  }

  if (data == 'h') {
    Serial.println(F("Checking MS-5611 barometer/distance."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_barometer();
  }

  if (data == 'i') {
    Serial.println(F("Checking raw GPS data."));
    check_gps();
  }

  if (data == 'j') {
    Serial.println(F("Checking HMC5883L compass."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_compass();
  }

  if (data == '1') {
    Serial.println(F("Check motor 1 (front right, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '2') {
    Serial.println(F("Check motor 2 (rear right, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '3') {
    Serial.println(F("Check motor 3 (rear left, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '4') {
    Serial.println(F("Check motor 4 (front lefft, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '5') {
    Serial.println(F("Check motor all motors."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '6') {
    gyro_signalen();
  }
}

void gyro_signalen(void) {
  //NOT WORKING
  //Read the MPU-6050 data. --READ IMU
  /*Wire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  Wire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                     //End the transmission.
  Wire.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.
*/
  //BNO055 specific code:
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
 
  acc_axis[1] = accelerometerData.acceleration.x;  /*Wire.read() << 8 | Wire.read();*/              //Add the low and high byte to the acc_x variable.
  acc_axis[2] = accelerometerData.acceleration.y;  /*Wire.read() << 8 | Wire.read();*/              //Add the low and high byte to the acc_y variable.
  acc_axis[3] = accelerometerData.acceleration.z;  /*Wire.read() << 8 | Wire.read();*/              //Add the low and high byte to the acc_z variable.
  //temperature = Wire.read() << 8 | Wire.read();              //Add the low and high byte to the temperature variable.
  gyro_axis[1] = angVelocityData.gyro.y;    //roll     /*Wire.read() << 8 | Wire.read();*/             //Read high and low part of the angular data.
  gyro_axis[2] = angVelocityData.gyro.z;    //pitch     /*Wire.read() << 8 | Wire.read();*/             //Read high and low part of the angular data.
  gyro_axis[3] = angVelocityData.gyro.x;    //yaw     /*Wire.read() << 8 | Wire.read();*/             //Read high and low part of the angular data.
  gyro_axis[2] *= -1;                                          //Invert gyro so that nose up gives positive value.
  gyro_axis[3] *= -1;                                          //Invert gyro so that nose right gives positive value.

  if (cal_int >= 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];                            //Subtact the manual gyro roll calibration value.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Subtact the manual gyro pitch calibration value.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Subtact the manual gyro yaw calibration value.
  }
  orientation[0] = orientationData.orientation.x;
  orientation[2] = orientationData.orientation.y;
  orientation[3] = orientationData.orientation.z;

/*
  Serial.print(orientationData.orientation.x);Serial.print(", ");
  Serial.print(magnetometerData.magnetic.x);Serial.print(", ");
  Serial.print(orientationData.orientation.y);Serial.print(", ");
  //Serial.print(magnetometerData.magnetic.y);Serial.print(", ");
  Serial.print(orientationData.orientation.z);
  //Serial.print(magnetometerData.magnetic.z);Serial.print(", ");
  Serial.println("gyro update called");
*/
  //Serial.print("*");
}

void red_led(int8_t level) {
  if (level) {
    
  } else {
    
  }
}

void green_led(int8_t level) {
  if (flip32)digitalWrite(PB3, !level);
  else digitalWrite(PB3, level);
}
  
bool getStickPositions() {
  if (Serial10.available()) {
    int h = Serial10.read();
    //sprintf(str, "%x, ", h);
    if (h == 0x04) {
      rfFirstChar=true;
      cnt=0;
    } else {
      if (rfFirstChar) {
        if ((cnt == 0) && (h == 0xdc)) {
          rfFlag=true;
        } else {
          rfFlag=false;
        }
      }
      rfFirstChar=false;
    }
    if (rfFlag) {
      input[cnt+1] = h;
      cnt++;
    }
    if (cnt == 31) {
      rfFlag=false;
      for (int i=0; i<4; i++) {
        pos[i] = input[21+i*2] + input[21+i*2+1] * 256;
      }
      sprintf(str, "%d, %d, %d, %d ", pos[0],pos[1],pos[2],pos[3]);
      //Serial.print(str);
      //Serial.println("");
      return true;
    }
   }
   return false;
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler() {

  Serial10.IrqHandler();
}
