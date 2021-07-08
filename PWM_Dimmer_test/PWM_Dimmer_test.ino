#define HAS_SERIAL 1

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

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BNO055 9 axis imu
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

uint16_t GPS_SAMPLERATE_DELAY_MS = 1200; //how often to read data
uint16_t LIDAR_SAMPLERATE_DELAY_MS = 50; //how often to read data
uint16_t DISPLAY_SAMPLERATE_DELAY_MS = 50; //how often to read data

//UART assignment to pins 0 and 1
Uart Serial10 (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0

//variables for reciever
byte input[32];
int cnt=0;
char str[32];
int pos[4];
bool rfFlag=false;
bool rfFirstChar=false;
int count=0;

//Create an instance for pwm_1 on pin PA20 in inverted mode
DimmerZero pwm_1(4,false);
//Create an instance for pwm_2 on pin PA21 in inverted mode
DimmerZero pwm_2(2,false);
//Create an instance for pwm_1 on pin PA16 in inverted mode
DimmerZero pwm_3(6,false);
//Create an instance for pwm_2 on pin PA17 in inverted mode
DimmerZero pwm_4(7,false);

int periodCounts;
int periodTimeMs;
int minPulseValue;
int maxPulseValue;

void setup() {
 #if HAS_SERIAL
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("cM Drone code");
#endif

  Wire.begin();
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

  // Initialize serial port for remote controll
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
  
}

// Initialize variables
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
unsigned long lastLidar=0,lastGPS=0,lastIMU=0,lastPrint=0, lastDisplayUpdate=0;
unsigned long loop_cnt=0;
sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
long latitude;
long longitude;
long altitude;
byte SIV;

void loop() {
  loop_cnt++;
  int max = pwm_1.getMaxValue();
  if (getStickPositions()) {
    pwm_1.setValue(pos[0]*3/4);
    pwm_2.setValue(pos[1]*3/4);
    pwm_3.setValue(pos[2]*3/4);
    pwm_4.setValue(pos[3]*3/4);
  }
  unsigned long tStart = micros();
  bool lidarFlag=false;
  char line1[32], line2[32], line3[32], line4[32];

  if (millis() - lastLidar > LIDAR_SAMPLERATE_DELAY_MS)
  {
    lastLidar = millis(); //Update the timer
    if( tfmP.getData(tfDist, tfFlux, tfTemp)) // Get data from the device.
    {
      lidarFlag=true;
    } else {
      lidarFlag=false;
#if HAS_SERIAL
      tfmP.printFrame();  // display the error and HEX data
#endif
    }
  }
  if (millis() - lastPrint > PRINT_DELAY_MS) {
    Serial.println(loop_cnt);
    lastPrint = millis();
    loop_cnt=0;
    //sprintf(line1, "CM: %d, Stren: %d", tfDist, tfFlux);
    //sprintf(line2, "Strength: %d", tfFlux);
    //sprintf(line3, "Temp: %d", tfTemp);
    line4[0]=0;
    //sprintf(line4, "Alt: %ld", altitude);
    //lcd_print(line1, line2, line3, line4);

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    printEvent(&gravityData);
  }
  
  if (millis() - lastIMU > BNO055_SAMPLERATE_DELAY_MS) {
    lastIMU = millis();
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  }

/**/
  if (millis() - lastGPS > GPS_SAMPLERATE_DELAY_MS) {
    lastGPS = millis();
    latitude = myGNSS.getLatitude();
    longitude = myGNSS.getLongitude();
    altitude = myGNSS.getAltitude();
    SIV = myGNSS.getSIV();
  }

  if (millis() - lastDisplayUpdate > DISPLAY_SAMPLERATE_DELAY_MS) {
    lastDisplayUpdate = millis();
    sprintf(line1, "CM: %d, Stren: %d", tfDist, tfFlux);
    sprintf(line2, "Lat: %d", latitude);
    sprintf(line3, "Long: %d", longitude);
    sprintf(line4, "SIV: %d", SIV);
    lcd_print(line1 ,line2, line3, line4);
    Serial.println(str);
  }
/**/
}

//function for tracking stick position on reciever
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
      
      //Serial.print(str);     //relocated to within display update in loop
      //Serial.println("");
      return true;
    }
   }
   return false;
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler()
{

  Serial10.IrqHandler();
}

void lcd_print(char *line1, char *line2, char *line3, char *line4) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  display.setCursor(0, 0);     // Start at top-left corner 
  display.write(line1);
  display.setCursor(0, 8);     // Start at top-left corner 
  display.write(line2);
  display.setCursor(0, 16);     // Start at top-left corner 
  display.write(line3);
  display.setCursor(0, 24);     // Start at top-left corner 
  display.write(line4);
  display.display();
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
