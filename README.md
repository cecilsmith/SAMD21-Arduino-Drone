# SAMD21-Arduino-Drone
This program utilizes IMU, GPS, and LiDAR data to fly an Arduino based drone on a SAMD 32 bit processor. The program is based on Joop Brokking's YMFC-32 open-source code for 32 bit STM Arduino processors but has been adapted to be used with different sensors.


### Credit to Joop Brokking for the majority of the software used. For general build instructions and more information, go to http://www.brokking.net/ymfc-32_main.html


*Download and run the test files to make sure sensors and motors are running correctly before attempting to run the drone controller program.*


#### **Libraries used:**

  * DimmerZero (Communication from receiver): https://github.com/Adminius/DimmerZero

  * TFMini (Lidar): https://github.com/budryerson/TFMini-Plus

  * U-Blox GNSS (GPS): https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

  * Arduino GFX library: https://github.com/adafruit/Adafruit-GFX-Library

  * SSD1306 (Display): https://github.com/adafruit/Adafruit_SSD1306

  * BNO055 (IMU): https://github.com/adafruit/Adafruit_BNO055

  * Adafruit Sensor Library: https://github.com/adafruit/Adafruit_Sensor

 

#### **Sensor Documentation:**

  * BNO055 IMU: https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf

 

#### **Only Tested on MKR WiFi 1010 Board**

MKR WiFi 1010 Pinout Info:
~TBA~
