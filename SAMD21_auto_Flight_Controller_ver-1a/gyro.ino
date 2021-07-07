///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the various registers of the MPU-6050 are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void) {
  /*Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
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
  Wire.endTransmission();                                      //End the transmission with the gyro.*/

  acc_pitch_cal_value  = EEPROM.read(0x16);
  acc_roll_cal_value  = EEPROM.read(0x17);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  cal_int = 0;                                                                        //Set the cal_int variable to zero.
  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                    //Change the led status every 125 readings to indicate calibration.
      gyro_signalen();                                                                //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
      delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    red_led(HIGH);                                                                     //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  /*Wire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  Wire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                     //End the transmission.
  Wire.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.
  acc_y = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_x variable.
  acc_x = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_y variable.
  acc_z = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();              //Add the low and high byte to the temperature variable.
  gyro_roll = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
  gyro_pitch = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data.
  gyro_yaw = Wire.read() << 8 | Wire.read();                 //Read high and low part of the angular data.
  gyro_pitch *= -1;                                            //Invert the direction of the axis.
  gyro_yaw *= -1;*/                                             //Invert the direction of the axis.

  //BNO055 specific code:
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
 
  acc_y = accelerometerData.acceleration.x;              //Add the low and high byte to the acc_x variable.
  acc_x = accelerometerData.acceleration.y;              //Add the low and high byte to the acc_y variable.
  acc_z = accelerometerData.acceleration.z;              //Add the low and high byte to the acc_z variable.
  
  gyro_roll = angVelocityData.gyro.y;    //roll          //Read high and low part of the angular data.
  gyro_pitch = angVelocityData.gyro.z;    //pitch        //Read high and low part of the angular data.
  gyro_yaw = angVelocityData.gyro.x;    //yaw            //Read high and low part of the angular data.
  gyro_pitch *= -1;                                        //Invert gyro so that nose up gives positive value.
  gyro_yaw *= -1;                                          //Invert gyro so that nose right gives positive value.

  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;                            //Subtact the manual gyro roll calibration value.
    gyro_pitch -= gyro_pitch_cal;                            //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= gyro_yaw_cal;                            //Subtact the manual gyro yaw calibration value.
  }

  //Why 0, 2, 3? Should it be 0, 1, 2?
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

  //Old code that may be unnecessary:
  if (level_calibration_on == 0) {
    acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
    acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
  }
  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;                                  //Subtact the manual gyro roll calibration value.
    gyro_pitch -= gyro_pitch_cal;                                //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= gyro_yaw_cal;                                    //Subtact the manual gyro yaw calibration value.
  }
}
