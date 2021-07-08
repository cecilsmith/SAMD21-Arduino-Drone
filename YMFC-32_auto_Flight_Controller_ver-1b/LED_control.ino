
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These functions handle the red and green LEDs. The LEDs on the flip 32 are inverted. That is why a Flip32 test is needed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int8_t red_level=0, green_level=0;
void red_led(int8_t level) {
  return;
  display.clearDisplay();
  display.drawRect(0, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  red_level = level;
  if (red_level) {
    display.fillRect(0, 0, display.width()/4-1, display.height()-1, SSD1306_WHITE);
  }
  if (green_level) {
    display.fillRect(display.width()/4*3-1, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  }
  display.display();
}
void green_led(int8_t level) {
  return;
  display.clearDisplay();
  display.drawRect(0, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  green_level = level;
  if (red_level) {
    display.fillRect(0, 0, display.width()/4-1, display.height()-1, SSD1306_WHITE);
  }
  if (green_level) {
    display.fillRect(display.width()/4*3-1, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  }
  display.display();
}

void blink_red_led()
{
  display.clearDisplay();
  display.drawRect(0, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  if (red_level) red_level=0; else red_level=1;
  if (red_level) {
    display.fillRect(0, 0, display.width()/4-1, display.height()-1, SSD1306_WHITE);
  }
  if (green_level) {
    display.fillRect(display.width()/4*3-1, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  }
  display.display();

}
void blink_green_led()
{
  display.clearDisplay();
  display.drawRect(0, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  if (green_level) green_level=0; else green_level=1;
  if (red_level) {
    display.fillRect(0, 0, display.width()/4-1, display.height()-1, SSD1306_WHITE);
  }
  if (green_level) {
    display.fillRect(display.width()/4*3-1, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  }
  display.display();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the error LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void) {
  if (error >= 100) red_led(HIGH);                                                         //When the error is 100 the LED is always on.
  else if (error_timer < millis()) {                                                       //If the error_timer value is smaller that the millis() function.
    error_timer = millis() + 250;                                                          //Set the next error_timer interval at 250ms.
    if (error > 0 && error_counter > error + 3) error_counter = 0;                         //If there is an error to report and the error_counter > error +3 reset the error.
    if (error_counter < error && error_led == 0 && error > 0) {                            //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      red_led(HIGH);                                                                       //Turn the LED on.
      error_led = 1;                                                                       //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
      red_led(LOW);                                                                        //Turn the LED off.
      error_counter++;                                                                     //Increment the error_counter variable by 1 to keep trach of the flashes.
      error_led = 0;                                                                       //Set the LED flag to indicate that the LED is off.
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the flight mode LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flight_mode_signal(void) {
  if (flight_mode_timer < millis()) {                                                      //If the error_timer value is smaller that the millis() function.
    flight_mode_timer = millis() + 250;                                                    //Set the next error_timer interval at 250ms.
    if (flight_mode > 0 && flight_mode_counter > flight_mode + 3) flight_mode_counter = 0; //If there is an error to report and the error_counter > error +3 reset the error.
    if (flight_mode_counter < flight_mode && flight_mode_led == 0 && flight_mode > 0) {    //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      green_led(HIGH);                                                                     //Turn the LED on.
      flight_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
      green_led(LOW);                                                                      //Turn the LED off.
      flight_mode_counter++;                                                               //Increment the error_counter variable by 1 to keep trach of the flashes.
      flight_mode_led = 0;                                                                 //Set the LED flag to indicate that the LED is off.
    }
  }
}
