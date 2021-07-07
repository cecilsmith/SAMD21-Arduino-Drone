void reading_receiver_signals(void) {
  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    delay(250);                                                                           //Print the receiver values on the screen every 250ms
    getStickPositions();
   
    if (Serial.available() > 0) {                                                         //If serial data is available
      data = Serial.read();                                                               //Read the incomming byte
      delay(100);                                                                         //Wait for any other bytes to come in
      while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer
    }
    
    //For starting the motors: throttle low and yaw left (step 1).
    if (pos[2] /*channel_3*/ < 1100 && pos[3]/*channel_4*/< 1100) {
      Serial.print("Start code 1");
      start = 1;
    }
    //When yaw stick is back in the center position start the motors (step 2).
    if (start == 1 && pos[2] /*channel_3*/ < 1100 && pos[3] /*channel_4*/ > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && pos[2] /*channel_3*/ < 1100 && pos[3] /*channel_4*/ > 1900)start = 0;

    Serial.print("Start:");
    Serial.print(start);

    Serial.print("  Roll:");
    if (pos[0] /*channel_1*/ - 1480 < 0)Serial.print("<<<");
    else if (pos[0] /*channel_1*/ - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(/*channel_1*/pos[0]);

    Serial.print("  Pitch:");
    if (/*channel_2*/pos[1] - 1480 < 0)Serial.print("^^^");
    else if (/*channel_2*/pos[1] - 1520 > 0)Serial.print("vvv");
    else Serial.print("-+-");
    Serial.print(/*channel_2*/pos[1]);

    Serial.print("  Throttle:");
    if (/*channel_3*/pos[2] - 1480 < 0)Serial.print("vvv");
    else if (/*channel_3*/pos[2] - 1520 > 0)Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(/*channel_3*/pos[2]);

    Serial.print("  Yaw:");
    if (/*channel_4*/pos[3] - 1480 < 0)Serial.print("<<<");
    else if (/*channel_4*/pos[3] - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.println(/*channel_4*/pos[3]);

    /*Serial.print("  CH5:");
    Serial.print(channel_5);

    Serial.print("  CH6:");
    Serial.println(channel_6);
    */
  }
  print_intro();
}
