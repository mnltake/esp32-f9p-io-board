//Core2: this task serves the Webpage and handles the GPS NMEAs

void Core2code( void * pvParameters ){
  
  DBG("\nTask2 running on core ");
  DBG((int)xPortGetCoreID(), 1);

  
// Start WiFi Client
 while (!EE_done){  // wait for eeprom data
  delay(10);
 }
 WiFi_Start_STA();
 if (my_WiFi_Mode == 0) WiFi_Start_AP(); // if failed start AP

 repeat_ser = millis();
  if ((IOSettings.AHRSbyte == 1)|(IOSettings.AHRSbyte == 3)) {   // Initialize the BNO055 if not done
	 if (imu_initialized == 0) {
		 initBNO055();
		 imu_initialized = 1;
	 }
	 else {		//  no IMU
		 imu_initialized = 0;
		 Head = 0;
		 Yaw = 0;
	 }
  }
  udpAOG.listen(portMy);
  UDPReceiveNtrip();

  
 for(;;){ // MAIN LOOP FOR THIS CORE
  WiFi_Traffic();
  Serial_Traffic();
  //* Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = millis();
  unsigned int time = currentTime;
  
  if (currentTime - lastTime >= LOOP_TIME)
  {
    dT = currentTime - lastTime;
    lastTime = currentTime;
 /*  
 if (IOSettings.IMU_type ==1){   // Initialize the BNO055 if not done
   if (imu_initialized==0){
     initBNO055(); 
     imu_initialized=1;
    }
    readEulData(EulCount);  // Read the x/y/z adc values   
    // Calculate the Euler angles values in degrees
    Head = (float)EulCount[0];
    Yaw = Head/16.;  
  }
*/

   //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = 20;

 if (IOSettings.Inclino_type ==1){
   // MMA8452 (1) Inclinometer
   if (accelerometer.acc_initialized==0){
    if(!accelerometer.init()) IOSettings.Inclino_type = 0;} // Try to Initialize MMA8452
   accelerometer.getRawData(&x_, &y_, &z_);
   roll=x_; //Conversion uint to int
   if (roll > 8500)  roll =  8500;
   if (roll < -8500) roll = -8500;
   roll -= IOSettings.roll_corr;  // 
   rollK = map(roll,-8500,8500,-480,480); //16 counts per degree (good for 0 - +/-30 degrees) 
  }

 if (IOSettings.Inclino_type ==2){
   // LSM9DS1 (2) Inclinometer
   imu.begin();
   if ( imu.gyroAvailable() )imu.readGyro();
   if ( imu.accelAvailable() )imu.readAccel();
   if ( imu.magAvailable() ) imu.readMag();
   float degroll = atan2(imu.ay, imu.az);
   degroll  *= 2880.0 / PI; //180*16=2880  16 counts per degree (good for 0 - +/-30 degrees) 
   //Serial.println(degroll, 4);
   degroll -= IOSettings.roll_corr;  // 
   rollK = int(degroll); //int 

   // LSM9DS1 mag Heading
  float heading;
  if (imu.my == 0)
    heading = (imu.mx < 0) ? PI : 0;
  else
    heading = atan2(imu.mx, imu.my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  Head = heading * 16;
  //Serial.print("Head:");
  //Serial.println(Head);
  }
 //Kalman filter
    Pc = P + varProcess;
    G = Pc / (Pc + varRoll);
    P = (1 - G) * Pc;
    Xp = XeRoll;
    Zp = Xp;
    XeRoll = G * (rollK - Zp) + Xp;

    switch (IOSettings.WorkSW_mode)
  {
  case 1:
    if (IOSettings.Invert_WorkSW == 0) workSwitch = digitalRead(WORKSW_PIN);    // read digital work switch
    if (IOSettings.Invert_WorkSW == 1) workSwitch = !digitalRead(WORKSW_PIN);    // read digital work switch
    break;
  case 2:
    AnalogValue = analogRead(WORKSW_PIN);
    delay(1);
    AnalogValue += analogRead(WORKSW_PIN);
    delay(1);
    AnalogValue += analogRead(WORKSW_PIN);
    delay(1);
    AnalogValue += analogRead(WORKSW_PIN);
    AnalogValue = AnalogValue >> 2;
    if (IOSettings.Invert_WorkSW == 0){
      if (AnalogValue < IOSettings.WorkSW_Threshold)   workSwitch = 1;
      else workSwitch = 0;
     }
    
    if (IOSettings.Invert_WorkSW == 1){
      if (AnalogValue > IOSettings.WorkSW_Threshold)   workSwitch = 1;
      else workSwitch = 0; 
     }
    break;
  }
  
  if (workSwitch != workSwitchOld) {
    if (workSwitch > 0) {
      Serial.println("workswitch: ON");
      workSwitchOld = workSwitch;
    }
    else {
      Serial.println("workSwitch: OFF");
      workSwitchOld = workSwitch;
    }
  }

    state_previous=steerEnable;    // Debug only
    if (pulseACount + pulseBCount >= IOSettings.pulseCountMax && pulseACount >0 && pulseBCount >0 && IOSettings.SWEncoder ){
       steerEnable=false;
       //watchdogTimer = 20;  // turn off steering
      }
if (steerEnable != state_previous) Serial.println("Steer-Break: Encoder.."); // Debug only
    steerSwitch = steerEnable;  //digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
    steerSwitch <<= 1; //put steerswitch status in bit 1 position
    switchByte = workSwitch | steerSwitch;

    SetRelays(); //turn on off sections

//steering position and steer angle
  switch (IOSettings.input_type) {
    case 1:  // ADS 1115 single
      steeringPosition = ads.readADC_SingleEnded(0);    delay(1);           //ADS1115 Standard Mode
      steeringPosition += ads.readADC_SingleEnded(0);    delay(1);
      steeringPosition += ads.readADC_SingleEnded(0);    delay(1);
      steeringPosition += ads.readADC_SingleEnded(0);     
      break;
    case 2:  // ADS 1115 differential
      steeringPosition = ads.readADC_Differential_0_1();    delay(1);    //ADS1115 Differential Mode 
      steeringPosition += ads.readADC_Differential_0_1();   delay(1);    //Connect Sensor GND to A0
      steeringPosition += ads.readADC_Differential_0_1();   delay(1);    //Connect Sensor Signal to A1
      steeringPosition += ads.readADC_Differential_0_1();
      break;
    case 3:  // ADS 1115 differential A3-5v A2-Singal
      steeringPosition = ads.readADC_Differential_2_3();    delay(1);    //ADS1115 Differential Mode 
      steeringPosition += ads.readADC_Differential_2_3();   delay(1);    //Connect Sensor 5V to A3
      steeringPosition += ads.readADC_Differential_2_3();   delay(1);    //Connect Sensor Signal to A2
      steeringPosition += ads.readADC_Differential_2_3();
      break;
   default: // directly to arduino
      steeringPosition = analogRead(W_A_S);    delay(1);
      steeringPosition += analogRead(W_A_S);    delay(1);
      steeringPosition += analogRead(W_A_S);    delay(1);
      steeringPosition += analogRead(W_A_S);
      break;

  }
    steeringPosition = steeringPosition >> 2; //divide by 4
    if (IOSettings.input_type == 3){ //map Raw to -45 +45degree
      steeringPosition = map(steeringPosition,steeringPositionRawMim,steeringPositionRawMax,-4500,4500);
    }
    actualSteerPos=steeringPosition; // stored for >zero< Funktion
    steeringPosition = ( steeringPosition -IOSettings.steeringPositionZero);   //center the steering position sensor  
    
    //invert position, left must be minus
    if (IOSettings.Invert_WAS) steeringPosition_corr = - steeringPosition;
    else steeringPosition_corr = steeringPosition;
    //convert position to steer angle
    steerAngleActual = (float)(steeringPosition_corr) /   IOSettings.steerSensorCounts; 

 if (IOSettings.Inclino_type > 0 ) steerAngleActual = steerAngleActual - (XeRoll * (IOSettings.Kd/800));     // add the roll
 else XeRoll=0;

   //close enough to center, remove any correction
   //if (distanceFromLine < 40 && distanceFromLine  -40) steerAngleSetPoint = 0;
   if (distanceFromLine <= 40 && distanceFromLine >= -40) corr = 0;
   else
    {
      //use the integal value to adjust how much per cycle it increases
      corr += IOSettings.Ki;

      //provide a limit - the old max integral value
      if (corr > maxIntegralValue) corr = maxIntegralValue;

      //now add the correction to fool steering position
      if (distanceFromLine > 40)
        {
         steerAngleSetPoint -= corr;
        }
      else
        {
         steerAngleSetPoint += corr;
        }
    }
 
 //Build Autosteer Packet: Send to agopenGPS **** you must send 10 Byte or 5 Int
  
 int temp;
    //actual steer angle
    temp = (100 * steerAngleActual);
    toSend[2] = (byte)(temp >> 8);
    toSend[3] = (byte)(temp);

    //imu heading --- * 16 in degrees
    temp = Head;
    toSend[4] = (byte)(temp >> 8);
    toSend[5] = (byte)(temp);

   //Vehicle roll --- * 16 in degrees
   temp = XeRoll;
   toSend[6] = (byte)(temp >> 8);
   toSend[7] = (byte)(temp);

    //switch byte
    toSend[8] = switchByte;

//Build Autosteer Packet completed
Send_UDP();  //transmit to AOG

//debug Prints
   //Send to agopenGPS **** you must send 5 numbers ****
   //Serial.print(steerAngleActual); //The actual steering angle in degrees
   //Serial.print(",");
   //Serial.print(switchByte); //The actual steering angle in counts
   //Serial.print(",");
   //Serial.print(XeRoll/16);   //the pwm value to solenoids or motor
   //Serial.print(",");
   // Serial.print(IMU.euler.head/16);   //the pwm value to solenoids or motor
   //Serial.println("");


  
  }  // End of timed loop ------ 
  //delay(10);

  switch (IOSettings.SteerSwitchType)
  { 
  case 0: 
    steerEnable = digitalRead(STEERSW_PIN);
    Serial.println(steerEnable);
    break;
  case 1:
    steerEnable = !digitalRead(STEERSW_PIN);
    break;
  case 3:
    byte tempvalue = analogRead(STEERSW_PIN);
    if (tempvalue < 800) { steerEnable = false; }
    if (tempvalue > 3200) { steerEnable = true; }
    break;
  } 
  //Serial.println(steerAngleSetPoint);
  if (watchdogTimer < 18 )
    {   

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error 
      calcSteeringPID();   //do the pid     
      motorDrive();       //out to motors the pwm value     
    }
  else
    {
      //we've lost the comm to AgOpenGPS
      state_previous=steerEnable;   // Debug only
      steerEnable= false;
        if (steerEnable != state_previous) Serial.println("Steer-Break: WDT runs out");    // Debug only
      pwmDrive = 0; //turn off steering motor
      motorDrive(); //out to motors the pwm value   
      pulseACount = pulseBCount =0; //Reset counters if Autosteer is offline    
    }
  if (toggleSteerEnable==1)
    {
      steerEnable = !steerEnable;
      Serial.println("Steer-Break: IRQ occured? Button pressed?");  // Debug only
      toggleSteerEnable=0;
      if (steerEnable) watchdogTimer = 0;
    }

} // End of (main core1)
} // End of core1code
//------------------------------------------------------------------------------------------
// Subs --------------------------------------

void udpNtripRecv()
{ //callback when received packets
  udpNtrip.onPacket([](AsyncUDPPacket packet) 
   {Serial.print("."); 
    for (int i = 0; i < packet.length(); i++) 
        {
          if(IOSettings.enableNtrip == 2) {
            //Serial.print(packet.data()[i],HEX); 
            Serial1.write(packet.data()[i]); 
          }
        }
   });  // end of onPacket call
}

//------------------------------------------------------------------------------------------
//Read serial GPS data
//-------------------------------------------------------------------------------------------
void Serial_Traffic(){

while (Serial1.available()) 
  { 
   c = Serial1.read();
   //Serial2.print(c);
   if (c == '$')      //if character is $=0x24 start new sentence
      {
        newSentence = true; // only store sentence, if time is over
        gpsBuffer[0] = '/0';
        i = 0;
      }
    
   if (c == 0x0D && newSentence)   //if character is CR, build UDP send buffer
      {
        char Sent_Buffer[]="???";
        gpsBuffer[i++] = 0x0D;  //last-1 byte =CR
        gpsBuffer[i++] = 0x0A;  //last-1 byte =CR
        //gpsBuffer[i++] = "/0";  //
        Sent_Buffer[0] = (char)gpsBuffer[3];
        Sent_Buffer[1] = (char)gpsBuffer[4];
        Sent_Buffer[2] = (char)gpsBuffer[5];

        
        if (strcmp(Sent_Buffer, "RMC")==0 || strcmp(Sent_Buffer, "GGA")==0 || strcmp(Sent_Buffer, "VTG")==0 || strcmp(Sent_Buffer, "ZDA")==0){
            switch (IOSettings.send_UDP_AOG){
              case 1:
                 udpNtrip.writeTo(gpsBuffer, i, ipDestination, portDestination );    
               break;
              case 2:
              #if (useBluetooth)
                 for (byte n = 0; n < i; n++){  //print gpsBuffer to Bluetooth
                   //SerialBT.print((char)gpsBuffer[n]);
                  }
              #endif                
              break;
            }          
               
            if (IOSettings.sendGGAsentence == 2 ){
               if (strcmp(Sent_Buffer, "GGA") == 0){
                  for (byte n = 0; n <= i; n++){
                    lastSentence[n] = gpsBuffer[n];
                   }
                  repeat_ser = millis(); //Reset timer
                }
              } 
          }
          i = 0;
          newSentence = false;
     }
   
   if (newSentence && i < 100) 
     {
       gpsBuffer[i++] = c;
	   if (debugmode) DBG(gpsBuffer[i]);
     }
  }

}  

// recv from AOG steerAngleSetPoint 
void udpSteerRecv()
{ //callback when received packets
Serial.println("<--");    
  udpAOG.onPacket([](AsyncUDPPacket packet) 
   {
      for (int i = 0; i < 10; i++) 
        {
          data[i]=packet.data()[i];
        }

      if (data[0] == 0x7F && data[1] == 0xFE) //Data Packet
        {
               //Serial.print(millis()-oldmillis);  //benchmark
               //oldmillis=millis(); 
               //Serial.println("<--");    
     
          
         relay = data[2];   // read relay control from AgOpenGPS     
         speeed = data[3] >> 2;  //actual speed times 4, single byte
  
         //distance from the guidance line in mm
         olddist = distanceFromLine;
         idistanceFromLine = (data[4] << 8 | data[5]);   //high,low bytes     
         distanceFromLine = (float)idistanceFromLine;
  
         //set point steer angle * 10 is sent
         isteerAngleSetPoint = ((data[6] << 8 | data[7])); //high low bytes 
         steerAngleSetPoint = (float)isteerAngleSetPoint * 0.01;  


        //auto Steer is off if 32020,Speed is too slow, Wheelencoder above Max
        if (distanceFromLine == 32020 | speeed < 1 | (pulseACount+pulseBCount >= IOSettings.pulseCountMax && pulseACount>0 && pulseBCount>0))
          { 
            state_previous=steerEnable;    // Debug only
            steerEnable=false;
            if (steerEnable != state_previous) Serial.println("Steer-Break:  AOG,Speed or Encoder.."); // Debug only
            
            watchdogTimer = 20;//turn off steering motor
            digitalWrite(Autosteer_Led, LOW); //turn LED off
            //led_off();
          }
         else          //valid conditions to turn on autosteer
          {
            if (olddist == 32020)  steerEnable = true;              // Take over AOG State on startup

            if (steerEnable == true)
              {
                digitalWrite(Autosteer_Led, HIGH);  //turn LED on 
                //led_on();
                watchdogTimer = 0;  //reset watchdog 
              }
            else
              {
                digitalWrite(Autosteer_Led, LOW);  //turn LED off 
                //led_off();
                watchdogTimer = 20;  // turn off steering
              }
          }
   
      //steerEnable=true;    //debug
       /*
      Serial.print(steerAngleActual);   //the pwm value to solenoids or motor
      Serial.print(",");
      Serial.println(XeRoll);
      */
      UDP_data_time = millis();
       }

    //autosteer settings packet
    if (data[0] == 0x7F && data[1] == 0xFC)
     {
      IOSettings.Kp = (float)data[2] * 1.0;   // read Kp from AgOpenGPS
      IOSettings.Ki = (float)data[3] * 0.001;   // read Ki from AgOpenGPS
      IOSettings.Kd = (float)data[4] * 1.0;   // read Kd from AgOpenGPS
      IOSettings.Ko = (float)data[5] * 0.1;   // read Ko from AgOpenGPS
      if (IOSettings.input_type == 3){ //map Raw to -45 +45degree
       IOSettings.steeringPositionZero = (IOSettings.SteerPosZero) + data[6];//read steering zero offset  
      }else{
      IOSettings.steeringPositionZero = (IOSettings.SteerPosZero-127) + data[6];//read steering zero offset  
      }
      IOSettings.minPWMValue = data[7]; //read the minimum amount of PWM for instant on
      maxIntegralValue = data[8]*0.1; //
      IOSettings.steerSensorCounts = data[9]; //sent as 10 times the setting displayed in AOG
      EEprom_write_all();
      
      for (int i = 0; i < 10; i++) 
       {
        
       Serial.print(data[i],HEX); Serial.print("\t");
       }

       }
   
       });  // end of onPacket call
}
