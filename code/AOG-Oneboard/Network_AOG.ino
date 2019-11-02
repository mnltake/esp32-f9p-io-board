//---------------------------------------------------------------------
void WiFi_Start_STA() {
  unsigned long timeout;

  WiFi.mode(WIFI_STA);   //  Workstation
  
  if (!WiFi.config(myip, gwip, mask, myDNS)) 
   {
    Serial.println("STA Failed to configure");
   }
  
  WiFi.begin(IOSettings.ssid, IOSettings.password);
  timeout = millis() + (timeoutRouter * 1000);
  LED_WIFI_time = millis();
  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
    delay(50);
    Serial.print(".");
    //WIFI LED blink in double time while connecting
    if (!LED_WIFI_ON) {
      if (millis() > (LED_WIFI_time + (LED_WIFI_pause >> 2))) {
        LED_WIFI_time = millis();
        LED_WIFI_ON = true;
        //digitalWrite(LED_PIN_WIFI, HIGH);
        led_on();
       }
     }
    if (LED_WIFI_ON) {
      if (millis() > (LED_WIFI_time + (LED_WIFI_pulse >> 2))) {
        LED_WIFI_time = millis();
        LED_WIFI_ON = false;
        //digitalWrite(LED_PIN_WIFI, LOW);
        led_off();
      }
    }
  }
  
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED) 
   {
    server.begin();
    my_WiFi_Mode = WIFI_STA;
    Serial.print("WiFi Client successfully connected to : ");
    Serial.println(IOSettings.ssid);
    Serial.print("Connected IP - Address : ");
    Serial.println( WiFi.localIP());
   } 
  else 
   {
    WiFi.mode(WIFI_OFF);
    Serial.println("WLAN-Client-Connection failed");
   }
}

//---------------------------------------------------------------------
void WiFi_Start_AP() {
  WiFi.mode(WIFI_AP);   // Accesspoint
  WiFi.softAP(ssid_ap, password_ap);
  while (!SYSTEM_EVENT_AP_START) // wait until AP has started
   {
    delay(100);
    Serial.print(".");
   }
   
  WiFi.softAPConfig(gwip, gwip, mask);  // set fix IP for AP
  IPAddress myIP = WiFi.softAPIP();
  my_WiFi_Mode = WIFI_AP;
  
  server.begin();
  Serial.print("Accesspoint started - Name : ");
  Serial.print(ssid_ap);
  Serial.print( " IP address: ");
  Serial.println(myIP);
}

//---------------------------------------------------------------------
void UDPReceiveNtrip()
{
  if(udpNtrip.listen(portMyNtrip)) 
    {
     Serial.print("NTRIP UDP Listening on IP: ");
     Serial.println(WiFi.localIP());
     udpNtripRecv();
    } 
}
//---------------------------------------------------------------------
void UDP_Start()
{
  if(udpAOG.listen(portAOG)) 
    {
     Serial.print("UDP Listening on IP: ");
     Serial.println(WiFi.localIP());
     udpSteerRecv();
    } 
}
//---------------------------------------------------------------------
void Send_UDP()
{
    //Send Packet
    udpAOG.listen(portMy);
    udpAOG.writeTo(toSend, sizeof(toSend), ipDestination, portDestination );
    udpAOG.listen(portAOG);
}
//---------------------------------------------------------------------
void WiFi_Traffic() {

  char my_char;
  int htmlPtr = 0;
  int myIdx;
  int myIndex;
  unsigned long my_timeout;
  

  // Check if a client has connected
  client_page = server.available();
  
  if (!client_page)  return;

  Serial.println("New client_page.");           // print a message out the serial port
  
  my_timeout = millis() + 250L;
  while (!client_page.available() && (millis() < my_timeout) ) delay(10);
  delay(10);
  if (millis() > my_timeout)  
    {
      Serial.println("Client connection timeout!");
      return;
    }
  //---------------------------------------------------------------------
  //htmlPtr = 0;
  char c;
  if (client_page) {                             // if you get a client,
    Serial.println("New client_page.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client_page.connected()) {            // loop while the client's connected
      if (client_page.available()) {             // if there's bytes to read from the client,
        char c = client_page.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
           
            make_HTML01();  // create Page array
           //---------------------------------------------------------------------
           // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
           // and a content-type so the client knows what's coming, then a blank line:
           strcpy(HTTP_Header , "HTTP/1.1 200 OK\r\n");
           strcat(HTTP_Header, "Content-Length: ");
           strcati(HTTP_Header, strlen(HTML_String));
           strcat(HTTP_Header, "\r\n");
           strcat(HTTP_Header, "Content-Type: text/html\r\n");
           strcat(HTTP_Header, "Connection: close\r\n");
           strcat(HTTP_Header, "\r\n");

           client_page.print(HTTP_Header);
           delay(20);
           send_HTML();
           
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') 
           { // if you got anything else but a carriage return character,
             currentLine += c;      // add it to the end of the currentLine
             if (currentLine.endsWith("HTTP")) 
               {
                if (currentLine.startsWith("GET ")) 
                 {
                  currentLine.toCharArray(HTML_String,currentLine.length());
                  Serial.println();
                  exhibit ("Request : ", HTML_String);
                  process_Request();
                 }  
               }
           }//end else
      } //end client available
    } //end while client_page.connected
    // close the connection:
    client_page.stop();
    Serial.print("Pagelength : ");
    Serial.print(strlen(HTML_String));
    Serial.println("   --> Client Disconnected.");
 }// end if client 
}
//---------------------------------------------------------------------
// Process given values
//---------------------------------------------------------------------
void process_Request()
{ 
  int myIndex;

  if (Find_Start ("/?", HTML_String) < 0 && Find_Start ("GET / HTTP", HTML_String) < 0 )
    {
      //nothing to process
      return;
    }
  action = Pick_Parameter_Zahl("ACTION=", HTML_String);

  // WiFi access data
  if ( action == ACTION_SET_SSID) {

    myIndex = Find_End("SSID_MY=", HTML_String);
    if (myIndex >= 0) {
       for (int i=0;i<24;i++) IOSettings.ssid[i]=0x00;
       Pick_Text(IOSettings.ssid, &HTML_String[myIndex], 24);
       exhibit ("SSID  : ", IOSettings.ssid);
      }
    myIndex = Find_End("Password_MY=", HTML_String);
    if (myIndex >= 0) {
       for (int i=0;i<24;i++) IOSettings.password[i]=0x00;
       Pick_Text(IOSettings.password, &HTML_String[myIndex], 24);
       exhibit ("Password  : ", IOSettings.password);
       EEprom_write_all();
      }
  }


  if ( action == ACTION_SET_NTRIPCAST) {
    myIndex = Find_End("CASTER=", HTML_String);
    if (myIndex >= 0) {
       for (int i=0;i<40;i++) IOSettings.host[i]=0x00;
       Pick_Text(IOSettings.host, &HTML_String[myIndex], 40);
       exhibit ("Caster:   ", IOSettings.host);
      }
    
    IOSettings.port = Pick_Parameter_Zahl("CASTERPORT=", HTML_String);
    exhibit ("Port  : ", IOSettings.port);
    
    myIndex = Find_End("MOUNTPOINT=", HTML_String);
    if (myIndex >= 0) {
       for (int i=0;i<40;i++) IOSettings.mountpoint[i]=0x00;
       Pick_Text(IOSettings.mountpoint, &HTML_String[myIndex], 40);
       exhibit ("Caster:   ", IOSettings.mountpoint);
      }
        
    myIndex = Find_End("CASTERUSER=", HTML_String);
    if (myIndex >= 0) {
       for (int i=0;i<40;i++) IOSettings.ntripUser[i]=0x00;
       Pick_Text(IOSettings.ntripUser, &HTML_String[myIndex], 40);
       exhibit ("Username : ", IOSettings.ntripUser);
      }
    myIndex = Find_End("CASTERPWD=", HTML_String);
    if (myIndex >= 0) {
       for (int i=0;i<40;i++) IOSettings.ntripPassword[i]=0x00;
       Pick_Text(IOSettings.ntripPassword, &HTML_String[myIndex], 40);
       exhibit ("Password  : ", IOSettings.ntripPassword);
       EEprom_write_all();
       connectCaster(); //reconnect Caster + calc _Auth
      }     
   }  

  if ( action == ACTION_SET_SENDPOS) {
    IOSettings.sendGGAsentence = Pick_Parameter_Zahl("POSITION_TYPE=", HTML_String);
    if (Pick_Parameter_Zahl("REPEATTIME=", HTML_String)==0) IOSettings.GGAfreq =1;
    if (Pick_Parameter_Zahl("REPEATTIME=", HTML_String)==1) IOSettings.GGAfreq =5;
    if (Pick_Parameter_Zahl("REPEATTIME=", HTML_String)==2) IOSettings.GGAfreq =10;
    
    if (Pick_Parameter_Zahl("BAUDRATESET=", HTML_String)==0) IOSettings.baudOut = 9600;
    if (Pick_Parameter_Zahl("BAUDRATESET=", HTML_String)==1) IOSettings.baudOut = 14400;
    if (Pick_Parameter_Zahl("BAUDRATESET=", HTML_String)==2) IOSettings.baudOut = 19200;
    if (Pick_Parameter_Zahl("BAUDRATESET=", HTML_String)==3) IOSettings.baudOut = 38400;
    if (Pick_Parameter_Zahl("BAUDRATESET=", HTML_String)==4) IOSettings.baudOut = 57600;
    if (Pick_Parameter_Zahl("BAUDRATESET=", HTML_String)==5) IOSettings.baudOut = 115200;   
    Serial.flush(); // wait for last transmitted data to be sent 
    Serial1.flush(); // wait for last transmitted data to be sent 
  Serial1.begin(IOSettings.baudOut, SERIAL_8N1, F9P_RX, F9P_TX);  //set new Baudrate
    DBG("\nRTCM/NMEA Baudrate: ");
    DBG(IOSettings.baudOut, 1);
    EEprom_write_all();
   }
  if ( action == ACTION_SET_RESTART) {
    if (EEPROM.read(2)==0){
       EEPROM.write(2,1);
       EEPROM.commit();
       delay(2000);
       ESP.restart();
     } 
   }

  if ( action == ACTION_SET_GGA) {
    myIndex = Find_End("GGA_MY=", HTML_String);
    if (myIndex >= 0) {
       for (int i=0;i<100;i++) IOSettings.GGAsentence[i]=0x00;
       Pick_Text(IOSettings.GGAsentence, &HTML_String[myIndex], 100);
       exhibit ("NMEA: ", IOSettings.GGAsentence);
      }
    EEprom_write_all();
  }

    if ( action == ACTION_SET_NMEAOUT) {
       IOSettings.send_UDP_AOG = Pick_Parameter_Zahl("SENDNMEA_TYPE=", HTML_String);
       byte old = IOSettings.enableNtrip;
       IOSettings.enableNtrip = Pick_Parameter_Zahl("ENABLENTRIP=", HTML_String);
       if (IOSettings.enableNtrip == 1 && old == 0 ) restart == 0; // 
       EEprom_write_all();    
     } 

   if ( action == ACTION_SET_AHRS) {

    for (int i = 0; i < 5; i++) {

      if (Pick_Parameter_Zahl("AHRS_TAG=", HTML_String) ==0) IOSettings.AHRSbyte = 0;
      if (Pick_Parameter_Zahl("AHRS_TAG=", HTML_String) ==1) IOSettings.AHRSbyte = 1;  
      if (Pick_Parameter_Zahl("AHRS_TAG=", HTML_String) ==2) IOSettings.AHRSbyte = 2;  
      if (Pick_Parameter_Zahl("AHRS_TAG=", HTML_String) ==3) IOSettings.AHRSbyte = 3;  
      if (Pick_Parameter_Zahl("AHRS_TAG=", HTML_String) ==4) IOSettings.AHRSbyte = 4;  
     }

    EEprom_write_all();

   }

     if ( action == ACTION_SET_OUTPUT_TYPE) {
     IOSettings.output_type = Pick_Parameter_Zahl("OUTPUT_TYPE=", HTML_String);
     EEprom_write_all();
    }
  
  if ( action == ACTION_SET_WAS_TYPE) {
     IOSettings.input_type = Pick_Parameter_Zahl("INPUT_TYPE=", HTML_String);
     EEprom_write_all();
    }

  if ( action == ACTION_SET_WAS_ZERO) {
     IOSettings.SteerPosZero= actualSteerPos; // >zero< Funktion Set Steer Angle to 0
     IOSettings.steeringPositionZero = actualSteerPos;
     EEprom_write_all();
    }
  
  if ( action == ACTION_SET_WAS_INVERT) {
     IOSettings.Invert_WAS = Pick_Parameter_Zahl("WAS_INVERT=", HTML_String);
     EEprom_write_all();
    }
   
  if ( action == ACTION_SET_IMU_TYPE) 
   {
    IOSettings.IMU_type = Pick_Parameter_Zahl("IMU_TYPE=", HTML_String);
    if (!IOSettings.IMU_type) imu_initialized=0;
    EEprom_write_all();
   }
  
  if ( action == ACTION_SET_INCLINO) 
   {
    IOSettings.Inclino_type = Pick_Parameter_Zahl("INCLINO_TYPE=", HTML_String);
    accelerometer.acc_initialized=0;
    EEprom_write_all();
   }
  
  if ( action == ACTION_SET_INCL_ZERO) {
    int roll_avg=0;
    for (int i=0; i<16; i++){
      roll_avg+=x_;
      delay(100);
    }
    IOSettings.roll_corr=roll_avg >> 4 ;
    EEprom_write_all();
   }
  
  if ( action == ACTION_SET_ENCODER) {
    IOSettings.SWEncoder= Pick_Parameter_Zahl("ENC_TYPE=", HTML_String);
    IOSettings.pulseCountMax= Pick_Parameter_Zahl("ENC_COUNTS=", HTML_String);
    EEprom_write_all();
   }
  if ( action == ACTION_SET_SWITCHES) {
   IOSettings.SteerSwitchType= Pick_Parameter_Zahl("SSWITCH_TYPE=", HTML_String);
   IOSettings.WorkSW_mode    = Pick_Parameter_Zahl("WSWITCH_TYPE=", HTML_String);
   IOSettings.Invert_WorkSW  = Pick_Parameter_Zahl("IWSWITCH_TYPE=", HTML_String);
   EEprom_write_all();
  }
  if ( action == ACTION_SET_THRESHOLD) {
    unsigned int WSThres_avg=0;
    for (int i=0; i<8; i++){
      WSThres_avg += analogRead(WORKSW_PIN);
      delay(100);
    }
    IOSettings.WorkSW_Threshold= WSThres_avg >> 3;
    EEprom_write_all();
   } 
}  
//---------------------------------------------------------------------
// HTML Seite 01 aufbauen
//---------------------------------------------------------------------
void make_HTML01() {

  strcpy( HTML_String, "<!DOCTYPE html>");
  strcat( HTML_String, "<html>");
  strcat( HTML_String, "<head>");
  strcat( HTML_String, "<title>AG Autosteer ESP Config Page</title>");
  strcat( HTML_String, "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0;\" />\r\n");
  //strcat( HTML_String, "<meta http-equiv=\"refresh\" content=\"10\">");
  strcat( HTML_String, "<style>divbox {background-color: lightgrey;width: 200px;border: 5px solid red;padding:10px;margin: 10px;}</style>");
  strcat( HTML_String, "</head>");
  strcat( HTML_String, "<body bgcolor=\"#66b3ff\">");
  strcat( HTML_String, "<font color=\"#000000\" face=\"VERDANA,ARIAL,HELVETICA\">");
  strcat( HTML_String, "<h1>ESP-F9P-IO-BORAD</h1>");

  //-----------------------------------------------------------------------------------------
  // WiFi Client Access Data
  strcat( HTML_String, "(experimental version by mnltake based on weder)<br>");
  strcat( HTML_String, "FOR ESP-F9P-IO-BORAD \"Setup Zone\"<br>");  
  strcat( HTML_String, "<hr><h2>WiFi Network Client Access Data</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "If access fails, an accesspoint will be created<br>");
  strcat( HTML_String, "(ESP-F9P-IO-BORAD_Net PW:"")<br><br>");
  strcat( HTML_String, "<table>");
  set_colgroup(200, 300, 150, 0, 0);

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Network Name</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"SSID_MY\" maxlength=\"22\" Value =\"");
  strcat( HTML_String, IOSettings.ssid);
  strcat( HTML_String, "\"></td>");
  
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_SSID);
  strcat( HTML_String, "\">Submit</button></td>");
  strcat( HTML_String, "</tr>");

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Password</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"Password_MY\" maxlength=\"22\" Value =\"");
  strcat( HTML_String, IOSettings.password);
  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "</tr>");


  
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");
//-------------------------------------------------------------  
// NTRIP Caster
  strcat( HTML_String, "<h2>NTRIP Caster Settings</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(150, 270, 150, 0, 0);
  
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Network Name:(IP only)</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"CASTER\" maxlength=\"40\" Value =\"");
  strcat( HTML_String, IOSettings.host);
  strcat( HTML_String, "\"></td>");
  
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_NTRIPCAST);
  strcat( HTML_String, "\">Submit</button></td>");
  strcat( HTML_String, "</tr>");

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Port:</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"CASTERPORT\" maxlength=\"4\" Value =\"");
  strcati( HTML_String, IOSettings.port);
  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "</tr>");
  
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Mountpoint:</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"MOUNTPOINT\" maxlength=\"40\" Value =\"");
  strcat( HTML_String, IOSettings.mountpoint);
  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "</tr>");

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Username:</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"CASTERUSER\" maxlength=\"40\" Value =\"");
  strcat( HTML_String, IOSettings.ntripUser);
  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "</tr>");
  
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Password:</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"CASTERPWD\" maxlength=\"40\" Value =\"");
  strcat( HTML_String, IOSettings.ntripPassword);
  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "</tr>");
  strcat( HTML_String, "<tr> <td colspan=\"3\">&nbsp;</td> </tr>");
  strcat( HTML_String, "<tr><td colspan=\"2\"><b>Restart NTRIP client for changes to take effect</b></td>");
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_RESTART);
  strcat( HTML_String, "\">Restart</button></td>");
  strcat( HTML_String, "</tr>");
  
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");
//-------------------------------------------------------------  
//baudOut
  strcat( HTML_String, "<h2>baudrate to F9P</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(150, 270, 150, 0, 0);
  for (int i = 0; i < 6; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0) strcat( HTML_String, "<td><b>Baudrate</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"BAUDRATESET\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if ((IOSettings.baudOut == 9600)   && i==0)strcat( HTML_String, " CHECKED");
    if ((IOSettings.baudOut == 14400)  && i==1)strcat( HTML_String, " CHECKED");
    if ((IOSettings.baudOut == 19200)  && i==2)strcat( HTML_String, " CHECKED");
    if ((IOSettings.baudOut == 38400)  && i==3)strcat( HTML_String, " CHECKED");
    if ((IOSettings.baudOut == 57600)  && i==4)strcat( HTML_String, " CHECKED");
    if ((IOSettings.baudOut == 115200) && i==5)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, baud_output[i]);
    strcat( HTML_String, "</label></td>");
  }
  
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");

  //-----------------------------------------------------------------------------------------
  // Output Driver
  strcat( HTML_String, "<h2>Output Driver</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(200, 300, 150, 0, 0);

  for (int i = 0; i < 6; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Select your output type</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"OUTPUT_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.output_type == i)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, output_driver_tab[i]);
    strcat( HTML_String, "</label></td>");
    if (i == 0){
      strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
      strcati(HTML_String, ACTION_SET_OUTPUT_TYPE);
      strcat( HTML_String, "\">Submit</button></td>");
      strcat( HTML_String, "</tr>");
     }
  }
  
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");

 //-----------------------------------------------------------------------------------------
 // Input device
  strcat( HTML_String, "<h2>Wheel Angle Sensor (WAS)</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(200, 300, 150, 10, 0);
 
  for (int i = 0; i < 4; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Select your Input type</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"INPUT_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.input_type == i)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, was_input_tab[i]);
    strcat( HTML_String, "</label></td>");
    if (i == 0){
      strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
      strcati(HTML_String, ACTION_SET_WAS_TYPE);
      strcat( HTML_String, "\">Submit</button></td>");
      strcat( HTML_String, "</tr>");
     }
  }
    
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><br><font size=\"+1\">WAS RAW Data</font></td>");
  strcat( HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
  strcati(HTML_String, steeringPosition_corr);
  strcat( HTML_String, "</b></font></divbox></td>");
  
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_WAS_TYPE);
  strcat( HTML_String, "\">Refresh</button></td>");
  strcat( HTML_String, "</tr>");
  
  strcat( HTML_String, "<tr>");  
  strcat( HTML_String, "<td>Center your Sensor to Zero</td>");
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_WAS_ZERO);
  strcat( HTML_String, "\">ZERO NOW</button></td>");
  strcat( HTML_String, "<td>Your Wheels should face straight ahead</td>");

 
  for (int i = 0; i < 2; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Invert WAS</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"WAS_INVERT\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.Invert_WAS == i)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, was_invert_tab[i]);
    strcat( HTML_String, "</label></td>");
    if (i == 0){
      strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
      strcati(HTML_String, ACTION_SET_WAS_INVERT);
      strcat( HTML_String, "\">Submit</button></td>");
      strcat( HTML_String, "</tr>");
     }
  }
      
  strcat( HTML_String, "</tr>");
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "Steering to the left must be minus"); 
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");

  //-----------------------------------------------------------------------------------------
  // IMU Heading Unit
  
  strcat( HTML_String, "<h2>IMU Heading Unit (Compass)</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(200, 300, 150, 0, 0);

  for (int i = 0; i < 2; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Select your IMU type</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"IMU_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.IMU_type == i)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, imu_type_tab[i]);
    strcat( HTML_String, "</label></td>");
    if (i == 0){
      strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
      strcati(HTML_String, ACTION_SET_IMU_TYPE);
      strcat( HTML_String, "\">Submit</button></td>");
      strcat( HTML_String, "</tr>");
     }
  }
  
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><br><font size=\"+1\">Heading</font></td>");
  strcat( HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
  strcatf(HTML_String, (Yaw));
  strcat( HTML_String, "</b></font></divbox>degree</td>");
  
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_IMU_TYPE);
  strcat( HTML_String, "\">Refresh</button></td>");
  strcat( HTML_String, "</tr>");

  
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");

  //-----------------------------------------------------------------------------------------
  // Inclinometer
  strcat( HTML_String, "<h2>Inclinometer Unit (Roll)</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(200, 300, 150, 0, 0);

  for (int i = 0; i < 3; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Select your Inclinometer type</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"INCLINO_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.Inclino_type == i)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, inclino_type_tab[i]);
    strcat( HTML_String, "</label></td>");
    if (i == 0){
      strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
      strcati(HTML_String, ACTION_SET_INCLINO);
      strcat( HTML_String, "\">Submit</button></td>");
      strcat( HTML_String, "</tr>");
     }
  }
  
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><br><font size=\"+1\">Tilt Angle</font></td>");
  strcat( HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
  strcatf(HTML_String, (XeRoll/16));
  strcat( HTML_String, "</b></font></divbox>degree</td>");
  
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_INCLINO);
  strcat( HTML_String, "\">Refresh</button></td>");
  strcat( HTML_String, "</tr>");
  
  strcat( HTML_String, "<tr>");  
  strcat( HTML_String, "<td>Calibrate Inclinometer</td>");
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_INCL_ZERO);
  strcat( HTML_String, "\">ZERO NOW</button></td>");
  strcat( HTML_String, "<td>Tilt Calibration takes place on a flat area with no slope</td>");

  

  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");

 //-----------------------------------------------------------------------------------------
  // Steerswitch Type
  strcat( HTML_String, "<h2>Switch Types</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(200, 300, 150, 0, 0);

  for (int i = 0; i < 5; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Steerswitch type</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"SSWITCH_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.SteerSwitchType == i)strcat( HTML_String, " CHECKED");
    if ( i == 4) strcat( HTML_String, " disabled");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, steersw_type_tab[i]);
    strcat( HTML_String, "</label></td>");
    if (i == 0){
      strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
      strcati(HTML_String, ACTION_SET_SWITCHES);
      strcat( HTML_String, "\">Submit</button></td>");
      strcat( HTML_String, "</tr>");
     }
  }
  
  for (int i = 0; i < 4; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Workswitch type</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"WSWITCH_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.WorkSW_mode == i)strcat( HTML_String, " CHECKED");
    if ( i == 3) strcat( HTML_String, " disabled");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, worksw_type_tab[i]);
    strcat( HTML_String, "</label></td>");
  }
 
  for (int i = 0; i < 2; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Invert Workswitch</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"IWSWITCH_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.Invert_WorkSW == i)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, worksw_invert_tab[i]);
    strcat( HTML_String, "</label></td>");
  }  
  
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><br><font size=\"+1\">Analog Workswitch Threshold value</font></td>");
  strcat( HTML_String, "<td><divbox align=\"right\"><font size=\"+2\"><b>");
  strcati(HTML_String, (IOSettings.WorkSW_Threshold));
  strcat( HTML_String, "</b></font></divbox>0-4095</td>");
  
  strcat( HTML_String, "<tr>");  
  strcat( HTML_String, "<td>Set Threshold</td>");
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_THRESHOLD);
  strcat( HTML_String, "\">Use Current</button></td>");
  strcat( HTML_String, "<td>Set Threshold value to current position</td>");

  
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br><hr>");
/*
//-----------------------------------------------------------------------------------------
// Steering Wheel Encoder
  strcat( HTML_String, "<h2>Steering Wheel Encoder</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(200, 300, 150, 0, 0);

for (int i = 0; i < 2; i++) {
    strcat( HTML_String, "<tr>");
    if (i == 0)  strcat( HTML_String, "<td><b>Encoder:</b></td>");
    else strcat( HTML_String, "<td> </td>");
    strcat( HTML_String, "<td><input type = \"radio\" name=\"ENC_TYPE\" id=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value=\"");
    strcati( HTML_String, i);
    strcat( HTML_String, "\"");
    if (IOSettings.SWEncoder == i)strcat( HTML_String, " CHECKED");
    strcat( HTML_String, "><label for=\"JZ");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, encoder_type_tab[i]);
    strcat( HTML_String, "</label></td>");
    if (i == 0){
      strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
      strcati(HTML_String, ACTION_SET_ENCODER);
      strcat( HTML_String, "\">Submit</button></td>");
      strcat( HTML_String, "</tr>");
     }
  }

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Counts to turn off Autosteer</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"ENC_COUNTS\" maxlength=\"3\" Value =\"");
  strcati( HTML_String, IOSettings.pulseCountMax);
  strcat( HTML_String, "\"></td>");
  
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_ENCODER);
  strcat( HTML_String, "\">Submit</button></td>");
  strcat( HTML_String, "</tr>");

  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br>");
*/

//-------------------------------------------------------------  
  strcat( HTML_String, "</font>");
  strcat( HTML_String, "</font>");
  strcat( HTML_String, "</body>");
  strcat( HTML_String, "</html>");
}

//--------------------------------------------------------------------------
void send_not_found() {

  DBG("\nSend Not Found\n");

  client_page.print("HTTP/1.1 404 Not Found\r\n\r\n");
  delay(20);
  //client_page.stop();
}

//--------------------------------------------------------------------------
void send_HTML() {
  char my_char;
  int  my_len = strlen(HTML_String);
  int  my_ptr = 0;
  int  my_send = 0;

  //--------------------------------------------------------------------------
  // in Portionen senden
  while ((my_len - my_send) > 0) {
    my_send = my_ptr + MAX_PACKAGE_SIZE;
    if (my_send > my_len) {
      client_page.print(&HTML_String[my_ptr]);
      delay(22);

      //Serial.println(&HTML_String[my_ptr]);

      my_send = my_len;
    } else {
      my_char = HTML_String[my_send];
      // Auf Anfang eines Tags positionieren
      while ( my_char != '<') my_char = HTML_String[--my_send];
      HTML_String[my_send] = 0;
      client_page.print(&HTML_String[my_ptr]);
      delay(22);
      
      //Serial.println(&HTML_String[my_ptr]);

      HTML_String[my_send] =  my_char;
      my_ptr = my_send;
    }
  }
  //client_page.stop();
}

//----------------------------------------------------------------------------------------------
void set_colgroup(int w1, int w2, int w3, int w4, int w5) {
  strcat( HTML_String, "<colgroup>");
  set_colgroup1(w1);
  set_colgroup1(w2);
  set_colgroup1(w3);
  set_colgroup1(w4);
  set_colgroup1(w5);
  strcat( HTML_String, "</colgroup>");

}
//------------------------------------------------------------------------------------------
void set_colgroup1(int ww) {
  if (ww == 0) return;
  strcat( HTML_String, "<col width=\"");
  strcati( HTML_String, ww);
  strcat( HTML_String, "\">");
}


//---------------------------------------------------------------------
void strcatf(char* tx, float f) {
  char tmp[8];

  dtostrf(f, 6, 2, tmp);
  strcat (tx, tmp);
}
//---------------------------------------------------------------------
//void strcatl(char* tx, long l) {
  //char tmp[sizeof l];
  //memcpy(tmp, l, sizeof l);
  //strcat (tx, tmp);
//}

//---------------------------------------------------------------------
void strcati(char* tx, int i) {
  char tmp[8];

  itoa(i, tmp, 10);
  strcat (tx, tmp);
}

//---------------------------------------------------------------------
void strcati2(char* tx, int i) {
  char tmp[8];

  itoa(i, tmp, 10);
  if (strlen(tmp) < 2) strcat (tx, "0");
  strcat (tx, tmp);
}

//---------------------------------------------------------------------
int Pick_Parameter_Zahl(const char * par, char * str) {
  int myIdx = Find_End(par, str);

  if (myIdx >= 0) return  Pick_Dec(str, myIdx);
  else return -1;
}
//---------------------------------------------------------------------
int Find_End(const char * such, const char * str) {
  int tmp = Find_Start(such, str);
  if (tmp >= 0)tmp += strlen(such);
  return tmp;
}

//---------------------------------------------------------------------
int Find_Start(const char * such, const char * str) {
  int tmp = -1;
  int ww = strlen(str) - strlen(such);
  int ll = strlen(such);

  for (int i = 0; i <= ww && tmp == -1; i++) {
    if (strncmp(such, &str[i], ll) == 0) tmp = i;
  }
  return tmp;
}
//---------------------------------------------------------------------
int Pick_Dec(const char * tx, int idx ) {
  int tmp = 0;

  for (int p = idx; p < idx + 5 && (tx[p] >= '0' && tx[p] <= '9') ; p++) {
    tmp = 10 * tmp + tx[p] - '0';
  }
  return tmp;
}
//----------------------------------------------------------------------------
int Pick_N_Zahl(const char * tx, char separator, byte n) {

  int ll = strlen(tx);
  int tmp = -1;
  byte anz = 1;
  byte i = 0;
  while (i < ll && anz < n) {
    if (tx[i] == separator)anz++;
    i++;
  }
  if (i < ll) return Pick_Dec(tx, i);
  else return -1;
}

//---------------------------------------------------------------------
int Pick_Hex(const char * tx, int idx ) {
  int tmp = 0;

  for (int p = idx; p < idx + 5 && ( (tx[p] >= '0' && tx[p] <= '9') || (tx[p] >= 'A' && tx[p] <= 'F')) ; p++) {
    if (tx[p] <= '9')tmp = 16 * tmp + tx[p] - '0';
    else tmp = 16 * tmp + tx[p] - 55;
  }

  return tmp;
}

//---------------------------------------------------------------------
void Pick_Text(char * tx_ziel, char  * tx_quelle, int max_ziel) {

  int p_ziel = 0;
  int p_quelle = 0;
  int len_quelle = strlen(tx_quelle);

  while (p_ziel < max_ziel && p_quelle < len_quelle && tx_quelle[p_quelle] && tx_quelle[p_quelle] != ' ' && tx_quelle[p_quelle] !=  '&') {
    if (tx_quelle[p_quelle] == '%') {
      tx_ziel[p_ziel] = (HexChar_to_NumChar( tx_quelle[p_quelle + 1]) << 4) + HexChar_to_NumChar(tx_quelle[p_quelle + 2]);
      p_quelle += 2;
    } else if (tx_quelle[p_quelle] == '+') {
      tx_ziel[p_ziel] = ' ';
    }
    else {
      tx_ziel[p_ziel] = tx_quelle[p_quelle];
    }
    p_ziel++;
    p_quelle++;
  }

  tx_ziel[p_ziel] = 0;
}
//---------------------------------------------------------------------
char HexChar_to_NumChar( char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 55;
  return 0;
}
//---------------------------------------------------------------------
void exhibit(const char * tx, int v) {
  DBG(tx);
  DBG(v, 1);
}
//---------------------------------------------------------------------
void exhibit(const char * tx, unsigned int v) {
  DBG(tx);
  DBG((int)v, 1);
}
//---------------------------------------------------------------------
void exhibit(const char * tx, unsigned long v) {
  DBG(tx);
  DBG((long)v, 1);
}
//---------------------------------------------------------------------
void exhibit(const char * tx, const char * v) {
  DBG(tx);
  DBG(v, 1);
}
