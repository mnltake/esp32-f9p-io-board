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

void led_on(){
  setByteI2C(0x43, 0x05, 0b00000100);
}

void led_off(){
  setByteI2C(0x43, 0x05, 0b00000000);
}

  
void SetRelays(void)
 {
    if (bitRead(relay,0)) digitalWrite(led1, HIGH);
    else digitalWrite(led1, LOW);
    if (bitRead(relay,1)) digitalWrite(led2, HIGH);
    else digitalWrite(led2, LOW); 
    //if (bitRead(relay,2)) digitalWrite(led3, HIGH);
    //else digitalWrite(led3, LOW); 
    //if (bitRead(relay,3)) digitalWrite(led4, HIGH);
    //else digitalWrite(led4, LOW); 
    
    //if (bitRead(relay,4)) bitSet(PORTB, 1); //Digital Pin 9
    //else bitClear(PORTB, 1); 
    //if (bitRead(relay,5)) bitSet(PORTB, 4); //Digital Pin 12
    //else bitClear(PORTB, 4); 
    //if (bitRead(relay,6)) bitSet(PORTC, 4); //analog Pin A4
    //else bitClear(PORTC, 4); 
    //if (bitRead(relay,7)) bitSet(PORTC, 5); //Analog Pin A5
    //else bitClear(PORTC, 5); 
  }

//--------------------------------------------------------------
//  EEPROM Data Handling
//--------------------------------------------------------------
#define EEPROM_SIZE 512
#define EE_ident1 0xED  // Marker Byte 0 + 1
#define EE_ident2 0xED


//--------------------------------------------------------------
//  Restore EEprom Data
//--------------------------------------------------------------
void restoreEEprom(){

  byte get_state = getByteI2C(0x43, 0x0F);
  get_state &= 0b00000001;
  get_state =~get_state;
  //byte get_state  = digitalRead(restoreDefault_PIN);
  if (debugmode) get_state = true;
  if (get_state ) DBG("State: restoring default values !\n");
  else DBG("State: read default values from EEPROM\n");
  
  if (EEprom_empty_check()==1 || get_state) { //first start?
    EEprom_write_all();     //write default data
   }
  if (EEprom_empty_check()==2 ) { //data available
    EEprom_read_all();
   }
  //EEprom_show_memory();  //
  EE_done =1;   
}

//--------------------------------------------------------------
byte EEprom_empty_check(){
    
  if (!EEPROM.begin(EEPROM_SIZE))  
    {
     DBG("failed to initialise EEPROM\n"); delay(1000);
     return false;
    }
  if (EEPROM.read(0)!= EE_ident1 || EEPROM.read(1)!= EE_ident2)
     return true;  // is empty
  
  if (EEPROM.read(0)== EE_ident1 && EEPROM.read(1)== EE_ident2)
     return 2;     // data available
     
 }
//--------------------------------------------------------------
void EEprom_write_all(){  // called if EEPROM empty
  EEPROM.write(0, EE_ident1);
  EEPROM.write(1, EE_ident2);
  EEPROM.write(2, 0); // reset Restart blocker
  EEPROM.put(3, IOSettings);
  EEPROM.commit();
}
//--------------------------------------------------------------
void EEprom_read_all(){
  
  EEPROM.get(3, IOSettings);
  
}
//--------------------------------------------------------------
void EEprom_show_memory(){
byte c2=0, data_;
  DBG(EEPROM_SIZE, 1);
  DBG(" bytes read from Flash . Values are:\n");
  for (int i = 0; i < EEPROM_SIZE; i++)
  { 
    data_=byte(EEPROM.read(i));
    if (data_ < 0x10) Serial.print("0");
    DBG(data_,HEX); 
    if (c2==15) {
       DBG(" ");
      }
    else if (c2>=31) {
           DBG("",1); //NL
           c2=-1;
          }
    else DBG(" ");
    c2++;
  }
}







   
