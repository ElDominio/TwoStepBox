#include "EEPROM.h"

//Function declarations
int readIntFromEEPROM(unsigned int);
void writeIntToEEPROM(unsigned int, int);

//


int readIntFromEEPROM(unsigned int address){ //reads an int from EEPROM memory, needs the address
  byte hiByte, loByte;              //which it uses to read highByte, then adds 1 for lowByte
  hiByte = EEPROM.read(address);
  loByte = EEPROM.read(address + 1);
  int intOutput = word(hiByte,loByte);
  return intOutput;
}

void writeIntToEEPROM(unsigned int address, int value){
  //write an int to EEPROM memory, needs the address
  //which it uses to write highByte, then adds 1 for lowByte
  EEPROM.update(address, highByte(value));
  EEPROM.update(address + 1, lowByte(value));
}


bool burnCalibration(byte positionSelect, int cutRPM, int cutTime){
  if ( (positionSelect & 0x01) == 0) { 
    EEPROM.update(positionSelect,highByte(cutRPM));
    EEPROM.update(positionSelect+1,lowByte(cutRPM));
    EEPROM.update(positionSelect+32,highByte(cutTime));
    EEPROM.update(positionSelect+33,lowByte(cutTime));
  }
  else{
    EEPROM.update(positionSelect+16,highByte(cutRPM));
    EEPROM.update(positionSelect+17,lowByte(cutRPM));
    EEPROM.update(positionSelect+48,highByte(cutTime));
    EEPROM.update(positionSelect+49,lowByte(cutTime));
  }
  EEPROM.update(40,true);// address 8 saved calibration status
  Serial.println("All calibration values are saved");
  return true; //to be used to set calibration flag
}

bool clearCalibration(){
  for (int i = 0; i < 41; i++){
    EEPROM.update(i,highByte(0));
  }
  Serial.println("Calibration Cleared");
  return false;
}


byte convertASCIItoByte(byte input){
  byte output = input - 48;
  return output;
}

