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

  EEPROM.update(positionSelect+positionSelect,highByte(cutRPM));
  EEPROM.update(positionSelect+positionSelect+1,lowByte(cutRPM));
  EEPROM.update(positionSelect+positionSelect+32,highByte(cutTime));
    EEPROM.update(positionSelect+positionSelect+33,lowByte(cutTime));
  EEPROM.update(250,true);// address 8 saved calibration status
  Serial.println("All calibration values are saved");
  return true; //to be used to set calibration flag
  /*
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
  }*/
 // EEPROM.update(40,true);// address 8 saved calibration status
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

/*
 * Calculates integer power values. Same as pow() but with ints
 */
inline long powint(int factor, int exponent)
{
  /*if (exponent < 0){
    long product = 0;
  }
  else{*/
   long product = 1;
   unsigned int counter = exponent;
   while ( (counter--) > 0) { product *= factor; }
   return product;
 // }
}

void writeDefaultSettings(){
  unsigned int value = 0;
  while(value <= 10){
    unsigned int cutRPM = 1500 + 500*value;
    unsigned int cutTime = 150 + 5*value;
    EEPROM.update(value+value,highByte(cutRPM));
    EEPROM.update(value+value+1,lowByte(cutRPM));
    EEPROM.update(value+value+32,highByte(cutTime));
    EEPROM.update(value+value+33,lowByte(cutTime));
    value++;
  }
  EEPROM.update(70,4);
  EEPROM.update(71,false);
  EEPROM.update(72,false);
  EEPROM.update(73,false);
  EEPROM.update(255, 23);// address 8 saved calibration status
    Serial.println("All calibration values are saved");
}
