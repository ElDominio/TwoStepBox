/* Memory Mapping
 *  0 - mode
 *  1 - timeToCut
 *  2,3 - high/lowByte of highRPM
 *  4,5 - high/lowByte of lowRPM
 */

#include "normanTools.h"

//Pin assignments
byte RPMinputPin = 3;
byte SparkOutput = 7;
byte ArmTriggerPin = 5;
byte bluePin = 9;
byte redPin = 10;

byte analogIn = A4;

byte pinOne = A0;
byte pinTwo = A1;
byte pinThree = A2;
byte pinFour = A3;

//Program variables
int cutRPM;

int timeToCut; 
//int armVoltage;
byte positionCheckcount = 0;
byte potPosition;
byte cylCount = 4;
byte RPMerror = 0;

byte bitField = B1000010; //field of working bits for states

#define BIT_POS_REP_FLAG  0 //Position Report enabled
#define BIT_DIAG_REQUEST  1 
#define BIT_CLUTCH_LOGIC  2
#define BIT_CUT_ACTIVE    3
#define BIT_ARM_TRIGGER   4
#define BIT_SPARK_STATE   5
#define BIT_TOYOTA_FAKE   6


/*
bool posRepFlag = true;
bool diagRequest = false;
bool clutchLogic = true;
bool cutActive = false;
bool armTriggerbool = false;
bool SparkState;
bool fakeOutState = false;
*/

//Working Variables
long RPM;
long RPMold;
unsigned long times;
unsigned long timeOld;

//bool correctionFlag = false;
byte trigCounter;
unsigned long cutTime;
unsigned long positionReport = 0;

//toyota function
byte toyotaDivision = 0;
byte timeCount = 0;
unsigned long timeTick = 0;
byte fakePin = 2;

void setup() {
	//Begin Serial comms
	Serial.begin(115200);

	//Set I/O
	pinMode(SparkOutput, OUTPUT);
 pinMode(bluePin,OUTPUT);
 pinMode(redPin,OUTPUT);
 pinMode(fakePin, OUTPUT);
	pinMode(ArmTriggerPin, INPUT_PULLUP);
	pinMode(RPMinputPin, INPUT);
  pinMode(pinOne, INPUT_PULLUP);pinMode(pinTwo, INPUT_PULLUP);pinMode(pinThree, INPUT_PULLUP);pinMode(pinFour, INPUT_PULLUP);
	digitalWrite(SparkOutput, HIGH); //activate spark output

  digitalWrite(bluePin,HIGH);
   Serial.println("Box Ready");
	 
   	//Initialize variables
	 RPM = 0;
	 RPMold = 0;
	 trigCounter = 0;
	 timeOld = 0;
	 times = 0;
  cutRPM = 4000;
	 /*armTriggerbool = false;
	 bitSet(bitField, BIT_SPARK_STATE);*/
  bitClear(bitField, BIT_DIAG_REQUEST);
  positionCheckcount = 0;
  trigCounter = 0;
  cutTime = 0;
  timeToCut = 5;
	cylCount = EEPROM.read(70);
 
 
  if (cylCount > 8){
    cylCount = 4;
  }
 bitWrite(bitField, BIT_CLUTCH_LOGIC, EEPROM.read(71));

	 //Attach RPM input interrupt
  attachInterrupt(digitalPinToInterrupt(3), triggerCounter, RISING);
}

void loop() {

  toyotaFunction();
  if (bitRead(bitField, BIT_CLUTCH_LOGIC)){

    bitWrite(bitField,BIT_ARM_TRIGGER,digitalRead(ArmTriggerPin));

  }
  else{  bitWrite(bitField,BIT_ARM_TRIGGER,!digitalRead(ArmTriggerPin));}
  //Serial.println(digitalRead(ArmTriggerPin));


  if (trigCounter > 0){
    RPMcounter();
  }
  if ((micros()- timeOld) > 670180){
    RPM = 0;
  }

    if (!bitRead(bitField, BIT_ARM_TRIGGER)){
      bitSet(bitField, BIT_SPARK_STATE);
      digitalWrite(bluePin, HIGH);
      digitalWrite(redPin, LOW);
      bitClear(bitField, BIT_CUT_ACTIVE);
      //Serial.println("Coil is On");
    }
    else{
      ignControl();
    }
     //Write ignition control out
    digitalWrite(SparkOutput, bitRead(bitField, BIT_SPARK_STATE));
   // Serial.println(bitRead(bitField, BIT_SPARK_STATE));

    if (Serial.available()){
      SerialComms();
    }

    
    if (bitRead(bitField,BIT_DIAG_REQUEST)){
      SerialDiag();
    }
    if (millis() % 2){
      positionCheckcount++;
    }
    if (positionCheckcount > 250){
      byte oldPotPos = (8*!digitalRead(pinFour)) + (4*!digitalRead(pinThree)) +(2*!digitalRead(pinTwo)) +(!digitalRead(pinOne)) ;
     potPosition = positionCheck();
     if (oldPotPos != potPosition){
      loadCalibration();
     }
      positionCheckcount = 0;
    }
}

byte positionCheck(){
  byte potPosition = (8*!digitalRead(pinFour)) + (4*!digitalRead(pinThree)) +(2*!digitalRead(pinTwo)) +(!digitalRead(pinOne));

    if ( (potPosition & 0x01) == 0) { 
      byte high = EEPROM.read(potPosition);
      byte low = EEPROM.read(potPosition+1);
      //Serial.print("cutRPM: ");Serial.println(word(high,low));
      cutRPM = word(high,low);
      high = EEPROM.read(potPosition + 32);
      low = EEPROM.read(potPosition + 33);
      timeToCut = word(high,low); 
     }
    else{
      byte high = EEPROM.read(potPosition+16);
      byte low = EEPROM.read(potPosition + 17);
      cutRPM = word(high,low);
      high = EEPROM.read(potPosition + 48);
      low = EEPROM.read(potPosition + 49);
      timeToCut = word(high,low);
    }
  return potPosition;
}

void toyotaFunction(){
	if(millis() > timeTick){
		toyotaDivision = times/4;
		timeTick = millis();
		timeCount++;
    //Serial.println("insideloop1");
		if(timeCount > toyotaDivision){
			timeCount = 0;
			bitClear(bitField, BIT_TOYOTA_FAKE);
      //    Serial.println(fakeOutState);
			digitalWrite(fakePin, bitRead(bitField, BIT_TOYOTA_FAKE));
		}
		if(timeCount == 1){
			bitSet(bitField, BIT_TOYOTA_FAKE);
        //  Serial.println(bitRead(bitField, BIT_TOYOTA_FAKE));
			digitalWrite(fakePin, bitRead(bitField, BIT_TOYOTA_FAKE));
		}
	}
}

void SerialDiag(){
  unsigned int posRep = millis() - positionReport;
  
  if (posRep >= 3000){
    Serial.print("Pot POsition: "); Serial.println(potPosition);
    Serial.print("cutRPM: "); Serial.println(cutRPM);
    Serial.print("Time to Cut: "); Serial.print(timeToCut);Serial.println("ms");
    Serial.print("Clutch Armed: ");Serial.println(bitRead(bitField,BIT_ARM_TRIGGER));
    Serial.print("CylMode: ");Serial.println(cylCount);
    //Serial.print("AnalogIn: ");Serial.println(armVoltage);
     Serial.print("Ignition Output: ");Serial.println(bitRead(bitField, BIT_SPARK_STATE));
     posRep = 0;
     positionReport = millis();
  }
  if (((posRep % 250) == 0) && (bitRead(bitField, BIT_POS_REP_FLAG))){
    Serial.print("RPM: "); Serial.println(RPM);
    bitClear(bitField, BIT_POS_REP_FLAG);
  }
  else if((posRep % 250) != 0){
    bitSet(bitField, BIT_POS_REP_FLAG);
  }
}

void SerialComms(){
  char commandChar = Serial.read();
  if (commandChar == 'w'){ //write command
    /* Memory Mapping
 *  0 - potposition
 *  1 - RPM hibyte
 *  2 - RPM lobyte
 *  3 - cutTime hi
 *  4 - cutTime lo
 *  Example Message:
 *  w0;205;50;
 */
    int i = 0;
    bool eepromFlag = false;
    Serial.println("Write mode activated, please enter the pot position (followed by a semicolon ';'");
    Serial.println("then the desired cut RPMs, followed by a semicolon,");
    Serial.println("and the the desired cut time, in ms, followed by a semicolon.");
    Serial.println("Ex. w14;3500;50; saves a 3500 RPM and 50ms cut to position 14");
    while (i < 4){
      
      if (Serial.available()){
        if (i == 0){
          String serialRead =Serial.readStringUntil(';');
            int serialInt = serialRead.toInt();
            if (serialInt > 17){ serialInt = 17;}
            byte serialByte = serialInt;
          potPosition = serialByte;
          Serial.print("i = "); Serial.print(i);Serial.print(" ");Serial.println(potPosition);
          i++;
        }
        else if (i == 1){
          if (Serial.available()){
            String serialRead =Serial.readStringUntil(';');
            int serialInt = serialRead.toInt();
            
            cutRPM = serialInt;
            Serial.print("i = "); Serial.print(i);Serial.print(" ");Serial.println(cutRPM);
            i++; // i = 2
          }
        }
        else if ( i == 2 ){
          if (Serial.available()){
            String serialRead =Serial.readStringUntil(';');
            int serialInt = serialRead.toInt();            
            timeToCut = serialInt;
            Serial.print("i = "); Serial.print(i);Serial.print(" ");Serial.println(timeToCut);
            i++; // i == 3
          }
        }// end elseif
      }// end main serial available
      if ( i == 3){
          Serial.print("i = "); Serial.print(i);Serial.print(" ");
          burnCalibration(potPosition, cutRPM, timeToCut);
          i++; // i == 4
      }
   }// end while
  } // End write command
  
  else if (commandChar == 'r'){
    bitWrite(bitField, BIT_DIAG_REQUEST, !bitRead(bitField, BIT_DIAG_REQUEST)); // activate diag


    Serial.print("diagRequest = "); Serial.println(bitRead(bitField, BIT_DIAG_REQUEST));
    if ( (potPosition & 0x01) == 0) { 
      byte high = EEPROM.read(potPosition);
      byte low = EEPROM.read(potPosition+1);
      Serial.print("cutRPM: ");Serial.println(word(high,low));
      high = EEPROM.read(potPosition + 32);

      low = EEPROM.read(potPosition + 33);
      Serial.print("timeToCut: ");Serial.println(word(high,low));

     }
    else{
      byte high = EEPROM.read(potPosition+16);
      byte low = EEPROM.read(potPosition + 17);
      Serial.print("cutRPM: ");Serial.println(word(high,low));
      high = EEPROM.read(potPosition + 48);
      low = EEPROM.read(potPosition + 49);
      Serial.print("timeToCut: ");Serial.println(word(high,low));
    }
  }
  else if (commandChar == 'l'){
    loadCalibration();




  }

 else if (commandChar == 'i'){

    bitWrite(bitField, BIT_CLUTCH_LOGIC, !(bitRead(bitField, BIT_CLUTCH_LOGIC)));
    EEPROM.update(71,(bitRead(bitField, BIT_CLUTCH_LOGIC)));
    Serial.println("Clutch logic inverted!");
  }// end I case
  
  else if (commandChar == 'c'){
    byte i = 0;
    Serial.println("Enter 'd' for distributor, 'w' for wasted spark, or 'c' for Coil On Plug");
    while (i < 1){
      if (Serial.available()){
        char serialRead = Serial.read();
        if (serialRead == 'd'){
          cylCount = 1;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == 'w'){
          cylCount = 2;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == 'c'){
          cylCount = 4;
          EEPROM.update(70,cylCount);
          i++;
        }
      } // end serial avail
    }// end while
    Serial.print("Spark mode set! Current mode is: ");Serial.println(cylCount);
  }// end C case
}

void triggerCounter(){
  trigCounter++;
  //Serial.println("TRIGGER");
}

void RPMcounter(){
 // Serial.print("micros "); Serial.print(micros());Serial.print("timeOld ");Serial.println(timeOld);
  times = micros()-timeOld;        //finds the time 
  //Serial.print("Times: "); Serial.println(times);
  RPM = (30108000/times);         //calculates rpm
 // Serial.println(RPM);
  RPM = RPM*(cylCount);
  if ((RPM - RPMold) > 10000){
    RPM = RPMold;
  }
  timeOld = micros();
  if ((RPM > 0) && (RPM < 12000)){
    RPMold = RPM;
  }
  
  trigCounter = 0;
}

void ignControl(){
  //Do ignition control methods

  if (timeToCut > 3000){
    if ((RPM > cutRPM) && (bitRead(bitField, BIT_SPARK_STATE) == HIGH) && (!bitRead(bitField,BIT_CUT_ACTIVE))){
      //Spark is cut
      bitClear(bitField, BIT_SPARK_STATE);
      bitSet(bitField,BIT_CUT_ACTIVE);
      digitalWrite(bluePin, LOW);
      digitalWrite(redPin, HIGH);
      cutTime = millis();
    }
    else if ((bitRead(bitField,BIT_CUT_ACTIVE)) && (millis() - cutTime > timeToCut)){
        bitSet(bitField, BIT_SPARK_STATE);
        cutTime = millis();
        digitalWrite(bluePin, HIGH);
        digitalWrite(redPin, LOW);
    }
  }
  else{
    if ((RPM > cutRPM) && (bitRead(bitField, BIT_SPARK_STATE) == HIGH) && (!bitRead(bitField,BIT_CUT_ACTIVE))){
        //Spark is cut
        bitSet(bitField,BIT_CUT_ACTIVE);
        cutTime = millis();
      }
      if ((RPM < cutRPM +700) && (bitRead(bitField,BIT_CUT_ACTIVE) && (millis() - cutTime > timeToCut))){
        bitWrite(bitField, BIT_SPARK_STATE, !bitRead(bitField, BIT_SPARK_STATE));
        cutTime = millis();
        digitalWrite(bluePin, bitRead(bitField, BIT_SPARK_STATE));
        digitalWrite(redPin, !bitRead(bitField, BIT_SPARK_STATE));
      }
      else if((RPM >= cutRPM + 700) && (bitRead(bitField,BIT_CUT_ACTIVE))){
        bitClear(bitField, BIT_SPARK_STATE);
        digitalWrite(bluePin, LOW);
        digitalWrite(redPin, HIGH);
      }
      if (RPM < cutRPM - 1500){
        digitalWrite(bluePin, HIGH);
        digitalWrite(redPin, LOW);
        bitClear(bitField,BIT_CUT_ACTIVE);
        bitSet(bitField, BIT_SPARK_STATE);
      }
  }
  /*
   if ((RPM > cutRPM) && (SparkState == HIGH) && (!cutActive){
      //Spark is cut
      bitClear(bitField, BIT_SPARK_STATE);
      cutActive = true;
      digitalWrite(bluePin, LOW);
      digitalWrite(redPin, HIGH);
      cutTime = millis();
    }
    else if (millis() - cutTime > timeToCut){
      
    }
    else if (millis() - cutTime > timeToCut){
      //Spark is restored after a set time
      if (RPM > cutRPM){
        RPM = cutRPM - 100;
      } // forces spark on even if RPM is greater than cut for one cycle
        bitSet(bitField, BIT_SPARK_STATE);
        digitalWrite(bluePin, HIGH);
        digitalWrite(redPin, LOW);
        cutTime = millis();
      }*/
}


void loadCalibration(){

  if ( (potPosition & 0x01) == 0) { 
      byte high = EEPROM.read(potPosition);
      byte low = EEPROM.read(potPosition+1);
      cutRPM = word(high,low);
      high = EEPROM.read(potPosition + 32);
      low = EEPROM.read(potPosition + 33);
      timeToCut = word(high,low); 
     }
    else{
      byte high = EEPROM.read(potPosition+16);
      byte low = EEPROM.read(potPosition + 17);
      cutRPM = word(high,low);
      high = EEPROM.read(potPosition + 48);
      low = EEPROM.read(potPosition + 49);
      timeToCut = word(high,low);

    }
}
