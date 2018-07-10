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

int analogIn = A4;

int pinOne = A0;
int pinTwo = A1;
int pinThree = A2;
int pinFour = A3;

//Program variables
int cutRPM;

int timeToCut; 
//int armVoltage;
int positionCheckcount = 0;
byte potPosition;
bool posRepFlag = true;
bool diagRequest = false;
byte cylCount = 4;
bool clutchLogic = true;
bool cutActive = false;



//Working Variables
long RPM;
long RPMold;
unsigned long times;
unsigned long timeOld;
bool armTriggerbool = false;
bool SparkState;
//bool correctionFlag = false;
byte trigCounter;
unsigned long cutTime;
unsigned long positionReport = 0;


void setup() {
	//Begin Serial comms
	Serial.begin(115200);

	//Set I/O
	pinMode(SparkOutput, OUTPUT);
 pinMode(bluePin,OUTPUT);
 pinMode(redPin,OUTPUT);
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
	 armTriggerbool = false;
	 SparkState = HIGH;
  positionCheckcount = 0;
  trigCounter = 0;
  cutTime = 0;
  timeToCut = 5;
	cylCount = EEPROM.read(70);
  if (cylCount > 8){
    cylCount = 4;
  }
  clutchLogic = EEPROM.read(71);

	 //Attach RPM input interrupt
  attachInterrupt(digitalPinToInterrupt(3), triggerCounter, RISING);
}

void loop() {
  /*armVoltage = analogRead(A4);
  if (armVoltage > 600){
    armTriggerbool = true;
  }
  else{armTriggerbool = false;}*/


  if (clutchLogic){

    armTriggerbool = digitalRead(ArmTriggerPin);

  }
  else{  armTriggerbool = !digitalRead(ArmTriggerPin);}
  //Serial.println(digitalRead(ArmTriggerPin));


  if (trigCounter > 0){
    RPMcounter();
  }

    if (!armTriggerbool){
      SparkState = HIGH;
      digitalWrite(bluePin, HIGH);
      digitalWrite(redPin, LOW);
      cutActive = false;
      //Serial.println("Coil is On");
    }
    else{
      ignControl();
    }
     //Write ignition control out
    digitalWrite(SparkOutput, SparkState);
   // Serial.println(SparkState);

    if (Serial.available()){
      SerialComms();
    }

    
    if (diagRequest){
      SerialDiag();
    }
    positionCheckcount++;
    if (positionCheckcount > 20000){
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
 /* switch (potPosition){
    case 15:
      cutRPM = 10000;
      timeToCut = 0;
      break;
    case 7:
      cutRPM = 2500;
      timeToCut = 2;
      break;
    case 11:
      cutRPM = 4000;
      timeToCut = 2;
      break;
    case 3:
      cutRPM = 5000;
      timeToCut = 2;
      break;
    case 13:
      cutRPM = 4500;
      timeToCut = 100;
      break;
    case 5:
      cutRPM = 5000;
      timeToCut = 200;
      break;
    case 9:
      cutRPM = 6000;
      timeToCut = 200;
      break;
  }*/
  
  return potPosition;
}

void SerialDiag(){
  unsigned int posRep = millis() - positionReport;
  
  if (posRep > 3000){
    Serial.print("Pot POsition: "); Serial.println(potPosition);
    Serial.print("cutRPM: "); Serial.println(cutRPM);
    Serial.print("Time to Cut: "); Serial.print(timeToCut);Serial.println("ms");
    Serial.print("Clutch Armed: ");Serial.println(armTriggerbool);
    Serial.print("CylMode: ");Serial.println(cylCount);
    //Serial.print("AnalogIn: ");Serial.println(armVoltage);
     Serial.print("Ignition Output: ");Serial.println(SparkState);
     posRep = 0;
     positionReport = millis();
  }
  if((posRep == 251) || (posRep == 501) || (posRep == 1001) || (posRep == 1251) || 
      (posRep == 1501) || (posRep == 1751) ||(posRep == 2001) || (posRep == 2251) || 
      (posRep == 2501) || (posRep == 2751)){
        posRepFlag = true;
      }
  if (((posRep == 250) || (posRep == 500) || (posRep == 1000) || (posRep == 1250) || 
      (posRep == 1500) || (posRep == 1750) ||(posRep == 2000) || (posRep == 2250) || 
      (posRep == 2500) || (posRep == 2750)) && posRepFlag){
    Serial.print("RPM: "); Serial.println(RPM);
    posRepFlag = false;
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
    diagRequest = !diagRequest; // activate diag


    Serial.print("diagRequest = "); Serial.println(diagRequest);
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

    clutchLogic = !clutchLogic;
    EEPROM.update(71,clutchLogic);
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
  //Serial.println("READ");
}

void RPMcounter(){
  times = millis()-timeOld;        //finds the time 
  RPM = (30108/times);         //calculates rpm
  //if (SparkState == 1){

    RPM = RPM*cylCount;
  //}
  if ((RPM - RPMold) > 10000){
    RPM = RPMold;
  }
  timeOld = millis();
 // Serial.print("RPMs are: "); Serial.println(RPM);
  // Serial.print("RPMOLDs are: "); Serial.println(RPMold);
  if ((RPM > 500) && (RPM < 7000)){
    RPMold = RPM;
  }
  trigCounter = 0;
}

void ignControl(){
  //Do ignition control methods

  if (timeToCut > 1000){
    if ((RPM > cutRPM) && (SparkState == HIGH) && (!cutActive)){
      //Spark is cut
      SparkState = LOW;
      cutActive = true;
      digitalWrite(bluePin, LOW);
      digitalWrite(redPin, HIGH);
      cutTime = millis();
    }
    else if ((cutActive) && (millis() - cutTime > timeToCut)){
        SparkState = HIGH;
        cutTime = millis();
        digitalWrite(bluePin, HIGH);
        digitalWrite(redPin, LOW);
    }
  }
  else{
    if ((RPM > cutRPM) && (SparkState == HIGH) && (!cutActive)){
        //Spark is cut
        cutActive = true;
        cutTime = millis();
      }
      if ((RPM < cutRPM +500) && (cutActive) && (millis() - cutTime > timeToCut)){
        SparkState = !SparkState;
        cutTime = millis();
        digitalWrite(bluePin, SparkState);
        digitalWrite(redPin, !SparkState);
      }
      else if((RPM >= cutRPM +500) && (cutActive)){
        SparkState = LOW;
        digitalWrite(bluePin, LOW);
        digitalWrite(redPin, HIGH);
      }
      if (RPM < cutRPM - 1500){
        cutActive = false;
        SparkState = HIGH;
      }
  }
  /*
   if ((RPM > cutRPM) && (SparkState == HIGH) && (!cutActive){
      //Spark is cut
      SparkState = LOW;
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
        SparkState = HIGH;
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
