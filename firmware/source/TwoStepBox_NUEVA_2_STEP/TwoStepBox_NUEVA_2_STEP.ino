
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
byte ArmTriggerPin = 14;
byte bluePin = 18;
byte redPin = 17;

byte analogIn = A4;

void loadCalibration();
//byte pinOne = A0;
//byte pinTwo = A1;
//byte pinThree = A2;
//byte pinFour = A3;


//Program variables
unsigned int cutRPM;

unsigned int timeToCut; 
//int armVoltage;
byte positionCheckcount = 0;
byte potPosition;
byte trigCount = 0;
uint8_t cylCount = 4;
byte RPMerror = 0;

byte bitField = B11000101; //field of working bits for states; first bit is 7, last is 0

#define BIT_POS_REP_FLAG  0 //Position Report enabled
#define BIT_DIAG_REQUEST  1 
#define BIT_CLUTCH_LOGIC  2
#define BIT_CUT_ACTIVE    3
#define BIT_ARM_TRIGGER   4
#define BIT_SPARK_STATE   5
#define BIT_TOYOTA_FAKE   6
#define BIT_RPM_FILTER    7


//Working Variables
unsigned int RPM;
unsigned int RPMold;
unsigned long times;
unsigned long timeOld;
byte oldPotPos = 0;
//bool correctionFlag = false;
byte trigCounter;
unsigned long positionReport = 0;

//toyota function
unsigned int toyotaDivision = 0;
uint16_t timeCount = 0;
uint32_t timeTick = 0;
bool timeTickOK = true;
byte fakePin = 2;
byte fakePinTwo = 4;
bool fakeOut = false;
bool divideRPM = false;
byte toyoCyl = 6;

void setup() {
  //Begin Serial comms
  Serial.begin(115200);

  //Set I/O
  pinMode(SparkOutput, OUTPUT);
 pinMode(bluePin,OUTPUT);
 pinMode(redPin,OUTPUT);
 pinMode(fakePin, OUTPUT);
  pinMode(fakePinTwo, OUTPUT);

  pinMode(ArmTriggerPin, INPUT_PULLUP);
  pinMode(RPMinputPin, INPUT);
//  pinMode(pinOne, INPUT_PULLUP);pinMode(pinTwo, INPUT_PULLUP);pinMode(pinThree, INPUT_PULLUP);pinMode(pinFour, INPUT_PULLUP);
  digitalWrite(SparkOutput, HIGH); //activate spark output

  digitalWrite(bluePin,HIGH);
   Serial.println("Box Ready");
//   Serial.println(bitField);
    //Initialize variables
   RPM = 0;
   RPMold = 0;
   trigCounter = 0;
   timeOld = 0;
   times = 0;
  cutRPM = 4000;
   /*armTriggerbool = false;
   bitSet(bitField, BIT_SPARK_STATE);*/
  //bitClear(bitField, BIT_DIAG_REQUEST);
  //bitSet(bitField, BIT_RPM_FILTER);
  //Serial.println(bitField);
  positionCheckcount = 0;
  trigCounter = 0;
  timeToCut = 5;
  bitSet(bitField, BIT_SPARK_STATE);
  //cylCount = EEPROM.read(70);
//  potPosition = (8*!digitalRead(pinFour)) + (4*!digitalRead(pinThree)) +(2*!digitalRead(pinTwo)) +(!digitalRead(pinOne)) ;
potPosition = 0;
  if (EEPROM.read(255) != 23){
    writeDefaultSettings();
  }
 loadCalibration();
 
 
  if (cylCount > 12){
    cylCount = 4;
  }

   //Attach RPM input interrupt
  attachInterrupt(digitalPinToInterrupt(3), triggerCounter, RISING);
}

void loop() {
  //Serial.println(bitRead(bitField, BIT_RPM_FILTER));
  if ((bitRead(bitField, BIT_RPM_FILTER)) && ((micros() - timeOld) > 4000)){
    attachInterrupt(digitalPinToInterrupt(3), triggerCounter, RISING);
    //Serial.println("Filter ok");
  }

  if (bitRead(bitField, BIT_TOYOTA_FAKE)){
    toyotaFunction();
   // Serial.println("Toyta");
  }
  
  if (bitRead(bitField, BIT_CLUTCH_LOGIC)){

    bitWrite(bitField,BIT_ARM_TRIGGER,digitalRead(ArmTriggerPin));

  }
  else{  bitWrite(bitField,BIT_ARM_TRIGGER,!digitalRead(ArmTriggerPin));}
//  Serial.println(digitalRead(ArmTriggerPin));


  if (trigCounter > trigCount){
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
      oldPotPos = potPosition;
      //potPosition = (8*!digitalRead(pinFour)) + (4*!digitalRead(pinThree)) +(2*!digitalRead(pinTwo)) +(!digitalRead(pinOne)) ;
     potPosition = 0;
     if (oldPotPos != potPosition){
      loadCalibration();
     }
      positionCheckcount = 0;
    }
}

void toyotaFunction(){
  if (trigCounter > 0){
    toyotaDivision = (times/toyoCyl)/100;
    // divides the current time between pulses by the number of cylinders to obtain the time one output will fire
   //times is the amount of time it took for one cycle
   timeTick=micros()/100;
  }
   if(((((micros()/100)- timeTick) % toyotaDivision) == 0) && (timeTickOK)){
    timeCount++;
    timeTickOK = false;
    //Serial.print("milllis - timeTick = "); Serial.println(millis()-timeTick);
  //  Serial.print(" ticking ttiime = ");Serial.println(timeCount);
     if (!bitRead(bitField, BIT_SPARK_STATE)){
        if (toyoCyl == 4){
          digitalWrite(fakePin, HIGH);
        //  Serial.println("4 cyl output high");
          if(timeCount > 3){ timeCount = 0;}
        }
        else if (toyoCyl == 6){
          fakeOut = !fakeOut;
          digitalWrite(fakePin, fakeOut);
          digitalWrite(fakePinTwo, !fakeOut);
          if(timeCount > 5){ timeCount = 0;}
        }
      }
   }
   else if( ((!timeTickOK) && (((micros()/100) - timeTick) % toyotaDivision)) == 0 ){}
   else{ 
      timeTickOK = true;
      if(toyoCyl == 4){
       // if((((micros()/100) - timeTick) % (toyotaDivision + (toyotaDivision/2))) == 0){
          digitalWrite(fakePin, LOW);
       // }
      }
    }
}

void SerialDiag(){
  unsigned int posRep = millis() - positionReport;
  
  if (posRep >= 1000){
   /* Serial.print("Pot POsition: "); Serial.println(potPosition);
    Serial.print("cutRPM: "); Serial.println(cutRPM);
    Serial.print("RPM hysterisis: "); Serial.print(timeToCut);Serial.println("ms");
    Serial.print("Clutch Armed: ");Serial.println(bitRead(bitField,BIT_ARM_TRIGGER));
    Serial.print("CylMode: ");Serial.println(cylCount);
    //Serial.print("AnalogIn: ");Serial.println(armVoltage);
     Serial.print("Ignition Output: ");Serial.println(bitRead(bitField, BIT_SPARK_STATE));*/
     Serial.print("v");
     Serial.print(potPosition);
     Serial.print(timeToCut);
     Serial.print(bitRead(bitField,BIT_ARM_TRIGGER));
     Serial.print(cylCount);
     Serial.print(bitRead(bitField,BIT_SPARK_STATE));
     Serial.print(bitRead(bitField, BIT_TOYOTA_FAKE));
     Serial.print(bitRead(bitField, BIT_RPM_FILTER));
     Serial.print(cutRPM);
     Serial.print(trigCount);
     posRep = 0;
     positionReport = millis();
  }
  if (((posRep % 150) == 0) && (bitRead(bitField, BIT_POS_REP_FLAG))){
    Serial.print("r"); Serial.print(RPM);
    bitClear(bitField, BIT_POS_REP_FLAG);
  }
  else if((posRep % 150) != 0){
    bitSet(bitField, BIT_POS_REP_FLAG);
  }
}

void SerialComms(){
  char commandChar = Serial.read();
  if (commandChar == 'p'){ //write command
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
    while (i < 4){
      byte serialByte;
      int serialInt;
      Serial.println(F("Entrar un numero del 0-9 (corresponden a 1-10) para la posicion\r\nque desea grabar"));
        while (i == 0){
          if (Serial.available()){
            serialByte = Serial.read();
            serialByte = serialByte - 48;
            serialByte = constrain(serialByte, 0, 9);
            potPosition = serialByte;
            i++;
          }
        }
        while (i == 1){ // enter RPM dialog
         byte readRPM[4];
         byte digits = 0;
         Serial.print(F("La posicion que se esta editando es: "));Serial.println(potPosition);Serial.println(F("------"));
         Serial.println(F("Entrar los RPM que desea que se active el corte"));
         while (digits < 4){
          if(Serial.available()){
            readRPM[digits] = Serial.read()-48;
            if (readRPM[digits] == 11){ break;}
            //Serial.println(digits);
           // Serial.println(readRPM[digits]);
            digits++;
          }
         }
          cutRPM = 0;
          for (int i = 0; i < digits; i++){
           // Serial.print(cutRPM);Serial.print(" + ");Serial.println(readRPM[i]*powint(10,digits-i-1));
            cutRPM = cutRPM + readRPM[i]*powint(10,digits-i-1);
            
          }           
          //cutRPM = readRPM[0]*1000 + readRPM[1]*100 + readRPM[2]*10 +readRPM[3];
          cutRPM = constrain(cutRPM, 0, 8000);
          i++;
        }
        while ( i == 2 ){ // enter cut time dialog
          byte readDC[4] ={255, 255, 255, 255};
         byte digits = 0;
         Serial.print(F("Los RPM de corte entrados fueron: ")); Serial.println(cutRPM);Serial.println(F("------"));
         Serial.println(F("Entre la ventana de corte deseada.\r\nEj. Si el corte esta a 4000RPM y desactive el corte a 3800RPM,\r\nentre 200, 4000-200 = 3800"));
         Serial.println(F("**Si usa un numero menor de 1000, por favor\r\nescriba ';' como su ultimo digito, sin los colchetes**"));
         while (digits < 4){
          if(Serial.available()){
            readDC[digits] = Serial.read()-48;
            if (readDC[digits] == 11){ break;}
            digits++;
          }
         }
          timeToCut = 0;
          for (int i = 0; i < digits; i++){
            timeToCut = timeToCut + readDC[i]*powint(10,digits-i-1);
          }
         i++;
         }
      if ( i == 3){
         Serial.print(F("La ventana de corte es: "));Serial.println(timeToCut);
//         Serial.print(F("Resultando en un punto de reactivacion a "));Serial.println(cutRPM-timeToCut);Serial.println(F("------"));
          burnCalibration(potPosition, cutRPM, timeToCut);
          i++; // i == 4
      }
    }// end while
  } // End write command
  
  else if (commandChar == 'r'){
    bitWrite(bitField, BIT_DIAG_REQUEST, !bitRead(bitField, BIT_DIAG_REQUEST)); // activate diag
    if(bitRead(bitField, BIT_DIAG_REQUEST)){
      Serial.print("ok");
      delay(100);
    }
    //Serial.print("bitField = ");Serial.println(bitField, BIN);
    //Serial.print("diagRequest = "); Serial.println(bitRead(bitField, BIT_DIAG_REQUEST));
  }
  else if (commandChar == 'l'){
    loadCalibration();
  }

 else if (commandChar == 'i'){

    bitWrite(bitField, BIT_CLUTCH_LOGIC, !(bitRead(bitField, BIT_CLUTCH_LOGIC)));
    EEPROM.update(71,(bitRead(bitField, BIT_CLUTCH_LOGIC)));
    Serial.println(F("Clutch logic inverted!"));
  }// end I case

  else if (commandChar == 't'){
    //bitWrite(bitField, BIT_TOYOTA_FAKE, !(bitRead(bitField, BIT_TOYOTA_FAKE)));
    //EEPROM.update(72,(bitRead(bitField, BIT_TOYOTA_FAKE)));
    //Serial.print(F("Toyota output inverted:"));Serial.println(bitRead(bitField, BIT_TOYOTA_FAKE));
    char tempCyl = 0;
    Serial.println("Enter 'f' for 4 cylinder Toyota, or 's' for six cylinder Toyota");
    while ( tempCyl == 0){
       if (Serial.available()){tempCyl = Serial.read();}
       if (tempCyl == 'f'){
        bitWrite(bitField, BIT_TOYOTA_FAKE, 1);
        EEPROM.update(72,(bitRead(bitField, BIT_TOYOTA_FAKE)));
        toyoCyl = 4;
        EEPROM.update(74, 4);
        Serial.println("Four cyl set!");
       }
       else if (tempCyl == 's'){
        bitWrite(bitField, BIT_TOYOTA_FAKE, 1);
        EEPROM.update(72,(bitRead(bitField, BIT_TOYOTA_FAKE)));
        toyoCyl = 6;
        EEPROM.update(74, 6);
        Serial.println("Six cyl set!");
       }
       else if (tempCyl == 'e'){
        bitWrite(bitField, BIT_TOYOTA_FAKE, 1);
        EEPROM.update(72,(bitRead(bitField, BIT_TOYOTA_FAKE)));
        toyoCyl = 8;
        EEPROM.update(74, 6);
        Serial.println("Six cyl set!");
       }
       else if(tempCyl == 'o'){
        bitWrite(bitField, BIT_TOYOTA_FAKE, 0);
        EEPROM.update(72,(bitRead(bitField, BIT_TOYOTA_FAKE)));
       }
       else{
        Serial.println("Please enter a valid value!");
        toyoCyl = 0;
        tempCyl = 0;
       }
    }
  }
  else if (commandChar == 'f'){
    bitWrite(bitField, BIT_RPM_FILTER, !(bitRead(bitField, BIT_RPM_FILTER)));
    EEPROM.update(73,(bitRead(bitField, BIT_RPM_FILTER)));
    Serial.print(F("RPM Filter inverted: "));Serial.println(bitRead(bitField, BIT_RPM_FILTER));
  }
  else if (commandChar == 'c'){
    byte i = 0;
    //Serial.println(F("Enter 'd' for distributor, 'w' for wasted spark, or 'c' for Coil On Plug"));
    while (i < 1){
      if (Serial.available()){
        char serialRead = Serial.read();
        if (serialRead == '1'){
          cylCount = 1;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '2'){
          cylCount = 2;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '3'){
          cylCount = 3;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '4'){
          cylCount = 4;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '5'){
          cylCount = 5;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '6'){
          cylCount = 6;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '7'){
          cylCount = 7;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '8'){
          cylCount = 8;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '9'){
          cylCount = 9;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '/'){
          divideRPM = true;
          EEPROM.update(75, divideRPM);
          i++;
        }
        else if (serialRead == '*'){
          divideRPM = false;
          EEPROM.update(75, divideRPM);
          i++;
        }
        else if (serialRead == 'a'){
          trigCount = 0;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'b'){
          trigCount = 1;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'c'){
          trigCount = 2;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'd'){
          trigCount = 3;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'e'){
          trigCount = 4;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'f'){
          trigCount = 5;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'g'){
          trigCount = 6;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'h'){
          trigCount = 7;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'i'){
          trigCount = 8;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'j'){
          trigCount = 9;
          EEPROM.update(76,trigCount);
          i++;
        }
        else if (serialRead == 'k'){
          trigCount = 10;
          EEPROM.update(76,trigCount);
          i++;
        }
        
       /* else if (serialRead == '10'){
          cylCount = 10;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '11'){
          cylCount = 11;
          EEPROM.update(70,cylCount);
          i++;
        }
        else if (serialRead == '12'){
          cylCount = 12;
          EEPROM.update(70,cylCount);
          i++;
        }*/
      } // end serial avail
    }// end while
    Serial.print(F("Spark mode set! Current mode is: "));Serial.println(cylCount);
  }// end C case
}

void triggerCounter(){
  if(bitRead(bitField, BIT_RPM_FILTER)){
    detachInterrupt(digitalPinToInterrupt(3));
  }
  trigCounter++;
  //Serial.println("TRIGGER!");
}

void RPMcounter(){
 // Serial.print("micros "); Serial.print(micros());Serial.print("timeOld ");Serial.println(timeOld);
  times = micros()-timeOld;        //finds the time 
  //Serial.print("Times: "); Serial.println(times);
  RPM = (30108000/times);         //calculates rpm
  RPM = RPM*cylCount;
  //Serial.println(RPM);
  /*if (divideRPM){RPM = RPM/(cylCount);}
  else{RPM = RPM*(cylCount);}*/
  
  if ((RPM > 0) && (RPM < 10000)){
    RPMold = RPM;
  }
  if ((RPM - RPMold) > 10000){
    RPM = RPMold;
  }
  timeOld = micros();
  
  
  trigCounter = 0;
}

void ignControl(){
  //Do ignition control methods

  if ((RPM > cutRPM) && !(bitRead(bitField, BIT_CUT_ACTIVE))){
    bitClear(bitField, BIT_SPARK_STATE);
    bitSet(bitField,BIT_CUT_ACTIVE);
    digitalWrite(bluePin, LOW);
    digitalWrite(redPin, HIGH);
  }
  else if ((RPM < (cutRPM - timeToCut)) && (bitRead(bitField, BIT_CUT_ACTIVE))){
    digitalWrite(bluePin, HIGH);
    digitalWrite(redPin, LOW);
    bitClear(bitField,BIT_CUT_ACTIVE);
    bitSet(bitField, BIT_SPARK_STATE);
  }
}


void loadCalibration(){
//  byte tempPotPos = (8*!digitalRead(pinFour)) + (4*!digitalRead(pinThree)) +(2*!digitalRead(pinTwo)) +(!digitalRead(pinOne));
    byte tempPotPos = 0;
    Serial.print(("tempPotPoggs = "));Serial.println(tempPotPos);
    
    cutRPM = word(EEPROM.read(tempPotPos+tempPotPos), EEPROM.read(tempPotPos+tempPotPos+1));
    Serial.print("cutRPM: ");Serial.println(cutRPM);
    timeToCut = word(EEPROM.read(tempPotPos+tempPotPos+32), EEPROM.read(tempPotPos+tempPotPos+33));
    Serial.print("RPM hysterisis: ");Serial.println(timeToCut);
    cylCount = EEPROM.read(70);
    trigCount = EEPROM.read(76);
    bitWrite(bitField, BIT_CLUTCH_LOGIC, EEPROM.read(71));
    bitWrite(bitField, BIT_TOYOTA_FAKE, EEPROM.read(72));
    bitWrite(bitField, BIT_RPM_FILTER, EEPROM.read(73));
    toyoCyl = EEPROM.read(74);
    divideRPM = EEPROM.read(75);

}
