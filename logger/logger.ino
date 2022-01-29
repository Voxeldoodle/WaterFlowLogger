#include <SPI.h>
#include <SD.h>

#include <U8g2lib.h>
#include <Wire.h>
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#include "RTClib.h"
RTC_DS1307 rtc;

#define RECPIN 6
#define SETPIN 7
#define IMPPIN 8
#define SDPIN 10

//rotary encoder pins
#define inputCLK 11
#define inputDT 12

#define STATE (state & 0x0F)
#define OLDSTATE (state & 0xF0)

#define CHANNELS 3
#define VOLDIGITS 4
#define REFRESHSTATES 0

int volumeUnit = 100;
//String measureUnit = "l/s";
int timeUnit = 1000;
long volume[CHANNELS] = {0};
float t0;
float t1;
float flow[CHANNELS] = {0};
float avgFlow[CHANNELS] = {0};

float alfa = 0.25;

short refreshRate = 1;

char strBuffer[30];

bool recording = false;
short state = 0;
//Optimized with bitmask
//short oldState = -1;

unsigned long pressTime = 0;

#define PRESS1 1
#define PRESS2 2
#define PRESS3 4
#define PRESSED1 (pressMask & PRESS1)
#define PRESSED2 (pressMask & PRESS2)
#define PRESSED3 (pressMask & PRESS3)

short pressMask = 0;

#define SETTIME 2000 //millis to press to enable settings

File impulseFile;
File avgFile;

//TODO: remove counter & encdir once obtained good R.E.
short counter = 0; 
short curStateCLK;
short prevStateCLK;
String encdir ="";

void u8g2_prepare() {
 u8g2.setFont(u8g2_font_6x10_tf);
 u8g2.setFontRefHeightExtendedText();
 u8g2.setDrawColor(1);
 u8g2.setFontPosTop();
 u8g2.setFontDirection(0);
}

void setup() {
  Serial.begin(9600);

  //commented reference garbage
  /*if (!SD.begin(SDPIN)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("SD initialization done."));

  avgFile = SD.open("averages.txt", FILE_WRITE);

  /*if (myFile) {
    Serial.print(F("Writing to test.txt..."));
    myFile.println(F("testing 1, 2, 3."));
    // close the file:
    myFile.close();
    Serial.println(F("done."));
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
  }

   /*myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println(F("test.txt:"));

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
  }*/

  pinMode(RECPIN,INPUT);
  pinMode(SETPIN,INPUT);
  pinMode(IMPPIN,INPUT);
  pinMode(inputCLK,INPUT);
  pinMode(inputDT,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  prevStateCLK = digitalRead(inputCLK);

  u8g2.begin();
  u8g2_prepare();
}

void loop() {
  curStateCLK =  digitalRead(inputCLK);
  readRotaryEnc();
  
  setRecording();
  if (recording){
    digitalLogFlowRate(IMPPIN);
  }
  setState();
  stateFunctions();
  refreshDisplay();

  prevStateCLK = curStateCLK;
}

void setRecording(){
  int rec = digitalRead(RECPIN);
  if (rec == LOW  && !PRESSED1){
    pressMask += PRESS1;
    Serial.println(F("Pressed1"));
  }else if(rec == HIGH && PRESSED1){
    pressMask -= PRESS1;
    recording = !recording;
    digitalWrite(LED_BUILTIN, recording);
    Serial.println(recording ? "Recording: ON" : "Recording: OFF");
    t0 =(float) millis()/timeUnit;
  }
}

void digitalLogFlowRate(int source){
  int impulso = digitalRead(source);
  if (impulso == LOW  && !PRESSED2){
    t1 = (float) millis()/timeUnit;
    pressMask += PRESS2;
    
    volume[source - IMPPIN] += volumeUnit;
    flow[source - IMPPIN] = (float) volumeUnit / (t1 - t0);

    float tmp = alfa * flow[source - IMPPIN];
    avgFlow[source - IMPPIN] = (float) ( isinf(tmp) ? 0 : tmp  + (1-alfa)*avgFlow[source - IMPPIN]);
    
    t0 = t1;

    //TODO: Save data on SD
  }else if(impulso == HIGH && PRESSED2){
    pressMask -= PRESS2;
  }
}

void setState(){
  //oldState = state;
  state = STATE + (STATE << 4);
  int action = digitalRead(SETPIN);
  if (action == LOW  && !PRESSED3){
    pressMask += PRESS3;
    pressTime = millis();
  }else if(action == HIGH && PRESSED3){
    pressMask -= PRESS3;

    if (STATE < CHANNELS)
      state = (STATE + 1) % CHANNELS + OLDSTATE;
    else if(STATE > CHANNELS || (STATE == CHANNELS && (millis() - pressTime) < SETTIME))
      state = (STATE + 1) % (CHANNELS + VOLDIGITS + REFRESHSTATES)+ OLDSTATE;
  }
  if(PRESSED3 && action == LOW && (millis() - pressTime) > SETTIME){
    state = CHANNELS + OLDSTATE;
  }
}

void stateFunctions(){
  if (STATE != OLDSTATE >> 4){
    //Serial.println(STATE);
    //If no function to add here (activation at state change), remove OLDSTATE and simplify state management.    
  }
  if (STATE >= CHANNELS){
      short digits[4] = {volumeUnit / 1000, (volumeUnit % 1000) /100, (volumeUnit % 100) /10, volumeUnit % 10};
      digits[STATE-CHANNELS] = abs(digits[STATE-CHANNELS] + readRotaryEnc()) % 10;
      volumeUnit = 1000*digits[0] + 100*digits[1] + 10*digits[2] + digits[3];
  }
}

void refreshDisplay(){
  u8g2.firstPage();
  do {

    if (recording){
      u8g2.drawCircle(122, 4, 4);
      u8g2.drawDisc(122, 4, 2);
    }else{
      u8g2.drawCircle(122, 4, 4);
    }
    ("State: " + String(STATE)).toCharArray(strBuffer,10);
    u8g2.drawStr(64, 0, strBuffer);
    
    if (STATE < CHANNELS){
      //commented garbage
      /*char str[20];
      printFloat(flow, str);
      sprintf(strBuffer, "Flow: %s%s  %s",str, measureUnit);*/
      
      ("Flow: " + String(flow[STATE],3)).toCharArray(strBuffer, 20);
      u8g2.drawStr(0, 10, strBuffer);

      //commented garbage
      /*char avg[20];
      printFloat(avgFlow, avg);
      sprintf(strBuffer, "Average: %s", avg);*/
      ("Average: " + String(avgFlow[STATE], 3)).toCharArray(strBuffer, 20);
      u8g2.drawStr(0, 20 , strBuffer);
    }

    if(STATE >= CHANNELS){
      sprintf(strBuffer, "Imp: %04d", volumeUnit);
      //("Imp: " + String(volumeUnit)).toCharArray(strBuffer, 20);
      u8g2.drawStr(0, 10, strBuffer);
      //("Imp:").length()*2
      u8g2.drawStr(6*2+(STATE)*6, 12, "_");
    }
  } while ( u8g2.nextPage() );
}

short readRotaryEnc(){
  short res =  0;
  if (curStateCLK != prevStateCLK){   
     // If the inputDT state is different than the inputCLK state then 
     // the encoder is rotating counterclockwise
     if (digitalRead(inputDT) != curStateCLK) { 
       counter --;
       encdir ="CCW";
       res = -1;
       
     } else {
       // Encoder is rotating clockwise
       counter ++;
       encdir ="CW";
       res = 1;
       
     }
     //commented debug garbage
     /*Serial.print("Direction: ");
     Serial.print(encdir);
     Serial.print(" -- Value: ");
     Serial.println(counter);*/
   }
   return res; 
}

//commented garbage
/*void printFloat(float num, char* str){
  float tmpVal = (num < 0) ? -num : num;

  int tmpInt1 = tmpVal;                  // Get the integer (678).
  float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
  int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

  // Print as parts, note that you need 0-padding for fractional bit.

  sprintf(str, "%s%d.%04d", (num < 0) ? "-" : "", tmpInt1, tmpInt2);
}*/
