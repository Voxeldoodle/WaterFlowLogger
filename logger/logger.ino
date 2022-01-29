/*
 * DataLogging script with following features:
 * - multiple channel reading of impulses representing established volumes of water
 * - display values for each channel about: instant flow, average flow, total volume
 * - log values on SD at established rate (same rate used for refreshing values on display)
 * - all preferences read from SD at boot time
 * 
 * Last update: 27/10/2021
 * Author: Antonio Casoli
 */

#include <SPI.h>
#include <SD.h>

#include <Arduino.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(U8X8_PIN_NONE);

#include "RTClib.h"
RTC_DS3231 rtc;

#define LEDPIN 4
#define RECPIN 5
#define SETPIN 6
#define IMPPIN1 7
#define IMPPIN2 8
#define IMPPIN3 9
#define SDPIN 10

#define CHANNELS 3
#define VOLDIGITS 4

int volumeUnit[CHANNELS] = {-1,-1,-1};
#define timeUnit 1000
unsigned long volume[CHANNELS] = {0};
float t0;
float t1;
float flow[CHANNELS] = {0};
float avgFlow[CHANNELS] = {0};
short impCount[3] = {0};

char volUnits[CHANNELS][4];
int timeUnits[CHANNELS] = {-1,-1,-1};
char logFiles[CHANNELS][12];

#define ALFA 0.25

short refreshRate[3] = {-1,-1,-1};
short refreshMask = 0;

char strBuffer[40];

bool recording = false;
short state = 0;
short oldState = -1;

#define PRESS1 1
#define PRESS2 2
#define PRESS3 4
#define PRESS4 8
#define PRESS5 16
#define PRESS6 32
#define PRESSED4 (pressMask & PRESS4)
#define PRESSED5 (pressMask & PRESS5)
#define PRESSED6 (pressMask & PRESS6)

short pressMask = 0;

#define SETTIME 2000 //millis to press to cycle view

File fileBuf;

void(* resetFunc) (void) = 0;

void dispError(char * err1, char * err2){
  Serial.println(F("Disp Err"));
  u8x8.clear();
  //err1.toCharArray(strBuffer,40);
  u8x8.drawString(0,1,err1);
  //err2.toCharArray(strBuffer,40);
  u8x8.drawString(0,2,err2);
  u8x8.drawString(0,3,"Premi set per OK");
  int set = 0;
  do{
    set = digitalRead(SETPIN);
  }while(set != LOW);
}

void parseSettings(){
  String buf;
  short i = 0;
  
  // read from the file until there's nothing else in it:
  while (fileBuf.available()) {
    buf = fileBuf.readStringUntil('\n');
    Serial.println(buf);
    //skip commented lines
    if (buf[0] == '%'){
      continue; 
    }
    buf.toCharArray(strBuffer,40);
    if(strncmp(strBuffer,"Nome", 4) ==0){
      i = atoi(&strBuffer[5]);
      strcpy(logFiles[i-1], &strBuffer[8]);
      
    }else if(strncmp(strBuffer,"Unità volume", 12) == 0){
      i = atoi(&strBuffer[13]);
      volumeUnit[i-1] = atoi(&strBuffer[17]);
      
      short j = 17;
      while(strBuffer[j] != ' '){
        j++;
      }
      strcpy(volUnits[i-1], &strBuffer[++j]);
      
    }else if(strncmp(strBuffer,"Unità tempo",11) == 0){
      i = atoi(&strBuffer[13]);
      switch(strBuffer[16]){
        case 's':
          timeUnits[i-1] = 1;
          break;
        case 'm':
          timeUnits[i-1] = 60;
          break;
        case 'h':
          timeUnits[i-1] = 3600;
          break;
        default:
          timeUnits[i-1] = -1;
          break;
      }
      
    }else if(strncmp(strBuffer,"Refresh rate",12) == 0){
      i = atoi(&strBuffer[13]);
      refreshRate[i-1] = atoi(&strBuffer[16]);
    }
  }
}

void checkVars(bool fileErr){
  Serial.println(F("Checking Vars"));
  if (fileErr){
    u8x8.clear();
    u8x8.drawString(0,1,"Err opening pref");
    u8x8.drawString(0,2,"Check SD");
    u8x8.drawString(0,3,"Reset in 10s");
    delay(10000);
    resetFunc();
  }
  for (short i=0; i<CHANNELS; i++){
    if (volumeUnit[i] < 0){
      volumeUnit[i] = 0;
      sprintf(strBuffer, "Volume %d error", i+1);
      if (!fileErr)
        dispError(strBuffer, "Default val 0");
    }
    if (timeUnits[i] < 0){
      timeUnits[i] = 1;
      sprintf(strBuffer, "TimeUnit %d error", i+1);
      if (!fileErr)
        dispError(strBuffer, "Default val s");
    }
    if (refreshRate[i] < 0){
      refreshRate[i] = 1;
      sprintf(strBuffer, "Refresh %d error", i+1);
      if (!fileErr)
        dispError(strBuffer, "Default val 1");
    }
    Serial.println(logFiles[i]);
    Serial.println(volumeUnit[i]);
    Serial.println(volUnits[i]);
    Serial.println(timeUnits[i]);
    Serial.println(refreshRate[i]);
  }
}

void writeHeaders(){
  for (short i = 0; i < CHANNELS; i++){
    if (!SD.exists(logFiles[i])){
      fileBuf = SD.open(logFiles[i], FILE_WRITE);
      if (!fileBuf)
        Serial.println(F("Error in opening file!!"));
      else{
        Serial.println(F("File opened succesfully!!"));
        sprintf(strBuffer, "Time, Volume");
        Serial.println(strBuffer);
        fileBuf.println(strBuffer);
      }
      fileBuf.close(); 
    }
  }
}

void setup() {
  Serial.begin(9600);  

  pinMode(RECPIN,INPUT);
  pinMode(SETPIN,INPUT);
  pinMode(IMPPIN1,INPUT);
  pinMode(IMPPIN2,INPUT);
  pinMode(IMPPIN3,INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);

  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    abort();
  }
  rtc.disable32K();

  bool fileErr = false;
  if (!SD.begin(SDPIN)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("SD initialization done."));
  
  //max filename 8 chars + .ext
  fileBuf = SD.open("pref.txt", FILE_READ);
  if (fileBuf) {
    Serial.println(F("opened pref"));
    parseSettings();
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening pref file"));
    fileErr = true;
  }
  // close the file:
  fileBuf.close();

  checkVars(fileErr);

  writeHeaders();

  Serial.println(F("Setup Complete!"));
}

void loop() {
  setRecording();
  if (recording){
    digitalLogFlowRate(IMPPIN1);
    digitalLogFlowRate(IMPPIN2);
    digitalLogFlowRate(IMPPIN3);
  }
  setState();
  //stateFunctions();
  refreshDisplay();

  oldState = state;
}

void setRecording(){
  int rec = digitalRead(RECPIN);
  if (rec == LOW  && !PRESSED4){
    pressMask += PRESS4;
    Serial.println(F("Pressed1"));
  }else if(rec == HIGH && PRESSED4){
    pressMask -= PRESS4;
    recording = !recording;
    digitalWrite(LEDPIN, recording);
    digitalWrite(LED_BUILTIN, recording);
    Serial.println(recording ? "Recording: ON" : "Recording: OFF");
    t0 =(float) millis()/timeUnit;
  }
}

void digitalLogFlowRate(int source){
  int impulso = digitalRead(source);
  short ind = source - IMPPIN1;
  if (impulso == LOW  && !(pressMask & (1 << ind))){
    t1 = (float) millis()/timeUnit;
    pressMask += 1 << ind;
    impCount[ind] += 1;

    if (impCount[ind] % refreshRate[ind] == 0)
      refreshMask += 1 << ind;
    
    volume[ind] += volumeUnit[ind];
    flow[ind] = (float) volumeUnit[ind] / (t1 - t0);

    float tmp = ALFA * flow[ind];
    avgFlow[ind] = (float) ( isinf(tmp) ? (1-ALFA)*avgFlow[ind] : tmp  + (1-ALFA)*avgFlow[ind]);
    
    t0 = t1;

    //TODO: log ONLY at refresh rate
    if (refreshMask & (1 << ind)){
      fileBuf = SD.open(logFiles[ind], FILE_WRITE);
      if (!fileBuf)
        Serial.println(F("Error in opening file!!"));
      else{
        Serial.println(F("File opened succesfully!!"));
        DateTime time = rtc.now();
        sprintf(strBuffer, "YYYY-MM-DD hh:mm:ss");
        sprintf(strBuffer, time.toString(strBuffer));
        sprintf(strBuffer, "%s, %d",strBuffer, volume[ind]);
        Serial.println(strBuffer);
        fileBuf.println(strBuffer);
      }
      fileBuf.close();    
    }    
  }else if(impulso == HIGH && (pressMask & (1 << ind))){
    pressMask -= 1 << ind;
  }
}

void setState(){
  int action = digitalRead(SETPIN);
  if (action == LOW  && !PRESSED5){
    pressMask += PRESS5;
  }else if(action == HIGH && PRESSED5){
    pressMask -= PRESS5;
    state = (state + 1) % 3;
  }
}
  
void refreshDisplay(){
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  if (state != oldState || refreshMask){
    if (state != oldState)
      u8x8.clear();

    char timeU = 'u';
    switch(state){
      case 0:
        //u8x8.clearLine(0);
        u8x8.drawString(0,0,"Portata ist.");
        for (short i = 0; i < CHANNELS; i++) {
          if (state != oldState || refreshMask & (1<<i)){
            u8x8.setCursor(0,i+1);
            //TODO: convert unit using right timeUnit
            double flo = (double) flow[i] * timeUnits[i];
            timeU = timeUnits[i] == 1 ? 's' : (timeUnits[i] == 60 ? 'm' : 'h');
            //sprintf(strBuffer, "%d: %s %s/%c", i+1, String(flo,3), volUnits[i], timeU); 
            (String(i+1) + ": " + String(flo,3) + " " + volUnits[i] + "/"+ timeU).toCharArray(strBuffer, 15);
            u8x8.print(strBuffer);
          }              
        }
        break;            
      case 1:
        u8x8.drawString(0,0,"Media");
        for (short i = 0; i < CHANNELS; i++) {
          if (state != oldState || refreshMask & (1<<i)){
            u8x8.setCursor(0,i+1);
            //TODO: convert unit using right timeUnit
            double avgFlo = (double) avgFlow[i] * timeUnits[i];
            timeU = timeUnits[i] == 1 ? 's' : (timeUnits[i] == 60 ? 'm' : 'h');
            (String(i+1) + ": " + String(avgFlo,3) + " " + volUnits[i] + "/"+ timeU).toCharArray(strBuffer, 15);
            u8x8.print(strBuffer);
          }
        }
        break;
      case 2:
        u8x8.drawString(0,0,"Volume");
        for (short i = 0; i < CHANNELS; i++) {
          if (state != oldState || refreshMask & (1<<i)){
            u8x8.setCursor(0,i+1);
            (String(i+1) + ": " + String(volume[i])+ " "+ volUnits[i]).toCharArray(strBuffer, 15);
            u8x8.print(strBuffer);
          }
        }
        break;
      default:
        //u8x8.setCursor(4,1);
        u8x8.drawString(4,1,"STATE ERROR!");
        break;
    }
    refreshMask = 0;
  }
}
