/*
 * DataLogging script with following features:
 * - multiple channel reading of impulses representing established volumes of water
 * - display values for each channel about: instant flow, average flow, total volume
 * - display values at established rate
 * - log values on SD at established rate
 * - set a recording date
 * - all preferences read from SD at boot time
 *
 * 7.0 update: conversion from impulse rate to time rate
 *
 * TODO:
 * - eventually add serial monitor initialization
 *
 * Many optimizations are possible, such as using more bitmask and refining
 * some processes, but it's already good enough for the resources available
 * on a Arduino Nano Every, ATMEGA328.
 *
 * Last update: 02/2022
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
//U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(U8X8_PIN_NONE);
//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

#include "RTClib.h"
RTC_DS3231 rtc;

#define LEDPIN A1
#define RECPIN 2
#define SETPIN 3
#define SDPIN 4 //Chip Select
#define ALARMPIN 5 //SQW from RTC
//Beware to change the IMPPIN order as IMPPpIN1 is used as reference in the readImpulse function
#define IMPPIN1 10
#define IMPPIN2 9
#define IMPPIN3 8
#define IMPPIN4 7
#define IMPPIN5 6

#define CHANNELS 3

#undef DEBUG
#define DEBUG 1

#ifdef DEBUG
#define PRINT(message) Serial.println(message);
#else
#define PRINT(message)
#endif

double volumeUnit[CHANNELS] = {-1,-1,-1};
#define timeUnit 1000
double volume[CHANNELS] = {0};
float flow[CHANNELS] = {0};
unsigned long lastImp[2*CHANNELS] = {0};

char volUnits[CHANNELS][4];
char timeUnitChar[CHANNELS][4];
int timeUnits[CHANNELS] = {-1,-1,-1};
char logFiles[CHANNELS][12];

int refreshRate[CHANNELS] = {-1,-1,-1};
int sdLogRate[CHANNELS] = {-1,-1,-1};
short refreshMask = 0;
short dispImpulseMask = 0;
short logMask = 0;
unsigned long refreshInterval[2] = {0};

unsigned long impulseDispInterval[CHANNELS] = {0};
// Blink time for impulse
#define IMPVIEWTIME 500


char strBuffer[40];

bool recording = false;
short state = 0;
short oldState = -1;

DateTime recDate;

bool screen = true;

bool longPress[2] = {false, false};
unsigned long pressTime[2] = {0};

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

#define SETTIME 2000 //millis for special func. Keep it lower than SETBUFFER.
// Define time interval buffer for a button to
// make a signal. This is to counteract jiggly and
// unreliable button presses.
#define RECBUFFER 1500 //millis to keep REC pressed for signal
#define SETBUFFER 100 //millis to keep SET pressed for signal

File fileBuf;

void(* resetFunc) (void) = 0;

void initAlarm(){
  // Making it so, that the alarm will trigger an interrupt
  pinMode(ALARMPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ALARMPIN), recordOnDate, FALLING);

  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  // stop oscillating signals at SQW Pin
  // otherwise setAlarm1 will fail
  rtc.writeSqwPinMode(DS3231_OFF);

  // turn off alarm 2 (in case it isn't off already)
  // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  rtc.disableAlarm(2);

  if(!rtc.setAlarm1(
          recDate,
          DS3231_A1_Date // this mode triggers the alarm when the seconds match. See Doxygen for other options
  )) {
      PRINT("Error, alarm wasn't set!")
  }else {
      PRINT("Alarm set!")
  }
}

/*
 * Diplay error on display
 * Useful for debugging without USB monitor
 */
void dispError(char * err1, char * err2){
  PRINT(F("Disp Err"))
  u8x8.clear();
  //err1.toCharArray(strBuffer,40);
  u8x8.drawString(0,1,err1);
  //err2.toCharArray(strBuffer,40);
  u8x8.drawString(0,2+1,err2);
  u8x8.drawString(0,4+1,"Premi set per OK");
  int set = 0;
  do{
    set = digitalRead(SETPIN);
  }while(set != LOW);
}

/*
 * setDefaults()
 * Dummy function useful to skip parsing from SD
 */
/*
void setDefaults(){
  for (short i=0; i<CHANNELS; i++){
    sprintf(logFiles[i], "Es%d.csv", i+1);
    volumeUnit[i] = 100;
    sprintf(volUnits[i], "m^3");
    timeUnits[i] = 1;
    refreshRate[i] = 2;
  }
}*/
/*
 * Utility function for comma support in float numbers
 * or anything else requiring substitution
 */
/*
char* replace_char(char* str, char find, char replace){
    char *current_pos = strchr(str,find);
    while (current_pos) {
        *current_pos = replace;
        current_pos = strchr(current_pos,find);
    }
    return str;
}
*/

/*
 * Main function for parsing the pref file
 * and initializing all the necessary variables
 */
void parseSettings(){
  String buf;
  short i = 0;
  bool schedRec = false;
  short j = 0;

  // read from the file until there's nothing else in it:
  while (fileBuf.available()) {
    buf = fileBuf.readStringUntil('\n');
    PRINT(buf)
    //skip commented lines
    if (buf[0] == '%'){
      continue;
    }
    buf.toCharArray(strBuffer,40);
    if(strncmp(strBuffer,"Nome", j = strlen("Nome")) ==0){

      i = atoi(&strBuffer[++j]);
      // TODO: Check why strcpy not working??
      //strcpy(logFiles[i-1], &strBuffer[8]);
      //Override in caso di errore
      sprintf(logFiles[i-1], "P%d.csv", i);

    }else if(strncmp(strBuffer,"VolumeUnit", j = strlen("VolumeUnit")) == 0){
      i = atoi(&strBuffer[++j]);

      j += 2;
      volumeUnit[i-1] = atof(&strBuffer[j++]);

      while(strBuffer[j] != ' '){
        j++;
      }
      strcpy(volUnits[i-1], &strBuffer[++j]);

    }else if(strncmp(strBuffer,"TimeUnit", j = strlen("TimeUnit")) == 0){
      i = atoi(&strBuffer[++j]);
      j += 2;
      while(strBuffer[j] != ' '){
        //PRINT(strBuffer[j])
        //PRINT(j)
        j++;
      }
      switch(strBuffer[++j]){
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
      strcpy(timeUnitChar[i-1], &strBuffer[j]);
      PRINT(timeUnits[i-1])
      PRINT(i)

    }else if(strncmp(strBuffer,"Refresh rate",j=strlen("Refresh rate")) == 0){
      i = atoi(&strBuffer[++j]);
      refreshRate[i-1] = (int) (atof(&strBuffer[16]) * 1000);
    }else if(strncmp(strBuffer,"SD log rate",j=strlen("SD log rate")) == 0){
      i = atoi(&strBuffer[++j]);
      sdLogRate[i-1] = (int) (atof(&strBuffer[15])* 1000);
    }else if(strncmp(strBuffer,"Registrazione Programmata:",strlen("Registrazione Programmata:")) == 0){
      schedRec = strncmp(&strBuffer[27],"ON",2) == 0;
      PRINT(strncmp(&strBuffer[27],"ON",2))
      PRINT(&strBuffer[26])
      PRINT(schedRec)
    }else if(strncmp(strBuffer,"Data&Ora:",9) == 0){
      //DateTime tmp(&strBuffer[10]);
      DateTime dt("2021-10-30T07:50:37");
      recDate = dt;
      PRINT(&strBuffer[10])
      strcpy(strBuffer, "DDD, DD MMM YYYY hh:mm:ss");
      PRINT(recDate.toString(strBuffer))
      PRINT(recDate > rtc.now())
    }
  }

  if (schedRec && recDate.isValid() && recDate > rtc.now())
    initAlarm();
}

/*
 * Verifier function to avoid illegal initialization
 */
void checkVars(bool fileErr){
  PRINT(F("Checking Vars"))
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
      PRINT(volumeUnit[i])
      volumeUnit[i] = 0;
      sprintf(strBuffer, "Volume %d error", i+1);
      PRINT(strBuffer)
      if (!fileErr)
        dispError(strBuffer, "Default val 0");
    }
    if (timeUnits[i] < 0){
      PRINT(timeUnits[i])
      timeUnits[i] = 1;
      sprintf(strBuffer, "TimeUnit %d error", i+1);
      PRINT(strBuffer)
      if (!fileErr)
        dispError(strBuffer, "Default val s");
    }
    if (refreshRate[i] < 0){
      PRINT(refreshRate[i])
      refreshRate[i] = 1;
      sprintf(strBuffer, "Refresh %d error", i+1);
      PRINT(strBuffer)
      if (!fileErr)
        dispError(strBuffer, "Default val 1");
    }
    PRINT(logFiles[i])
    PRINT(volumeUnit[i])
    PRINT(volUnits[i])
    PRINT(timeUnits[i])
    PRINT(refreshRate[i])
  }
}

/*
 * If not present, create
 * log files with required headers
 */
void writeHeaders(){
  for (short i = 0; i < CHANNELS; i++){
    if (!SD.exists(logFiles[i])){
      fileBuf = SD.open(logFiles[i], FILE_WRITE);
      if (!fileBuf)
        PRINT(F("Error in opening file!!"))
      else{
        PRINT(F("File opened succesfully!!"))
        // TODO: Check strange newline for both arrays
        sprintf(strBuffer, "#Unità espresse in %s e %s", volUnits[i], timeUnitChar[i]);
        fileBuf.println(logFiles[i]);
        fileBuf.println(strBuffer);
        fileBuf.println("Time, Volume, Portata ist.");
      }
      fileBuf.close();
    }
  }
}

/*
 * Special initialization for some variables
 */
void initVars(){
  for (short i = 0; i < CHANNELS; i++) {
    lastImp[i] = 0;
    lastImp[i+CHANNELS] = 0;
  }
}

void setup() {
  Serial.begin(9600);

  //Set all pins
  pinMode(RECPIN,INPUT);
  pinMode(SETPIN,INPUT);
  pinMode(IMPPIN1,INPUT);
  pinMode(IMPPIN2,INPUT);
  pinMode(IMPPIN3,INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize display Lib
  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  //u8x8.setFont(u8x8_font_px437wyse700a_2x2_f);


  //Initialize RTC module
  if (!rtc.begin()) {
    PRINT(F("Couldn't find RTC"))
    dispError("RTC Error", "Check RTC");
    Serial.flush();
    delay(2);
    resetFunc();
  }
  rtc.disable32K();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //Initialize SD Lib
  bool fileErr = false;
  if (!SD.begin(SDPIN)) {
    PRINT(F("initialization failed!"))
    dispError("SD Error", "Check SD");
    delay(2);
    resetFunc();
  }
  PRINT(F("SD initialization done."))

  initVars();

  //max filename 8 chars + .ext
  fileBuf = SD.open("pref.txt", FILE_READ);
  if (fileBuf) {
    PRINT(F("opened pref"))
    parseSettings();
  } else {
    // if the file didn't open, print an error:
    PRINT(F("error opening pref file"))
    fileErr = true;
  }
  // close the file:
  fileBuf.close();

  checkVars(fileErr);

  writeHeaders();

  PRINT(F("Setup Complete!"))
}

void loop() {
  setRecording();
  readImpulse(IMPPIN1);
  readImpulse(IMPPIN2);
  readImpulse(IMPPIN3);
  calculateFlow();
  setState();
  refreshDisplay();

  oldState = state;
}

void setRecording(){
  int rec = digitalRead(RECPIN);
  if (rec == LOW  && !PRESSED4){
    pressMask += PRESS4;
    pressTime[0] = millis();
    PRINT(F("Pressed1"))
  }else if(rec == HIGH && PRESSED4){
    pressMask -= PRESS4;
    longPress[0] = false;
  }if(PRESSED4 && rec == LOW && (millis() - pressTime[0]) > RECBUFFER){
    pressTime[0] = millis();
    longPress[0] = true;

    recording = !recording;
    record(recording);
  }
}

void record(bool activation){
  digitalWrite(LEDPIN, activation);
  digitalWrite(LED_BUILTIN, activation);
  PRINT(activation ? "Recording: ON" : "Recording: OFF")
  unsigned long instant = millis();
  for (short i = 0; i < CHANNELS; i++) {
    volume[i] = 0;
    flow[i] = 0;
    lastImp[i] = instant;
  }
}

void readImpulse(int source){
  int impulso = digitalRead(source);
  short ind = IMPPIN1 - source;
  if (impulso == LOW  && !(pressMask & (1 << ind))){
    pressMask += 1 << ind;
    dispImpulseMask |= 1 << ind;
    volume[ind] += volumeUnit[ind];
    lastImp[ind+CHANNELS] = lastImp[ind];
    lastImp[ind] = millis();
  }else if(impulso == HIGH && (pressMask & (1 << ind))){
    pressMask -= 1 << ind;
  }
}

void calculateFlow(){
  unsigned long instant = millis();
  for (short i = 0; i < CHANNELS; i++) {
    bool dispRefresh = instant - refreshInterval[0] >= refreshRate[i];
    bool logRefresfh = instant - refreshInterval[1] >= sdLogRate[i];
    if(lastImp[i] != lastImp[i+CHANNELS] || dispRefresh || logRefresfh){
      if (dispRefresh){
        refreshInterval[0] = instant;
        refreshMask += 1 << i;
      }
      if (logRefresfh){
        refreshInterval[1] = instant;
        logMask += 1 << i;
      }

      unsigned long interval = lastImp[i]-lastImp[i+CHANNELS];

      if (lastImp[i] > 0 && instant - lastImp[i] < interval) {
        flow[i] = volumeUnit[i] * timeUnits[i] / (((float)interval)/timeUnit);
      }else if(lastImp[i] > 0 && instant - lastImp[i] >= interval){
        flow[i] = volumeUnit[i] * timeUnits[i] / (((float)(instant-lastImp[i]))/timeUnit);
      }else{
        flow[i] = 0;
      }

      //PRINT(logMask & (1 << i))
      if (recording && logMask & (1 << i)){
        fileBuf = SD.open(logFiles[i], FILE_WRITE);
        if (!fileBuf)
          PRINT(F("Error in opening file!!"))
        else{
          PRINT(F("File opened succesfully!!"))
          DateTime time = rtc.now();
          sprintf(strBuffer, "YYYY-MM-DD hh:mm:ss");
          sprintf(strBuffer, time.toString(strBuffer));
          sprintf(strBuffer, "%s, %.2f, %.2f",strBuffer, volume[i], flow[i]);
          fileBuf.println(strBuffer);
        }
        fileBuf.close();
        PRINT(strBuffer)
        logMask = logMask ^ (logMask & (1 << i));
        //PRINT(logMask & (1 << i))
      }
    }
  }
}

void setState(){
  int action = digitalRead(SETPIN);
  unsigned long instant = millis();
  if (action == LOW  && !PRESSED5){
    pressMask += PRESS5;
    pressTime[1] = instant;
  }else if(action == HIGH && PRESSED5  && (instant - pressTime[1]) > SETBUFFER){
    pressMask -= PRESS5;

    //avoid changing state if pressing for special function
    if(!longPress[1])
      state = (state + 1) % 3;
    longPress[1] = false;
  }
  if(PRESSED5 && action == LOW && (instant - pressTime[1]) > SETTIME){
    screen = !screen;
    digitalWrite(LED_BUILTIN, screen);
    if (!screen)
      u8x8.clear();
    else
      refreshMask = 1+2+4+8+16+32;


    pressTime[1] = instant;
    longPress[1] = true;
  }
}

void refreshDisplay(){
  if (screen){
    //u8x8.setFont(u8x8_font_chroma48medium8_r);
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
              u8x8.clearLine(2*i+2);
              u8x8.setCursor(0,2*i+2);
              double flo = (double) (isinf(flow[i]) ? 0 : flow[i]);
              timeU = timeUnits[i] == 1 ? 's' : (timeUnits[i] == 60 ? 'm' : 'h');
              //sprintf(strBuffer, "%d: %s %s/%c", i+1, String(flo,2), volUnits[i], timeU);
              (String(i+1) + ": " + String(flo,2) + " " + volUnits[i] + "/"+ timeU).toCharArray(strBuffer, 15);
              u8x8.print(strBuffer);
            }
          }
          break;
        case 1:
          u8x8.drawString(0,0,"Volume");
          for (short i = 0; i < CHANNELS; i++) {
            if (state != oldState || refreshMask & (1<<i)){
              u8x8.clearLine(2*i+2);
              u8x8.setCursor(0,2*i+2);
              (String(i+1) + ": " + String(volume[i])+ " "+ volUnits[i]).toCharArray(strBuffer, 15);
              u8x8.print(strBuffer);
            }
          }
          break;
        case 2:
          u8x8.drawString(0,0,"Time");
          DateTime nowTime = rtc.now();
//          PRINT(nowTime.unixtime())
          sprintf(strBuffer, "DD-MM-YYYY");
          sprintf(strBuffer, nowTime.toString(strBuffer));
//          sprintf(strBuffer,"%hhu-%hhu-%hu" , nowTime.day(), nowTime.month(), nowTime.year());
          u8x8.drawString(2,3,strBuffer);
          sprintf(strBuffer, "hh:mm");
          sprintf(strBuffer, nowTime.toString(strBuffer));
          u8x8.drawString(5,5,strBuffer);
          break;
        default:
          //u8x8.setCursor(4,1);
          u8x8.drawString(4,1,"STATE ERROR!");
          break;
      }

      refreshMask = 0;
    }
    if (dispImpulseMask){
      unsigned long instant = millis();
      for (short i = 0; i < CHANNELS; i++) {
        if (dispImpulseMask & (1<<i)){
          u8x8.setCursor(i*4,7);
          u8x8.print(" ***");
          if(instant - lastImp[i] >= IMPVIEWTIME){
            dispImpulseMask ^= dispImpulseMask & (1 << i);
            u8x8.setCursor(i*4,7);
            u8x8.print("    ");
          }
        }
      }
    }
  }
}

void recordOnDate(){
  record(true);
  if(rtc.alarmFired(1)) {
    rtc.clearAlarm(1);
    PRINT("Alarm cleared")
  }
}
