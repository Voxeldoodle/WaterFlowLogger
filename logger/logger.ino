/*
 * DataLogging script with following features:
 * - multiple channel reading of impulses representing established volumes of water
 * - display values for each channel about: instant flow, average flow, total volume
 * - display values at established rate
 * - log values on SD at established rate
 * - set a recording date
 * - all preferences read from SD at boot time
 * 
 * 6.1 update: added float values for volume input
 * 
 * TODO: make debug prints cleaner through #define debug
 * 
 * Many optimizations are possible, such as using more bitmask and refining
 * some processes, but it's already good enough for the resources available
 * on a Arduino Nano Every, ATMEGA328.
 * 
 * Last update: 01/2022
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
//Beware to change the IMpPIN order as IMPpIN1 is used as reference in the digitalLogFlowRate function
#define IMPPIN1 10
#define IMPPIN2 9
#define IMPPIN3 8
#define IMPPIN4 7
#define IMPPIN5 6

#define CHANNELS 3

/*
 * TODO: Clean debug messages
#define DEBUG(str) {\
  if(debug) \
    Serial.println(str);\
}

bool debug = true;
*/

double volumeUnit[CHANNELS] = {-1,-1,-1};
#define timeUnit 1000
double volume[CHANNELS] = {0};
float t0;
float t1;
float flow[CHANNELS] = {0};
float avgFlow[CHANNELS] = {0};
short impCount[3] = {0};

char volUnits[CHANNELS][4];
int timeUnits[CHANNELS] = {-1,-1,-1};
char logFiles[CHANNELS][12];

// Alpha parameter for computing average
#define ALFA 0.25

short refreshRate[CHANNELS] = {-1,-1,-1};
short sdLogRate[CHANNELS] = {-1,-1,-1};
short refreshMask = 0;
short logMask = 0;

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

#define SETTIME 2000 //millis for special func

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
      Serial.println("Error, alarm wasn't set!");
  }else {
      Serial.println("Alarm set!");  
  }
}

/*
 * Diplay error on display
 * Useful for debugging without USB monitor
 */
void dispError(char * err1, char * err2){
  Serial.println(F("Disp Err"));
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
      volumeUnit[i-1] = atof(&strBuffer[17]);
      
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
    }else if(strncmp(strBuffer,"SD log rate",11) == 0){
      i = atoi(&strBuffer[12]);
      sdLogRate[i-1] = atoi(&strBuffer[15]);
    }else if(strncmp(strBuffer,"Registrazione Programmata:",26) == 0){
      schedRec = strncmp(&strBuffer[27],"ON",2) == 0;
      Serial.println(strncmp(&strBuffer[27],"ON",2));
      Serial.println(&strBuffer[26]);
      Serial.println(schedRec);
    }else if(strncmp(strBuffer,"Data&Ora:",9) == 0){
      //DateTime tmp(&strBuffer[10]);
      DateTime dt("2021-10-30T07:50:37");
      recDate = dt;
      Serial.println(&strBuffer[10]);
      strcpy(strBuffer, "DDD, DD MMM YYYY hh:mm:ss");
      Serial.println(recDate.toString(strBuffer));
      Serial.println(recDate > rtc.now());
    }
  }

  if (schedRec && recDate.isValid() && recDate > rtc.now())
    initAlarm();
}

/*
 * Verifier function to avoid illegal initialization
 */
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

/*
 * If not present, create 
 * log files with required headers
 */
void writeHeaders(){
  for (short i = 0; i < CHANNELS; i++){
    if (!SD.exists(logFiles[i])){
      fileBuf = SD.open(logFiles[i], FILE_WRITE);
      if (!fileBuf)
        Serial.println(F("Error in opening file!!"));
      else{
        Serial.println(F("File opened succesfully!!"));
        sprintf(strBuffer, "#Unità espress in %s", volUnits[i]);
        fileBuf.println(logFiles[i]);
        fileBuf.println(strBuffer);
        fileBuf.println("Time, Volume, Portata ist., Portata media");
      }
      fileBuf.close(); 
    }
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
    Serial.println(F("Couldn't find RTC"));
    dispError("RTC Error", "Check RTC");
    Serial.flush();
    delay(2);
    resetFunc();
  }
  rtc.disable32K();

  //Initialize SD Lib
  bool fileErr = false;
  if (!SD.begin(SDPIN)) {
    Serial.println(F("initialization failed!"));
    dispError("SD Error", "Check SD");
    delay(2);
    resetFunc();
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
  digitalLogFlowRate(IMPPIN1);
  digitalLogFlowRate(IMPPIN2);
  digitalLogFlowRate(IMPPIN3);
  setState();
  refreshDisplay();

  oldState = state;
}

void setRecording(){
  int rec = digitalRead(RECPIN);
  if (rec == LOW  && !PRESSED4){
    pressMask += PRESS4;
    pressTime[0] = millis();
    Serial.println(F("Pressed1"));
  }else if(rec == HIGH && PRESSED4){
    pressMask -= PRESS4;
    longPress[0] = false;
  }if(PRESSED4 && rec == LOW && (millis() - pressTime[0]) > SETTIME){  
    pressTime[0] = millis();
    longPress[0] = true;

    recording = !recording;
    record(recording);
  }
}

void record(bool activation){
  recording = !activation;
  digitalWrite(LEDPIN, activation);
  digitalWrite(LED_BUILTIN, activation);
  Serial.println(activation ? "Recording: ON" : "Recording: OFF");
  t0 =(float) millis()/timeUnit;
  for (short i = 0; i < CHANNELS; i++) {
    volume[i] = 0;
    flow[i] = 0;
    avgFlow[i] = 0;      
  }
}

void digitalLogFlowRate(int source){
  int impulso = digitalRead(source);
  short ind = IMPPIN1 - source;
  if (impulso == LOW  && !(pressMask & (1 << ind))){
    t1 = (float) millis()/timeUnit;
    pressMask += 1 << ind;
    impCount[ind] += 1;

    if (impCount[ind] % refreshRate[ind] == 0)
      refreshMask += 1 << ind;
    if (impCount[ind] % sdLogRate[ind] == 0)
      logMask += 1 << ind;
    
    volume[ind] += volumeUnit[ind];
    flow[ind] = (float) volumeUnit[ind] / (t1 - t0);

    float tmp = ALFA * flow[ind];
    avgFlow[ind] = (float) ( isinf(tmp) ? (1-ALFA)*avgFlow[ind] : tmp  + (1-ALFA)*avgFlow[ind]);
    
    t0 = t1;
    //Serial.println(logMask & (1 << ind));
    if (recording && logMask & (1 << ind)){
      fileBuf = SD.open(logFiles[ind], FILE_WRITE);
      if (!fileBuf)
        Serial.println(F("Error in opening file!!"));
      else{
        Serial.println(F("File opened succesfully!!"));
        DateTime time = rtc.now();
        sprintf(strBuffer, "YYYY-MM-DD hh:mm:ss");
        sprintf(strBuffer, time.toString(strBuffer));
        sprintf(strBuffer, "%s, %.02f, %.02f, .02f",strBuffer, volume[ind], flow[ind], avgFlow[ind]);
        Serial.println(strBuffer);
        fileBuf.println(strBuffer);
      }
      fileBuf.close();
      //TODO: check this piece of code, should be correct
      logMask = logMask ^ (logMask & (1 << ind));
      //Serial.println(logMask & (1 << ind));
    }    
  }else if(impulso == HIGH && (pressMask & (1 << ind))){
    pressMask -= 1 << ind;
  }
}

void setState(){
  int action = digitalRead(SETPIN);
  if (action == LOW  && !PRESSED5){
    pressMask += PRESS5;
    pressTime[1] = millis();
  }else if(action == HIGH && PRESSED5){
    pressMask -= PRESS5;
    if(!longPress[1])
      state = (state + 1) % 3;
    longPress[1] = false;
  }
  if(PRESSED5 && action == LOW && (millis() - pressTime[1]) > SETTIME){
    screen = !screen;
    digitalWrite(LED_BUILTIN, screen);
    if (!screen)
      u8x8.clear();
    else
      refreshMask = 1+2+4+8+16;
    
    
    pressTime[1] = millis();
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
              u8x8.setCursor(0,2*i+2);
              double flo = (double) (isinf(flow[i]) ? 0 : flow[i] * timeUnits[i]);
              timeU = timeUnits[i] == 1 ? 's' : (timeUnits[i] == 60 ? 'm' : 'h');
              //sprintf(strBuffer, "%d: %s %s/%c", i+1, String(flo,2), volUnits[i], timeU); 
              (String(i+1) + ": " + String(flo,2) + " " + volUnits[i] + "/"+ timeU).toCharArray(strBuffer, 15);
              u8x8.print(strBuffer);
            }              
          }
          break;            
        case 1:
          u8x8.drawString(0,0,"Media");
          for (short i = 0; i < CHANNELS; i++) {
            if (state != oldState || refreshMask & (1<<i)){
              u8x8.setCursor(0,2*i+2);
              double avgFlo = (double) (isinf(avgFlow[i]) ? 0 :  avgFlow[i] * timeUnits[i]);
              timeU = timeUnits[i] == 1 ? 's' : (timeUnits[i] == 60 ? 'm' : 'h');
              (String(i+1) + ": " + String(avgFlo,2) + " " + volUnits[i] + "/"+ timeU).toCharArray(strBuffer, 15);
              u8x8.print(strBuffer);
            }
          }
          break;
        case 2:
          u8x8.drawString(0,0,"Volume");
          for (short i = 0; i < CHANNELS; i++) {
            if (state != oldState || refreshMask & (1<<i)){
              u8x8.setCursor(0,2*i+2);
              (String(i+1) + ": " + String(volume[i])+ " "+ volUnits[i]).toCharArray(strBuffer, 15);
              u8x8.print(strBuffer);
            }
          }
          break;
        /*TODO: display time as last state
        case 3:
          u8x8.drawString(0,0,"Time");
          DateTime nowTime = rtc.now();
          DateTime time = rtc.now();
          sprintf(strBuffer, time.toString("DD-MM-YYYY"));
          drawString(2,3,strBuffer);
          sprintf(strBuffer, time.toString("hh:mm"));
          drawString(5,5,strBuffer);     
          break;*/
        default:
          //u8x8.setCursor(4,1);
          u8x8.drawString(4,1,"STATE ERROR!");
          break;
      }
      refreshMask = 0;
    } 
  }
  
}

void recordOnDate(){
  record(true);
  if(rtc.alarmFired(1)) {
    rtc.clearAlarm(1);
    Serial.println("Alarm cleared");
  }
}
