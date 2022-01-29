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
U8X8_SSD1306_128X32_UNIVISION_SW_I2C u8x8(/* clock=*/ A1, /* data=*/ A0, /* reset=*/ U8X8_PIN_NONE);
//U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(U8X8_PIN_NONE);

#include "RTClib.h"
RTC_DS1307 rtc;

//Display pins
//SDA A4
//SCK A5
#define LEDPIN 4
#define RECPIN 5
#define SETPIN 6
#define IMPPIN1 7
#define IMPPIN2 8
#define IMPPIN3 9
#define SDPIN 10

//PINS 10, 11, 12 reserved for SD by Shield
//PINS A5, A4 reserved for RTC by Shield

//rotary encoder pins
#define inputCLK 2
#define inputDT 3

#define CHANNELS 3
#define VOLDIGITS 4
#define REFRESHSTATES 0

int volumeUnit[CHANNELS] = {100,100,100};
#define timeUnit 1000
int volume[CHANNELS] = {0};
float t0;
float t1;
float flow[CHANNELS] = {0};
float avgFlow[CHANNELS] = {0};
short impCount[3] = {0};

/*const char cubeM[] PROGMEM = "m^3";
const char litres[] PROGMEM = "l";
const char secs[] PROGMEM = "s";
const char mins[] PROGMEM = "m";
const char hours[] PROGMEM = "h";
const char *const volUnits[] PROGMEM = {cubeM, litres};
const char *const timeUnits[] PROGMEM = {secs, mins, hours};*/
char * volUnits[] = {"m^3","l"};
char * timeUnits[] = {"s","m","h"};
short channelUnit[CHANNELS] = {0};

#define ALFA 0.25

short refreshRate = 1;
short refreshMask = 0;

char strBuffer[40];

#define STATE (state & 0x0F)
#define SUBSTATE (state & 0xF0)
#define OLDSTATE (oldState & 0x0F)
#define OLDSUBSTATE (oldState & 0xF0)

bool recording = false;
short state = 0;
short oldState = -1;

//To save 1 byte, longPress could be saved in
//the last bit of the state variable
//#define SUBSTATE (state & 0x70) //or maybe is 0xE0, need to test
//#define LONGPRESS (state & 0x10) //or maybe is 0x80, need to test
bool longPress = false;
unsigned long pressTime = 0;

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

#define SETTIME 2000 //millis to press to enable settings

File fileBuf;

bool curStateCLK;
bool prevStateCLK;

//disp library with buffer
/*void u8g2_prepare() {
 //u8g2.setFont(u8g2_font_6x10_tf);
 u8x8.setFont(u8x8_font_chroma48medium8_r);  
}*/

void setup() {
  Serial.begin(9600);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  
  if (!SD.begin(SDPIN)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("SD initialization done."));

  fileBuf = SD.open("pref.ini", FILE_READ);
  if (fileBuf) {
  
    // read from the file until there's nothing else in it:
    while (fileBuf.available()) {
      Serial.write(fileBuf.read());
    }
    // close the file:
    fileBuf.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening pref file"));
  }

  pinMode(RECPIN,INPUT);
  pinMode(SETPIN,INPUT);
  pinMode(IMPPIN1,INPUT);
  pinMode(IMPPIN2,INPUT);
  pinMode(IMPPIN3,INPUT);
  pinMode(inputCLK,INPUT);
  pinMode(inputDT,INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  prevStateCLK = digitalRead(inputCLK);

  u8x8.begin();
}

void loop() {
  curStateCLK =  digitalRead(inputCLK);
  
  //function here just for debug purposes; remove once R.E. works;
  readRotaryEnc();
  
  setRecording();
  if (recording){
    digitalLogFlowRate(IMPPIN1);
    digitalLogFlowRate(IMPPIN2);
    digitalLogFlowRate(IMPPIN3);
  }
  setState();
  stateFunctions();
  refreshDisplay();

  
  oldState = state;
  prevStateCLK = curStateCLK;
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

    if (impCount[ind] % refreshRate == 0)
      refreshMask += 1 << ind;
    
    volume[ind] += volumeUnit[ind];
    flow[ind] = (float) volumeUnit[ind] / (t1 - t0);

    float tmp = ALFA * flow[ind];
    avgFlow[ind] = (float) ( isinf(tmp) ? (1-ALFA)*avgFlow[ind] : tmp  + (1-ALFA)*avgFlow[ind]);
    
    t0 = t1;
    
    fileBuf = SD.open("test.csv", FILE_WRITE);
    if (!fileBuf)
      Serial.println(F("Error in opening file!!"));
    else{
      Serial.println(F("File opened succesfully!!"));
      DateTime time = rtc.now();
      sprintf(strBuffer, "YYYY-MM-DD hh:mm:ss");
      sprintf(strBuffer, time.toString(strBuffer));
      sprintf(strBuffer, "%s, %d",strBuffer, volume[ind]);
      Serial.println(strBuffer);
      //fileBuf.println(strBuffer);
    }
    fileBuf.close();
    
  }else if(impulso == HIGH && (pressMask & (1 << ind))){
    Serial.println(1 << ind);
    pressMask -= 1 << ind;
  }
}

void setState(){
  int action = digitalRead(SETPIN);
  if (action == LOW  && !PRESSED5){
    pressMask += PRESS5;
    pressTime = millis();
  }else if(action == HIGH && PRESSED5){
    pressMask -= PRESS5;

    if (!longPress){
      // used switch instead of if..else to allow for flexibility in case I wanted changes
      // in respective states. If no changes are made, revert to if..else.
      switch(STATE){
        case 0:
          state = STATE + ((((SUBSTATE >> 4) + 1) % 3) << 4);
          break;
        case 1:
        case 2:
        case 3:
          if (((SUBSTATE >> 4) + 1) % (VOLDIGITS + 1) == VOLDIGITS)
            state = (STATE + 1) % 5;
          else
            state = STATE + ((((SUBSTATE >> 4) + 1) % VOLDIGITS) << 4);
          break;
        case 4:
          if (((SUBSTATE >> 4) + 1) % 5 == 4)
            state = (STATE + 1) % 5;
          else
            state = STATE + ((((SUBSTATE >> 4) + 1) % 4) << 4);
          break;
  
        default:
          state = 0 + ((((SUBSTATE >> 4) + 1) % 3) << 4);
          break;
      }
    }
    longPress = false;
  }
  if(PRESSED5 && action == LOW && (millis() - pressTime) > SETTIME){
    state = (STATE + 1) % 5;
    pressTime = millis();
    longPress = true;
  }
}

void stateFunctions(){
  if (state != oldState){
    Serial.print(F("State: "));
    Serial.println(STATE);
    Serial.print(F("Substate: "));
    Serial.println(SUBSTATE>>4);
    for (short i = 0; i < CHANNELS; i++){
      impCount[i] = 0;
    }
  }
  
  short digits[4] = {0};
  switch(STATE){
      case 1:
      case 2:
      case 3:
        digits[0] = volumeUnit[STATE - 1] / 1000;
        digits[1] = (volumeUnit[STATE - 1] % 1000) /100;
        digits[2] = (volumeUnit[STATE - 1] % 100) /10;
        digits[3] = volumeUnit[STATE - 1] % 10;
        digits[SUBSTATE>>4] = abs(digits[SUBSTATE>>4] + readRotaryEnc()) % 10;
        volumeUnit[STATE-1] = 1000*digits[0] + 100*digits[1] + 10*digits[2] + digits[3];
        if (abs(readRotaryEnc()))
          Serial.println(volumeUnit[STATE-1]);
        refreshMask += 1 << 3;
        break;
      case 4:
        digits[0] = refreshRate / 1000;
        digits[1] = (refreshRate % 1000) /100;
        digits[2] = (refreshRate % 100) /10;
        digits[3] = refreshRate % 10;
        
        if (digits[0] == 0 && digits[1] == 0 && digits[2] == 0 && SUBSTATE >> 4 == 3)
          digits[3] = abs(digits[3] - 1 + readRotaryEnc()) % 9 + 1;
        else
          digits[SUBSTATE>>4] = abs(digits[SUBSTATE>>4] + readRotaryEnc()) % 10;
        refreshRate = 1000*digits[0] + 100*digits[1] + 10*digits[2] + digits[3];
        if (abs(readRotaryEnc()))
          Serial.println(refreshRate);
        refreshMask += 1 << 4;
        break;
      default:
        break;
    }
}

void refreshDisplay(){
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  if (state != oldState || refreshMask){
    switch(STATE){
      case 0:
        if (state != oldState)
          u8x8.clear();
        switch(SUBSTATE>>4){
          case 0:
            //u8x8.clearLine(0);
            u8x8.drawString(0,0,"Portata ist.");
            for (short i = 0; i < CHANNELS; i++) {
              if (state != oldState || refreshMask & (1<<i)){
                u8x8.setCursor(0,i+1);
                (String(i+1) + ": " + String(flow[i],3) + " " + volUnits[1] + "/"+ timeUnits[0]).toCharArray(strBuffer, 15);
                u8x8.print(strBuffer);
              }              
            }
            break;            
          case 1:
            u8x8.drawString(0,0,"Media");
            for (short i = 0; i < CHANNELS; i++) {
              if (state != oldState || refreshMask & (1<<i)){
                u8x8.setCursor(0,i+1);
                (String(i+1) + ": " + String(avgFlow[i],3) + " " + volUnits[1] + "/"+ timeUnits[0]).toCharArray(strBuffer, 15);
                u8x8.print(strBuffer);
              }
            }
            break;
          case 2:
            u8x8.drawString(0,0,"Volume");
            for (short i = 0; i < CHANNELS; i++) {
              if (state != oldState || refreshMask & (1<<i)){
                u8x8.setCursor(0,i+1);
                (String(i+1) + ": " + String(volume[i])+ " "+ volUnits[1]).toCharArray(strBuffer, 15);
                u8x8.print(strBuffer);
              }
            }
            break;
          default:
            //u8x8.setCursor(4,1);
            u8x8.drawString(4,1,"STATE 0 ERROR!");
            break;
        }
        break;
      case 1:
      case 2:
      case 3:
        if (STATE != OLDSTATE)
          u8x8.clear();
        if (SUBSTATE != OLDSUBSTATE)
          u8x8.clearLine(2);
        //u8x8.setCursor(0,0);
        sprintf(strBuffer, "Unita' impulso %d", STATE);
        u8x8.drawString(0,0,strBuffer);
  
        //u8x8.setCursor(0,1);
        sprintf(strBuffer, "%04d", volumeUnit[STATE-1]);
        u8x8.drawString(0,1,strBuffer);

        
        /*char buffer[5];
        strcpy_P(buffer, (char *)pgm_read_word(&(volUnits[0])));
        strcpy_P(strBuffer, (char *)pgm_read_word(&(timeUnits[0])));*/
        u8x8.setCursor(6,1);
        sprintf(strBuffer, "%s/%s", volUnits[1 % 2],timeUnits[0 % 3]);
        Serial.println(strBuffer);
        u8x8.print(strBuffer);

        u8x8.drawGlyph((SUBSTATE>>4), 2, '-');
        //u8x8.setCursor((SUBSTATE>>4),2);
        //u8x8.print("-");       
        break;
      case 4:
        if (STATE != OLDSTATE)
          u8x8.clear();
        if (SUBSTATE != OLDSUBSTATE)
          u8x8.clearLine(2);
        
        //u8x8.setCursor(0,0);
        u8x8.drawString(0,0,"Refresh rate");
  
        //u8x8.setCursor(0,1);
        sprintf(strBuffer, "Ogni %04d imp.", refreshRate);
        u8x8.drawString(0,1,strBuffer);
  
        //u8x8.setCursor((SUBSTATE>>4)+5,2);
        //u8x8.print("-");
        u8x8.drawGlyph((SUBSTATE>>4)+5, 2, '-');   
        break;
      default:
        u8x8.setCursor(4,1);
            u8x8.print("STATE OVER ERROR!");
        break;
    }
    refreshMask = 0;
  }
}

short readRotaryEnc(){
  short res =  0;
  if (curStateCLK != prevStateCLK){   
    // If the inputDT state is different than the inputCLK state then 
    // the encoder is rotating counterclockwise
    if (digitalRead(inputDT) != curStateCLK) {
      res = -1;
    } else {
      res = 1;
    }
  }
  return res; 
}
