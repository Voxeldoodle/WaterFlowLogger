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
#define SUBSTATE (state & 0xF0)

#define CHANNELS 3
#define VOLDIGITS 4
#define REFRESHSTATES 0

short 
int volumeUnit[CHANNELS] = {100,100,100};
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

//oldstate is useful only for debugging purposes
//remove in final version
//short oldstate = -1;

bool longPress = false;
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
//String encdir ="";

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
  if (!SD.begin(SDPIN)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("SD initialization done."));

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
  
  //function here just for debug purposes; remove once R.E. works;
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
    
    volume[source - IMPPIN] += volumeUnit[source - IMPPIN];
    flow[source - IMPPIN] = (float) volumeUnit[source - IMPPIN] / (t1 - t0);

    float tmp = alfa * flow[source - IMPPIN];
    avgFlow[source - IMPPIN] = (float) ( isinf(tmp) ? (1-alfa)*avgFlow[source - IMPPIN] : tmp  + (1-alfa)*avgFlow[source - IMPPIN]);
    
    t0 = t1;

    //TODO: Save data on SD
    //(if RAM allows)
    /*
    sprintf(strBuffer, "Impulse%d", source - IMPPIN);
    impulseFile = SD.open(strBuffer, FILE_WRITE);
    DateTime time = rtc.now();
    sprintf(strBuffer, "%d", volume);
    impulseFile.println(time.timestamp(DateTime::TIMESTAMP_FULL) + ", " + strBuffer);
    impulseFile.close();*/
  }else if(impulso == HIGH && PRESSED2){
    pressMask -= PRESS2;
  }
}

void setState(){
  int action = digitalRead(SETPIN);
  if (action == LOW  && !PRESSED3){
    pressMask += PRESS3;
    pressTime = millis();
  }else if(action == HIGH && PRESSED3){
    pressMask -= PRESS3;

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
  if(PRESSED3 && action == LOW && (millis() - pressTime) > SETTIME){
    state = (STATE + 1) % 5;
    pressTime = millis();
    longPress = true;
  }
}

void stateFunctions(){
  /*
  if (state != oldstate){
    Serial.print(F("State: "));
    Serial.println(STATE);
    Serial.print(F("Substate: "));
    Serial.println(SUBSTATE>>4);
  }
  oldstate = state;
  */
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
        break;
      default:
        break;
    }
}

void refreshDisplay(){
  u8g2.firstPage();
  do {

    if (recording){
      u8g2.drawCircle(122, 24, 4);
      u8g2.drawDisc(122, 24, 2);
    }else{
      u8g2.drawCircle(122, 24, 4);
    }

     switch(STATE){
      case 0:
         switch(SUBSTATE>>4){
          case 0:
            u8g2.drawStr(84, 0, "Portata");
            for (short i = 0; i < CHANNELS; i++) {
              (String(i+1) + ": " + String(flow[i],3)).toCharArray(strBuffer, 15);
              u8g2.drawStr(0, i*10, strBuffer);
            }
            break;            
          case 1:
            u8g2.drawStr(84, 0, "Media");
            for (short i = 0; i < CHANNELS; i++) {
              (String(i+1) + ": " + String(avgFlow[i],3)).toCharArray(strBuffer, 15);
              u8g2.drawStr(0, i*10, strBuffer);
            }
            break;
          case 2:
            u8g2.drawStr(84, 0, "Volume");
            for (short i = 0; i < CHANNELS; i++) {
              (String(i+1) + ": " + String(volume[i])).toCharArray(strBuffer, 15);
              u8g2.drawStr(0, i*10, strBuffer);
            }
            break;
          default:
            u8g2.drawStr(40, 20, "STATE 0 ERROR!");
            break;
        }
        break;
      case 1:
      case 2:
      case 3:
        sprintf(strBuffer, "Unita' impulso %d", STATE);
        //("Unita' impulso "+String(STATE, DEC)).toCharArray(strBuffer, 16);
        u8g2.drawStr(20, 0, strBuffer);

        sprintf(strBuffer, "%04d", volumeUnit[STATE-1]);
        u8g2.drawStr(6, 10, strBuffer);

        u8g2.drawStr(6 +(SUBSTATE>>4)*6, 12, "_");       
        break;
      case 4:
        u8g2.drawStr(40, 0, "Refresh rate");
        
        sprintf(strBuffer, "Ogni %04d impulsi", refreshRate);
        u8g2.drawStr(0, 10, strBuffer);

        u8g2.drawStr(30 +(SUBSTATE>>4)*6, 12, "_");       
        break;
      default:
        u8g2.drawStr(40, 20, "STATE OVER ERROR!");
        break;
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
       //encdir ="CCW";
       res = -1;
       
     } else {
       // Encoder is rotating clockwise
       counter ++;
       //encdir ="CW";
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
