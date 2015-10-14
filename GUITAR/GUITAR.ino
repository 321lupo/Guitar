#include <MIDI.h>
#include <Wire.h>
#define MIDI_CHAN 4                                       //check if choseposnote function works, otherwise place back as before
                                                          //ADD LEDS
                                                          //TRY OUT MIDDLE MODE...
#define FSRSHORT_N 6                                     //pressure sensors 
int fsrshortPins[] = {A6, A9, A2, A0, A1, A3};
int fsrshortReadings[FSRSHORT_N];   
#define FSRSHORTTHRESH 800
#define FSRLONG_N 2
int fsrlongPins[] = {A7, A8};   
int fsrlongReadings[FSRLONG_N]; 
#define FSRLONGTHRESH 900
bool fsrlongPress[FSRLONG_N];
#define POS_N 2
int posPins[] = {A11, A10};  
int posReadings[POS_N];      
int poslongPitch [POS_N];      
int posChange [POS_N]; 
int posnotePos [POS_N];


#define BUTTON1 11                                        //buttons (dont need bounce)
#define BUTTON2 12 
int buttonLeft = 0;
int buttonRight = 0;

int16_t accel_x, accel_y, accel_z;                        //accelerometer
#define accel_module (0x53)                               // for now using only x
byte values [6];
char output [512];

int bank = 0;
int bankold = 0;
#define CTRL_START 61
#define POSSCALE_N 13
int posnote[POS_N];
int posnoteold[POS_N];
int16_t scale0[] = {40,41,42,43,44,45,46,47,48,49,50,51,52};
int16_t scale1[] = {45,46,47,48,49,50,51,52,53,54,55,56,57};
int posnotevel[POS_N];                             //note velocities

unsigned long startTime[FSRLONG_N];              //millis bounce function
unsigned long millisTime;
bool fsrlongBounce[FSRLONG_N];
int bounceTime = 50;


void setup(void) {
  
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
  initFsrs();
  initAccel();
  initButtons();
  for(int i=0; i<FSRLONG_N; i++){ 
    fsrlongPress[i]=false; 
  }
  for(int i=0; i<FSRLONG_N; i++){             //reset all debounce startTimes
    startTime[i]=0;
    fsrlongBounce[i]=true;
  }
  
}

void loop(void) {

  while (usbMIDI.read()) ; // read and discard any incoming MIDI messages
  
  readAccel();
  readfsrShorts();
  readfsrLongs(); 
  //readPos();                        //not good to read pos pins all the time better only read at certain point after fsr event when pos pin is stablizied
  readButtons();

  Serial.println ("");

  millisTime = millis();                  //time var running

  if (buttonLeft == LOW) {                                           //BANK 0
    Serial.println ("LEFT");
    bankold = bank;                                                  //MIDI RESET to make sure all notes from the other banks are stopped
    bank = 0;
    if (bank!=bankold) {
    resetScales();
    }
  }
  else if (buttonLeft == HIGH && buttonRight == HIGH) {              //BANK 1 send pos notes plus pitchbend
    Serial.println ("MIDDLE");
    bankold = bank;                                                  //MIDI RESET to make sure all notes from the other banks are stopped
    bank = 1;
    if (bank!=bankold) {
      resetScales();
      Serial.println ("MIDI RESET");
    }
//NEW
    for(int i=0; i<FSRLONG_N; i++) {
      Serial.print (posReadings[i]);
      Serial.print (" ");
      Serial.print (posnote[i]);
      Serial.print (" ");
      
      if (fsrlongReadings[i]<=FSRLONGTHRESH) { 
        if (fsrlongPress[i]==false){   
          delay (35);                                 //THIS IN MILLIS
          posReadings[i] = analogRead(posPins[i]); 
          posnotePos[i] = posReadings[i];
          choseposNote();                             //maps posreading to a note of the respective scale, chosen according to the guitar frets (see below)
          posnotevel[i] = map (fsrlongReadings[i], FSRLONGTHRESH, 0, 0, 127); 
          if (i==0){
            usbMIDI.sendNoteOn(scale0[posnote[i]], posnotevel[i], MIDI_CHAN); 
          }
          else if (i==1){
            usbMIDI.sendNoteOn(scale1[posnote[i]], posnotevel[i], MIDI_CHAN); 
          }  
        }
        posReadings[i] = analogRead(posPins[i]); 
        fsrlongPress[i]=true;
        posChange[i] = (posReadings[i] - posnotePos[i]) * 16; 
        poslongPitch[i] = 8192 + posChange[i];
        poslongPitch[i] = constrain(poslongPitch[i],0,16383);
        usbMIDI.sendPitchBend(poslongPitch[i], MIDI_CHAN);

      } 
      else if (fsrlongReadings[i]>FSRLONGTHRESH && fsrlongPress[i] == true) {
        fsrlongPress[i] = false;
        delay(30);
        for(int x=0; x<POSSCALE_N;x++){
          if (i==0) usbMIDI.sendNoteOff(scale0[x], 127, MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOff(scale1[x], 127, MIDI_CHAN);   
        }  
      }
    }

//NEW
    
  }
  
  else if (buttonRight == LOW) {                                     //BANK 2
    Serial.println ("RIGHT");
    bankold = bank;                                                  //MIDI RESET to make sure all notes from the other banks are stopped
    bank = 2;
    if (bank!=bankold) {
      resetScales();
      Serial.println ("MIDI RESET");
    }
    for(int i=0; i<FSRLONG_N; i++) {
      Serial.print (posReadings[i]);
      Serial.print (" ");
      Serial.print (posnote[i]);
      Serial.print (" ");
      if (fsrlongReadings[i]<=FSRLONGTHRESH) { 
        if (fsrlongPress[i]==false){   
          delay (35);                                 //THIS IN MILLIS
          posReadings[i] = analogRead(posPins[i]); 
          choseposNote();                                                               //maps posreading to a note of the respective scale, chosen according to the guitar frets (see below)
          posnotevel[i] = map (fsrlongReadings[i], FSRLONGTHRESH, 0, 0, 127); 
          if (i==0){
            usbMIDI.sendNoteOn(scale0[posnote[i]], posnotevel[i], MIDI_CHAN); 
          }
          else if (i==1){
            usbMIDI.sendNoteOn(scale1[posnote[i]], posnotevel[i], MIDI_CHAN); 
          }  
          posnoteold[i]=posnote[i];
        }
        posReadings[i] = analogRead(posPins[i]);
        delay(20);                                          //THIS IN MILLIS 
        choseposNote();                                                                  //maps posreading to a note of the respective scale, chosen according to the guitar frets (see below)
        if (posnoteold[i] != posnote[i]) {                                              //NEEDS A BOUNCE
          if (i==0) usbMIDI.sendNoteOff(scale0[posnoteold[i]], 127, MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOff(scale1[posnoteold[i]], 127, MIDI_CHAN); 
          
          if (i==0) usbMIDI.sendNoteOn(scale0[posnote[i]], posnotevel[i], MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOn(scale1[posnote[i]], posnotevel[i], MIDI_CHAN);
          delay(20);
        }
        posnoteold[i]=posnote[i];
        fsrlongPress[i]=true;
      } 
      else if (fsrlongReadings[i]>FSRLONGTHRESH && fsrlongPress[i] == true) {
        fsrlongPress[i] = false;
        delay(30);
        for(int x=0; x<POSSCALE_N;x++){
          if (i==0) usbMIDI.sendNoteOff(scale0[x], 127, MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOff(scale1[x], 127, MIDI_CHAN);   
        }  
      }
    }
  }

  
  for(int i=0; i<FSRSHORT_N; i++){
    if (fsrshortReadings[i]<=FSRSHORTTHRESH && bank==2){
      if (accel_x<-150) accel_x=-150;
      if (accel_x>250) accel_x=250;
      int xctrllevel = map (accel_x, -150, 250, 0, 127);  
      usbMIDI.sendControlChange(CTRL_START+i, xctrllevel, MIDI_CHAN); 
      //Serial.println(String("CTRL") + (CTRL_START+i)); 
    } 
  }
}






void initFsrs(){
  pinMode(A0, INPUT_PULLUP); 
  pinMode(A1, INPUT_PULLUP); 
  pinMode(A2, INPUT_PULLUP); 
  pinMode(A3, INPUT_PULLUP); 
  pinMode(A6, INPUT_PULLUP); 
  pinMode(A7, INPUT_PULLUP); 
  pinMode(A8, INPUT_PULLUP); 
  pinMode(A9, INPUT_PULLUP); 
} 
void initAccel(){
  Wire.begin();
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(0);
  Wire.endTransmission();
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(16);
  Wire.endTransmission();
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(8);
  Wire.endTransmission();
}

void initButtons(){
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
}


void readfsrShorts(){                          //reads and prints fsr values
  for(int i=0; i<FSRSHORT_N; i++){
    fsrshortReadings[i] = analogRead(fsrshortPins[i]);
  }
  Serial.print (" fsrshorts ");
  for(int i=0;i<FSRSHORT_N; i++){
    Serial.print (fsrshortReadings[i]);
    Serial.print (" ");
  }
}

void readfsrLongs(){
  for(int i=0; i<FSRLONG_N; i++){
    fsrlongReadings[i] = analogRead(fsrlongPins[i]);
  }
  Serial.print (" fsrlongs ");
  for(int i=0;i<FSRLONG_N; i++){
    Serial.print (fsrlongReadings[i]);
    Serial.print (" ");
  }
}

void readPos(){
  for(int i=0; i<POS_N; i++){
    posReadings[i] = analogRead(posPins[i]);
  }
  Serial.print (" pos's ");
  for(int i=0; i<POS_N; i++){
    Serial.print (posReadings[i]);
    Serial.print (" ");
  }  
}

void choseposNote(){                                              //determine the values of posreading according to the guitar fret pos reading, all in halftone steps
  int i;
  if (posReadings[i]<=1023 && posReadings[i]>980) posnote[i]=0;
  if (posReadings[i]<=980 && posReadings[i]>855) posnote[i]=1;
  if (posReadings[i]<=855 && posReadings[i]>750) posnote[i]=2;
  if (posReadings[i]<=750 && posReadings[i]>640) posnote[i]=3;
  if (posReadings[i]<=640 && posReadings[i]>540) posnote[i]=4;
  if (posReadings[i]<=540 && posReadings[i]>445) posnote[i]=5;
  if (posReadings[i]<=445 && posReadings[i]>350) posnote[i]=6;
  if (posReadings[i]<=350 && posReadings[i]>275) posnote[i]=7;
  if (posReadings[i]<=275 && posReadings[i]>190) posnote[i]=8;
  if (posReadings[i]<=190 && posReadings[i]>110) posnote[i]=9;
  if (posReadings[i]<=110 && posReadings[i]>60) posnote[i]=10;
  if (posReadings[i]<=60 && posReadings[i]>20) posnote[i]=11;
  if (posReadings[i]<=20 && posReadings[i]>=0) posnote[i]=12;
}


void readAccel() {
   int16_t xyzregister = 0x32;

   Wire.beginTransmission(accel_module);
   Wire.write(xyzregister);
   Wire.endTransmission();
   Wire.beginTransmission(accel_module);
   Wire.write(xyzregister);
   Wire.endTransmission();
   Wire.beginTransmission(accel_module);
   Wire.requestFrom(accel_module, 6);
   int16_t i = 0;
   while(Wire.available()){
     values[i] = Wire.read();
     i++;
   }
   Wire.endTransmission();
   
   accel_x = (((int16_t)values[1]) << 8) | values [0];
   accel_y = (((int16_t)values[3]) << 8) | values [2];
   accel_z = (((int16_t)values[5]) << 8) | values [4];

  Serial.print (" X ");
  Serial.print(accel_x); 
  Serial.print (" Y ");
  Serial.print(accel_y);
  Serial.print (" Z ");
  Serial.print(accel_z);
}

void readButtons(){
  buttonLeft = digitalRead(BUTTON1);
  buttonRight = digitalRead(BUTTON2);
  //ButtonLeft.update();
  //ButtonRight.update();
}

void resetScales(){
  for(int x=0; x<POSSCALE_N;x++){
    usbMIDI.sendNoteOff(scale0[x], 127, MIDI_CHAN); 
    Serial.print (String("Scale0") + scale0[x] + ("OFF"));
    usbMIDI.sendNoteOff(scale1[x], 127, MIDI_CHAN);
    Serial.print (String("Scale1") + scale1[x] + ("OFF"));
  }
}

 
