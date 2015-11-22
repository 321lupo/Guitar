#include <MIDI.h>                   
#include <Wire.h>
#define MIDI_CHAN 4                                           //maybe put in a timer after the notes, so that a note cannot be send sooner then something after another note?
                                   
#define GREEN1 5                      //LEDS
#define BLUE1 3
#define RED1 4
#define GREEN2 6
#define BLUE2 10
#define RED2 9   

int LED[]={BLUE1, GREEN1, RED1, BLUE2, GREEN2, RED2};   
int ledalways=20;              
                                                         
#define FSRSHORT_N 6                                     //pressure sensors 
int fsrshortPins[] = {A6, A9, A2, A0, A1, A3};
int fsrshortReadings[FSRSHORT_N];   
bool fsrshortSend[FSRSHORT_N];
int fsrshortonTime[FSRSHORT_N]={0,0,0,0,0,0};
int fsrshortoffTime[FSRSHORT_N]={0,0,0,0,0,0};
#define FSRSHORTBOUNCE 750                               //fsr short bounce time (fsr functions like on off switch for ctlr msgs
#define FSRSHORTTHRESH 550

#define FSRLONG_N 2
int fsrlongPins[] = {A7, A8};   
int fsrlongReadings[FSRLONG_N]; 
#define FSRLONGTHRESH 850
#define FSRLONGTHRESHHIGH 400
bool fsrlongPress[FSRLONG_N];
#define POS_N 2
int posPins[] = {A11, A10};  
int posReadings[POS_N];      
int poslongPitch [POS_N];      
int posChange [POS_N]; 
int posnotePos [POS_N];



#define BUTTON1 11                                        //buttons (dont need bounce: switch button, three possible states)
#define BUTTON2 12 
int buttonLeft = 0;
int buttonRight = 0;

int16_t accel_x, accel_y, accel_z;                        //accelerometer
#define accel_module (0x53)                               // for now using only x
byte values [6];
char output [512];

int bank = 0;
int bankold = 0;
#define CTRL_START 55
#define POSSCALE_N 13
int posnote[POS_N];
int posnoteold[POS_N];
int posnotenew[POS_N];
bool posnotechangeBool[POS_N]={false, false};
int16_t scale0[] = {40,41,42,43,44,45,46,47,48,49,50,51,52};
int16_t scale1[] = {45,46,47,48,49,50,51,52,53,54,55,56,57};
int posnotevel[POS_N];                                    //note velocities
#define POS_DELAY 12                                      //INTEGRATED DELAYS
                                                              
unsigned long millisTime;                                 //millis bounce variables
unsigned long poschangeTime[POS_N]={0,0};
#define CHANGENOTE_TIME 60

void setup(void) {
  
  Serial.begin(9600);                                     
  initFsrs();                                              //all init functions below
  initAccel();
  initButtons();
  initLeds();  

  analogWrite(BLUE1, ledalways);
  analogWrite(BLUE2, ledalways);
  
  for(int i=0; i<FSRLONG_N; i++){                          
    fsrlongPress[i]=false;                                 //reset all press bools
    posnotechangeBool[i]=false;
    poschangeTime[i]=0;
  }
  for(int i=0; i<FSRSHORT_N; i++) {
    fsrshortSend[i]=false;
  }
}

void loop(void) {

  while (usbMIDI.read()) ;                                 //read and discard any incoming MIDI messages
  
  readAccel();
  readfsrShorts();
  readfsrLongs(); 
  //readPos();
  readButtons();

  Serial.println ("");

  millisTime = millis();                                   //time var running

  for(int i=0; i<FSRSHORT_N; i++){                         //send ctrl values when fsr pressed, do not send when pressed again independent from bank
    if (fsrshortReadings[i]<=FSRSHORTTHRESH && fsrshortSend[i]==false && (millisTime-fsrshortoffTime[i]>=FSRSHORTBOUNCE)) {
      fsrshortSend[i]=true;
      fsrshortonTime[i] = millisTime;
    }
    else if (fsrshortReadings[i]<=FSRSHORTTHRESH && fsrshortSend[i]==true && (millisTime-fsrshortonTime[i]>=FSRSHORTBOUNCE)) {
      fsrshortSend[i]=false;
      fsrshortoffTime[i]=millisTime; 
      analogWrite(LED[i], 0);  
      if (fsrshortSend[0]==false && fsrshortSend[1]==false && fsrshortSend[2]==false && fsrshortSend[3]==false && fsrshortSend[4]==false && fsrshortSend[5]==false) {
        analogWrite(BLUE1, ledalways);
        analogWrite(BLUE2, ledalways);                                       //if all fsrshorts are off, switch back to light blue
      }
    }
    if (fsrshortSend[i]==true){
      if (accel_x<-150) accel_x=-150;
      if (accel_x>250) accel_x=250;
      int xctrllevel = map (accel_x, -150, 250, 0, 127);  
      usbMIDI.sendControlChange(CTRL_START+i, xctrllevel, MIDI_CHAN); 
      int ledval = map (accel_x, -150, 30, 0, 255);  
      analogWrite(LED[i], ledval);                                               //turn Led on respective to fsrshort
      Serial.print("SEND CTRL ");
      Serial.println(i);
    } 
  }

  if (buttonLeft == LOW) {                                 //BANK 0 BUTTON LEFT - do nothing
    Serial.println ("LEFT");
    bankold = bank;                                        //MIDI RESET to make sure all notes from the other banks are stopped
    bank = 0;
    if (bank!=bankold) {
    resetScales();                                         //when bank switch occurs reset scales (send midi offs for all notes of the scales)
    usbMIDI.sendPitchBend(8192, MIDI_CHAN);                //also reset pitchbend back to standard
    
    }
  }
  else if (buttonLeft == HIGH && buttonRight == HIGH) {    //BANK 1 BUTTON MIDDLE - send pos notes plus pitchbend
    Serial.println ("MIDDLE");
    bankold = bank;                                        //MIDI RESET to make sure all notes from the other banks are stopped
    bank = 1;
    if (bank!=bankold) {
      resetScales();
      usbMIDI.sendPitchBend(8192, MIDI_CHAN);
    }
    for(int i=0; i<FSRLONG_N; i++) {                       //FSR send notes on pos sensors plus subsequent pitchbend
      Serial.print (posReadings[i]);
      Serial.print (" ");
      Serial.print (posnote[i]);
      Serial.print (" ");
      
      if (fsrlongReadings[i]<=FSRLONGTHRESH) { 
        if (fsrlongPress[i]==false){   
          delay (POS_DELAY);                               //delay to let pos sensor stablizie
          posReadings[i] = analogRead(posPins[i]); 
          posnotePos[i] = posReadings[i];
          choseposNote();                                  //maps posreading to a note of the respective scale, chosen according to the guitar frets (see below)
          posnotevel[i] = map (fsrlongReadings[i], FSRLONGTHRESH, 0, 70, 127); 
          if (i==0) usbMIDI.sendNoteOn(scale0[posnote[i]], posnotevel[i], MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOn(scale1[posnote[i]], posnotevel[i], MIDI_CHAN);
          analogWrite(BLUE1, 255); 
          analogWrite(BLUE2, 255); 
        }
        posReadings[i] = analogRead(posPins[i]); 
        fsrlongPress[i]=true;
        posChange[i] = (posnotePos[i]-posReadings[i]) * 16; 
        poslongPitch[i] = 8192 + posChange[i];
        poslongPitch[i] = constrain(poslongPitch[i],0,16383);
        usbMIDI.sendPitchBend(poslongPitch[i], MIDI_CHAN);

      } 
      else if (fsrlongReadings[i]>FSRLONGTHRESH && fsrlongPress[i] == true) {
        fsrlongPress[i] = false;
        delay(POS_DELAY);
        for(int x=0; x<POSSCALE_N;x++){
          if (i==0) usbMIDI.sendNoteOff(scale0[x], 127, MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOff(scale1[x], 127, MIDI_CHAN);  
          analogWrite(BLUE1, ledalways); 
          analogWrite(BLUE2, ledalways);  
          delay (POS_DELAY);   
        }  
      }
    }
  }
  
  else if (buttonRight == LOW) {                           //BANK 2
    Serial.println ("RIGHT");
    bankold = bank;                                        //MIDI RESET to make sure all notes from the other banks are stopped
    bank = 2;
    if (bank!=bankold) {
      resetScales();
      usbMIDI.sendPitchBend(8192, MIDI_CHAN);
      Serial.println ("MIDI RESET");
    }
      for(int i=0; i<FSRLONG_N; i++) {
      Serial.print (posReadings[i]);
      Serial.print (" ");
      Serial.print (posnote[i]);
      Serial.print (" ");
      if (fsrlongReadings[i]<=FSRLONGTHRESH) { 
        if (fsrlongPress[i]==false){                                                  //if fsr was not already been pressed
          delay (POS_DELAY);                                                          //let pos sensor stabilize
          posReadings[i] = analogRead(posPins[i]);                                    //read pos sensor
          choseposNote();                                                             //maps posreading to a note of the respective scale, chosen according to the guitar frets (see below)
          posnotevel[i] = map (fsrlongReadings[i], FSRLONGTHRESH, 0, 70, 127); 
          if (i==0) usbMIDI.sendNoteOn(scale0[posnote[i]], posnotevel[i], MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOn(scale1[posnote[i]], posnotevel[i], MIDI_CHAN); 
          analogWrite(BLUE1, 255); 
          analogWrite(BLUE2, 255); 
          //DELAY HERE?
          posnoteold[i]=posnote[i];                                                     //save the note as old note for later comparison
        }
        fsrlongPress[i]=true;                                                         //fsr is now being pressed
        posReadings[i] = analogRead(posPins[i]);                                      //keep reading pos sensor for sensor changes
        choseposNote();                                                               //maps posreading to a note of the respective scale, chosen according to the guitar frets (see below)
        
        if (posnoteold[i]!=posnote[i] && (millisTime-poschangeTime[i]>=CHANGENOTE_TIME)) {                                              //if a change has been read
          poschangeTime[i]=millisTime;                                              //start bouncer
          for(int x=0; x<POSSCALE_N;x++){                                             //send note off
            if (i==0) usbMIDI.sendNoteOff(scale0[x], 127, MIDI_CHAN); 
            else if (i==1) usbMIDI.sendNoteOff(scale1[x], 127, MIDI_CHAN);   
          }  
          if (i==0) usbMIDI.sendNoteOn(scale0[posnote[i]], posnotevel[i], MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOn(scale1[posnote[i]], posnotevel[i], MIDI_CHAN);
          posnoteold[i]=posnote[i]; 
        } 
      }
      
      else if (fsrlongReadings[i]>FSRLONGTHRESH && fsrlongPress[i] == true) {       //if reading go above threshold and fsr has been pressed
        fsrlongPress[i] = false;                                                    //fsr not pressed anymore                              
        for(int x=0; x<POSSCALE_N;x++){                                             //send note off
          if (i==0) usbMIDI.sendNoteOff(scale0[x], 127, MIDI_CHAN); 
          else if (i==1) usbMIDI.sendNoteOff(scale1[x], 127, MIDI_CHAN);    
        }  
        analogWrite(BLUE1, ledalways); 
        analogWrite(BLUE2, ledalways);  
      }
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

void initLeds() {
  pinMode(BLUE1, OUTPUT);
  pinMode(GREEN1, OUTPUT);
  pinMode(RED1, OUTPUT);
  pinMode(BLUE2, OUTPUT);
  pinMode(GREEN2, OUTPUT);
  pinMode(RED2, OUTPUT);
}

void clearLeds() {
  analogWrite(BLUE1, 0); 
  analogWrite(BLUE2, 0);
  analogWrite(RED1, 0);
  analogWrite(RED2, 0);
  analogWrite(GREEN1, 0);
  analogWrite(GREEN2, 0);
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

void choseposNote(){                           //determine the values of posreading according to the guitar fret pos reading, all in halftone steps
  for(int i=0; i<FSRLONG_N; i++) {
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
}
 
void resetScales(){
  for(int x=0; x<POSSCALE_N;x++){
    usbMIDI.sendNoteOff(scale0[x], 127, MIDI_CHAN); 
    Serial.print (String("Scale0") + scale0[x] + ("OFF"));
    usbMIDI.sendNoteOff(scale1[x], 127, MIDI_CHAN);
    Serial.print (String("Scale1") + scale1[x] + ("OFF"));
  }
}

 
