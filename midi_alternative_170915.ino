/*
 * Arduno_setup.cpp
 *
 * Created: 8/6/2015 8:53:02 AM
 *  Author: Vic
 */ 
//#define F_CPU 16000000
//#define ARDUINO 164
#include "Arduino.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include  <LiquidCrystalFast.h>  // use cpp instead of h 
#include  <LiquidCrystalFast.h>
#include <Wire.h>
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>




MIDI_CREATE_DEFAULT_INSTANCE();
LiquidCrystalFast lcd (2, 12,3, 10, 11, 5, 7 ); //
using namespace midi;

const byte MajorNote[]= { 0,2,4,5,7,9,11,12,14,16,17,19,21,23,24} ;  // major
const byte MinorNote[]={ 0,2,3,5,7,8,10,12,14,15,17,19,20,22,24}; // minor
const byte ChromNote[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}; //chromatic

const byte noteReference[] = {0,10,0,8,1,10,1,8,2,10,3,10,3,8,4,10,4,8,5,10,5,8,6,10,0,9,0,8,1,9,1,8,2,9,3,9,3,8,4,9,4,8,5,9,5,8,6,9 };

const char mainNote[] = "CDEFGABb#2 " ; //note string values 7 notes and flat and sharp which are double length ?
int ledReference[9]; // reference number for led on voltage 


const byte  waveSaw[] = {  10,31,47,63,79,95,111,127,0,31,47,63,79,95,111,127}; //lfo saw values lut
const byte waveTri[17] = { 0,15,31,47,63,79,95,111,127,111,95,79,63,47,32,15 }; //lfo triangle values lut
byte potValues [65] = {};  // note potValues  storage 8x8
byte potPosition=0; // potValues pointer
byte modSwitches [9] ={0,0,0,0,0,0,0,0 }; // function switch storage ,all values are 0-2  
 byte ccValues[6]={}; //cc memories storage 
byte currentValues [9] = { 1,1,1,1,1,1,1,1 };  //note switches on,stop,off  storage
byte currentValuesB [9] = { 1,1,1,1,1,1,1,1 };  // note switch  storage for lcd
byte pitchValues[9] = { 0,0,0,0,0,0,0,0 }; // remember pitch values of key pot
byte noteDuration[9] = { 0,0,0,0,0,0,0,0 }; // note length array
byte displayMem[9] = { 0,0,0,0,0,0,0,0}; // past stored values for lcd
int lcdTopA[17] ={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // top row memory
int lcdTopB[17] ={ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte memNote[9] = { 0,0,0,0,0,0,0,0 };  // current note values for lcd
int tempValue[9]={ 0,0,0,0,0,0,0,0 };
int tempValueB[9]={ 0,0,0,0,0,0,0,0 };
byte potRange=54; // range of potvalues 8/14
byte eventSchedule[9]= {1,2,3,2,1,2,3,2 };   //schedule operations 1= read controls 2= lcd operations 3= serial writes
byte eventCounter=0; // event timer

boolean controlMem=false; // enable control memory for 8 bar loop
byte controlStore [41] ={};  // control settings store 8x5 , first 8 is current
byte modSwitchesMem[5]={}; // modSwitch temp storage
boolean writeCycle=true; // switch write and read

byte lfoWave[4]={};
byte lfolastccA[4]={};

byte levelGraph1[8]={B00000,B00000,B00000,B00000,B00000,B00000,B00000,B01110, };
byte levelGraph2[8]={B00000,B00000,B00000,B00000,B00000,B00000,B11111,B00000, };
byte levelGraph3[8]={B00000,B00000,B00000,B00000,B00000,B11011,B00000,B00000, };
byte levelGraph4[8]={B00000,B00000,B00000,B00000,B10101,B00000,B00000,B00000, };
byte levelGraph5[8]={B00000,B00000,B00000,B11011,B00000,B00000,B00000,B00000, };
byte levelGraph6[8]={B00000,B00000,B11111,B00000,B00000,B00000,B00000,B00000, };
byte levelGraph7[8]={B00000,B11111,B00000,B00000,B00000,B00000,B00000,B00000, };
byte levelGraph8[8]={B01110,B00000,B00000,B00000,B00000,B00000,B00000,B00000, };
byte lfoSelect=0;
byte modWheel[4]={};
boolean displayFirst=true; // display loop firs round
boolean lfoUp=false; // lfo direction
boolean lfoPrint=false;   
boolean lfofirstRound=true;  // lfo display first round
boolean lfoEnable=true; 
boolean lfoStored=false;
boolean lfomodeToggle=false;
boolean lfoTrigger=false; // reset lfo for sync
const byte lfodisplayLUT[8]= {0,3,6,9,12,15,16}; // lfo display values position
byte lfoSettings[30]={}; // lfo settings store  4 sets , 0 is pointer
byte lfoSettingsB[17]={};
byte lfodisplayMem[7]={};
byte lfocounterVar=0;
byte lfoGain=2;
byte lfoDelay=0;
byte modLFO =0;
byte noteint=0;

byte modMulti=0;
byte modStep=0;
boolean modEnable=true;
byte Note=0;
boolean loopMode=false;
byte octKey = 0;
boolean NoteSlide=false;
byte NoteBuffer=0;
byte patternType=0;
byte keyPot=0;
byte keyPotB=5;
byte keyPotC=0;
volatile byte counterVar=0;
byte switchOne=0;
byte outNote=0;

byte barCount=0;
int barCountB=1;// dec 1-8 
byte barCountC=1;
byte barCountTotal=0;  // total note position in 32 bars
int lastbarCountB=0; // ( dec signed barcountB-2)  0-7 pointer
byte nextbarCountB=0; // next barcounter
boolean firstbarWrite=true;  // write during first 8 bars
boolean ccEnable=false; // enable reading cc value from memory
volatile int  midiTimer =0; // timing from midiclock to notes
volatile int isrCount=0; // ISR counter
volatile int i=0; // song position
int isrCountB=0;
byte lcdCounter=0;
boolean ledToggle=false;
boolean nextStep=false;  // send note only once
boolean nextStepB=false; // send midi recorded note only once
byte electPitch=64; // Electribe pitch

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

boolean noteLength=false;
volatile byte tempo=20; //delay timer
boolean switchLed =false ;
byte noteMem=0;
byte midiMemory[70]={}; //midi storage for incoming
byte midSettings[]={1,1,0,0}; // midi channel and lfo cc setting
byte midiBarcount=0; // note off var rollover on 8 
volatile int  t =0;
volatile long timExe=0;
byte timeCount=0; //time execute schedule once per note
byte noteTraffic[4]={1,48,0,0}; // remember if note was sent from midi or sequencer : switchone , outNote, midi stored value ,0
boolean noteToggle=false; // flip to enable sending next note every 12 clocks
boolean noteOffToggle=false; // flip for note off after 11 clocks
boolean midireadEnable=false; // enable reading midi port
void selectMultiplexer() {
	digitalWrite (A0, bitRead (counterVar,0));
	digitalWrite (A1, bitRead (counterVar,1));
	digitalWrite (A2, bitRead (counterVar,2));
	
} //multiplexer selector use counterVar to change value
void analogInputloop () { 
	for (counterVar=0; counterVar<8; counterVar++) {
		selectMultiplexer();digitalWrite(4,HIGH);
		nextbarCountB=barCountB;bitClear(nextbarCountB,3); // set next bar position
		tempValue[counterVar]=(analogRead(A4)+analogRead(A4)+analogRead(A4)+analogRead(A4))/4; // read pots and store to temp Never do formula with analogRead !!!!  460 - 800
		if (lfomodeToggle==true) {tempValueB[counterVar]=tempValue[counterVar];} // copy values after using lfomode 
		if (lfomodeToggle==true && counterVar==7) lfomodeToggle=false; // reset lfo toggle at end of bar
		if (counterVar<7) { lfoSettingsB[counterVar+7]=((tempValue[counterVar])-ledReference[counterVar])/2;lfoSettingsB[counterVar+7]=lfoSettingsB[counterVar+7]/(28-(lfoSettingsB[counterVar+7]/16))  ;// store values for lfo 
		 if (lfoSettingsB[counterVar+7]>9) {lfoSettingsB[counterVar+7]=9;}     ; }  // stop lfo value overflow
		
		if (tempValue[counterVar]> 860) tempValue[counterVar]=ledReference[counterVar]; // reject pot drop outs
		if (firstbarWrite==true) {potValues[counterVar+((barCountB-1)*8)]=((tempValue[counterVar])-ledReference[counterVar])/potRange;tempValueB[counterVar]=tempValue[counterVar];} // write everything during first 8 bars
			else {
		
		
		if (modSwitches[5]!=2) {
		if (((tempValue[counterVar]+16)<tempValueB[counterVar])  |  ((tempValue[counterVar]-16)>tempValueB[counterVar]) ) 
		{potValues[counterVar+((barCountB-1)*8)]=((tempValue[counterVar])-ledReference[counterVar])/potRange;potValues[counterVar+(nextbarCountB*8)]=potValues[counterVar+((barCountB-1)*8)];tempValueB[counterVar]=tempValue[counterVar];   } ;} 
			 //  check value for noise and store to pot value
			;}
		digitalWrite(4,LOW); }
	counterVar=0;} // loop for reading analog inputs

void lfoOutput() {


if (modSwitches[1]==2) {modStep=4;}    // mod rate multiplier
if (modSwitches[1]==0) {modStep=2;} 	// mod rate multiplier faster
if (modSwitches[1]==1) {modStep=1;}    // mod rate multiplier fastest remember to divide by 2
// lfoDelay=modStep+8;bitClear(lfoDelay,4); // delay pointer 4 bit
for (lfoSelect=0;lfoSelect<4;lfoSelect++) {

// modStep=modStep+(lfoSettings[1+(lfoSelect*6)]);


if (lfoEnable==true) {
	if (modSwitches[0]==2)  {modWheel[lfoSelect]=(((barCountTotal+(lfoSettings[4+(lfoSelect*6)]) )/(4/modStep)) *(4*modStep));bitClear(modWheel[lfoSelect],7);} // lfo speed and steps full range 
	modWheel[lfoSelect]=((127-modWheel[lfoSelect])/(19-(lfoSettings[2+(lfoSelect*6)]*2) )) +(lfoSettings[3+(lfoSelect*6)]*14); // gain and offset
	
	// lfoWave[lfoSelect]=(lfoWave[lfoSelect]+modStep); if (bitRead(lfoWave[0],7)==1)  {lfoUp=!lfoUp;}  bitClear(lfoWave[lfoSelect],7);}// reset steps overflow , flip lfo direction
// if (lfoTrigger==true) {lfoWave[lfoSelect]=0;} // rset lfo value to 0 when enabled

// if (modSwitches[0]==0) {modWheel[lfoSelect]=(lfoWave[lfoSelect]>>6)*(lfoSettings[2+(lfoSelect*6)]*14) +(lfoSettings[3+(lfoSelect*6)]*14); modEnable=true;} else if (modSwitches[0]==2)   {modWheel[lfoSelect]=((127-lfoWave[lfoSelect])/(10-lfoSettings[2+(lfoSelect*6)] )) +(lfoSettings[3+(lfoSelect*6)]*14) ; modEnable=true;} else 
	//  {if(lfoUp==true) {modWheel[lfoSelect]=127-lfoWave[lfoSelect] ; modEnable=true;} else if(lfoUp==false) {modWheel[lfoSelect]=lfoWave[lfoSelect] ; modEnable=true;} }   // enable lfo saw or triangle + calculate

 if (modWheel[lfoSelect]>127) modWheel[lfoSelect]=127; // overflow stop and clip
;} // process 4 lfos
 
}
}



void displayLoop () {
	
	if (lfofirstRound==false && displayFirst==true) {lcd.setCursor(0,0);lcd.print("               ");lcd.setCursor(0,1);lcd.print("               ");lcdTopB[0]++;lcdTopB[5]++;}
	for (lcdCounter=0;lcdCounter<8;lcdCounter++) {
		//if (isrCount>0) break;  //break out for midi clock
		
		if (lcdTopA[lcdCounter]!=lcdTopB[lcdCounter])  {
		lcd.setCursor ( lcdCounter*2,0) ; lcd.print (lcdTopA[lcdCounter]);}
		lcdTopB[lcdCounter]=lcdTopA[lcdCounter];                 // top row values
		if (potValues[(lcdCounter+((barCountB-1)*8))]!=99) {potPosition=(lcdCounter+((barCountB-1)*8));} else potPosition=lcdCounter; 
		
		
		if (modSwitches[2]==0)  {memNote[lcdCounter]= MajorNote [potValues[potPosition]]+keyPot ;}  // noteValues calculated
		if (modSwitches[2]==1)  {memNote[lcdCounter]= MinorNote [potValues[potPosition]]+keyPot ;}  // noteValues calculated
		if (modSwitches[2]==2)  {memNote[lcdCounter]= ChromNote [potValues[potPosition]]+keyPot ; } // noteValues calculated
		
		if  (currentValues[lcdCounter]!=currentValuesB[lcdCounter] && currentValues[lcdCounter]==1) { lcd.setCursor (lcdCounter*2,1);lcd.print("  ");} // print blanks
		
		if  (currentValues[lcdCounter]!=currentValuesB[lcdCounter] && currentValues[lcdCounter]==0) { lcd.setCursor (lcdCounter*2,1);lcd.print("X ");} // print x
		
		if  ((currentValues[lcdCounter]!=currentValuesB[lcdCounter] && currentValues[lcdCounter]==15)| (displayFirst==true)) {displayMem[lcdCounter]++;}  // change note val if changed to on
		
		if  (displayMem[lcdCounter]!=memNote[lcdCounter] && currentValues[lcdCounter]==15)  {
			
		lcd.setCursor (lcdCounter*2,1); lcd.print (mainNote[noteReference[memNote[lcdCounter]*2]]);lcd.setCursor ((lcdCounter*2)+1,1);  lcd.print (mainNote[noteReference[(memNote[lcdCounter]*2)+1]]); }
		
		displayMem[lcdCounter]=memNote[lcdCounter];  // reset note values
		currentValuesB[lcdCounter]=currentValues[lcdCounter];   // reset note switch values
	 
	}
displayFirst=false;

timeCount=3;	;}

void lfoMode () {
	if (lfofirstRound==true) {lcd.setCursor(0,0);lcd.print("LfRt Gn Of Ph Cc"); lcd.setCursor(0,1);lcd.print("                ");} 
	counterVar=0;selectMultiplexer();digitalWrite(4,HIGH);
	
	if ((lfoSettings[0]!=((analogRead(A4)-460)/96))  |   (lfofirstRound==true)  ) {lfoSettings[0]=((analogRead(A4)-460)/96);lfocounterVar=lfoSettings[0]*6;	 lcd.setCursor(0,1);lcd.print("                ");  
	for (counterVar=1;counterVar<7;counterVar++) {lcd.setCursor((lfodisplayLUT[counterVar])-1,1);lcd.print((lfoSettings[counterVar+lfocounterVar]));
		lfodisplayMem[counterVar]=lfoSettings[counterVar+lfocounterVar];} // write stored values
	lfoPrint=true;}        
	
	lcd.setCursor(0,1);lcd.print(lfoSettings[0]);lcd.print(" "); //select lfo 
	for (counterVar=1;counterVar<7;counterVar++) {selectMultiplexer();lfoSettingsB[counterVar]=(((analogRead(A4)+analogRead(A4)+analogRead(A4)+analogRead(A4))/4)-ledReference[counterVar])/2;lfoSettingsB[counterVar]=lfoSettingsB[counterVar]/(38-(lfoSettingsB[counterVar]/8))  ; if (lfoSettingsB[counterVar]>9) {lfoSettingsB[counterVar]=9;} ;} // read pots
	
	// for (counterVar=1;counterVar<5;counterVar++) {selectMultiplexer();lfoSettingsB[counterVar]=(analogRead(A4)-460)/4; lfoSettingsB[counterVar]=lfoSettingsB[counterVar]/( 23-(lfoSettingsB[counterVar]/8));  } // read pots 1-4  use formula non linear
	
	for (counterVar=1;counterVar<7;counterVar++) { if (lfoSettingsB[counterVar]!=lfoSettingsB[counterVar+7]) { lfoSettings[counterVar+lfocounterVar]=lfoSettingsB[counterVar] ;}   ; }  // compare to stored and update settings
	
	for (counterVar=1;counterVar<7;counterVar++) { if (lfoSettings[counterVar+lfocounterVar]!=lfodisplayMem[counterVar]) {lcd.setCursor((lfodisplayLUT[counterVar])-1,1);lcd.print((lfoSettings[counterVar+lfocounterVar]));
			;lfodisplayMem[counterVar]=lfoSettings[counterVar+lfocounterVar];} } // print updated values and store in displaymem
	
	lfofirstRound=false; digitalWrite(4,LOW);	
	timeCount=2; // don't change
	if (lfomodeToggle==false) {lfomodeToggle=true;}
	
	}
	
	

void clearNotes () { for (Note=0; Note<128;Note++) MIDI.sendNoteOff(Note,0,1);
{
for (Note=0;Note<54;Note++){potValues[Note+8]=99;}
Note=0;  // turn off notes on ch1 also set 8-64 in potValues to 99
barCount=0;
 barCountB=1;//clear vars
 barCountC=1;
firstbarWrite=true;
MIDI.sendControlChange(120,0,1); // all sound off on Microkorg ch1 or 123 for all notes off
MIDI.sendProgramChange(42,1);
MIDI.sendProgramChange(8,10);
MIDI.sendRealTime(Start);
	 //turn off all notes on ch1 and set program
}
}


void handleControlChange(byte channel, byte number, byte value) {ccValues[barCountB+((barCountC-1)*10)]=channel;ccValues[barCountB+((barCountC+3)*10)]=number;ccValues[barCountB+((barCountC+7)*10)]=value;MIDI.sendControlChange(number,value,channel);} // store 1 cc value every bar

void ESedit () {counterVar=6; selectMultiplexer();
	
	
	
	while (digitalRead(A3)==LOW) {midSettings[2]=15-(analogRead(A6)>>6);midSettings[3]=127-(analogRead(A7)>>3);
	
	lcd.setCursor (0,0) ; lcd.print("Ch:");
	lcd.print (midSettings[0]/10);lcd.print (midSettings[0]%10);lcd.print (" PC: ");lcd.print (midSettings[1]/100);lcd.print ((midSettings[1]/10)%10);lcd.print (midSettings[1]%10);
	
	if (midSettings[2]!=15-(analogRead(A6)>>6)) {midSettings[0]=midSettings[2];}  // detect pot change
	if (midSettings[3]!=127-(analogRead(A7)>>3)) {midSettings[1]=midSettings[3];}
	
	
	
	}
	lcd.setCursor (0,0) ;
	lcd.print F(("   Tempo ")) ;
	lcdTopB[0]=9; lcdTopB[5]=109;
	for (counterVar=0;counterVar<70;counterVar++) {midiMemory[counterVar]=0;} // reset midi memory
	clearNotes ();
	modSwitches[6]=0;
	MIDI.sendProgramChange(midSettings[1],midSettings[0]); 
	
} // ES editor or midi settings


void handleNoteOn(byte channel, byte pitch, byte velocity) {midiBarcount=(barCount/12);midiMemory[midiBarcount]=pitch ; midiMemory[midiBarcount+33]=velocity;
	 midireadEnable=false;
	 if (midiMemory[midiBarcount]>83) {midiMemory[midiBarcount]=0; } // clear midi note of higher than value
	
	//midiMemory[midiBarcount+1]=pitch;midiMemory[midiBarcount+34]=0;
	
	 } // called automatically from midi read creates automatic noteoff later
//void handleNoteOff(byte channel, byte pitch, byte velocity) {midiMemory[(barCount/3)+2]=pitch; midiMemory[(barCount/3)+34]=velocity;}

void setup() {

	
	
	
	
	
	
	//Serial.begin(9600);
	
	MIDI.setHandleControlChange(handleControlChange);
	MIDI.setHandleNoteOn(handleNoteOn);
	//MIDI.setHandleNoteOff(handleNoteOff);  // need these for callback to work
	
	MIDI.begin();  // enable all through on ch 2
	MIDI.turnThruOff(); // turn off thru
	
	
	lcd.begin(16, 2);
	
	
	// change adc prescaler
	
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library
	ADCSRA |= PS_64;    // set our own prescaler to 64
	// setup timer

	cli();//disable interrupts

	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	
	OCR1A = 19990;//  set for 1000us interrupt
	
	TCCR1B |= (1 << WGM12);    // turn on CTC mode
	
	TCCR1B |= (0 << CS12) | (1<<CS11) |(0 << CS10);  //ALWAYS set all 3 bits for correct speed !!!!!   0 1 0 is 8 divide
	
	TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

	sei();//enable interrupts

	pinMode (A0, OUTPUT);
	pinMode (A1 , OUTPUT );
	pinMode (A2, OUTPUT );  // set multiplexer driver outputs
	pinMode(A3,INPUT_PULLUP);	
	pinMode(A4,INPUT);
	pinMode(A5,INPUT_PULLUP);
	pinMode(A6,INPUT);
	pinMode(A7,INPUT);
	pinMode(4,OUTPUT); 
	lcd.begin ( 16, 2 );
	
	lcd.print ("Good evening! ");
	delay ( 2000);
	lcd.clear();
	lcd.setCursor (3,0) ;
	lcd.print ("Tempo") ;
	
	clearNotes();
	
	lcd.createChar(0, levelGraph1);
	lcd.createChar(1, levelGraph2);
	lcd.createChar(2, levelGraph3);
	lcd.createChar(3, levelGraph4);
	lcd.createChar(4, levelGraph5);
	lcd.createChar(5, levelGraph6);
	lcd.createChar(6, levelGraph7);
	lcd.createChar(7, levelGraph8);
	
	// MidiFilterMode(Full);  // Turn on through mode
	//for (counterVar=0; counterVar<8; counterVar++) {selectMultiplexer();digitalWrite(4,HIGH);delay(10);EEPROM.write(counterVar+10, (analogRead(A4)-400));}  // do once write values to eeprom
	for (counterVar=0; counterVar<8; counterVar++) {ledReference[counterVar]=(EEPROM.read(counterVar+10))+400;}  // read led zero values fro eeprom
}

//ISR Section

ISR(TIMER1_COMPA_vect){     //timer1 interrupt
	isrCount++ ;
	
}




void loop() {

	MIDI.read();
	if ((timeCount==0) | (timeCount==3))   {
		
		tempo=(analogRead(A6)>>2);  lcdTopA[5] =50000/(tempo+200);     // tempo pot    set reading all controls to a certain speed  changed for D8
		
		cli();//disable interrupts
		OCR1A = 19999+(tempo*100);//  set for 10-25ms-ish interrupt
		sei();//enable  interrupts
		
		
		if ((i>6) | (loopMode=true)) {keyPot=7-(analogRead(A7)>>7); lcdTopA[0]=keyPot;} else lcdTopA[0]=7-(analogRead(A7)>>7);//key pot changed to A7 from D9
		
		// read 8 step controls loop
		loopMode=false;  // set non looping
		for (counterVar=0; counterVar<8; counterVar++) {
			 
			//digitalWrite(A4,LOW); 
			digitalWrite(A5,LOW);  // DO THESE EARLY 
			digitalWrite(A3,LOW);
			digitalWrite(4,LOW);
			selectMultiplexer();
			
			if (digitalRead (A3)==HIGH) modSwitches[counterVar]=2; else  {digitalWrite(A3,HIGH);if (digitalRead (A3)==HIGH) modSwitches[counterVar]=0;else  modSwitches[counterVar]=1;}
			digitalWrite(A3,LOW);// read modSwitches
			
			
			if (barCountB==8 && i==7 && writeCycle==true && modSwitches[5]==1 ) writeCycle=false;  // at the end of 8 bars disable write
			if ((counterVar<4) && writeCycle==true && modSwitches[5]==1 ) {controlStore[(counterVar+1)+(barCountB*4)]=modSwitches[counterVar];}  // store first 4 switches over 8 bar into controlStore for the first 8 barrs then disable
			if (counterVar<4) modSwitchesMem[counterVar]=modSwitches[counterVar]; // store current modswitch to mem
			//if (modSwitches[5]==1) {controlMem=false; for (byte mx=1;mx<5;mx++){modSwitches[mx-1]=controlStore[mx+(barCountB*4)];}  ;} else controlMem=false; //rewrite modswitch values from memory if enabled
			//if (modSwitches[5]==2) {for (byte mx=1;mx<5;mx++){controlStore[mx+(barCountB*4)]=0;};  writeCycle=true;}   //clear values if off and enable writing
			//if (modSwitches[5]==0) {  writeCycle=true;}   //memory off reset writecycle
			
			if (digitalRead (A5)==HIGH) currentValues[counterVar]=15; else  {digitalWrite(A5,HIGH);if (digitalRead (A5)==HIGH) currentValues[counterVar]=0;else  currentValues[counterVar]=1;}  //read  note switches for 3 state
			digitalWrite(A5,LOW);
			
			 
		}
if (timeCount==0) timeCount=1; else timeCount=04;
	}
	
	if (timeCount==1) {analogInputloop(); timeCount=2;} // read potvalues
	
	counterVar=i;selectMultiplexer(); if (currentValues[counterVar]==15) digitalWrite(4,HIGH);  // led shows current step
	
	// nextStep=false; // reset step enable

	if (isrCount>(0))  {MIDI.sendRealTime(Clock);isrCountB=isrCount; barCount++; isrCount=0;
			if (midiTimer==0){noteToggle=true;lfoEnable=true;} midiTimer++;
		   if (midiTimer==5 && modSwitches[5]==1) {noteOffToggle=true;}
		   if (midiTimer==6 && modSwitches[5]==1) {noteToggle=true;}
		   if (midiTimer==11) {noteOffToggle=true;}
		   
		   if (midiTimer==12)  {i++;nextStep=true ;midiTimer=0;ledToggle=!ledToggle; digitalWrite(13, ledToggle) ;timeCount=0;midireadEnable=true;}   if (i>7)  i=0;              }        // real time clock count to tempo value then reset , barCounter and midiTimer counter

	
	
	if (currentValues[i]==0) {i=0;loopMode=true;} // if looping on reset i and enable loopMode
	
		if (barCount>95 && nextStep==true) {barCountB++;barCount=0;ccEnable=true;}   // bar count 8 bar
		if (barCountB==9) {barCountB=1;barCountC++;firstbarWrite=false; MIDI.sendRealTime(Start);}   // every 8 bars reset barCount (1-8)
		lastbarCountB=(barCountB+6); bitClear(lastbarCountB,3); // add 6  then reset after 3 bits so its 1 bar behind with value 0-7
		if (barCountC==5) {barCountC=1;} // 4x8 bar counter  send midi start and
		
		lcdTopA[7]=(barCountC*10) + barCountB;
		barCountTotal=(((barCount)/3)/4) + ((barCountB-1)*8)+ ((barCountC-1) *64) ; // total position on notes in 32 bars/ 256 notes
		// switch modifiers
		 if (modSwitches[7]==2)       { keyPot= potValues[barCountB-1];  }   // use pot values to modify keyPot
			if (modSwitches[7]==1)       { keyPot= modStep/2;  } // use modStep to control keyPot 


		//modMulti=modSwitches[1]*2;//lfo  bar length
		lfoOutput(); // lfo operations	
		if (modSwitches[3]==0) octKey=0; else if (modSwitches[3]==1) octKey=12 ; else octKey=-12;   // octave shift switch
		if (modSwitches[4]==0)  patternType=2; else if (modSwitches[4]==1) patternType=1; else patternType=4 ; //pattern type modifier 
		
		if (modSwitches[5]==1) ( noteLength=true); else {noteLength =false ; }   // noteLength switch enable
		//modWheel[0]=modWheel[0]/lfoGain;lfoDelay=lfoDelay/lfoGain; // set gains on lfos

		// note value section
		
		//  midiTimer = barCount; //shift note times by stored value
		//   i=barCount/(patternType*6);// miditimer divided 0-7
		// stop overflow
		
		 
		
		switchOne=currentValues[i]; // note switch position read
		if (midiTimer==0 &&  nextStep==true && switchOne==15) nextStep=true; else nextStep=false;  // enable next note playback
		if (potValues[(i+((barCountB-1)*8))]!=99) {potPosition=(i+((barCountB-1)*8));} else potPosition=i;  // pass only position if changed from 99 
		
		if (nextStep==true && modSwitches[2]==0)     { Note  = MajorNote [potValues [potPosition]]+keyPot ;potRange=46;}//read Note value and modify to major or minor scale with switch 3 (llok at keypot bracket)
		if (nextStep==true && modSwitches[2]==1)     { Note  = MinorNote [potValues [potPosition]]+keyPot ;potRange=46;}//read Note value and modify to major or minor scale with switch 3
		if (nextStep==true && modSwitches[2]==2)     { Note  = ChromNote [potValues [potPosition]]+keyPot ;potRange=30;}//read Note value and modify to major or minor scale with switch 3
		electPitch=66+ (Note*3);  // Electribe pitch control

		
	if (timeCount==2 && modSwitches[5]!=2) {displayLoop();lfofirstRound=true; } else if (timeCount==2 && modSwitches[5]==2) {lfoMode() ;displayFirst=true;} //lcd  printing section
		
		
		MIDI.read();
		
		if (modSwitches[6]==1)  {  ESedit() ; } //Enter and Print MIDI and cc using tempo and pitch
			midiBarcount=(barCount/12);
			
			
			if (noteToggle==true) {lcd.setCursor(1,0);lcd.print(lfoSettingsB[2]);}
			
			
			
			//while (ccEnable==true) {if (ccValues[barCountB+((barCountC-1)*10)]!=0) { MIDI.sendControlChange((ccValues[barCountB+((barCountC+3)*10)]),(ccValues[barCountB+((barCountC+7)*10)]),(ccValues[barCountB+((barCountC-1)*10)]));   }ccEnable=false;} //send stored cc
			
			while (noteToggle==true){noteTraffic[0]=switchOne;
		
			
			if (lfoEnable==true && modSwitches[5]!=2  ) {for (lfoSelect=0;lfoSelect<4;lfoSelect++) { lfolastccA[lfoSelect]=(lfoSettings[5+(lfoSelect*6)]*10)+lfoSettings[6+(lfoSelect*6)];MIDI.sendControlChange(lfolastccA[lfoSelect],modWheel[lfoSelect],1);}
				lcd.setCursor(4,0);lcd.write(byte(modWheel[0]/16));lcd.write(byte(modWheel[1]/16));lcd.write(byte(modWheel[2]/16));lcd.write(byte(modWheel[3]/16));  lfoEnable=false;
				} // send cc and print
			if (lfoEnable==true && modSwitches[5]==2) { for (lfoSelect=0;lfoSelect<4;lfoSelect++) { MIDI.sendControlChange(lfolastccA[lfoSelect],modWheel[lfoSelect],1); }    lfoEnable=false;} // use last cc value don't change
			
			// if (modEnable==true)  {MIDI.sendControlChange (midSettings[1],modWheel[0],midSettings[0]); MIDI.sendControlChange(98,33,10); MIDI.sendControlChange(99,05,10);MIDI.sendControlChange(6,(modWheel[0]),10);
			//	 MIDI.sendControlChange(98,25,10); MIDI.sendControlChange(99,05,10);MIDI.sendControlChange(6,lfoDelay,10);}  // send modWheel[0] value  if enabled as well ES filter+pan for no5	
			
			if (switchOne==15){  noteTraffic[1]=outNote=Note+48+octKey; MIDI.sendNoteOn(noteTraffic[1],120,midSettings[0]); nextStep=false;} //MIDI.sendControlChange(98,32,10);MIDI.sendControlChange(99,05,10);MIDI.sendControlChange(6,electPitch,10);
			if (switchOne==1) {noteTraffic[2]=midiMemory[midiBarcount];if (noteTraffic[2]!=0) {MIDI.sendNoteOn(noteTraffic[2],midiMemory[midiBarcount+33],midSettings[0]);nextStepB=false;} }
			
			
			noteToggle=false;
			
			}  // send midi note on section only during note is toggled on once
		
		while (noteOffToggle==true){
			if (noteTraffic[0]==15) {MIDI.sendNoteOff(noteTraffic[1],0,midSettings[0]); nextStep=false;noteOffToggle=false;} //MIDI.sendControlChange(98,32,10);MIDI.sendControlChange(99,05,10);MIDI.sendControlChange(6,electPitch,10)
				if (noteTraffic[0]==1 && (noteTraffic[2]!=0) ) {MIDI.sendNoteOff(noteTraffic[2],midiMemory[midiBarcount+33],midSettings[0]);nextStepB=false;noteOffToggle=false; }
				noteOffToggle=false;
				}  // send midi note off
		
		
	}
	
	
