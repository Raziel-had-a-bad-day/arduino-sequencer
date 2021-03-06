/*
 * Arduno_setup.cpp
 *
 * Created: 6/7/2018 8:53:02 AM
 *  Author: Vic
 */ 
//#define F_CPU 16000000
//#define ARDUINO 164
#include "Arduino.h"
// #include <EEPROM.cpp>
//#include <SoftwareSerial.h>
// #include  <LiquidCrystal.cpp>  // use cpp instead of h 
//#include <LiquidCrystal_I2C.h>
//#include <USBComposite.h>
//#include <Wire.h>
#include <LiquidCrystal.h>
#include <EEPROM.h> 
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = PC15, en = PC14, d4 = PB12, d5 = PB13, d6 = PB14, d7 = PB15;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// MIDI_CREATE_INSTANCE (HardwareSerial, Serial1, MIDI);  // set input to serial1 PA10 
//USBMIDI midi;
//LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//using namespace midi;
HardwareTimer pwmtimer(3);
HardwareTimer pwmtimer2(1);
//HardwareTimer timer(4); // basic timer for sync
volatile word n;
long x;  // calc timer note
long y;
word pwmVel;
word pwmVelB;
//const byte tempoLUT[];  // lookup table for tempo instead of calculate
const byte MajorNote[]= { 0,2,4,6,7,9,11,13,14,16,18,19,21,23,25,26} ;  // major
const byte MinorNote[]={ 0,2,3,5,7,8,10,12,14,15,17,19,20,22,24}; // minor
const byte ChromNote[]={0,2,3,5,6,8,9,11,12,14,15,17,18,20,21}; //chromatic, diminished
//const byte noteReference[] = {10,10,10,10,0,10,0,8,1,10,1,8,2,10,3,10,3,8,4,10,4,8,5,10,5,8,6,10,0,11,0,8,1,11,1,8,2,11,3,11,3,8,4,11,4,8,5,9,5,8,6,9,10,11,10,20,10,11,10,12,10,13,10,14,10,15,10,16,10,17,10,18,10,19,11,20,11,11 };// cant read last
const word timerValues[]= {34400,32469,30647,28927,27303,25771,24324,22959,21670,20454,19306,18222,17200,16234,15323,14463,13651,12885,12162,11479,10835,10227,9653,9111,8600,8117,7661
,7231,6825,6442,6081,5739,5417,5113,4826,4555,4300,4058,3830,3615,3412,3221,3040,2869,2708,2556,2413,2277} ;   // timer values C2-C6 
const word sampleNoteadd[]= {1715,1817,1925,2039,2160,2289,2425,2569,2722,2884,3055,3237,3429,3633,3849,4078,4320,4577,4849,5138,5443,5767
,6110,6473,6858,7266,7698,8156,8641,9155,9699,10276,10887,11534,12220,12947,13717,14532,15396,16312,17282,18310,19398,20552,21774,23069,24440,25894};  // 20khz add c2-c6  /1024  for correct value , 0-511 samples of whatever (can be simple +1)
long sampleAccu;   // accu for sample output or lookup
word sampleNote;  // hold sampleNoteadd value
bool playWave; 
const char menuList[] = "NOTEL1L2SWITRPT KEY#TimeNTE2       ";   // top menu 
const char mainNote[] = " CDEFGABCDEF 0123456789+>* " ; //note string values 7 notes and flat and sharp which are double length ? 11=1  20=0 21-
//const char mainNoteLow[] = "cdefgab_^: " ; // note string for off values 
const byte noteSwitch[] = { 255,0,204,51,170,85,240,15,17,252,0 };  // note switching using bitread , alternating pattern on, off, 11001100, 00110011,10101010,01010101,11110000,00001111,00010001,11101110
const word lfoGainValues[8] = {}; // gain adder values
const byte  waveSaw[17] = {  10,31,47,63,79,95,111,127,0,31,47,63,79,95,111,127}; //lfo saw values lut ,not used
  
const byte waveTri[17] = { 0,15,31,47,63,79,95,111,127,111,95,79,63,47,32,15 }; //lfo triangle values lut ,not used
byte potValues [80] = {0,0,0,0,0,0,0,0,5,5,5,5,5,5,5,5};  // note potValues  8*10 matrix
byte potValuesLFO[80] = {}; //potvalues for lofo
byte potValuesVelocity[9] = {64,64,64,64,64,64,64,64}; // velocity potvalues default should be 127
word noteVelocity=0;  // for lfo1
word noteVelocity2=0;  // lfo2
 word noteVelt=0;
 word noteVelt2=0;// lfo2
word noteVelocityB2;  // lfo2
byte notePos2;
bool lfoNext; 
word noteVelocityB;
byte noteTiming;  // set timing shift 
bool noteTimeFlag; 
byte noteTimebit[]= {0,170,85,240,15,204,51,153,102,165,90,0}   ;   // bit switch for adjustimg rythm 
word noteAccu; //adder for left over timing
word tempAccuA;  // accu for pots
volatile byte promValue; //eprom
volatile byte promValues[64]; // hold original potvalues here
byte noteLength=16; // lenght of note 16 ticks default
byte noteDelay; // initial delay for trigger 
word noteBar[257]={};  //   8 bar data , start , end ,vel,wave * 8  3*wave note length cant be longer than next start 
byte potPosition=0; // potValues pointer
byte modSwitches [9] ={0,0,0,0,0,0,0,0 }; // function switch storage ,all values are 0-2  
 byte ccValues[6]={}; //cc memories storage 
byte currentValues [10] = {  };  //note switches on,stop,off  storage 15=on 0=off 1=loop
byte currentValuesB [10] = {  };  // note switch  storage for lcd
byte pitchValues[9] = {  }; // remember pitch values of key pot
byte noteDuration[9] = {  }; // note length array
byte displayMem[9] = { }; // past stored values for lcd
byte displayMemVelocity[9] = {}; 
byte lcdBufferString; // string storage 
byte lcdBufferCounter;  // counter in lcd buffer
byte lcdBufferCounterB;  // counter in lcd buffer
char lcdBufferMem[33] = {};  // string memory for lcdbuffering
char lcdBufferMemNew[33] = {};  // string memory for lcdbuffering incoming
byte memNote[10] = { 0,0,0,0,0,0,0,0,0 };  // current note values for lcd
byte tempValue[80]={ }; // last value is menu control
byte tempValueB[80]={ };
word tempValueA; 
byte potRange=54; // range of potvalues 8/14
// byte modMatrix[65]={};  // storage for all modulation options, new
const char modTop[] = "Nt_WvRtWdTyDp"; // modulation top lcd row : Note(1-8),LFO wave type(0-3) 0 is off , Rate(1-8), Pulse width(1-8) , velocity pitch or time modulation(1-3) , depth goes +- for inversion (+5 -5) 
boolean modEnable=false;   // true when menu pot above 0
byte menuSelect;     // menu selector pot 
byte menuSelectB;  // old menu data
byte menuSelectX; // menu horizontal
byte menuSelectX2; // old menu 
boolean firstRound=true; 
boolean modMenu=false; 
byte modRate; // lfo rate 
byte modPhase; // lfo phase
byte modDelay; // lfo initialdelay
byte modPosition; // like notePosition but freewheel
byte modPositionB; // like modPosition but shifted
int modGain;// moddepth setting ,not sure yet can be -
int modPitch;// pitch modifier
int modBarPitch; // changes bar pitch , 8 bars
byte modPointer; // mod pointer
byte modOut; // mod output
bool modStep;// mod up count
bool centreLed;  // locate positions with led
byte oldNote; //store previous note
byte oldNote2;
byte eventCounter=0; // event timer
boolean writeCycle=true; // switch write and read
byte notePosition; // seq position for noteplayback 8x8 for now 0-63
byte lfoWave[4]={};
byte lfolastccA[4]={};
byte lfoPhase; 
boolean displayFirst=true; // display loop firs round
word isrDivide; // counter for isrcount since notimer for now 
volatile byte  Note=0;
volatile byte NoteB; 
boolean loopMode=false;
byte octKey = 0;
boolean NoteSlide=false;
byte NoteBuffer=0;
byte patternType=0;
byte keyPot=0; // make it real time only from now 
byte keyPotB=5;
byte keyPotC=0; // storage for analogeread
volatile byte counterVar=0;
byte counterVarB;  
byte switchOne=0;
bool outNote;  // detect note process
boolean currentValuesWrite=false; // variable for changing pot values if reset to original value before change 
boolean patternEditor=true; // enable pattern editor
byte patternEditPos=1; // bar position for pattern edit 
int valueOriginal; // stored value  needs signed 
int valueCurrent; //current setting
byte valueReturn; // final value out
byte valueChange[9]={1,1,1,1,1,1,1,1}; // enable writing 
byte barCount=0;  // counts +1 per 8 bars ie 
int barCountB=1;// dec 1-8 
byte barCountC=1;
word barCountTotal=0;  // total note position in 32 bars 95 * 8 , 16bit 
int lastbarCountB=0; // ( dec signed barcountB-2)  0-7 pointer
byte nextbarCountB=0; // next barcounter
boolean firstbarWrite=true;  // write during first 8 bars
byte firstbarLoop; 
byte firstbarLoopB;// bit shifted
boolean ccEnable=false; // enable reading cc value from memory
volatile int  midiTimer =0; // timing from midiclock to notes
 
volatile long isrTemp; 
byte isrCountC;  
volatile byte i=0; // song position
word isrPoint;
word isrPointB;  // for noteBar jump by 4 
	bool noteSetup; 
volatile long isrCount; // ISR counter tcik 32 bit
volatile word isrCountB; //  512-820 per tick 16bit
volatile word isrMask;  // tempo mask for tick  16 bit
byte lcdCounter=0;
boolean ledToggle=false;
boolean nextStep=false;  // send note only once
boolean nextStepB=false; // send midi recorded note only once
byte electPitch=64; // Electribe pitch
volatile byte tempo=20; //delay timer
byte tempoTemp; 
long timerVar ;  // multipler for timer1 lenght for freq , too big needs to be 16bit
long timerVar2; // timer 2
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
byte changeNote=2; // note on off flags
 
void analogInputloop () {       // might make menuselect a master override for everything  if chnaged then halt writing new values until screen refreshed 
  
    
  for (counterVar=0; counterVar<3; counterVar++) {
     
	 tempAccuA=0; // reset accu
	 for (i=0;i<4;i++) {
    
    switch (counterVar) {
case 2 :  tempValueA=(analogRead(PA4)); break ;      
  case 1 :  tempValueA=(analogRead(PA5)); break ;   // menuSelectx  79
  case 0 : tempValueA=analogRead(PA6); break;  // menuSelect 78 
  
  default :   break ; 
   
	}
	tempAccuA = tempAccuA+tempValueA; //read 4 times 
		}
	
		tempValueA=tempAccuA>>6; // simple 4 average
		
		switch(counterVar){
		case 0 : menuSelect=7-(tempValueA>>5); break; //0-8
		case 1 : menuSelectX=7-(tempValueA>>5);  break ; //0-7
		
		}
		counterVarB=menuSelectX+(menuSelect*8);  // select mem page 10*8  and location pointer  77 
  }  // countervar loop
   	
	
	if  ( counterVarB!=tempValue[77]) {modEnable=true;lcd.setCursor((menuSelectX*2) , 1);}   else modEnable=false;   //compared to stored value 
	tempValue[77]=counterVarB; 
	
	tempValueA=(tempValueA*12)>>8;  // divide by 21 ish , 0-10 works ok 
		tempValueA=constrain(tempValueA,0,10);   //limit values just in case
   
         tempValue[counterVarB]=tempValueA;   // read and store pots 0-200 ,writemenu to single pos , 
	
 
 
 if ((modEnable) or (firstRound))  tempValueB[counterVarB]=tempValue[counterVarB];   // needs replicate values after moving menupot to avoidwriting new values needs to move twice not good
     valueCurrent=tempValue[counterVarB]; // incoming value  10*8 blocks  
    valueOriginal=tempValueB[counterVarB]; // read original stored value    might just make  all separate regardless 
    
   
   //if ((valueCurrent>valueOriginal+1) | (valueCurrent+1<valueOriginal)   ) {                // triggered by anytime the pot is moved seems ok now
     if (valueCurrent!=valueOriginal ) {  
   tempValueB[counterVarB]=valueCurrent;   potValues[counterVarB]=10-(valueCurrent);  }    //perfect now   , change to 0-9 here
  
	firstRound=false; 
  //for (i=0;i>80;i++) {EEPROM.write(i,potValues[i]);  } 
  counterVar=0;} // loop for reading analog inputs
void displayLoop () {					// read often as possible wont affect much unless data changes random 
 byte lcdCountTempA;
 
  //if (modEnable==true)  {lcd.setCursor (0,0) ;  lcd.print (menuList[menuSelect*2]) ; lcd.setCursor (1,0) ;  lcd.print (menuList[(menuSelect*2)+1]) ;modEnable=false; }  // only print when changed  also end mod here for now
if (menuSelect) modMenu=true; else modMenu=false; 
lcdBufferMemNew [0]=menuList[menuSelect*4] ;  lcdBufferMemNew [2]=menuList[(menuSelect*4)+1];lcdBufferMemNew [4]=menuList[(menuSelect*4)+2];lcdBufferMemNew [6]=menuList[(menuSelect*4)+3]; modEnable=false; 
	  
		  for (lcdCounter=0;lcdCounter<8;lcdCounter++) {
  lcdCountTempA=lcdCounter*2;  // just a multiplier
 
        
memNote[lcdCounter] = potValues[lcdCounter+(menuSelect*8)] + (modMenu*13);  // noteValues calculated deleted keypot ,byte value
//if (potValues[lcdCounter]) lcdBufferMemNew [lcdCountTempA+1]=mainNote[14+lcdCounter]; else lcdBufferMemNew [lcdCountTempA+1]=mainNote[12]; 
	  
 //if ( ((isrCount>>7) & 7)==lcdCounter) lcdBufferMemNew [lcdCountTempA+1]=mainNote[25]; else if (potValues[lcdCounter])  lcdBufferMemNew [lcdCountTempA+1]=mainNote[14+lcdCounter]; else lcdBufferMemNew [lcdCountTempA+1]=mainNote[12]; 
	 if ( ((isrPoint>>7) & 7)==lcdCounter) lcdBufferMemNew [lcdCountTempA+1]=mainNote[14+lcdCounter];  else lcdBufferMemNew [lcdCountTempA+1]=mainNote[12]; 
lcdBufferMemNew [lcdCountTempA+17]=  mainNote[memNote[lcdCounter]];
// else lcdBufferMemNew [lcdCountTempA+16] =  mainNote[12]; // move cursor
if ( ((isrCount>>4) & 7)==lcdCounter) lcdBufferMemNew [lcdCountTempA+16]=mainNote[25];  else  lcdBufferMemNew [lcdCountTempA+16]=mainNote[12];  
if (lcdCounter==menuSelectX)  lcdBufferMemNew [lcdCountTempA+16] =  mainNote[24]; 
  }
displayFirst=false;
timeCount=3;  ;}
void displayBuffer (){        // reads all positions only swaps what changed ,lcd print here    working as it should leave alone , only 1 char per round for now
  
  //for (lcdBufferCounter=0;lcdBufferCounter<16;lcdBufferCounter++) {
    lcdBufferCounter++;  lcdBufferCounter= lcdBufferCounter & 15;  // count to 15 
    if (lcdBufferMem[lcdBufferCounter]!=lcdBufferMemNew[lcdBufferCounter]) {  // checks old data stored here   this is all string memory not values
      
      lcd.setCursor ( lcdBufferCounter,0) ;lcd.write(lcdBufferMemNew[lcdBufferCounter]) ; (lcdBufferMem[lcdBufferCounter]=lcdBufferMemNew[lcdBufferCounter]);}   // print to lcd if diff from store first line, only printing single so lcd  row 0
  //  }  // if lcdbufferMemnew changed from lcdbuffermem print , then write over old value 
  
  
//for (lcdBufferCounter=16;lcdBufferCounter<32;lcdBufferCounter++) {
lcdBufferCounterB=lcdBufferCounter+16;  // counter in lcd buffer
  if (lcdBufferMem[lcdBufferCounterB]!=lcdBufferMemNew[lcdBufferCounterB]) {
    
  lcd.setCursor ( lcdBufferCounter,1) ;lcd.write(lcdBufferMemNew[lcdBufferCounterB]) ; (lcdBufferMem[lcdBufferCounterB]=lcdBufferMemNew[lcdBufferCounterB]);}  
 
	  // print to lcd if diff from store second line   row 1 
//}
}
void timerNote  ()  {      //this runs always 
	
	
	
	//Note=0;
	
	//if (isrPoint!=(isrCount & 1023)) {   // only when counter changed 
	
	isrPoint=isrCount & 1023;  // fetch data j/c  16 * 64 10bit
	n=0;
	while (n<255) {
		if ((noteBar[n+1] == isrPoint) &&  (changeNote==2)) {  NoteB=noteBar[n];changeNote=4;noteVelocityB=noteBar[n+3];
				}		// trigger note only after note off  , search until find value
		
		n=n+4; 
				}
	
		if (NoteB==31)  Serial.print(Note); // no issues  
		n=0;
	velocityLFO();  // insert lfo
		
	while (n<255) {
		if ((noteBar[n+2] == isrPoint) && (changeNote==3))  {pwmWrite(PB1,1);changeNote=5; }// trigger note only after note off  , search
		n=n+4;
		}
	
	NoteB=NoteB & 31;	
	
		if (changeNote==3) sampling(); 	
	
	if (changeNote==5) { NoteB=0;  sampling();changeNote=2;  }   // count up once played note problem is it stops for 8 bars
if (changeNote==4)	{      // square gen
	//	Serial.print(isrPoint);Serial.print(" "); Serial.print(noteBar[isrPointB+1]);Serial.print(" "); Serial.print(noteBar[isrPointB+2]);
	//Serial.println(); 
	
	timerVar= timerValues[NoteB+24]; // calc freq
	
	pwmVel=timerVar>>1; //just 50 for now 
	if (!NoteB) pwmVel=1; // mute for note off
	
	pwmtimer.pause();
  
	pwmtimer.setPrescaleFactor(32);   // +8 per octave only use 2>>x or it sounds bad (potValues[40])     2250/ms now
	pwmtimer.setOverflow(timerVar - 1); //      1ms=9000  c4=261.63hz 1.06 multi or 1/1.06   c4#=3.81 * 0.944   or (381*944*72)/ 100  long var 
	//pwmtimer.setCompare(TIMER_CH1, pwmVel- 1);  // calc duty cycle  7bit full is 50% 
	pwmtimer.refresh();  //32369 c4   x0.944 c4#  or +1   32369*967*note  >>10   
	pwmtimer.resume();
 
	//pwmVel=(pwmVel * waveTri[((potValues[8] & 7) *(isrPoint>>5) & 15 )])>>8; 
	Serial.print(noteVelocityB);
	Serial.println();
	Serial.print(noteVelocityB2);
	Serial.println();
	pwmVel=(noteVelocityB*noteVelocityB*pwmVel)>>12; 
	pwmWrite(PB1,pwmVel) ; //send pwm  
	changeNote=3;
}
	
	
//	}
}
	
  
  void handler3 (){   // 50 us
playWave=true;
sampleAccu=sampleNote+sampleAccu;    // add samples 
				// sampleAccu=sampleAccu & 523776;  // clear overflow
if (sampleAccu>>19) sampleAccu=sampleAccu>>19; 
  if ((isrCountB & 1023) == isrMask) {isrTemp++;isrCountB=0;} // tick over inc tempo 16/beat 
isrCountB++ ;}  // 16 bit
 
 
 
 void  sampling (){
	if (NoteB) sampleNote=sampleNoteadd[NoteB+24]; else sampleNote=0;   // sample adder  
//sampleNote=sampleNoteadd[24+potValues[46]];
pwmVelB=sampleAccu>>10;  //downconvert to 8b for now
pwmVelB=(pwmVelB*noteVelocityB2)>>7; 	
	if (!NoteB) pwmVelB=511; else pwmVelB=512-pwmVelB;
	
  pwmWrite(PA8,pwmVelB) ; //send pwm 
  }
void noteCalc(){
// enter note info into noteBar 64*4 Note ,note start , end ,vel,   for now  , 16 * 64  ticks
//noteVelocity=0;
n=0;
//noteVelocity=noteVelocity & 127;  // needs to be here or just goes bananas
	while  (n<256){
	
	notePosition= n>>2;  // 
notePosition=notePosition & 63 ; // just 64 count for now
notePos2=(notePosition>>4)<<1; // count 0-8 by 2 
potPosition=notePosition & 7;   //clear 5 msb 
modPointer=potPosition; // select tables ,horizontal now , working now dont disable !!
	
	
modBarPitch= potValues[32+(notePosition>>3)];  // if (modBarPitch) (modBarPitch=modBarPitch-4) ; // add pitch per bar
	modPitch=potValues[modPointer+16]; //default off
	
	
	Note  =potValues [potPosition]; 
	Note=Note*bitRead(noteSwitch[modPitch], (notePosition>>3)) ;  // pattern note switching
	if (Note) Note=Note+modBarPitch; 
Note=constrain(Note,0,15); // stop overflow just in case
Note=MinorNote[Note]; 
	
	noteBar[n]=Note ;  // for now 1 wave only
	// if (potValues[42]  &&  potPosition)  noteTiming= (bitRead(noteTimebit[potValues[42]],potPosition) * (20+potValues[43])) +16-potValues[43];  else noteTiming=0;  // adjust timing except first note
	
	
	noteTimeFlag= bitRead(noteTimebit[potValues[40+notePos2]],potPosition);    // get bit for time mod
	if (potValues[40+notePos2])  noteTiming= (potValues[41+notePos2]*(!noteTimeFlag))+(noteTimeFlag*(16-potValues[41+notePos2]))+7   ;  else noteTiming=15;  // adjust timing except first note 8 or 24 or off with defualt timing
	
	//if (potValues[42])  noteTiming= 12;  else noteTiming=15;  // adjust timing except first note 8 or 24 or off with defualt timing
	//	if (potValues[42])  noteTiming= (potValues[43]*(!noteTimeFlag))+(noteTimeFlag*(20-potValues[43])) +10  ;  else noteTiming=15; 
	
	if (potPosition) {
	
	noteBar[n+1]= noteBar[n-2]+1 ;   // check end of last note +1 
	noteBar[n+2]= noteBar[n+1]+noteTiming; // add note length
	
	}
	
	else {
	noteBar[n+1]= (n*4);   // always the first note in the bar 
	noteBar[n+2]= noteBar[n+1]+noteTiming;  // first note untouched start but add note length 7 or 23 
			
	}
	
	 noteBar[n+3]=64; // nothing atm
	
	n=n+4;
}
	
	
}
void velocityLFO (){      // works ok 
	
	
	if (((isrCount & 15) ==10 ) && !lfoNext)   {  // reset lfo enable when at 0 
	noteVelocity=noteVelocity+(potValues[8]+1);  // count up lfo rate
	noteVelocity2=noteVelocity2+(potValues[10]+1);  // count up lfo rate						
	if (lfoPhase!=potValues[9]) {noteVelocity=noteVelocity+(potValues[9]*12);  lfoPhase=potValues[9]; }			// adjust phase when changed 
	if (lfoPhase!=potValues[11]) {noteVelocity2=noteVelocity2+(potValues[11]*12);  lfoPhase=potValues[11]; }							
							noteVelocity=noteVelocity & 255; 
						noteVelocity2=noteVelocity2 & 255; 
						
						if (noteVelocity>127)	noteVelt=255-noteVelocity ;  else noteVelt=noteVelocity;
						if (noteVelocity2>127)	noteVelt2=255-noteVelocity2 ;  else noteVelt2=noteVelocity2;		//lfo2				
												lfoNext=true;
	}
	if (((isrCount & 15) ==13 ) && lfoNext)	 lfoNext=false; 
	noteVelocityB2=noteVelt2;	// lfo2
	noteVelocityB=(noteVelt>>1)+10;		// always on
}
void setup() {
//USBComposite.setProductId(0x0031);
  //  midi.begin();
    //while (!USBComposite);
Serial.begin(9600);
 // pinMode(PA0,INPUT_PULLUP);
 // pinMode(PA1,INPUT_PULLUP);
 // pinMode(PA2,INPUT_PULLUP);  // set multiplexer driver outputs obsolete, a0-a7 pot inputs now
 // pinMode(PA3,INPUT_PULLUP);  
  pinMode(PA4,INPUT_PULLUP);
  pinMode(PA5,INPUT_PULLUP);  
  pinMode(PA6,INPUT_PULLUP);
 // pinMode(PA7,INPUT_PULLUP);
 pinMode(PB0,INPUT_PULLUP);
 //pinMode(PC15,OUTPUT); // 
 // pinMode(PB4,OUTPUT); 
  pinMode(PC13,OUTPUT);  //Led 
 pinMode (PB1,PWM); // pwm out
pinMode (PA8,PWM); // pwm out
 //lcd.begin ( 16, 2 );
 
Timer2.pause();   //  setup isr counter
Timer2.setPeriod(50); // in microseconds  20khz
Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
    Timer2.setCompare1(1);   // Interrupt 1 count after each update
   Timer2.refresh();  // seems to work
 Timer2.attachCompare1Interrupt(handler3);
	
	Timer2.resume();
 pwmtimer2.pause();
  
 // pwmtimer2.setPrescaleFactor( (potValues[41]+1)*32);
	pwmtimer2.setPrescaleFactor(4); // 18 Mhz no change for now
  pwmtimer2.setOverflow (512 - 1); //    70 khz
  //pwmtimer2.setCompare(TIMER_CH1, 1-pwmVel);  // 8bit duty cycle for writing waveform
  
  pwmtimer2.refresh();  //
  pwmtimer2.resume();
 
  lcd.begin(16,2);
pwmWrite(PB1,1) ;
   lcd.clear();
  lcd.setCursor (3,0) ; //works now ???
  lcd.print ("Gool evening! ");
  delay ( 2000);
  lcd.clear();
  lcd.setCursor (0,0) ; // lcd.print ("2 3 4 5 6 7 8") ;  // print top row for menuselector
	
	// isrCountB=19;
	isrMask=256; // def tempo 120ish
// Status = EEPROM.format();   Status = EEPROM.init();  use this to format
for (i=0;i<64;i++) { potValues[i]= EEPROM.read(i); promValues[i]=potValues[i];
} // works fine
  noteTiming=24;
  delay(1000); 
}
void loop() {    // was about 500us when all on always
bool loopCount;
if (isrTemp==isrCount)  loopCount=true;  else {loopCount=false;displayBuffer ();  isrCount=isrTemp; }   // about 12ms ish  update  isrPoint++;
if (((isrCount & 7)==1) && !outNote)  {analogInputloop();  // 100 msec
noteCalc();
displayLoop();
 
 if (promValue<64) promValue=promValue+1 ; else promValue=0;  // fetch eeprom   nogo
if ((promValues[promValue] ) !=(potValues[promValue]))  EEPROM.write(promValue,(potValues[promValue]));   //  not too happy can totally kill speed  will have to put elsewhere
promValues[promValue] =potValues[promValue];
 
 
outNote=true; }   // read pots at certain times reasoanbly regular  about 1 per beat ie 500ms  , arm note,  working ok 
if ((isrCount & 7)==2) outNote=false; // important 
if (((isrCount & 63)==5) && !loopCount)  {ledToggle=!ledToggle; digitalWrite(PC13, ledToggle) ;  
}  // 1 sec  solid
 //problem
timerNote();
 

  }// 
