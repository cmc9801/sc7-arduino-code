#include "sc7-can-libinclude.h"
#include <Metro.h>
#include <Switch.h>
#include <serLCD.h>

#include <SPI.h>
 
 #define BIT(n)                   ( 1<<(n) ) 
 #define BIT_SET(y, mask)         ( y |=  (mask) ) 
 #define BIT_CLEAR(y, mask)       ( y &= ~(mask) ) 
 #define BIT_FLIP(y, mask)        ( y ^=  (mask) )
 #define BIT_DIFFERENT(y, x, mask)( y & mask == x & mask )
 #define BIT_CHECK(y, bit, mask)  ( BIT_DIFFERENT(y, (bit)*mask, mask) )

/* Bit masks */
#define FWD_GEAR BIT(0)
#define REV_GEAR BIT(1)
#define HEADLIGHT BIT(2)
#define HAZARDLIGHT BIT(3)
#define CRUISE_CONTROL BIT(4)
#define HORN BIT(5)
#define LEFT_TURN BIT(6)
#define RIGHT_TURN BIT(7)
char young = 1;
char old;

//Steering Wheel LCD Info
const int SOC = 8;
const int V = 6;
const int GEAR = 13;
const int CC = 11;
const int LIGHT = 12;
const int RIGHT = 15;
const int LEFT = 1;

//set up pins that connect to switch terminals
   const int fgp =   9;
   const int rgp =   8;
   const int hp =    7;
   const int hzp =   6;
   const int ccp =   3;
   const int hornp = 10;
   const int ltp =   4;
   const int rtp =   5;

//set up metro timer
  //1st: switch state reading timer
  Metro switch_timer = Metro(100);
  //2nd: CAN Transmission timer
  Metro CAN_TX = Metro(1000);
  //3rd: CAN Reception timer
  Metro CAN_RX = Metro(1000);
  //4th: Notification Timer
  Metro notif_timer = Metro(2000,NOUPDATE_ON_CHECK);
  //5th: Display Timer
  Metro display_timer = Metro(500);
  //6th: Turn signal blinking timer
  Metro blinking_timer = Metro(500);

//CAN parameters
const byte	   CAN_CS 	 = A0;
const byte	   CAN_INT	 = 1;
const uint16_t CAN_BAUD_RATE = 1000;
const byte     CAN_FREQ      = 16;
const uint16_t RXM0      = MASK_NONE;
const uint16_t RXM1      = MASK_NONE;
const uint16_t RXF0      = MASK_NONE;
const uint16_t RXF1      = MASK_NONE;
const uint16_t RXF2      = MASK_NONE;
const uint16_t RXF3      = MASK_NONE;
const uint16_t RXF4      = MASK_NONE;
const uint16_t RXF5      = MASK_NONE;
      uint16_t CAN_errors;
      
CAN_IO CanControl(CAN_CS,CAN_INT,CAN_BAUD_RATE,CAN_FREQ);

Switch cruisecontrol(ccp);
Switch horn(hornp);

//boolean cruisecontroltoggle = false;

serLCD screen(Serial1);

//setup a display structure to store the shenanigans that we neeed to display on LCD
struct LCD{
  char CCdisplay;
  char geardisplay;
  String Lightsdisplay;
  int SOCdisplay;
  int Veldisplay;
  boolean LTdisplay;
  boolean RTdisplay;
  boolean turnsignal_on;
  char notification[16];
};
LCD steering_wheel;

String situation = "nothing";

void setup() {
  // Pin Modes
  pinMode(fgp, INPUT_PULLUP);
  pinMode(rgp, INPUT_PULLUP);
  pinMode(hp, INPUT_PULLUP);
  pinMode(hzp, INPUT_PULLUP);
  pinMode(ccp, INPUT_PULLUP);
  pinMode(hornp, INPUT_PULLUP);
  pinMode(ltp, INPUT_PULLUP);
  pinMode(rtp, INPUT_PULLUP);

/*SPST - left turn, right turn, horn, cruise control
SPDT - forward/neutral/reverse, headlight/no light/hazard*/

  //set Serial baud rate to 9600bps
  Serial1.begin(9600);
  Serial.begin(9600);

 //CAN setup
  CANFilterOpt filter;
  filter.setRB0(MASK_NONE,DC_DRIVE_ID,0);
  filter.setRB1(MASK_NONE,DC_SWITCHPOS_ID,0,0,0);
  CanControl.Setup(filter, &CAN_errors);
  
screen.begin();
screen.clear();
screen.setBrightness(25);

//Initialize turnsignal_on state
steering_wheel.turnsignal_on = false;

//screen.print("It works up to here");
}

inline void switchBitFromPin(byte pin, char& out, byte mask){
  switchBit(digitalRead(pin),out, mask);
}

inline void switchBit(bool b, char& out, byte mask) {
  if (b){
    BIT_SET(out,mask);
  }else{
    BIT_CLEAR(out,mask);
  }
}

/*copy over the blink function from the LCD testing code, used to blink the sides of the display for the turning signals*/

inline void blnk(int a, boolean on){
 if (on)
 {
   screen.setCursor(1,a);
   if (a == LEFT){
     screen.print("<<");
   }
   else{
     screen.print(">>");
   }
   screen.setCursor(2,a);
   if (a == LEFT){
     screen.print("<<");
   }
   else{
     screen.print(">>");
   }
 }
 else
 {
   screen.setCursor(1,a);
   screen.print("  ");
   screen.setCursor(2,a);
   screen.print("  ");
 }
}

inline void defaultdisplay(){
  screen.clear();
  screen.setCursor(1,4);
  screen.print("SOC  %");
  screen.setCursor(1,SOC);
  screen.print(steering_wheel.SOCdisplay);
  screen.setCursor(1,LIGHT);
  screen.print(steering_wheel.Lightsdisplay);
  screen.setCursor(2,4);
  screen.print("V:");
  screen.setCursor(2,V);
  screen.print(steering_wheel.Veldisplay);
  screen.setCursor(2,CC);
  screen.print(steering_wheel.CCdisplay);
  screen.setCursor(2,GEAR);
  screen.print(steering_wheel.geardisplay);
  if(steering_wheel.LTdisplay){
    blnk(LEFT,steering_wheel.turnsignal_on);
  }
  
  if(steering_wheel.RTdisplay){
    blnk(RIGHT,steering_wheel.turnsignal_on);
  }
  /*else{
    screen.setCursor(1,LEFT);
    screen.print("  ");
    screen.setCursor(2,LEFT);
    screen.print("  ");
    screen.setCursor(1,RIGHT);
    screen.print("  ");
    screen.setCursor(2,RIGHT);
    screen.print("  ");
  }*/
}

inline void displayNotification(){
  screen.clear();
  screen.selectLine(1);
  screen.print(situation);
}
  

void loop() {  
  old = young;
  
  /*if the metro timer runs out, then check the states of all the switches
    assign the values to the 'young' byte. Reset switch timer.*/
  if (switch_timer.check() == 1){
     switchBitFromPin(fgp,  young,FWD_GEAR);
     switchBitFromPin(rgp,  young,REV_GEAR);
     switchBitFromPin(hp,   young,HEADLIGHT);
     switchBitFromPin(hzp,  young,HAZARDLIGHT);
     switchBitFromPin(ltp,  young,LEFT_TURN);
     switchBitFromPin(rtp,  young,RIGHT_TURN);
    	
    cruisecontrol.poll();
    if(cruisecontrol.pushed()){
      BIT_FLIP(young,CRUISE_CONTROL);
    }    
    horn.poll();
	switchBit(horn.on(), young, HORN);
    switch_timer.reset();
  }
  
  if (old != young || display_timer.check()){
    Serial.println(byte(~young),BIN);
    
    //Switch turnsignal_on on and off at regular intervals
    steering_wheel.turnsignal_on = !steering_wheel.turnsignal_on;
    
    //Display shenanigans
  
    if(!(~young & (FWD_GEAR|REV_GEAR)) && steering_wheel.geardisplay != 'N'){
      steering_wheel.geardisplay = 'N';
      situation = String("Neutral Gear");
      notif_timer.reset();
      Serial.print("ResetNEU");
    }
    if((~young & FWD_GEAR) && steering_wheel.geardisplay != 'F'){
      steering_wheel.geardisplay = 'F';
      situation = String("Forward Gear");
      notif_timer.reset();
      Serial.print("ResetFWD");
    }
    if((~young & REV_GEAR) && steering_wheel.geardisplay != 'R'){
      steering_wheel.geardisplay = 'R';
      situation = String("Reverse Gear");
      notif_timer.reset();
      Serial.print("ResetREV");
    }
   
    if((~young & HEADLIGHT) && steering_wheel.Lightsdisplay != "H "){
      steering_wheel.Lightsdisplay = "H ";
      situation = String("Headlights");
      notif_timer.reset();
    }
    if((~young & HAZARDLIGHT) && steering_wheel.Lightsdisplay != "HZ"){
      Serial.print(steering_wheel.Lightsdisplay);
      Serial.println(".");
      steering_wheel.Lightsdisplay = "HZ";
      situation = String("Hazardlights");
      notif_timer.reset();
    } 
    if(!(~young & (HAZARDLIGHT|HEADLIGHT)) && steering_wheel.Lightsdisplay != "  "){
      steering_wheel.Lightsdisplay = "  ";
      situation = String("All lights off");
      notif_timer.reset();
    }
    
    if((~young & CRUISE_CONTROL) && steering_wheel.CCdisplay != 'C'){
      steering_wheel.CCdisplay = 'C';
      situation = String("CruiseControl on");
      notif_timer.reset();
    }  
    if((young & CRUISE_CONTROL) && steering_wheel.CCdisplay != ' '){
      steering_wheel.CCdisplay = ' ';
      situation = String("CruiseControlOff");
      notif_timer.reset();
    }
      
    if((~young & LEFT_TURN)){
      steering_wheel.LTdisplay = true;
    }
    else steering_wheel.LTdisplay = false;
    
    if((~young & RIGHT_TURN)){
      steering_wheel.RTdisplay = true;
    }
    else steering_wheel.RTdisplay = false;
    
    if (!notif_timer.check()){
      displayNotification();
    }
    else{
      defaultdisplay();
    }
  }
  
  
  
      
  /*If this byte is different from the one in the void setup() or the CAN_TX timer runs out, send CAN packetxxxxx
    and reset CAN_TX timer.*/
  //if(young != old || CAN_TX.check()){
    //Serial.print("ERRORS:");
    //Serial.println(CAN_errors,BIN);
    //Serial.println(young,BIN);
    //CanControl.Send(SW_Data(young),TXB0);
    //CAN_TX.reset();
    //old = young;
 // }

 // if (CanControl.Available()){
   /*Use available CAN packets (BMS SOC and MC Velocity) to assign values to appropriate members of the data structures
    Frame& f = CanControl.Read();
    if (f.id == MC_BUS_STATUS_ID){
      MC_BusStatus receivedMC(f);
      screen.selectLine(1);
      screen.print("MC Bus Current: ");
      screen.selectLine(2);
      screen.print(receivedMC.bus_current);
      CAN_RX.reset();
	}
    else if (f.id == MC_VELOCITY_ID){
      MC_Velocity receivedVel(f);
      screen.selectLine(1);
      screen.print("Velocity: ");
      screen.selectLine(2);
      screen.print(receivedVel.car_velocity);
      CAN_RX.reset();
    }*/
  //}
  //else if (CAN_RX.check()){
  //  screen.print("Communic. lost  with DrivCont");
  //}         
}