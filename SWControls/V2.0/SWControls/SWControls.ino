#include "sc7-can-libinclude.h"
#include <Metro.h>
#include <Switch.h>
#include <serLCD.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <math.h>

void setup();
void switchBitFromPin();
void switchBit();
void blnk();
void defaultDisplay();
void displayNotification();
void loop();
void initializePins();
void checkProgrammingMode();

//--------------------------------MACROS---------------------------------------//

// bit manipulations
#define BIT(n)              ( 1<<(n) ) 
#define BIT_SET(y, mask)    ( y |=  (mask) )                       
#define BIT_CLEAR(y, mask)  ( y &= ~(mask) )                      
#define BIT_FLIP(y, mask)   ( y ^=  (mask) )

// bit masks (byte 1)
#define FWD_GEAR BIT(0)
#define REV_GEAR BIT(1)
#define HEADLIGHT BIT(2)
#define HAZARDLIGHT BIT(3)
#define HORN BIT(5)
#define LEFT_TURN BIT(6)
#define RIGHT_TURN BIT(7)

// bit masks (byte 2)
#define CRUISE_ON BIT(0)
#define CRUISE_OFF BIT(1)

//-------------------------------CONSTANTS--------------------------------------//

// misc constants
//#define LOOPBACK
#define DEBUG
#define MPS_TO_MPH 2.2369f  // conversion factor for display

// LCD screen constants
//const int SOC = 8;    // state of charge (from CAN)
const int V = 6;        // velocity (from CAN)
const int GEAR = 13;    // forward/reverse/neutral
const int LT = 8;       // lap timer
const int LIGHT = 11;   // headlights/hazardlights/no lights
const int RIGHT = 15;   // right turn signals
const int LEFT = 1;     // left turn signals
const int TELM = 0;     // telemetry indicator

// pin constants
const int fgp =   6;  // forward gear
const int rgp =   7;  // reverse gear
const int hp =    9;  // headlights
const int hzp =   8;  // hazardlights
const int lapp =  4;  // lap timer reset
const int hornp = 5;  // horn
const int ltp =   3;  // left turn
const int rtp =   A2; // right turn
const int conp =  0;  // cruise on
const int coffp = 0;  // cruise off
const do_not_run_set_pins;

// CAN constants
const byte CAN_CS = 10;
const byte CAN_INT = 2; // Interrupt #1
const uint16_t CAN_BAUD_RATE = 1000;
const byte CAN_FREQ = 16;

//---------------------------TYPE DEFINITIONS----------------------------------//

// display structure to store the shenanigans that we neeed to display on LCD
struct LCD{
  char lapdisplay[4];    
  int lapmindisplay;      // displays the minutes in the lap
  int lapsecdisplay;      // displays the seconds in the lap
  char geardisplay;       // 'F' = forward, 'R' = reverse, 'N' = neutral
  char ccdisplay;         // 'C' = cruise on, ' ' = cruise off
  char telemetrydisplay;  // 'T' = telemetry on, ' ' = telemetry off
  String lightsdisplay;   // "H" = headlights, "HZ" = hazardlights, " " = no lights
  float Veldisplay;       // velocity (from CAN)
  bool LTdisplay;         // left turn?
  bool RTdisplay;         // right turn?
  bool turnsignal_on;     // whether turn signal is on/off
  String notification;    // notification string
};

//-----------------------------GLOBAL DATA-------------------------------------//

// timers
Metro switch_timer = Metro(100); // switch read wait
Metro CAN_TX = Metro(1000); // can tx wait (if switch states unchanged)
Metro CAN_RX = Metro(1000); // can read wait
Metro notif_timer = Metro(500); // notification display duration
Metro display_timer = Metro(500); // display refresh wait (if no change)
Metro blinking_timer = Metro(500); // turn signal blinking timer
Metro debug_timer = Metro(200); // wait for writing debug data over serial
Metro telmetry_timer = Metro(2500); // wait for telemetry heartbeat

// bytes for current state
char byte0 = 0xFF; // current switch states
char byte1 = 0xFF; // current switch states cont'd
char byte0_prev;
char byte1_prev;

// CAN objects
uint16_t errors;
CAN_IO CanControl(CAN_CS, CAN_INT, CAN_BAUD_RATE, CAN_FREQ); //Try initializing without interrupts for now

// switch objects (for debouncing, based on the included Switch library)
Switch laptimerreset(lapp);
Switch horn(hornp);

// serLCD object (display, based on the NUserLCD library)
serLCD_buffered screen(Serial1);
LCD steering_wheel; // info struct

// debug data
unsigned long loopStartTime = 0;
unsigned long loopSumTime = 0;
unsigned long loopCount = 0;
unsigned long previousmillis = 0;

//------------------------------FUNCTIONS------------------------------------//

void setup() {
  // pin Modes
  pinMode(fgp, INPUT_PULLUP);
  pinMode(rgp, INPUT_PULLUP);
  pinMode(hp, INPUT_PULLUP);
  pinMode(hzp, INPUT_PULLUP);
  pinMode(lapp, INPUT_PULLUP);
  pinMode(hornp, INPUT_PULLUP);
  pinMode(ltp, INPUT_PULLUP);
  pinMode(rtp, INPUT_PULLUP);
  pinMode(conp, INPUT_PULLUP);
  pinMode(coffp, INPUT_PULLUP);

  // set Serial and screen baud rate to 9600bps
  Serial.begin(9600);
  screen.begin();
  delay(500); // Allow MCP2515 to run for 128 cycles and LCD to boot

  // hazards must be set to on to allow programming
  checkProgrammingMode();

  // initialize the pin states
  initializePins();

  // CAN setup
  CanControl.filters.setRB0(MASK_Sxxx,BMS_SOC_ID,0); // read SOC packets
  CanControl.filters.setRB1(MASK_Sxxx,MC_VELOCITY_ID,0,0,0); // read velocity packets
  CanControl.Setup(RX0IE|RX1IE);

#ifdef LOOPBACK 
  Serial.print("Set Loopback"); 
  CanControl.controller.Mode(MODE_LOOPBACK); 
#endif

  // enable WDT
  wdt_enable(WDTO_4S);

  // init info struct
  steering_wheel.turnsignal_on = false;
  steering_wheel.lapmindisplay = 0;
  steering_wheel.lapsecdisplay = 0;
  steering_wheel.geardisplay = ' ';
  steering_wheel.telemetrydisplay = ' ';
  steering_wheel.lightsdisplay = "  ";
  steering_wheel.Veldisplay = 0.0;
  steering_wheel.LTdisplay = false;
  steering_wheel.RTdisplay = false;
  steering_wheel.notification = "";
  
#ifdef DEBUG
  Serial.print("CANINTE: " );
  Serial.println(CanControl.controller.Read(CANINTE), BIN);
#endif
}

/*
 * This function runs at startup and checks whether the headlights/hazards switch is set to hazards.
 * If it is, the board is in "Programming Mode". For some reason, the pro micro won't program corectly
 * while running in the main loop. It is necessary to put the micro into this state before programming.
 */
void checkProgrammingMode() {    
  while (digitalRead(hzp) == LOW) 
  {
    // do nothing if hazards is on, allowing programming to happen.
    // this delay must go before the screen printing, for some random reason.
    // also, do not call screen.clear in here.
    screen.home();
    screen.print("Turn off Hazards to Exit PrgMd  ");
    screen.update();
    delay(500); 
  }
}

inline void initializePins() {
  steering_wheel.ccdisplay = ' ';
  
  if(digitalRead(fgp) == LOW) {
    steering_wheel.geardisplay = 'F';
  }
  else if(digitalRead(rgp) == LOW) {
    steering_wheel.geardisplay = 'R';
  }
  else {
    steering_wheel.geardisplay = 'N';
  }
  
  if(digitalRead(hp) == LOW) {
    steering_wheel.lightsdisplay = "H ";
  }
  else if(digitalRead(hzp) == LOW) {
    steering_wheel.lightsdisplay = "HZ";
  }
  else {
    steering_wheel.lightsdisplay = "  ";
  }
}

/*
 * Main loop.
 */
void loop() {  
  #ifdef DEBUG
    loopStartTime = micros();
  #endif
  
  wdt_reset();
  byte0_prev = byte0;
  byte1_prev = byte1;
  
  // if the metro timer runs out, update young byte
  if (switch_timer.check()) {
    switchBitFromPin(fgp, byte0, FWD_GEAR);
    switchBitFromPin(rgp, byte0, REV_GEAR);
    switchBitFromPin(hp, byte0, HEADLIGHT);
    switchBitFromPin(hzp, byte0, HAZARDLIGHT);
    switchBitFromPin(ltp, byte0, LEFT_TURN);
    switchBitFromPin(rtp, byte0, RIGHT_TURN);
    switchBitFromPin(conp, byte1, CRUISE_ON);
    switchBitFromPin(coffp, byte1, CRUISE_OFF);
    
    // poll the lap timer and horn and change value of bit accordingly
    laptimerreset.poll();
    if(laptimerreset.pushed()){
      previousmillis = millis();
    }
    horn.poll();
    switchBit(!horn.on(), byte0, HORN);

    // reset switch timer
    switch_timer.reset();
  }  
  
  if (byte0 != byte0_prev || byte1 != byte1_prev || display_timer.check()) {
    // toggle turn signal
    steering_wheel.turnsignal_on = !steering_wheel.turnsignal_on;

    //Display shenanigans
    /*What is generally happening is the code is checking whether the switch states have changed by juxtaposing the young and old bytes and then
    changing the members of the display structure accordingly and also changing the notification string*/

    // update gear info
    if(!(~byte0 & (FWD_GEAR|REV_GEAR)) && steering_wheel.geardisplay != 'N'){
      steering_wheel.geardisplay = 'N';
      steering_wheel.notification = String("Neutral Gear");
      notif_timer.reset();
    }
    if((~byte0 & FWD_GEAR) && steering_wheel.geardisplay != 'F'){
      steering_wheel.geardisplay = 'F';
      steering_wheel.notification = String("Forward Gear");
      notif_timer.reset();
    }
    if((~byte0 & REV_GEAR) && steering_wheel.geardisplay != 'R'){
      steering_wheel.geardisplay = 'R';
      steering_wheel.notification = String("Reverse Gear");
      notif_timer.reset();
    }

    // update lights info
    if((~byte0 & HEADLIGHT) && steering_wheel.lightsdisplay != "H "){
      steering_wheel.lightsdisplay = "H ";
      steering_wheel.notification = String("Headlights");
      notif_timer.reset();
    }
    if((~byte0 & HAZARDLIGHT) && steering_wheel.lightsdisplay != "HZ"){
      steering_wheel.lightsdisplay = "HZ";
      steering_wheel.notification = String("Hazardlights");
      notif_timer.reset();
    } 
    if(!(~byte0 & (HAZARDLIGHT|HEADLIGHT)) && steering_wheel.lightsdisplay != "  "){
      steering_wheel.lightsdisplay = "  ";
      steering_wheel.notification = String("All lights off");
      notif_timer.reset();
    }

    // update turn signal info
    if((~byte0 & LEFT_TURN)){
      steering_wheel.LTdisplay = true;
    }
    else steering_wheel.LTdisplay = false;

    if((~byte0 & RIGHT_TURN)){
      steering_wheel.RTdisplay = true;
    }
    else steering_wheel.RTdisplay = false;

    // update cruise control info
    if((~byte1 & CRUISE_ON) && steering_wheel.ccdisplay != 'C'){
      steering_wheel.ccdisplay = 'C';
      steering_wheel.notification = String("Cruisectrl on");
      notif_timer.reset();
    }  
    if((~byte1 & CRUISE_OFF) && steering_wheel.ccdisplay != ' '){
      steering_wheel.ccdisplay = ' ';
      steering_wheel.notification = String("Cruisectrl off");
      notif_timer.reset();
    }

    // update telemetry heartbeat info
    if (telmetry_timer.check()){
      steering_wheel.telemetrydisplay = ' ';
    }

    if (notif_timer.running()){
      defaultdisplay();
      displayNotification();
    }
    else{
      defaultdisplay();
    }

    // update lap timer info
    steering_wheel.lapsecdisplay = (millis() - previousmillis)/1000;
    if (steering_wheel.lapsecdisplay >= 60) {
      steering_wheel.lapmindisplay = steering_wheel.lapsecdisplay/60;
      steering_wheel.lapsecdisplay -= steering_wheel.lapmindisplay*60;
    }
    else {
      steering_wheel.lapmindisplay = 0;
    }
    
    // write screen
    screen.update();
  }

  // send CAN packet
  if(byte0 != byte0_prev || byte1 != byte1_prev || CAN_TX.check()) {
    CanControl.Send(SW_Data(byte0, byte1), TXB0);
    CAN_TX.reset();
  }

  wdt_reset();
  
  // check if CAN packet is available for read
  CanControl.Fetch(); 
  if (CanControl.Available()){
    // read CAN packet
    Frame& f = CanControl.Read();
 #ifdef DEBUG
    Serial.print("Received: " );
    Serial.println(f.id, HEX);
 #endif
    switch (f.id) {
      case MC_VELOCITY_ID: {
        MC_Velocity packet(f);
        steering_wheel.Veldisplay = packet.car_velocity*MPS_TO_MPH;
        #ifdef DEBUG
          Serial.print(steering_wheel.Veldisplay);
        #endif
        CAN_RX.reset();
        break;
      }
      case TEL_STATUS_ID: {
        TEL_Status packet(f);
        if (packet.sql_connected && packet.com_connected) {
          steering_wheel.telemetrydisplay = 'T';
        }
        else {
          steering_wheel.telemetrydisplay = ' ';
        }
        telmetry_timer.reset();
        break;
      }
    }
  }
  
  // CAN debug
  CanControl.FetchErrors();
  CanControl.FetchStatus();
  #ifdef DEBUG
    loopSumTime += (micros() - loopStartTime);
    loopCount += 1;
  #endif
  
  #ifdef DEBUG
    if (debug_timer.check())
    {
      Serial.print("Switches:");
      Serial.println(byte0,BIN);
      Serial.print("TEC/REC: ");
      Serial.print(CanControl.tec); Serial.print(", "); Serial.println(CanControl.rec);
      Serial.print("CANSTATUS: ");
      Serial.println(CanControl.canstat_register);
      Serial.print("CANINTF: ");
      Serial.println(CanControl.controller.Read(CANINTF), BIN);
      Serial.print("Average Loop Time (us): ");
      Serial.println(loopSumTime/loopCount);
      Serial.print("System time: ");
      Serial.println(millis());
      
      loopSumTime = 0;
      loopCount = 0;
    }
  #endif
}

/*
 * Non-notification display function (what is displayed onto LCD if 
 * we're not displaying a notification).
 */
inline void defaultdisplay(){
  screen.clear();
  screen.setCursor(1,4);
  screen.print("LAP ");
  screen.setCursor(1,LT);
  sprintf(steering_wheel.lapdisplay,"%d:%02d",steering_wheel.lapmindisplay,steering_wheel.lapsecdisplay);
  screen.print(steering_wheel.lapdisplay);
  screen.setCursor(1,GEAR);
  screen.print(steering_wheel.geardisplay);
  screen.setCursor(2,4);
  screen.print("V:");
  screen.setCursor(2,V);
  screen.print(min(99,int(steering_wheel.Veldisplay)));
  screen.setCursor(2,LIGHT);
  screen.print(steering_wheel.lightsdisplay);
  screen.setCursor(2,TELM);
  screen.print(steering_wheel.telemetrydisplay);
  if(steering_wheel.LTdisplay || steering_wheel.lightsdisplay=="HZ"){
    blnk(LEFT,steering_wheel.turnsignal_on);
  }

  if(steering_wheel.RTdisplay || steering_wheel.lightsdisplay=="HZ"){
    blnk(RIGHT,steering_wheel.turnsignal_on);
  }
}

/*
 * Notification display function.
 */
inline void displayNotification() {
  //screen.clear();
  screen.clearLine(1);
  screen.selectLine(1);
  screen.print(steering_wheel.notification);
}

/*
 * Assigns appropriate value to the bit from the state of the pin.
 */
inline void switchBitFromPin(byte pin, char& out, byte mask){
  switchBit(digitalRead(pin),out, mask);
}

inline void switchBit(bool b, char& out, byte mask) {
  if (b){
    BIT_SET(out,mask);
  }
  else{
    BIT_CLEAR(out,mask);
  }
}

/*
 * Blink function used for the turn signals.
 */
inline void blnk(int a, boolean on) {
  if (on) {
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
  else {
    screen.setCursor(1,a);
    screen.print("  ");
    screen.setCursor(2,a);
    screen.print("  ");
  }
}


