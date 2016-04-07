/*
 * DriverControls.ino
 * Contains code to run the driver controls
 * board for sc7.
 */

#include <stdint.h>
#include <Metro.h>
#include <SPI.h>
#include "sc7-can-libinclude.h"

//------------------------------CONSTANTS----------------------------//

// debugging
const bool DEBUG       = true;    // change to true to output debug info over serial
const int  SERIAL_BAUD = 115200;  // baudrate for serial (maximum)

// pins
const byte IGNITION_PIN   = 44;
const byte BRAKE_PIN      = 9;
const byte ACCEL_PIN      = A0;
const byte REGEN_PIN      = A1;
const byte INTERRUPT_PIN  = 7;
const byte CS_PIN         = 3;
const byte HORN_PIN       = 2;
const byte RIGHT_TURN_PIN = 11;
const byte LEFT_TURN_PIN  = 12;
const byte HEADLIGHT_PIN  = 10;
const byte BRAKELIGHT_PIN = 13;
const byte BOARDLED       = 13;

// CAN parameters
const uint16_t BAUD_RATE = 1000;
const byte     FREQ      = 16;
const int      MAX_CAN_PACKETS_PER_LOOP = 10; // Maximum number of receivable CAN packets per loop
const uint16_t DC_ID               = 0x00C7;  // For SC7
const uint16_t DC_SER_NO           = 0x0042;  // Don't panic!

const uint16_t RXM0      = MASK_Sxxx;
const uint16_t RXF0      = 0;
const uint16_t RXF1      = BMS_VOLT_CURR_ID;

const uint16_t RXM1      = MASK_Sxxx;
const uint16_t RXF2      = SW_DATA_ID;
const uint16_t RXF3      = TEL_STATUS_ID;
const uint16_t RXF4      = MC_VELOCITY_ID;
const uint16_t RXF5      = MC_BUS_STATUS_ID;

// timer intervals (all in ms)
const uint16_t MC_HB_INTERVAL    = 1000;  // motor controller heartbeat
const uint16_t SW_HB_INTERVAL    = 1500;  // steering wheel heartbeat
const uint16_t BMS_HB_INTERVAL   = 1000;  // bms heartbeat
const uint16_t TEL_HB_INTERVAL   = 1500;  // telemetry heartbeat
const uint16_t DC_DRIVE_INTERVAL = 150;   // drive command packet
const uint16_t DC_INFO_INTERVAL  = 150;   // driver controls info packet
const uint16_t DC_STAT_INTERVAL  = 150;   // driver controls status packet
const uint16_t DC_HB_INTERVAL    = 1000;  // driver controls heartbeat packet
const uint16_t DC_POWER_INTERVAL = 1000;  // driver controls power packet
const uint16_t WDT_INTERVAL      = 5000;  // watchdog timer
const uint16_t TOGGLE_INTERVAL   = 500;   // toggle interval for right/left turn signals, hazards
const uint16_t DEBUG_INTERVAL    = 500;   // interval for debug calls output

// drive parameters
const uint16_t MAX_ACCEL_VOLTAGE   = 1024;    // max possible accel voltage
const float    MAX_ACCEL_RATIO     = 0.8;     // maximum safe accel ratio
const uint16_t MAX_REGEN_VOLTAGE   = 1024;    // max possible regen voltage
const float    MAX_REGEN_RATIO     = 1.0;     // maximum safe regen ratio
const float    MIN_PEDAL_TOLERANCE = 0.07;    // anything less is basically zero
const uint8_t  MIN_BRAKE_COUNT     = 10;      // Minimum # of LOW reads on the brake pin it takes to enable the brake state (for de-noising)
const float    FORWARD_VELOCITY    = 100.0f;  // velocity to use if forward
const float    REVERSE_VELOCITY    = -100.0f; // velocity to use if reverse
const float    MAX_MOTOR_CURRENT   = 1.0;     // sent to the motor to set the maximum amount of current to draw
const float    GEAR_CHANGE_CUTOFF  = 5.0f;    // cannot change gear unless velocity is below this threshold
const float    M_PER_SEC_TO_MPH    = 2.237f;  // conversion factor from m/s to mph
const bool     ENABLE_REGEN        = false;   // flag to enable/disable regen
const bool     ENABLE_CRUISE_CTRL  = true;    // flag to enable/disable cruise control

// steering wheel parameters
const byte NEUTRAL_RAW = 0x03;
const byte FORWARD_RAW = 0x02;
const byte REVERSE_RAW = 0x01;
const byte SW_ON_BIT   = 0;        // value that corresponds to on for steering wheel data
const bool NO_STEERING = false;    // set to true to read light, horn, gear controls directly from board (also automatically enabled when comm with SW is lost).

// BMS parameters
const float MAX_CURRENT_THRESH          = 68000; // mA
const float CONT_CURRENT_THRESH         = 40000; // current may exceed this value no more than 7 times in a row
const int   OVERCURRENTS_ALLOWED        = 7;     // max number of overcurrent values allowed before trip

// status flags 1
const byte HORN_REQ  = 0x01;
const byte RT_REQ    = 0x02;
const byte LT_REQ    = 0x04;
const byte HEAD_REQ  = 0x08;
const byte HAZ_REQ   = 0x10;
const byte CON_REQ   = 0x20;
const byte COFF_REQ  = 0x40;

// status flags 1
const byte RT_ON      = 0x01;
const byte LT_ON      = 0x02;
const byte CRUISE_ON  = 0x04;

// error flags
const byte MC_TIMEOUT  = 0x01; // motor controller timed out
const byte BMS_TIMEOUT = 0x02; // bms timed out
const byte SW_TIMEOUT  = 0x04; // sw timed out
const byte TEL_TIMEOUT = 0x08; // telemetry timed out
const byte SW_BAD_GEAR = 0x10; // bad gearing from steering wheel
const byte BMS_OVER_CURR = 0x20; // detected BMS overcurrent, tripped car.
const byte RESET_MCP2515 = 0x40; // reset the MCP2515
const byte RESET_BOARD   = 0x80; // reset Arduino

//----------------------------TYPE DEFINITIONS------------------------//

/*
 * Enum to represet the possible gear states.
 */
enum GearState { REVERSE = 0x01, FORWARD = 0x02, NEUTRAL = 0x03, BRAKE = 0x04, REGEN = 0x05 };

/*
 * Enum to represent ignition states
 */
enum IgnitionState { IGNITION_START = 0x0040, IGNITION_RUN = 0x0020, IGNITION_PARK = 0x0010 };

/*
 * Struct to hold informations about the car state.
 */
struct CarState {
  //-----------RAW DATA----------//
  // pedals
  bool brakeEngaged;
  unsigned long brakeCountRaw; // internal counter use for de-noising
  uint16_t regenRaw; // raw voltage reading from regen input
  uint16_t accelRaw; // raw voltage reading from accel input
  
  // steering wheel info (if we need to go to digital controls on the driver box)
  byte gearRaw;       // 00 = neutral, 01 = forward, 10 = reverse, 11 = undefined
  bool horn;          // true if driver wants horn on (no toggle)
  bool headlights;    // true if driver wants headlights on (no toggle)
  bool rightTurn;     // true if driver wants right turn on (no toggle)
  bool leftTurn;      // true if driver wants left turn on (no toggle)
  bool hazards;       // true if driver wants hazards on (no toggle)
  bool cruiseCtrlOn;  // true if driver wants cruise control on
  bool cruiseCtrlOff; // true fi driver wants cruise control off
  
  // motor info
  float motorVelocity;  // rotational speed of motor (rpm)
  float carVelocity;    // velocity of car (mph)
  int16_t busCurrent;
  
  // bms info
  float bmsPercentSOC;                      // percent state of charge of bms
  float bmsCurrent;                         // Current reading from BMS (negative is out of the batteries)
  int numOvercurrents;                      // number of current values in buffer over threshold
  bool currentAvailable;                    // set to true when a BMS current packet comes in.
  
  // ignition
  IgnitionState ignitionRaw;                // ingition requested by ignition switch
  
  // debugging
  bool wasReset;       // true on initilization, false otherwise
  byte canstat_reg;    // holds value of canstat register on the MCP2515
  
  //-----------DERIVED DATA-----------//
  // pedals
  float accelRatio; // ratio of accel voltage to max voltage, constrained for safety
  float regenRatio; // ratio of regen voltage to max voltage, constrained for safety
                    
  // motor current
  float accelCurrent;   // current to use if in drive/reverse
  float regenCurrent;   // current to use if in regen
                    
  // cruise control
  bool cruiseCtrl;       // true if cruise control should be active
  float cruiseCtrlRatio; // pedal ratio to use if cruise control active
                    
  // gearing and ignition
  GearState gear;         // brake, foward, reverse, regen, neutral
  IgnitionState ignition; // start, run, park
  bool tripped;           // flag for overcurrent  
  
  // outputs
  bool rightTurnOn;   // true if we should turn rt signal on
  bool leftTurnOn;    // true if we should turn lt signal on

  // status/errors
  uint16_t canErrorFlags; // keep track of errors with CAN bus
  byte dcErrorFlags;      // keep track of other errors
  byte statusFlags1;      // first byte of status flags
  byte statusFlags2;      // second byte of status flags
};

//----------------------------DATA/VARIABLES---------------------------//

// CAN variables
CAN_IO canControl(CS_PIN, INTERRUPT_PIN, BAUD_RATE, FREQ);

// car state
CarState state;

// timers
Metro mcHbTimer(MC_HB_INTERVAL);       // motor controller heartbeat
Metro swHbTimer(SW_HB_INTERVAL);       // steering wheel heartbeat
Metro bmsHbTimer(BMS_HB_INTERVAL);     // bms heartbeat
Metro telHbTimer(TEL_HB_INTERVAL);     // telemetry heartbeat
Metro dcHbTimer(DC_HB_INTERVAL);       // driver controls heartbeat packet
Metro dcDriveTimer(DC_DRIVE_INTERVAL); // driver controls drive command
Metro dcPowerTimer(DC_POWER_INTERVAL); // driver controls power command
Metro dcInfoTimer(DC_INFO_INTERVAL);   // dirver controls info packet
Metro dcStatusTimer(DC_STAT_INTERVAL); // driver controls status packet
Metro hazardsTimer(TOGGLE_INTERVAL);   // timer for toggling hazards
Metro rightTurnTimer(TOGGLE_INTERVAL); // timer for toggling right turn signal
Metro leftTurnTimer(TOGGLE_INTERVAL);  // timer for toggling left turn signal
Metro debugTimer(DEBUG_INTERVAL);      // timer for debug output over serial

// debugging variables
byte debugStep   = 0;       // split serial out into 3 steps
long loopStartTime = 0;
long loopSumTime = 0;
int loopCount = 0;


//--------------------------HELPER FUNCTIONS--------------------------//

/*
 * Reads general purpose input and updates car state.
 */
void readInputs() {
  // read brake
  if (digitalRead(BRAKE_PIN) == LOW)
    state.brakeCountRaw = min(MIN_BRAKE_COUNT, state.brakeCountRaw + 1);
  else if (!state.brakeEngaged && state.brakeCountRaw >= 1)
    state.brakeCountRaw--;
  else 
    state.brakeCountRaw = 0;

  if (state.brakeCountRaw >= MIN_BRAKE_COUNT)
    state.brakeEngaged = true;
  else
    state.brakeEngaged = false;
  
  // read accel and regen pedals pedal
  state.accelRaw = analogRead(ACCEL_PIN);
  if (ENABLE_REGEN) { // will stay 0 if disabled
    state.regenRaw = analogRead(REGEN_PIN);
  }
  
  // read ignition switch
  state.ignitionRaw = digitalRead(IGNITION_PIN) == LOW ? IGNITION_START : IGNITION_PARK;
  
  // modify state if steering wheel disconnected
  if (state.dcErrorFlags & SW_TIMEOUT) {
    // set to forward so we can keep driving
    state.gearRaw = FORWARD_RAW;

    // disable cruise control
    state.cruiseCtrl = false;
  }
}

/*
 * Reads packets from CAN message queue and updates car state.
 */
void readCAN() {
  int safetyCount = 0;  
  while(canControl.Available() && safetyCount <= MAX_CAN_PACKETS_PER_LOOP) { // there are messages
    safetyCount++;                // Increment safety counter
    Frame& f = canControl.Read(); // read one message
    
    // determine source and update heartbeat timers
    if ((f.id & MASK_Sx00) == BMS_BASEADDRESS) { // source is bms
      bmsHbTimer.reset();
      state.dcErrorFlags &= ~BMS_TIMEOUT; // clear flag
    }
    else if ((f.id & MASK_Sx00) == MC_BASEADDRESS) { // source is mc
      mcHbTimer.reset();
      state.dcErrorFlags &= ~MC_TIMEOUT; // clear flag
    }
    else if ((f.id & MASK_Sx00) == SW_BASEADDRESS) { // source is sw
      swHbTimer.reset();
      state.dcErrorFlags &= ~SW_TIMEOUT; // clear flag
    }
    else if ((f.id & MASK_Sx00) == TEL_BASEADDRESS) { // source is tel
      telHbTimer.reset();
      state.dcErrorFlags &= ~TEL_TIMEOUT; // clear flag
    }
    
    // check for specific packets
    if (f.id == MC_BUS_STATUS_ID) { // motor controller bus status
      MC_BusStatus packet(f);
      state.busCurrent = packet.bus_current;
    }
    else if (f.id == MC_VELOCITY_ID) { // motor controller velocity
      MC_Velocity packet(f);
      state.motorVelocity = packet.motor_velocity;
      state.carVelocity = packet.car_velocity * M_PER_SEC_TO_MPH;
    }
    else if (f.id == BMS_SOC_ID) { // bms state of charge
      BMS_SOC packet(f);
      state.bmsPercentSOC = packet.percent_SOC;
    }
    else if (f.id == SW_DATA_ID) { // steering wheel data
      SW_Data packet(f);
      
      // read data
      state.gearRaw =        packet.gear;
      state.horn =          (packet.horn == SW_ON_BIT);
      state.rightTurn =     (packet.rts == SW_ON_BIT);
      state.leftTurn =      (packet.lts == SW_ON_BIT);
      state.headlights =    (packet.headlights == SW_ON_BIT);
      state.hazards =       (packet.hazards == SW_ON_BIT);
      state.cruiseCtrlOn =  (packet.cruiseon == SW_ON_BIT);
      state.cruiseCtrlOff = (packet.cruiseoff == SW_ON_BIT);
    }
    else if (f.id == BMS_VOLT_CURR_ID) { // BMS Voltage Current Packet
      BMS_VoltageCurrent packet(f);
      state.bmsCurrent = packet.current;
      state.currentAvailable = true;
    }
  }
}

/*
 * Checks heartbeat timers to make sure all systems are still connected.
 * If any timer has expired, updates the error state.
 */
void checkTimers() {
  // check motor controller
  if (mcHbTimer.check()) { // motor controller timeout
    state.dcErrorFlags |= MC_TIMEOUT; // set flag
  }
  
  // check bms
  if (bmsHbTimer.check()) { // bms timeout
    state.dcErrorFlags |= BMS_TIMEOUT; // set flag
  }
  
  // check steering wheel
  if (swHbTimer.check()) { // steering wheel timeout
    state.dcErrorFlags |= SW_TIMEOUT; // set flag
  }
  
  // check telemetry
  if (telHbTimer.check()) { // telemetry timeout
    state.dcErrorFlags |= TEL_TIMEOUT; // set flag
  }
}

/*
 * Process information read from GPIO and CAN and updates
 * the car state accordingly.
 */
void updateState() {
  // calculate accel, regen ratios
  state.accelRatio = constrain(float(state.accelRaw)/MAX_ACCEL_VOLTAGE,
                               0.0f,
                               MAX_ACCEL_RATIO);
  state.regenRatio = constrain(float(state.regenRaw)/MAX_REGEN_VOLTAGE,
                               0.0f,
                               MAX_REGEN_RATIO);
  
  // update gear state
  state.dcErrorFlags &= ~SW_BAD_GEAR; // clear bad gear flag
  if (state.brakeEngaged) { // brake engaged, overrides all other gears
    state.gear = BRAKE;
  }
  else if (state.regenRatio > MIN_PEDAL_TOLERANCE) {  // regen engaged
    state.gear = REGEN;
  }
  else { // accel or nothing engaged
    switch (state.gearRaw){
    case NEUTRAL_RAW:
      state.gear = NEUTRAL; // can always change to neutral
      break;
    case FORWARD_RAW:
      if (state.carVelocity > -GEAR_CHANGE_CUTOFF) { // going forward or velocity less than cutoff, gear switch ok
        state.gear = FORWARD;
      }
      break;
    case REVERSE_RAW:
      if (state.carVelocity < GEAR_CHANGE_CUTOFF) { // going backward or velocity less than cutoff, gear switch ok
        state.gear = REVERSE;
      }
      break;
    default: // unknown gear
      state.gear = NEUTRAL; // safe default gear?
      state.dcErrorFlags |= SW_BAD_GEAR; // flag bad gear
      break;
    }
  }
  
  // update lights state
  // check hazards
  if (state.hazards) { // hazards active
    if (hazardsTimer.check()) { // timer expired, toggle
      state.rightTurnOn = !state.rightTurnOn;
      state.leftTurnOn = state.rightTurnOn; // make sure they have same value
      hazardsTimer.reset();
    }
  }
  else { // hazards inactive
    // check right turn signal
    if (state.rightTurn) { // right turn signal active
      if (rightTurnTimer.check()) { // timer expired, toggle
        state.rightTurnOn = !state.rightTurnOn;
        rightTurnTimer.reset();
      }
    }
    else { // right turn signal inactive
      state.rightTurnOn = false;
    }
    // check left turn signal
    if (state.leftTurn) { // left turn signal active
      if (leftTurnTimer.check()) { // timer expired, toggle
        state.leftTurnOn = !state.leftTurnOn;
        leftTurnTimer.reset();
      }
    }
    else { // left turn signal inactive
      state.leftTurnOn = false;
    }
  }
  
  // update cruise control state
  if (state.cruiseCtrlOn) { // cruise control not on, but driver wants on
   state.cruiseCtrl = true;
   state.cruiseCtrlRatio = state.accelRatio;
  }
  if (state.gear != FORWARD || state.cruiseCtrlOff) { // not in forward or driver wants cruise off
   state.cruiseCtrl = false;
  }
  if (!ENABLE_CRUISE_CTRL) { // cruise control disabled
    state.cruiseCtrl = false;
  }
  
  // update current values to be sent to motor controller
  state.regenCurrent = state.regenRatio < MIN_PEDAL_TOLERANCE ? 
                       0 : 
                       state.regenRatio;
  state.accelCurrent = state.accelRatio < MIN_PEDAL_TOLERANCE ? 
                       0 : 
                       state.accelRatio; 
  
  if (state.cruiseCtrl) { // cruise control on, set to cruise control ratio
   state.accelCurrent = state.cruiseCtrlRatio < MIN_PEDAL_TOLERANCE ?
                        0 :
                        state.cruiseCtrlRatio;
  }
  
  // check for trip current condition from BMS
  if (state.currentAvailable) {
    float absBMSCurrent = abs(state.bmsCurrent);

    if (absBMSCurrent >= CONT_CURRENT_THRESH) { // increment overcurrent counter
      state.numOvercurrents++;
    }
    else { // set overcurrent counter to 0
      state.numOvercurrents = 0;
    }
    
    // check for trip condition
    if (absBMSCurrent >= MAX_CURRENT_THRESH || state.numOvercurrents > OVERCURRENTS_ALLOWED) { // kill car
      state.tripped = true;
      state.dcErrorFlags |= BMS_OVER_CURR;
    }

    // mark this update request handled
    state.currentAvailable = false;
  }
  
  // update ignition state
  if (state.tripped) { // kill car
    state.ignition = IGNITION_PARK;
  }
  else {
    state.ignition = state.ignitionRaw;
  }

  // update status flags
  state.statusFlags1 = 0;
  if (state.horn) state.statusFlags1 |= HORN_REQ;
  if (state.rightTurn) state.statusFlags1 |= RT_REQ;
  if (state.leftTurn) state.statusFlags1 |= LT_REQ;
  if (state.headlights) state.statusFlags1 |= HEAD_REQ;
  if (state.hazards) state.statusFlags1 |= HAZ_REQ;
  if (state.cruiseCtrlOn) state.statusFlags1 |= CON_REQ;
  if (state.cruiseCtrlOff) state.statusFlags1 |= COFF_REQ;

  state.statusFlags2 = 0;
  if (state.rightTurnOn) state.statusFlags2 |= RT_ON;
  if (state.leftTurnOn) state.statusFlags2 |= LT_ON;
  if (state.cruiseCtrl) state.statusFlags2 |= CRUISE_ON;
}

/*
 * Sets general purpose output according to car state.
 */
void writeOutputs() {
  digitalWrite(HORN_PIN, state.horn ? HIGH : LOW);
  digitalWrite(HEADLIGHT_PIN, state.headlights ? HIGH : LOW);
  digitalWrite(BRAKELIGHT_PIN, state.brakeEngaged ? HIGH : LOW);
  digitalWrite(RIGHT_TURN_PIN, state.rightTurnOn ? HIGH : LOW);
  digitalWrite(LEFT_TURN_PIN, state.leftTurnOn ? HIGH : LOW);  
}

/*
 * Checks timers to see if packets need to be sent out over the CAN bus.
 * If so, sends the appropriate packets.
 */
void writeCAN() {
  // see if motor controller packet needs to be sent
  if (dcDriveTimer.check() && !state.tripped) { // ready to send drive command
    // determine velocity, current
    float MCvelocity, MCcurrent;
    switch (state.gear) {
    case FORWARD:
      MCvelocity = FORWARD_VELOCITY;
      MCcurrent = state.accelCurrent;
      break;
    case REVERSE:
      MCvelocity = REVERSE_VELOCITY;
      MCcurrent = state.accelCurrent;
      break;
    case REGEN: // drive current back through batteries to recharge them
      MCvelocity = 0;
      MCcurrent = state.regenCurrent; 
      break;
    case BRAKE: // do regen while braking
      MCvelocity = 0;
      MCcurrent = state.regenCurrent;
      break;
    case NEUTRAL: // coast
      MCvelocity = 0;
      MCcurrent = 0;
      break;
    }
    
    // create and send packet
    bool trysend = canControl.Send(DC_Drive(MCvelocity, MCcurrent), TXBANY);
    
    // reset timer
    if (trysend) {
      dcDriveTimer.reset();
    }

  }
  
  // check if driver controls heartbeat needs to be sent
  if (dcHbTimer.check()) {
    // create and send packet
    canControl.Send(DC_Heartbeat(DC_ID, DC_SER_NO), TXBANY);

    // reset timer
    dcHbTimer.reset(); 
  }

  // check if driver controls info packet needs to be sent
  if (dcInfoTimer.check()) {
    // create and send packet
    DC_Info packet(state.ignition, ((state.ignition != IGNITION_PARK) ? true : false),
      state.accelRatio, state.regenRatio, state.numOvercurrents, state.gear);                            
    bool trysend = canControl.SendVerified(packet, TXBANY);
    
    // reset timer
    if (trysend) 
      dcInfoTimer.reset();
    
    // clear non-persistent error flags
    state.dcErrorFlags &= ~RESET_MCP2515;
    state.dcErrorFlags &= ~RESET_BOARD;
  }

  // check if driver controls status packet needs to be sent
  if (dcStatusTimer.check()) {
    // create and send packet
    DC_Status packet(state.canErrorFlags, state.dcErrorFlags, 0, 
      state.statusFlags1, state.statusFlags2);
    bool trysend = canControl.SendVerified(packet, TXBANY);

    if (trysend)
      dcStatusTimer.reset();
  }
  
  if (dcPowerTimer.check()) {
    bool trysend = canControl.Send(DC_Power(MAX_MOTOR_CURRENT), TXBANY);
    
    if (trysend) 
      dcPowerTimer.reset();
  }
}

/*
 * Checks the CAN controller and any other components for errors.
 * If errors exist, updates the error state.
 */
void checkErrors() {
  //Check the can bus for errors
  canControl.FetchErrors();
  state.canErrorFlags = canControl.errors;
  
  // Reset the MCP if we are heading towards a bus_off condition
  if (canControl.tec > 200 || canControl.rec > 200) {
    if (DEBUG) {
      Serial.println("Reseting MCP2515");
      Serial.print("TEC/REC: ");
      Serial.print(canControl.tec); Serial.print(" / "); Serial.println(canControl.rec);
    }
    canControl.ResetController();
    if (DEBUG) {
      Serial.println("Reset MCP2515");
    }

    state.dcErrorFlags |= RESET_MCP2515;
  }
  
  // Check the mode of the MCP2515 (sometimes it is going to sleep randomly)
  canControl.FetchStatus();

  if ((canControl.canstat_register & 0b00100000) == 0b00100000) {
    canControl.ResetController();
    canControl.FetchStatus(); // check that everything worked
    if (DEBUG) {
       Serial.print("MCP2515 went to sleep. CANSTAT reset to: ");
       Serial.println(canControl.canstat_register);
    }
  }
}

//--------------------------MAIN FUNCTIONS---------------------------//

void setup() {
  // setup pin I/O
  pinMode(IGNITION_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);
  pinMode(BRAKELIGHT_PIN, OUTPUT);
  pinMode(RIGHT_TURN_PIN, OUTPUT);
  pinMode(LEFT_TURN_PIN, OUTPUT);
  pinMode(BOARDLED,OUTPUT);

  digitalWrite(BOARDLED,HIGH); // Turn on durring initialization
  
  // debugging [ For some reason the board doesn't work unless I do this here instead of at the bottom ]
  if (DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Serial Initialized");
  }
  
  // init car state
  state = {}; // init all members to 0
  state.gear = FORWARD;
  state.gearRaw = FORWARD_RAW;
  state.dcErrorFlags |= RESET_BOARD;
  state.ignition = IGNITION_PARK;
  state.tripped = false;
    
  // set the watchdog timer interval
  WDT_Enable(WDT, 0x2000 | WDT_INTERVAL | ( WDT_INTERVAL << 16 ));
  
  // reset timers
  mcHbTimer.reset();
  swHbTimer.reset();
  bmsHbTimer.reset();
  dcDriveTimer.reset();
  dcInfoTimer.reset();
  dcHbTimer.reset();
  debugTimer.reset();
  
  
  // setup CAN
  canControl.filters.setRB0(RXM0, RXF0, RXF1);
  canControl.filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  canControl.Setup(RX0IE | RX1IE | TX0IE | TX1IE | TX2IE);
 
  digitalWrite(BOARDLED,LOW);   // Turn of led after initialization
  
  if (DEBUG && canControl.errors != 0) {
    Serial.print("Init CAN error: ");
    Serial.println(canControl.errors, HEX);
  }
}

void loop() {
  // Start timer
  if (DEBUG) {
    loopStartTime = micros();
  }
  
  WDT_Restart(WDT); // clear watchdog timer
  readInputs(); // read GPIO
  canControl.Fetch(); // get any CAN messages that have come in
  readCAN(); // read CAN
  WDT_Restart(WDT); // clear watchdog timer
  checkTimers(); // check timers
  updateState(); // process information that was read
  canControl.Fetch(); // get any CAN messages that have come in 
  writeOutputs(); // write GPIO
  writeCAN(); // write CAN
  WDT_Restart(WDT); // clear watchdog timer
  checkErrors(); // check for errors and fix them
  
  // Add the loop time to the sum time
  if (DEBUG) { 
    loopSumTime += micros() - loopStartTime;
    loopCount += 1;
  }
  
  // debugging printout
  if (DEBUG && debugTimer.check()) {
    
    byte Txstatus[3] = {0,0,0};
    Txstatus[0] = canControl.controller.Read(TXB0CTRL);
    Txstatus[1] = canControl.controller.Read(TXB1CTRL);
    Txstatus[2] = canControl.controller.Read(TXB2CTRL);
    byte canintf = 0; canintf = canControl.last_interrupt;
    byte canctrl = 0; canctrl = canControl.controller.Read(CANCTRL);
    
    Serial.println("TXnCTRL: ");
    Serial.println(Txstatus[0], BIN);
    Serial.println(Txstatus[1], BIN);
    Serial.println(Txstatus[2], BIN);
    Serial.print("Last Interrupt: ");
    Serial.println(canintf, BIN);
    Serial.print("CANCTRL: ");
    Serial.println(canctrl, BIN);
    Serial.print("CANSTAT: ");
    Serial.println(state.canstat_reg, BIN);   
    Serial.print("Loop time (us): ");
    Serial.println(loopSumTime / loopCount);
    Serial.print("System time: ");
    Serial.println(millis());
    Serial.println();
    
    switch (debugStep) {
      case 0:
        Serial.print("Brake pin: ");
        Serial.println(state.brakeEngaged ? "pressed" : "not pressed");
        Serial.print("Accel pedal raw: ");
        Serial.println(state.accelRaw);
        Serial.print("Accel ratio: ");
        Serial.println(state.accelRatio);
        Serial.print("Accel current: ");
        Serial.println(state.accelCurrent);
        Serial.print("Regen pedal raw: ");
        Serial.println(state.regenRaw);
        Serial.print("Regen ratio: ");
        Serial.println(state.regenRatio);
        Serial.print("Regen current: ");
        Serial.println(state.regenCurrent);
        Serial.print("Gear: ");
        switch (state.gear) {
        case BRAKE:
          Serial.println("BRAKE");
          break;
        case FORWARD:
          Serial.println("FORWARD");
          break;
        case REVERSE:
          Serial.println("REVERSE");
          break;
        case REGEN:
          Serial.println("REGEN");
          break;
        case NEUTRAL:
          Serial.println("NEUTRAL");
          break;
        }
        Serial.print("Gear Raw: ");
        Serial.println(state.gearRaw);
        Serial.print("Car tripped: ");
        Serial.println(state.tripped ? "YES" : "NO");
        break;
      case 1:
        Serial.print("Ignition: ");
        Serial.println(state.ignition,HEX);
        Serial.print("Horn: ");
        Serial.println(state.horn ? "ON" : "OFF");
        Serial.print("Headlights: ");
        Serial.println(state.headlights ? "ON" : "OFF");
        Serial.print("Brakelights: ");
        Serial.println(state.brakeEngaged ? "ON" : "OFF");
        Serial.print("Right turn signal: ");
        Serial.println(state.rightTurn ? "ON" : "OFF");
        Serial.print("Right turn singal active: ");
        Serial.println(state.rightTurnOn ? "YES" : "NO");
        Serial.print("Left turn signal: ");
        Serial.println(state.leftTurn ? "ON" : "OFF");
        Serial.print("Left turn signal active: ");
        Serial.println(state.leftTurnOn ? "YES" : "NO");
        Serial.print("Hazards: ");
        Serial.println(state.hazards ? "YES" : "NO");
        break;
      case 2: 
        //Serial.print("Cruise control: ");
        //Serial.println(state.cruiseCtrl ? "ON" : "OFF");
        //Serial.print("Cruise control previous: ");
        //Serial.println(state.cruiseCtrlPrev ? "ON" : "OFF");
        //Serial.print("Cruise control active: ");
        //Serial.println(state.cruiseCtrlOn ? "YES" : "NO");
        //Serial.print("Cruise control ratio: ");
        //Serial.println(state.cruiseCtrlRatio);
        Serial.print("CAN error: ");
        Serial.println(canControl.errors, HEX);
        Serial.print("TEC/REC: ");
        Serial.print(canControl.tec);
        Serial.print(" / "); 
        Serial.println(canControl.rec);
        Serial.print("Interrupt Counter: ");
        Serial.println(canControl.int_counter);
        Serial.print("RX buffer counter: ");
        Serial.println(canControl.RXbuffer.size());
        Serial.print("Board error: ");
        Serial.println(state.dcErrorFlags, HEX);
        Serial.print("BMS Current: ");
        Serial.println(state.bmsCurrent);
        break;
    }
    
    debugStep = (debugStep+1) % 3;
    debugTimer.reset();
    
    // Reset loop timer variables
    loopSumTime = 0;
    loopCount = 0;
  }
}

