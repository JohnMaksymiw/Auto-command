#include <Wire.h>
#include <Servo.h>
#include <HX711_ADC.h>
#if defined(AVR)
#include <EEPROM.h>
#endif
#include <stdlib.h>
#include <ctype.h>

// ============================================================
// LINEAR ACTUATOR + POWDER DISPENSER
// I2C slave address: 0x0A  (Mixer master uses prefix: l<cmd>)
// 115200 baud
//
// --- Linear (NEMA 23, TB6600) ---
//   DIR=2, STEP=3, Motor relay=8, Power relay=7, Hall=A2
//
// --- Dispenser (Auger TB6600) ---
//   STEP=A0, DIR=13, Auger relay=9
//   HX711: dout=5, sck=4
//   Gate servo=12        (open/close/auto)
//   Mixer Extraction Servo=6   (o/c)
//   Manifold Servo B=A3  (o2/c2)
//   Vacuum relay=10      (vacuumon/vacuumoff / auto during dispensing)
//
// Serial commands:
//   home / left / right / centre / mix / dispense / stop / pos
//   hall / hallstream / halloff
//   poweron / poweroff / toggle / status
//   start / stopdispense / t / weight / weightstream / weightoff
//   open / close / auto   (gate servo)
//   o / c / o2 / c2       (manifold servos)
//   vacuumon / vacuumoff
// ============================================================

// ===== Linear pin assignments =====
const uint8_t DIR_PIN          = 2;
const uint8_t STEP_PIN         = 3;
const uint8_t RELAY_PIN        = 7;   // main power relay (user-controlled)
const uint8_t NEMA23_RELAY_PIN = 8;   // NEMA 23 motor power relay (auto)
#define HALL_PIN A2

// ===== Dispenser pin assignments =====
const uint8_t AUGER_STEP_PIN        = A0;
const uint8_t AUGER_DIR_PIN         = 13;
const uint8_t AUGER_RELAY_PIN       = 9;
const uint8_t SERVO_GATE_PIN        = 12;
const uint8_t MANIFOLD_SERVO_A_PIN  = 6;
const uint8_t MANIFOLD_SERVO_B_PIN  = A3;
const uint8_t VACUUM_RELAY_PIN      = 10;
const uint8_t SERVO_A_RELAY_PIN     = A1;  // shared pin for both servo A and dispenser servo
const uint8_t SERVO_B_RELAY_PIN     = A1;  // same pin
const uint8_t GATE_RELAY_PIN        = 11;
const int     HX711_DOUT            = 5;
const int     HX711_SCK             = 4;

// ===== Dispenser servo angles =====
const uint8_t SERVO_GATE_OPEN_ANGLE   = 180;
const uint8_t SERVO_GATE_CLOSED_ANGLE = 100;

const uint8_t MANIFOLD_SERVO_A_OPEN_ANGLE   = 125;
const uint8_t MANIFOLD_SERVO_A_CLOSED_ANGLE = 35;

const uint8_t MANIFOLD_SERVO_B_OPEN_ANGLE   = 65;
const uint8_t MANIFOLD_SERVO_B_CLOSED_ANGLE = 168;

Servo gateServo;
Servo manifoldServoA;
Servo manifoldServoB;

// ===== Relay (linear main) =====
const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;
bool relayState = false;

void motorPowerOn() {
  digitalWrite(NEMA23_RELAY_PIN, HIGH);
  delay(20);
}

void motorPowerOff() {
  digitalWrite(NEMA23_RELAY_PIN, LOW);
}

void relayPowerOn() {
  digitalWrite(RELAY_PIN, RELAY_ON);
  relayState = true;
  Serial.println(F("Power ON"));
}

void relayPowerOff() {
  digitalWrite(RELAY_PIN, RELAY_OFF);
  relayState = false;
  Serial.println(F("Power OFF"));
}

void relayToggle() {
  if (relayState) relayPowerOff();
  else            relayPowerOn();
}

// ===== Linear step timing =====
const uint16_t PULSE_US    = 10;
const uint16_t INTERVAL_US = 2000;

// ===== Linear motor config =====
const uint16_t STEPS_PER_REV = 1600;
const uint16_t HALF_ROTATION = STEPS_PER_REV / 2;
const bool     INVERT_DIR    = true;

// ===== Homing =====
const int  HALL_FAR_THRESHOLD = 900;
const byte HALL_WINDOW        = 100;
const long HOME_MAX_STEPS     = 100000;
const bool HOME_DIRECTION     = true;

// ===== Preset positions =====
const long CENTRE_POS   = 5000;
const long DISPENSE_POS = 36000;

// ===== Linear state =====
long  linearPosSteps = 0;
bool  isHomed        = false;
bool  homingActive   = false;
bool  stopRequested  = false;

// ===== Non-blocking movement state =====
long          moveTarget      = 0;
bool          movementActive  = false;
long          homingStepCount = 0;
unsigned long lastStepUs      = 0;

// ===== I2C busy flag =====
volatile bool linearBusy = false;

// ===== Hall sensor =====
struct HallMonitor {
  uint8_t pin;
  int     farThreshold;
  byte    window;
  bool    isNear;
  byte    samplesSinceLow;
};

HallMonitor homeHall = {HALL_PIN, HALL_FAR_THRESHOLD, HALL_WINDOW, false, 0};

void updateHallState() {
  int mag = analogRead(homeHall.pin);
  if (mag < homeHall.farThreshold) {
    homeHall.samplesSinceLow = 0;
    homeHall.isNear = false;
  } else {
    if (homeHall.samplesSinceLow < homeHall.window) homeHall.samplesSinceLow++;
    homeHall.isNear = (homeHall.samplesSinceLow >= homeHall.window);
  }
}

void printHall() {
  int raw = analogRead(HALL_PIN);
  Serial.print(F("Hall A2: "));
  Serial.print(raw);
  Serial.print(F(" -> "));
  Serial.println(homeHall.isNear ? F("NEAR") : F("FAR"));
}

bool hallStreaming = false;
unsigned long lastHallPrintMs = 0;
const unsigned long HALL_PRINT_INTERVAL_MS = 100;

bool weightStreaming = false;
unsigned long lastWeightStreamMs = 0;

void printHallStream() {
  int raw = analogRead(HALL_PIN);
  Serial.print(millis());
  Serial.print(F(","));
  Serial.print(raw);
  Serial.print(F(","));
  Serial.println(homeHall.isNear ? F("NEAR") : F("FAR"));
}

// ===== Linear step function =====
void stepOnce(bool moveLeft) {
  bool dir = moveLeft ^ INVERT_DIR;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP_PIN, LOW);
  linearPosSteps += moveLeft ? -1 : 1;
}

// ===== Non-blocking movement =====
void startMoveTo(long target, bool requireHomed = true) {
  if (requireHomed && !isHomed) {
    Serial.println(F("Run HOME first"));
    linearBusy = false;
    return;
  }
  moveTarget     = target;
  movementActive = true;
  lastStepUs     = micros();
  motorPowerOn();
}

void startHoming() {
  Serial.println(F("Homing started..."));
  stopRequested   = false;
  homingActive    = true;
  homingStepCount = 0;
  for (byte i = 0; i < HALL_WINDOW + 1; i++) updateHallState();
  lastStepUs = micros();
  motorPowerOn();
}

void runMovementStep() {
  unsigned long now = micros();
  if (now - lastStepUs < (unsigned long)INTERVAL_US) return;
  lastStepUs = now;

  if (movementActive) {
    long delta = moveTarget - linearPosSteps;
    if (delta == 0) {
      movementActive = false;
      linearBusy     = false;
      motorPowerOff();
      Serial.print(F("Position reached. Pos="));
      Serial.println(linearPosSteps);
      return;
    }
    stepOnce(delta < 0);
  }
  else if (homingActive) {
    if (stopRequested) {
      homingActive = false;
      linearBusy   = false;
      motorPowerOff();
      Serial.println(F("HOME stopped"));
      return;
    }
    updateHallState();
    if (homeHall.isNear) {
      linearPosSteps  = 0;
      isHomed         = true;
      homingActive    = false;
      linearBusy      = false;
      motorPowerOff();
      Serial.println(F("HOME complete. Position set to 0"));
      return;
    }
    if (homingStepCount >= HOME_MAX_STEPS) {
      homingActive = false;
      linearBusy   = false;
      motorPowerOff();
      Serial.println(F("HOME failed: step limit reached"));
      return;
    }
    stepOnce(HOME_DIRECTION);
    homingStepCount++;
  }
}

// ===== Dispenser - Load cell =====
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);
const float calibrationValue  = 1939.037353f;
float currentWeight           = 0.0f;
bool  loadCellReady           = false;

void serviceLoadCell() {
  if (!loadCellReady) return;
  if (LoadCell.update()) currentWeight = LoadCell.getData();
}

// ===== Vacuum relay =====
// Sequence: vacuumon 3s before auger starts, stays on while auger runs,
//           then stays on for 15s after gate opens, gate closes at end of cooldown.
const unsigned long VACUUM_PRE_DELAY_MS  = 3000;
const unsigned long VACUUM_POST_DELAY_MS = 15000;

bool          vacuumOn            = false;  // physical relay state
bool          vacuumManual        = false;  // true = user override, skip auto logic
bool          vacuumPreWaiting    = false;  // in 3s pre-auger warm-up
unsigned long vacuumPreStartMs    = 0;      // when pre-wait began
bool          vacuumPostWaiting   = false;  // in 5s post-gate cooldown
unsigned long vacuumPostStartMs   = 0;      // when cooldown began

// ===== Servo / gate power relays =====
const unsigned long SERVO_RELAY_HOLD_MS = 5000UL;  // hold time after servo move

bool          servoARelayState        = false;
bool          servoBRelayState        = false;
bool          gateRelayState          = false;
bool          servoRelayVacuumPinned  = false;  // true while vacuum is on
bool          gateRelayDispensePinned = false;  // true while dispense sequence active
unsigned long servoARelayHoldUntil    = 0;
unsigned long servoBRelayHoldUntil    = 0;
unsigned long gateRelayHoldUntil      = 0;

void applyServoARelay() {
  bool on = servoRelayVacuumPinned || (millis() < servoARelayHoldUntil);
  if (on != servoARelayState) {
    servoARelayState = on;
    digitalWrite(SERVO_A_RELAY_PIN, on ? HIGH : LOW);
    Serial.println(on ? F("Mixer Extraction Servo relay ON") : F("Mixer Extraction Servo relay OFF"));
  }
}

void applyServoBRelay() {
  bool on = servoRelayVacuumPinned || (millis() < servoBRelayHoldUntil);
  if (on != servoBRelayState) {
    servoBRelayState = on;
    digitalWrite(SERVO_B_RELAY_PIN, on ? HIGH : LOW);
    Serial.println(on ? F("Servo B relay ON") : F("Servo B relay OFF"));
  }
}

void applyGateRelay() {
  bool on = gateRelayDispensePinned || (millis() < gateRelayHoldUntil);
  if (on != gateRelayState) {
    gateRelayState = on;
    digitalWrite(GATE_RELAY_PIN, on ? HIGH : LOW);
    Serial.println(on ? F("Gate relay ON") : F("Gate relay OFF"));
  }
}

void triggerServoARelay()  { servoARelayHoldUntil = millis() + SERVO_RELAY_HOLD_MS; }
void triggerServoBRelay()  { servoBRelayHoldUntil = millis() + SERVO_RELAY_HOLD_MS; }
void triggerGateRelay()    { gateRelayHoldUntil   = millis() + SERVO_RELAY_HOLD_MS; }

void runServoRelayLogic() {
  applyServoARelay();
  applyServoBRelay();
  applyGateRelay();
}

void vacuumRelayOn() {
  if (!vacuumOn) {
    digitalWrite(VACUUM_RELAY_PIN, HIGH);
    vacuumOn = true;
    Serial.println(F("Vacuum ON"));
  }
  // Hold servo A & B relays on while vacuum is running
  servoRelayVacuumPinned = true;
}

void vacuumRelayOff() {
  if (vacuumOn) {
    digitalWrite(VACUUM_RELAY_PIN, LOW);
    vacuumOn = false;
    Serial.println(F("Vacuum OFF"));
  }
  // Release servo and gate relay pins
  servoRelayVacuumPinned  = false;
  gateRelayDispensePinned = false;
}

// ===== Dispenser - Auger relay =====
const uint32_t AUGER_RAMP_START_US = 2000;
const uint32_t AUGER_RAMP_STEP_US  = 8;
bool     augerRelayState      = false;
bool     augerAtSpeed         = false;
uint32_t augerCurrentDelayUs  = AUGER_RAMP_START_US;

void augerPowerOn() {
  if (!augerRelayState) {
    digitalWrite(AUGER_RELAY_PIN, HIGH);
    augerRelayState = true;
    augerAtSpeed = false;              // reset ramp on each fresh power-on
    augerCurrentDelayUs = AUGER_RAMP_START_US;
    delay(50);
  }
}

void augerPowerOff() {
  if (augerRelayState) {
    digitalWrite(AUGER_RELAY_PIN, LOW);
    augerRelayState = false;
  }
}

// ===== Dispenser - Auger step helpers =====
const uint16_t AUGER_STEPS_PER_REV  = 1600;  // 1/8 microstep on TB6600
const uint16_t STEP_PULSE_HIGH_US   = 5;
const uint16_t STEP_PULSE_LOW_US    = 5;
const uint16_t DIR_SETUP_US         = 50;
const bool     AUGER_DIR_INVERT     = false;
const bool     AUGER_DIR_ACTIVE_LOW = false;

// Acceleration ramp constants are declared above with the relay state variables

static inline void pulseAuger() {
  digitalWrite(AUGER_STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_HIGH_US);
  digitalWrite(AUGER_STEP_PIN, LOW);
  delayMicroseconds(STEP_PULSE_LOW_US);
}

static inline void waitUsHX(uint32_t us) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < us) serviceLoadCell();
}

void augerChunk(uint32_t steps, uint16_t rpm) {
  if (steps == 0 || rpm == 0) return;
  bool actualDir = true ^ AUGER_DIR_INVERT;
  digitalWrite(AUGER_DIR_PIN, (actualDir ^ AUGER_DIR_ACTIVE_LOW) ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);

  uint32_t targetUs = 60000000UL / ((uint32_t)rpm * AUGER_STEPS_PER_REV);
  uint32_t pulseTotal = STEP_PULSE_HIGH_US + STEP_PULSE_LOW_US;
  if (targetUs > pulseTotal) targetUs -= pulseTotal;
  else targetUs = 0;

  for (uint32_t i = 0; i < steps; i++) {
    pulseAuger();
    uint32_t waitUs = augerAtSpeed ? targetUs : augerCurrentDelayUs;
    if (waitUs) waitUsHX(waitUs);
    else serviceLoadCell();

    // Ramp toward target speed
    if (!augerAtSpeed) {
      if (augerCurrentDelayUs > targetUs + AUGER_RAMP_STEP_US) {
        augerCurrentDelayUs -= AUGER_RAMP_STEP_US;
      } else {
        augerCurrentDelayUs = targetUs;
        augerAtSpeed = true;
      }
    }
  }
}

// ===== Dispenser - State =====
bool dispensingEnabled  = false;
bool gateOpen           = false;
bool gateManualOverride = false;
unsigned long gateOpenedAt = 0;

void stopDispensingNow() {
  dispensingEnabled  = false;
  augerPowerOff();
  gateManualOverride = false;
  gateServo.write(SERVO_GATE_CLOSED_ANGLE);
  gateOpen = false;
  // Vacuum: if pre-waiting, cancel. If running, start post-cooldown.
  if (!vacuumManual) {
    if (vacuumPreWaiting) {
      vacuumPreWaiting = false;
      vacuumRelayOff();
    } else if (vacuumOn) {
      vacuumPostWaiting  = true;
      vacuumPostStartMs  = millis();
    }
  }
  Serial.println(F("Dispensing stopped, gate closed"));
}

// ===== Forward declarations =====
void processCommand(const char* cmd);
void requestI2C();

// ===== I2C slave =====
#define LINEAR_I2C_ADDRESS 0x0A

volatile char i2cRxBuffer[32];
volatile byte i2cRxIndex      = 0;
volatile bool i2cCommandReady = false;

void receiveI2C(int bytes) {
  i2cRxIndex = 0;
  while (Wire.available() && i2cRxIndex < (byte)(sizeof(i2cRxBuffer) - 1)) {
    i2cRxBuffer[i2cRxIndex++] = Wire.read();
  }
  i2cRxBuffer[i2cRxIndex] = '\0';
  // Reject noise: all valid commands start with a lowercase letter
  if (i2cRxIndex == 0 || i2cRxBuffer[0] < 'a' || i2cRxBuffer[0] > 'z') return;
  i2cCommandReady = true;
  linearBusy = true;
}

void requestI2C() {
  Wire.write(linearBusy ? 0x01 : 0x00);
}

void handleI2CCommands() {
  if (!i2cCommandReady) return;
  i2cCommandReady = false;
  processCommand((const char*)i2cRxBuffer);
  // Block until any movement/homing started by this command finishes,
  // keeping linearBusy=true the whole time (matches Bowl behaviour).
  while (movementActive || homingActive) {
    runMovementStep();
  }
  linearBusy = false;
}

// ===== Serial command buffer =====
char serialBuffer[32];
byte serialIndex = 0;

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        processCommand(serialBuffer);
        serialIndex = 0;
      }
    } else if (serialIndex < (sizeof(serialBuffer) - 1)) {
      serialBuffer[serialIndex++] = c;
    }
  }
}

// ===== Command processor =====
void processCommand(const char* cmd) {

  // --- Linear movement (non-blocking) ---
  if (strcmp(cmd, "home") == 0) {
    startHoming();
  }
  else if (strcmp(cmd, "left") == 0) {
    startMoveTo(linearPosSteps - HALF_ROTATION, false);
    Serial.println(F("LEFT started"));
  }
  else if (strcmp(cmd, "right") == 0) {
    startMoveTo(linearPosSteps + HALF_ROTATION, false);
    Serial.println(F("RIGHT started"));
  }
  else if (strcmp(cmd, "centre") == 0) {
    startMoveTo(CENTRE_POS);
  }
  else if (strcmp(cmd, "mix") == 0) {
    startMoveTo(0);
  }
  else if (strcmp(cmd, "dispense") == 0) {
    startMoveTo(DISPENSE_POS);
  }
  else if (strncmp(cmd, "moveto=", 7) == 0) {
    long target = atol(cmd + 7);
    startMoveTo(target);
    Serial.print(F("Moving to "));
    Serial.println(target);
  }

  // --- Stop: halts movement AND dispensing ---
  else if (strcmp(cmd, "stop") == 0) {
    if (homingActive)        { stopRequested = true; Serial.println(F("HOME stop requested")); }
    else if (movementActive) { movementActive = false; linearBusy = false; motorPowerOff(); Serial.println(F("Movement stopped")); }
    else                     { linearBusy = false; }
    if (dispensingEnabled)   { stopDispensingNow(); }
    if (!homingActive && !movementActive && !dispensingEnabled) Serial.println(F("Nothing running"));
  }

  // --- Dispenser ---
  else if (strcmp(cmd, "start") == 0) {
    // Auto-tare with settling time before and after
    Serial.println(F("Settling before tare..."));
    { uint32_t t = millis(); while (millis() - t < 500) serviceLoadCell(); }
    LoadCell.tareNoDelay();
    Serial.println(F("Taring..."));
    { uint32_t t = millis();
      while (millis() - t < 3000) {
        serviceLoadCell();
        if (LoadCell.getTareStatus()) break;
      }
    }
    Serial.println(F("Tare complete. Settling..."));
    { uint32_t t = millis(); while (millis() - t < 500) serviceLoadCell(); }

    dispensingEnabled       = true;
    gateRelayDispensePinned = true;   // gate relay stays on until vacuum turns off
    gateManualOverride      = false;  // always reset to auto on new dispense
    if (!vacuumManual) {
      vacuumPreWaiting = true;
      vacuumPreStartMs = millis();
      vacuumPostWaiting = false;
      vacuumRelayOn();
      Serial.println(F("Vacuum pre-warming (3s)..."));
    }
    Serial.println(F("Dispensing ENABLED - auger starts after vacuum pre-warm"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "stopdispense") == 0) {
    stopDispensingNow();
    linearBusy = false;
  }
  else if (strcmp(cmd, "t") == 0) {
    LoadCell.tareNoDelay();
    Serial.println(F("Tare started"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "weight") == 0) {
    Serial.print(F("Weight: "));
    Serial.println(currentWeight);
    linearBusy = false;
  }
  else if (strcmp(cmd, "weightstream") == 0) {
    weightStreaming = true;
    Serial.println(F("Weight streaming ON"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "weightoff") == 0) {
    weightStreaming = false;
    Serial.println(F("Weight streaming OFF"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "open") == 0) {
    gateManualOverride = true;
    gateServo.write(SERVO_GATE_OPEN_ANGLE);
    gateOpen = true;
    triggerGateRelay();
    Serial.println(F("Gate MANUAL OPEN"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "close") == 0) {
    gateManualOverride = true;
    gateServo.write(SERVO_GATE_CLOSED_ANGLE);
    gateOpen = false;
    triggerGateRelay();
    Serial.println(F("Gate MANUAL CLOSED"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "auto") == 0) {
    gateManualOverride = false;
    Serial.println(F("Gate AUTO mode"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "o") == 0) {
    manifoldServoA.write(MANIFOLD_SERVO_A_OPEN_ANGLE);
    triggerServoARelay();
    Serial.println(F("Mixer Extraction Servo -> OPEN"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "c") == 0) {
    manifoldServoA.write(MANIFOLD_SERVO_A_CLOSED_ANGLE);
    triggerServoARelay();
    Serial.println(F("Mixer Extraction Servo -> CLOSED"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "o2") == 0) {
    manifoldServoB.write(MANIFOLD_SERVO_B_OPEN_ANGLE);
    triggerServoBRelay();
    Serial.println(F("Manifold Servo B -> OPEN"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "c2") == 0) {
    manifoldServoB.write(MANIFOLD_SERVO_B_CLOSED_ANGLE);
    triggerServoBRelay();
    Serial.println(F("Manifold Servo B -> CLOSED"));
    linearBusy = false;
  }

  // --- Vacuum manual override ---
  else if (strcmp(cmd, "vacuumon") == 0) {
    vacuumManual = true;
    vacuumRelayOn();
    linearBusy = false;
  }
  else if (strcmp(cmd, "vacuumoff") == 0) {
    vacuumManual = false;
    vacuumPreWaiting  = false;
    vacuumPostWaiting = false;
    vacuumRelayOff();
    linearBusy = false;
  }

  // --- Linear instant commands ---
  else if (strcmp(cmd, "pos") == 0) {
    Serial.print(F("Linear pos = "));
    Serial.print(linearPosSteps);
    Serial.println(isHomed ? F(" (homed)") : F(" (NOT homed)"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "hall") == 0) {
    updateHallState();
    printHall();
    linearBusy = false;
  }
  else if (strcmp(cmd, "hallstream") == 0) {
    hallStreaming = true;
    Serial.println(F("Hall streaming ON"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "halloff") == 0) {
    hallStreaming = false;
    Serial.println(F("Hall streaming OFF"));
    linearBusy = false;
  }
  else if (strcmp(cmd, "poweron") == 0) {
    relayPowerOn();
    linearBusy = false;
  }
  else if (strcmp(cmd, "poweroff") == 0) {
    relayPowerOff();
    linearBusy = false;
  }
  else if (strcmp(cmd, "toggle") == 0) {
    relayToggle();
    linearBusy = false;
  }
  else if (strcmp(cmd, "status") == 0) {
    Serial.print(F("Power relay: "));
    Serial.print(relayState ? F("ON") : F("OFF"));
    Serial.print(F(" | Dispensing: "));
    Serial.print(dispensingEnabled ? F("YES") : F("NO"));
    Serial.print(F(" | Vacuum: "));
    Serial.println(vacuumOn ? F("ON") : F("OFF"));
    linearBusy = false;
  }
  else if (strlen(cmd) > 0) {
    Serial.print(F("Unknown cmd: "));
    Serial.println(cmd);
    linearBusy = false;
  }
}

// ===== Vacuum auto logic (called from loop) =====
void runVacuumLogic() {
  if (vacuumManual) return;

  unsigned long now = millis();

  // Pre-warm: vacuum is already on, waiting 3s before allowing auger
  if (vacuumPreWaiting && (now - vacuumPreStartMs >= VACUUM_PRE_DELAY_MS)) {
    vacuumPreWaiting = false;
    Serial.println(F("Vacuum pre-warm done - auger starting"));
  }

  // Post cooldown: 15s after gate opened, close gate then turn vacuum off
  if (vacuumPostWaiting && (now - vacuumPostStartMs >= VACUUM_POST_DELAY_MS)) {
    vacuumPostWaiting = false;
    if (gateOpen && !gateManualOverride) {
      gateServo.write(SERVO_GATE_CLOSED_ANGLE);
      gateOpen = false;
      Serial.println(F("Gate closed after 15s cooldown"));
    }
    vacuumRelayOff();
    Serial.println(F("Vacuum OFF"));
  }
}

// ===== setup =====
void setup() {
  Serial.begin(115200);

  // --- I2C slave ---
  Wire.begin(LINEAR_I2C_ADDRESS);
  Wire.onReceive(receiveI2C);
  Wire.onRequest(requestI2C);

  // --- Linear stepper pins ---
  pinMode(DIR_PIN,          OUTPUT);
  pinMode(STEP_PIN,         OUTPUT);
  pinMode(RELAY_PIN,        OUTPUT);
  pinMode(NEMA23_RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN,        RELAY_OFF);
  digitalWrite(NEMA23_RELAY_PIN, LOW);
  relayState = false;

  // --- Hall sensor ---
  pinMode(HALL_PIN, INPUT);

  // --- Auger relay ---
  pinMode(AUGER_RELAY_PIN, OUTPUT);
  digitalWrite(AUGER_RELAY_PIN, LOW);
  augerRelayState = false;

  // --- Auger stepper pins ---
  pinMode(AUGER_STEP_PIN, OUTPUT);
  pinMode(AUGER_DIR_PIN,  OUTPUT);
  digitalWrite(AUGER_STEP_PIN, LOW);
  digitalWrite(AUGER_DIR_PIN,  LOW);

  // --- Vacuum relay ---
  pinMode(VACUUM_RELAY_PIN, OUTPUT);
  digitalWrite(VACUUM_RELAY_PIN, LOW);
  vacuumOn = false;

  // --- Servo / gate power relays ---
  pinMode(SERVO_A_RELAY_PIN, OUTPUT);
  pinMode(SERVO_B_RELAY_PIN, OUTPUT);
  pinMode(GATE_RELAY_PIN,    OUTPUT);
  digitalWrite(SERVO_A_RELAY_PIN, LOW);
  digitalWrite(SERVO_B_RELAY_PIN, LOW);
  digitalWrite(GATE_RELAY_PIN,    LOW);

  // --- Dispenser servos ---
  gateServo.attach(SERVO_GATE_PIN);
  gateServo.write(SERVO_GATE_CLOSED_ANGLE);
  manifoldServoA.attach(MANIFOLD_SERVO_A_PIN);
  manifoldServoB.attach(MANIFOLD_SERVO_B_PIN);
  manifoldServoA.write(MANIFOLD_SERVO_A_CLOSED_ANGLE);
  manifoldServoB.write(MANIFOLD_SERVO_B_CLOSED_ANGLE);

  // --- Load cell ---
  LoadCell.begin();
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println(F("HX711 timeout - check wiring. Continuing without load cell."));
    loadCellReady = false;
  } else {
    LoadCell.setCalFactor(calibrationValue);
    loadCellReady = true;
    unsigned long primeStart = millis();
    while (millis() - primeStart < 300) serviceLoadCell();
    Serial.println(F("Load cell ready"));
  }

  Serial.println(F("Linear + Dispenser ready"));
  Serial.println(F("--- Linear ---"));
  Serial.println(F("  home/left/right/centre/mix/dispense/stop/pos"));
  Serial.println(F("  hall/hallstream/halloff"));
  Serial.println(F("  poweron/poweroff/toggle/status"));
  Serial.println(F("--- Dispenser ---"));
  Serial.println(F("  start/stopdispense/t/weight/weightstream/weightoff"));
  Serial.println(F("  open/close/auto        (gate servo)"));
  Serial.println(F("  o/c  o2/c2             (manifold servos)"));
  Serial.println(F("  vacuumon/vacuumoff"));
}

// ===== loop =====
void loop() {
  serviceLoadCell();
  handleI2CCommands();
  handleSerialCommands();
  runMovementStep();
  runVacuumLogic();
  runServoRelayLogic();

  if (!movementActive && !homingActive) updateHallState();

  // Hall streaming
  if (hallStreaming) {
    unsigned long now = millis();
    if (now - lastHallPrintMs >= HALL_PRINT_INTERVAL_MS) {
      lastHallPrintMs = now;
      printHallStream();
    }
  }

  // Weight streaming (on demand or while dispensing)
  if (loadCellReady && (weightStreaming || dispensingEnabled)) {
    unsigned long now = millis();
    if (now - lastWeightStreamMs >= 200) {
      lastWeightStreamMs = now;
      Serial.print(F("Weight: "));
      Serial.println(currentWeight);
    }
  }

  // Tare completion
  if (loadCellReady && LoadCell.getTareStatus()) {
    Serial.println(F("Tare complete"));
  }

  // ---- Dispenser logic (only after "start" AND vacuum pre-warm complete) ----
  if (dispensingEnabled && !vacuumPreWaiting) {
    augerPowerOn();

    if (currentWeight < 300) {
      augerChunk(160, 60);
    } else if (currentWeight < 375) {
      augerChunk(80, 30);
    } else if (currentWeight < 435) {
      augerChunk(16, 10);
    } else if (currentWeight < 450) {
      augerChunk(8, 3);
    } else {
      // Target reached: stop auger, open gate, start vacuum post-cooldown
      dispensingEnabled = false;
      augerPowerOff();
      Serial.println(F("Target reached - dispensing stopped"));

      if (!gateManualOverride) {
        gateServo.write(SERVO_GATE_OPEN_ANGLE);
        gateOpen     = true;
        gateOpenedAt = millis();
        Serial.println(F("Gate opening"));
      }

      // Start vacuum 5s cooldown after gate opens
      if (!vacuumManual) {
        vacuumPostWaiting = true;
        vacuumPostStartMs = millis();
      }
      return;
    }

    // Relay stays ON while dispensing — do not power off between chunks

    // AUTO gate: open at target, close after 30s
    if (!gateManualOverride) {
      if (!gateOpen && currentWeight >= 450) {
        gateServo.write(SERVO_GATE_OPEN_ANGLE);
        gateOpen     = true;
        gateOpenedAt = millis();
      }
      if (gateOpen && (millis() - gateOpenedAt >= 30000UL)) {
        gateServo.write(SERVO_GATE_CLOSED_ANGLE);
        gateOpen = false;
      }
    }
  }
}
