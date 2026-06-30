#include <Wire.h>
#include <SPI.h>
#include "MCP41HVX1.h"
#include <math.h>
#include <EEPROM.h>

// === I2C Slave ===
const uint8_t BOWL_I2C_ADDRESS   = 0x09;
const uint8_t LINEAR_I2C_ADDRESS = 0x0A;

// === HX711 (load cell) ===
#include <HX711_ADC.h>
const int HX711_dout = 5;
const int HX711_sck  = 4;
HX711_ADC LoadCell(HX711_dout, HX711_sck);

float calibrationValue = 552.908142f;
const int EEPROM_ADDR_CAL = 0;   // float uses 4 bytes

const unsigned long START_TARE_PRE_DELAY_MS  = 1000;  // adjust as needed
const unsigned long START_TARE_POST_DELAY_MS = 1000;  // adjust as needed

// ================= Pins (UNO R4 Minima) =================
#define WLAT_PIN 8
#define SHDN_PIN 9
#define CS_PIN   10
#define MAG_PIN  A3
#define HEIGHT_HALL_PIN A0

MCP41HVX1 pot(CS_PIN, SHDN_PIN, WLAT_PIN);

// ================= Relay control =================
const int relayPin = 6;
const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;
bool relayState = false;

// ================= Stepper power relay (NEW) =================
const int stepperRelayPin = 7;
bool stepperPowerState = false;

// ================= Digipot (MCP41HVX1) =================
static const int DIGIPOT_MAX_STEP = 255;
static const int WIPER_MIN = 0;
static const int WIPER_MAX = DIGIPOT_MAX_STEP - 2;   // 253
static const int WIPER_START = 23;                   // start here

// Mixer NEMA 17 only
// Commands:
// mu = mixer up (1/2 turn)
// md = mixer down (1/2 turn)
// home = home mixer height until A1 = NEAR
// mix = move to mix height
// raise = move to raise height
// pos = print current stored position
// hall = print A1 hall state
// stop = stop homing
// start = relay ON
// end = relay OFF

const uint8_t dirMixer  = 3;
const uint8_t stepMixer = 2;

const uint16_t STEPS_PER_REV = 1000;   // set for your driver microstep setting
const uint16_t HALF_ROTATION = STEPS_PER_REV;

const uint16_t PULSE_US    = 5;
const uint16_t INTERVAL_US = 1000;

const bool invertMixer = false;   // flip mixer direction if backwards

#define MIXER_PIN A3

// ================= Height hall sensing =================
const int HEIGHT_FAR_THRESHOLD = 200;
const byte HALL_WINDOW = 30;

struct HallMonitor {
  uint8_t pin;
  int farThreshold;
  byte window;
  bool isNear;
  byte samplesSinceLow;
};

HallMonitor heightHall = {HEIGHT_HALL_PIN, HEIGHT_FAR_THRESHOLD, HALL_WINDOW, false, 0};

// ================= Homing / preset settings =================
// false = md direction
// true  = mu direction
const bool HOME_MIXER_UP = false;
const long HOME_MAX_STEPS = 50000;

// EDIT THESE POSITIONS AFTER HOMING / JOGGING
const long MIX_HEIGHT_POS   = 1000;
const long RAISE_HEIGHT_POS = 21000;

// ================= Position / command state =================
long mixerPosSteps = 0;
bool mixerIsHomed = false;
bool homingActive = false;
bool stopRequested = false;

char serialBuffer[24];
byte serialIndex = 0;

// ================================================
// Per-rev timing & averaging (for the current revolution only)
// ================================================
unsigned long revStartMs = 0;

// explicit ladder of 13 positions (12 increments) from 23 to 85 inclusive
static const int WIPER_POSITIONS[] = {
  23, 28, 33, 38, 44, 49, 54, 59, 64, 70, 75, 80, 85
};

uint8_t wiperIndex = 0;

// Ignore the first 2 full timed revolutions after each speed change,
// then record the next 3 revolutions.
const uint8_t NUM_SETTLING_REVS = 2;
uint8_t settlingRevsRemaining = NUM_SETTLING_REVS;

// ===== Logging (measurement -> series -> cycle) =====
const uint8_t NUM_SERIES = sizeof(WIPER_POSITIONS) / sizeof(WIPER_POSITIONS[0]); // 13
const uint8_t NUM_CYCLES = 3; // recorded cycles per series
const uint16_t MAX_MEASUREMENTS = 20; // choose how many sweeps to keep

// Aggregation mode for the 3 recorded revolutions in each series.
// With NUM_CYCLES = 3, MEDIAN and TRIMMED_MEAN return the same middle value.
enum SeriesAggregateMode {
  AGG_MEAN,
  AGG_MEDIAN,
  AGG_TRIMMED_MEAN
};

const SeriesAggregateMode SERIES_AGG_MODE = AGG_MEAN;

struct LogEntry {
  float rpm;
  float avgLoad;
  uint32_t tMs;
};

LogEntry logData[NUM_SERIES][NUM_CYCLES];

enum SweepStatus { STATUS_OK, STATUS_STARTUP, STATUS_TRANSITION, STATUS_SUSPECT };

struct FitResult {
  float slope;
  float intercept;
  float r2;
  float rmse;
  float maxStep;
  SweepStatus status;
};

FitResult fitResults[MAX_MEASUREMENTS];
uint16_t measurement = 0;

// ================= Magnet sensing =================
const int LOW_MAX = 100;    // <= LOW_MAX => 'L'
const int HIGH_MIN = 800;   // >= HIGH_MIN => 'H'

// ================= Switches =================
bool firstRotation = false;
bool risingEdge = false;

// ================= Timers =================
unsigned long nowMs = 0;
unsigned long deltaMs = 0;
unsigned long startCommandMs = 0;

// ================= Holders =================
float loadSum = 0.0f;
uint32_t loadCount = 0;
bool firstRotationCompleted = false;
unsigned long lastMs = 0;
int cycle = 1;
int series = 1;

// ================= Objects =================

// Magnet level state (stateless hysteresis + single stored level)
char previousMagLevel = 'L';   // last quantized level

// ================= Function prototypes =================
void handleSerialCommands();
void processCommand(const char* cmd);

void updateHallState(HallMonitor &hall);
void printHeightHall();

void stepperPowerOn();
void stepperPowerOff();

void stepMixerOnce(bool up);
void moveMixer(bool up, long steps);
void moveMixerTo(long targetSteps);

void homeMixerAxis();
void home2();
void moveToHeightPreset(long targetSteps, const char* name);
void waitForBowl(unsigned long timeoutMs = 30000);
void waitForLinear(unsigned long timeoutMs = 120000);

void relayStart();
void relayEnd();
void mixCycle();

void loadCalibrationFromEEPROM();
void saveCalibrationToEEPROM();
void waitWithLoadCellUpdates(unsigned long waitMs);
void runStartTareSequence();

void resetRotationCapture();
void clearCurrentMeasurementData();

inline char levelHyst(int reading);
inline bool magnetRisingEdge(int reading);
inline void setWiper(int step);
inline float rpmCalculator(unsigned long dtMs);
void printMeasurementSummaryAndFit(uint16_t mIdx);
float aggregateSeriesValue(const float* values, uint8_t count);
void sortFloatArray(float* values, uint8_t count);

// ================= Relay functions =================
void relayStart() {
  // prepare tare before the run actually starts
  runStartTareSequence();

  digitalWrite(relayPin, RELAY_ON);
  relayState = true;
  startCommandMs = millis();

  cycle = 1;
  series = 1;
  wiperIndex = 0;
  setWiper(WIPER_POSITIONS[wiperIndex]);

  resetRotationCapture();

  Serial.println(F("Mixer relay START"));
}

void relayEnd() {
  if (relayState) {
    // Wait for the next rising edge so the paddle always stops at the same position.
    // Motor stays on until the magnet passes the sensor, then power cuts.
    Serial.println(F("Waiting for magnet edge to park paddle..."));
    const unsigned long EDGE_TIMEOUT_MS = 15000UL;
    unsigned long t0 = millis();
    bool edgeFound = false;

    while ((millis() - t0) < EDGE_TIMEOUT_MS) {
      if (LoadCell.update()) LoadCell.getData();
      if (magnetRisingEdge(analogRead(MAG_PIN))) {
        edgeFound = true;
        break;
      }
    }

    if (!edgeFound) {
      Serial.println(F("Edge wait timed out — cutting power now"));
    }
  }

  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;

  resetRotationCapture();

  Serial.println(F("Mixer relay END"));
}

// ================= Mix Cycle sequence =================
void mixCycle() {
  const int highestWiper = WIPER_POSITIONS[NUM_SERIES - 1]; // 85

  // Ensure relay is off and analysis state is clean before starting
  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;
  resetRotationCapture();

  Serial.println(F("mixcycle: Step 1 - Bowl to mix position..."));
  sendToBowl("moveto=8950");
  waitForBowl();

  Serial.println(F("mixcycle: Step 2 - Mixer to mix position..."));
  moveToHeightPreset(MIX_HEIGHT_POS, "MIX");

  Serial.println(F("mixcycle: Step 3 - Bowl rotate on..."));
  sendToBowl("on");

  Serial.println(F("mixcycle: Step 4 - Dispenser Servo close, Mixer Extraction Servo open, Vacuum on..."));
  sendToLinear("c2");
  sendToLinear("o");
  sendToLinear("vacuumon");

  Serial.println(F("mixcycle: Step 5 - Relay on, wiper to max..."));
  setWiper(highestWiper);
  digitalWrite(relayPin, RELAY_ON);
  relayState = true;

  Serial.println(F("mixcycle: Step 6 - Running 30s..."));
  waitWithLoadCellUpdates(30000);

  Serial.println(F("mixcycle: Step 7 - Relay off..."));
  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;

  Serial.println(F("mixcycle: Step 8 - Bowl rotate off..."));
  sendToBowl("off");

  Serial.println(F("mixcycle: Step 9 - Mixer raise to 7000..."));
  moveToHeightPreset(7000, "PARTIAL_RAISE");

  Serial.println(F("mixcycle: Step 10 - Wait 30s..."));
  waitWithLoadCellUpdates(30000);

  Serial.println(F("mixcycle: Step 11 - Mixer return to mix position..."));
  moveToHeightPreset(MIX_HEIGHT_POS, "MIX");

  Serial.println(F("mixcycle: Step 12 - Relay on, wiper to max..."));
  setWiper(highestWiper);
  digitalWrite(relayPin, RELAY_ON);
  relayState = true;

  Serial.println(F("mixcycle: Step 13 - Bowl rotate on..."));
  sendToBowl("on");

  Serial.println(F("mixcycle: Step 14 - Running 2 minutes..."));
  waitWithLoadCellUpdates(120000);

  Serial.println(F("mixcycle: Step 15 - Relay off, Bowl rotate off..."));
  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;
  sendToBowl("off");

  Serial.println(F("mixcycle: Step 16 - Vacuum off, Mixer Extraction Servo close, Dispenser Servo open..."));
  sendToLinear("vacuumoff");
  sendToLinear("c");
  sendToLinear("o2");

  Serial.println(F("mixcycle: complete."));
}

// ================= Stepper power relay functions (NEW) =================
void stepperPowerOn() {
  if (!stepperPowerState) {
    digitalWrite(stepperRelayPin, RELAY_ON);
    stepperPowerState = true;
    delay(50);
  }
}

void stepperPowerOff() {
  if (stepperPowerState) {
    digitalWrite(stepperRelayPin, RELAY_OFF);
    stepperPowerState = false;
    digitalWrite(stepMixer, LOW);
    delay(10);
  }
}

// ================= Height hall functions =================
void updateHallState(HallMonitor &hall) {
  int mag = analogRead(hall.pin);

  if (mag < hall.farThreshold) {
    hall.samplesSinceLow = 0;
    hall.isNear = false;
  } else {
    if (hall.samplesSinceLow < hall.window) {
      hall.samplesSinceLow++;
    }
    hall.isNear = (hall.samplesSinceLow >= hall.window);
  }
}

void printHeightHall() {
  int raw = analogRead(HEIGHT_HALL_PIN);

  Serial.print(F("Height A1: "));
  Serial.print(raw);
  Serial.print(F(" -> "));
  Serial.println(heightHall.isNear ? F("NEAR") : F("FAR"));
}

// ================= Stepper functions =================
void stepMixerOnce(bool up) {
  digitalWrite(dirMixer, (up ^ invertMixer) ? HIGH : LOW);

  digitalWrite(stepMixer, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(stepMixer, LOW);
  delayMicroseconds(INTERVAL_US);

  mixerPosSteps += up ? 1 : -1;
}

void moveMixer(bool up, long steps) {
  if (steps <= 0) return;

  stepperPowerOn();

  for (long i = 0; i < steps; i++) {
    stepMixerOnce(up);
  }

  stepperPowerOff();
}

void moveMixerTo(long targetSteps) {
  long delta = targetSteps - mixerPosSteps;

  if (delta > 0) moveMixer(true, delta);
  else if (delta < 0) moveMixer(false, -delta);
}

// ================= Homing / preset functions =================
void homeMixerAxis() {
  Serial.println(F("Homing started..."));
  Serial.println(F("Type STOP to cancel"));

  homingActive = true;
  stopRequested = false;

  for (byte i = 0; i < HALL_WINDOW + 1; i++) {
    updateHallState(heightHall);
    delay(2);
  }

  long steps = 0;

  stepperPowerOn();

  while (steps < HOME_MAX_STEPS) {
    handleSerialCommands();

    if (stopRequested) {
      homingActive = false;
      stepperPowerOff();
      Serial.println(F("HOME stopped by user"));
      return;
    }

    updateHallState(heightHall);

    if (heightHall.isNear) {
      mixerPosSteps = 0;
      mixerIsHomed = true;
      homingActive = false;
      stepperPowerOff();
      Serial.println(F("HOME complete"));
      printHeightHall();
      return;
    }

    // md direction until near is sensed
    stepMixerOnce(HOME_MIXER_UP);
    steps++;
  }

  homingActive = false;
  stepperPowerOff();
  Serial.println(F("HOME failed: hall sensor not found before limit"));
  printHeightHall();
}

void home2() {
  Serial.println(F("home2: Step 1 - Homing Mixer..."));
  homeMixerAxis();

  if (!mixerIsHomed) {
    Serial.println(F("home2: Mixer homing FAILED, aborting"));
    return;
  }

  Serial.println(F("home2: Step 2 - Moving Mixer to RAISE position..."));
  moveToHeightPreset(RAISE_HEIGHT_POS, "RAISE");

  Serial.println(F("home2: Step 3 - Homing Bowl..."));
  sendToBowl("home");
  Serial.println(F("home2: Bowl home command sent"));
}

void finish() {
  Serial.println(F("finish: Step 1 - Bowl to Mix position..."));
  sendToBowl("mix");
  waitForBowl();

  Serial.println(F("finish: Step 2 - Raising Mixer..."));
  moveToHeightPreset(RAISE_HEIGHT_POS, "RAISE");

  Serial.println(F("finish: Step 3 - Homing Linear..."));
  sendToLinear("home");
  waitForLinear();

  Serial.println(F("finish: Step 4 - Mixer to Mix position..."));
  moveToHeightPreset(MIX_HEIGHT_POS, "MIX");
  Serial.println(F("finish: complete."));
}

void homeAll() {
  Serial.println(F("homeall: Step 1 - Homing Mixer..."));
  homeMixerAxis();
  if (!mixerIsHomed) {
    Serial.println(F("homeall: Mixer homing FAILED, aborting"));
    return;
  }

  Serial.println(F("homeall: Step 2 - Mixer to Raise position..."));
  moveToHeightPreset(RAISE_HEIGHT_POS, "RAISE");

  Serial.println(F("homeall: Step 3 - Homing Bowl..."));
  sendToBowl("home");
  delay(3000);   // tune: time (ms) for Bowl to finish homing

  Serial.println(F("homeall: Step 4 - Bowl to Mix position..."));
  sendToBowl("mix");
  waitForBowl();

  Serial.println(F("homeall: Step 5 - Homing Linear..."));
  sendToLinear("home");
  waitForLinear();
  Serial.println(F("homeall: complete."));
}

void refillCement() {
  Serial.println(F("refillcement: Step 1 - Raising Mixer to top..."));
  moveToHeightPreset(RAISE_HEIGHT_POS, "RAISE");

  Serial.println(F("refillcement: Step 2 - Bowl main to 1800..."));
  sendToBowl("mainmoveto=1800");
  waitForBowl();

  Serial.println(F("refillcement: Step 3 - Linear to 62400..."));
  sendToLinear("moveto=62400");
  waitForLinear();

  Serial.println(F("refillcement: Step 4 - Bowl axis to 4000..."));
  sendToBowl("moveto=4000");
  waitForBowl();

  Serial.println(F("refillcement: Step 5 - Main to -950..."));
  sendToBowl("mainmoveto=-950");
  waitForBowl();

  Serial.println(F("refillcement: Step 6 - Bowl axis to 8950..."));
  sendToBowl("moveto=8950");
  waitForBowl();

  Serial.println(F("refillcement: Step 7 - Main to -550..."));
  sendToBowl("mainmoveto=-550");
  waitForBowl();

  Serial.println(F("refillcement: complete."));
}

void moveToHeightPreset(long targetSteps, const char* name) {
  if (!mixerIsHomed) {
    Serial.println(F("Run HOME first"));
    return;
  }

  moveMixerTo(targetSteps);

  Serial.print(name);
  Serial.print(F(" position reached. Pos="));
  Serial.println(mixerPosSteps);
}

void loadCalibrationFromEEPROM() {
  EEPROM.get(EEPROM_ADDR_CAL, calibrationValue);

  if (isnan(calibrationValue) || calibrationValue == 0.0f) {
    calibrationValue = 552.908142f;
  }

  LoadCell.setCalFactor(calibrationValue);

  Serial.print(F("Calibration loaded: "));
  Serial.println(calibrationValue, 6);
}

void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_ADDR_CAL, calibrationValue);

  Serial.print(F("Calibration saved: "));
  Serial.println(calibrationValue, 6);
}

// ================= Command handling =================
void processCommand(const char* cmd) {
  if (strcmp(cmd, "stop") == 0) {
    if (homingActive) {
      stopRequested = true;
      Serial.println(F("STOP requested"));
    } else {
      Serial.println(F("Nothing is currently homing"));
    }
    return;
  }

  if (homingActive) {
    Serial.println(F("Homing in progress - type STOP to cancel"));
    return;
  }

  if (strcmp(cmd, "start") == 0) {
    relayStart();
  }
  else if (strcmp(cmd, "end") == 0) {
    relayEnd();
  }
  else if (strcmp(cmd, "mu") == 0) {
    moveMixer(true, HALF_ROTATION);
    Serial.print(F("Mixer: UP (mu) | pos="));
    Serial.println(mixerPosSteps);
  }
  else if (strcmp(cmd, "md") == 0) {
    moveMixer(false, HALF_ROTATION);
    Serial.print(F("Mixer: DOWN (md) | pos="));
    Serial.println(mixerPosSteps);
  }
  else if (strcmp(cmd, "home") == 0) {
    homeMixerAxis();
  }
  else if (strcmp(cmd, "home2") == 0) {
    home2();
  }
  else if (strcmp(cmd, "homeall") == 0) {
    homeAll();
  }
  else if (strcmp(cmd, "finish") == 0) {
    finish();
  }
  else if (strcmp(cmd, "refillcement") == 0) {
    refillCement();
  }
  else if (strcmp(cmd, "addmortar") == 0) {
    Serial.println(F("addmortar: Step 1 - Raising Mixer..."));
    moveToHeightPreset(RAISE_HEIGHT_POS, "RAISE");

    Serial.println(F("addmortar: Step 2 - Bowl to mix position..."));
    sendToBowl("mix");
    waitForBowl();

    Serial.println(F("addmortar: Step 3 - Bowl axis to 0..."));
    sendToBowl("moveto=0");
    waitForBowl();

    Serial.println(F("addmortar: complete."));
  }
  else if (strcmp(cmd, "mix") == 0) {
    moveToHeightPreset(MIX_HEIGHT_POS, "MIX");
  }
  else if (strcmp(cmd, "mixcycle") == 0) {
    mixCycle();
  }
  else if (strcmp(cmd, "raise") == 0) {
    moveToHeightPreset(RAISE_HEIGHT_POS, "RAISE");
  }
  else if (strcmp(cmd, "poweron") == 0) {
    sendToBowl("poweron");
    sendToLinear("poweron");
    Serial.println(F("Power ON sent to Bowl and Linear."));
  }
  else if (strcmp(cmd, "poweroff") == 0) {
    sendToBowl("poweroff");
    sendToLinear("poweroff");
    Serial.println(F("Power OFF sent to Bowl and Linear."));
  }
  else if (strcmp(cmd, "dispense") == 0) {
    Serial.println(F("dispense: Step 1 - Raising Mixer to top..."));
    moveToHeightPreset(RAISE_HEIGHT_POS, "RAISE");

    Serial.println(F("dispense: Step 2 - Bowl to Mix position..."));
    sendToBowl("mix");
    waitForBowl();

    Serial.println(F("dispense: Step 3 - Bowl axis to 6300..."));
    sendToBowl("moveto=6300");
    waitForBowl();

    Serial.println(F("dispense: Step 4 - Linear to 27200 (wait 65s)..."));
    sendToLinear("moveto=27200");
    waitWithLoadCellUpdates(65000);

    Serial.println(F("dispense: Step 5 - Bowl axis to 4250..."));
    sendToBowl("moveto=4250");
    waitForBowl();

    Serial.println(F("dispense: Step 6 - Main axis to 1650..."));
    sendToBowl("mainmoveto=1650");
    waitForBowl();

    Serial.println(F("dispense: Step 7 - Bowl axis to -500..."));
    sendToBowl("moveto=-500");
    waitForBowl();

    Serial.println(F("dispense: Step 8 - Main axis to 4450..."));
    sendToBowl("mainmoveto=4450");
    waitForBowl();

    Serial.println(F("dispense: Holding for 10 seconds..."));
    waitWithLoadCellUpdates(10000);

    Serial.println(F("dispense: complete."));
  }
  else if (strcmp(cmd, "returnafterdispense") == 0) {
    Serial.println(F("returnafterdispense: Step 1 - Main axis to 4450..."));
    sendToBowl("mainmoveto=4450");
    waitForBowl();

    Serial.println(F("returnafterdispense: Step 2 - Main axis to 1650..."));
    sendToBowl("mainmoveto=1650");
    waitForBowl();

    Serial.println(F("returnafterdispense: Step 3 - Bowl axis to 4250..."));
    sendToBowl("moveto=4250");
    waitForBowl();

    Serial.println(F("returnafterdispense: Step 4 - Main axis to 0..."));
    sendToBowl("mainmoveto=0");
    waitForBowl();

    Serial.println(F("returnafterdispense: Step 5 - Bowl axis to 6300..."));
    sendToBowl("moveto=6300");
    waitForBowl();

    Serial.println(F("returnafterdispense: Step 6 - Linear to 0 (wait 65s)..."));
    sendToLinear("mix");
    waitWithLoadCellUpdates(65000);

    Serial.println(F("returnafterdispense: Step 7 - Bowl axis to 8900..."));
    sendToBowl("moveto=8950");
    waitForBowl();

    Serial.println(F("returnafterdispense: Step 8 - Mixer to Mix height..."));
    moveToHeightPreset(MIX_HEIGHT_POS, "MIX");

    Serial.println(F("returnafterdispense: complete."));
  }
  else if (strcmp(cmd, "pos") == 0) {
    Serial.print(F("Mixer height pos = "));
    Serial.println(mixerPosSteps);
  }
  else if (strcmp(cmd, "hall") == 0) {
    for (byte i = 0; i < HALL_WINDOW + 1; i++) {
      updateHallState(heightHall);
      delay(2);
    }
    printHeightHall();
  }
  else if (strcmp(cmd, "cal?") == 0) {
    Serial.print(F("Calibration = "));
    Serial.println(calibrationValue, 6);
  }
  else if (strncmp(cmd, "wiper=", 6) == 0) {
    int val = atoi(cmd + 6);
    setWiper(val);
    Serial.print(F("Wiper set to "));
    Serial.println(val);
  }
  else if (strcmp(cmd, "savecal") == 0) {
    saveCalibrationToEEPROM();
  }
  else if (strncmp(cmd, "setcal=", 7) == 0) {
    calibrationValue = atof(cmd + 7);
    LoadCell.setCalFactor(calibrationValue);

    Serial.print(F("Calibration set to: "));
    Serial.println(calibrationValue, 6);
  }
  else if (strncmp(cmd, "b", 1) == 0 && strlen(cmd) > 1) {
    sendToBowl(cmd + 1);   // strip "b" prefix and forward to Bowl
  }
  else if (strncmp(cmd, "l", 1) == 0 && strlen(cmd) > 1) {
    sendToLinear(cmd + 1); // strip "l" prefix and forward to Linear
  }
  else if (strlen(cmd) > 0) {
    Serial.println(F("Unknown cmd (use start, end, mu, md, home, home2, homeall, finish, mix, raise, dispense, poweron, poweroff, pos, hall, stop, cal?, setcal=x, savecal, b<cmd>, l<cmd>)"));
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        processCommand(serialBuffer);
        serialIndex = 0;
      }
    }
    else if (c != ' ' && c != '\t') {
      if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';

      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      }
    }
  }
}

inline char levelHyst(int reading) {
  if (reading <= LOW_MAX) return 'L';
  if (reading >= HIGH_MIN) return 'H';
  return previousMagLevel;
}

inline bool magnetRisingEdge(int reading) {
  char currentMagLevel = levelHyst(reading);   // turns the magnet reading into an L or an H
  bool rising = (currentMagLevel == 'H' && previousMagLevel == 'L');
  previousMagLevel = currentMagLevel;
  return rising;
}

// Set the wiper position. Includes a delay on the HX711 so it reads once the motor has accelerated/deccelerated.
int wiperPosition = WIPER_START;

inline void setWiper(int step) {
  step = constrain(step, WIPER_MIN, WIPER_MAX);
  pot.WiperSetPosition(step);
  wiperPosition = step;
}

inline float rpmCalculator(unsigned long dtMs) {
  return (60000.0f / (float)dtMs);
}

void sortFloatArray(float* values, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }
}

float aggregateSeriesValue(const float* values, uint8_t count) {
  if (count == 0) return NAN;

  if (SERIES_AGG_MODE == AGG_MEAN) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
      sum += values[i];
    }
    return sum / (float)count;
  }

  float sorted[NUM_CYCLES];
  for (uint8_t i = 0; i < count; i++) {
    sorted[i] = values[i];
  }
  sortFloatArray(sorted, count);

  if (SERIES_AGG_MODE == AGG_MEDIAN) {
    if ((count & 1U) != 0U) {
      return sorted[count / 2];
    }
    return 0.5f * (sorted[(count / 2) - 1] + sorted[count / 2]);
  }

  // Trimmed mean: drop lowest and highest when possible.
  if (count <= 2) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
      sum += sorted[i];
    }
    return sum / (float)count;
  }

  float trimmedSum = 0.0f;
  uint8_t trimmedCount = 0;
  for (uint8_t i = 1; i < (count - 1); i++) {
    trimmedSum += sorted[i];
    trimmedCount++;
  }

  return (trimmedCount > 0) ? (trimmedSum / (float)trimmedCount) : NAN;
}

void printMeasurementSummaryAndFit(uint16_t mIdx) {
  float x[NUM_SERIES];   // rpm
  float y[NUM_SERIES];   // load
  uint8_t n = 0;

  Serial.println();
  Serial.println(F("===== MEASUREMENT SUMMARY ====="));
  Serial.print(F("Measurement,"));
  Serial.println(mIdx + 1);
  Serial.println(F("Index,Series,AvgLoad,AvgRPM"));

  // Aggregate the 3 recorded cycles for each series
  for (uint8_t s = 0; s < NUM_SERIES; s++) {
    float seriesLoads[NUM_CYCLES];
    float seriesRpms[NUM_CYCLES];

    for (uint8_t c = 0; c < NUM_CYCLES; c++) {
      seriesLoads[c] = logData[s][c].avgLoad;
      seriesRpms[c]  = logData[s][c].rpm;
    }

    float avgLoadSeries = aggregateSeriesValue(seriesLoads, NUM_CYCLES);
    float avgRpmSeries  = aggregateSeriesValue(seriesRpms, NUM_CYCLES);

    x[n] = avgRpmSeries;
    y[n] = avgLoadSeries;

    Serial.print(n);          // index
    Serial.print(",");
    Serial.print(s + 1);      // series number
    Serial.print(",");
    Serial.print(avgLoadSeries, 3);
    Serial.print(",");
    Serial.println(avgRpmSeries, 3);

    n++;
  }

  // Need at least 2 points for a line
  if (n < 2) {
    Serial.println(F("Not enough points for linear fit"));
    return;
  }

  // Linear regression: Load = a*RPM + c
  double sumX = 0.0;
  double sumY = 0.0;
  double sumXY = 0.0;
  double sumX2 = 0.0;

  for (uint8_t i = 0; i < n; i++) {
    sumX  += x[i];
    sumY  += y[i];
    sumXY += (double)x[i] * (double)y[i];
    sumX2 += (double)x[i] * (double)x[i];
  }

  double denom = (n * sumX2) - (sumX * sumX);

  if (fabs(denom) < 1e-12) {
    Serial.println(F("Linear fit failed: denominator too small"));
    return;
  }

  double a = ((n * sumXY) - (sumX * sumY)) / denom;
  double c = (sumY - (a * sumX)) / n;

  // R^2 on Load as the dependent variable
  double yMean = sumY / n;
  double ssRes = 0.0;
  double ssTot = 0.0;

  for (uint8_t i = 0; i < n; i++) {
    double yPred = (a * x[i]) + c;
    double err   = y[i] - yPred;
    double dev   = y[i] - yMean;

    ssRes += err * err;
    ssTot += dev * dev;
  }

  double r2 = (ssTot > 1e-12) ? (1.0 - (ssRes / ssTot)) : 1.0;

  float rmse = sqrt((float)(ssRes / n));

  // Max single step between consecutive load values
  float maxStep = 0.0f;
  for (uint8_t i = 1; i < n; i++) {
    float step = fabs(y[i] - y[i - 1]);
    if (step > maxStep) maxStep = step;
  }

  // Determine status
  SweepStatus status;
  if (mIdx == 0) {
    status = STATUS_STARTUP;
  } else if (maxStep > 50.0f) {
    status = STATUS_TRANSITION;
  } else if (rmse > 20.0f) {
    status = STATUS_SUSPECT;
  } else {
    status = STATUS_OK;
  }

  fitResults[mIdx].slope     = (float)a;
  fitResults[mIdx].intercept = (float)c;
  fitResults[mIdx].r2        = (float)r2;
  fitResults[mIdx].rmse      = rmse;
  fitResults[mIdx].maxStep   = maxStep;
  fitResults[mIdx].status    = status;

  const char* statusStr =
    (status == STATUS_STARTUP)    ? "STARTUP"    :
    (status == STATUS_TRANSITION) ? "TRANSITION" :
    (status == STATUS_SUSPECT)    ? "SUSPECT"    : "OK";

  float timeSec = (millis() - startCommandMs) / 1000.0f;

  Serial.println();
  Serial.println(F("Linear fit for this measurement:"));
  Serial.print(F("Load = "));
  Serial.print(a, 4);
  Serial.print(F(" * RPM + "));
  Serial.println(c, 4);
  Serial.print(F("R^2 = "));
  Serial.println(r2, 4);
  Serial.print(F("RMSE = "));
  Serial.println(rmse, 2);
  Serial.print(F("MaxStep = "));
  Serial.println(maxStep, 2);
  Serial.print(F("Status = "));
  Serial.println(statusStr);

  // Machine-readable sweep summary line
  // SWEEP,<measurement>,<time_s>,<intercept>,<slope>,<rmse>,<max_step>,<status>
  Serial.print(F("SWEEP,"));
  Serial.print(mIdx + 1);
  Serial.print(F(","));
  Serial.print(timeSec, 1);
  Serial.print(F(","));
  Serial.print(c, 2);
  Serial.print(F(","));
  Serial.print(a, 4);
  Serial.print(F(","));
  Serial.print(rmse, 2);
  Serial.print(F(","));
  Serial.print(maxStep, 2);
  Serial.print(F(","));
  Serial.println(statusStr);

  Serial.print(F("Time since START (s) = "));
  Serial.println(timeSec, 3);

  Serial.println(F("==============================="));
  Serial.println();
}

void waitWithLoadCellUpdates(unsigned long waitMs) {
  unsigned long t0 = millis();
  while ((millis() - t0) < waitMs) {
    handleSerialCommands();
    updateHallState(heightHall);

    if (LoadCell.update()) {
      // keep HX711 conversions running during the delay
      // but do not accumulate these samples into the test data
      LoadCell.getData();
    }
  }
}

void runStartTareSequence() {
  Serial.println(F("Start tare: pre-delay"));
  waitWithLoadCellUpdates(START_TARE_PRE_DELAY_MS);

  Serial.println(F("Start tare: taring"));
  LoadCell.tareNoDelay();

  while (!LoadCell.getTareStatus()) {
    handleSerialCommands();
    updateHallState(heightHall);

    if (LoadCell.update()) {
      LoadCell.getData();
    }
  }

  Serial.println(F("Start tare: post-delay"));
  waitWithLoadCellUpdates(START_TARE_POST_DELAY_MS);

  // make sure no pre/post tare samples leak into the first logged revolution
  loadSum = 0.0f;
  loadCount = 0;
}

void resetRotationCapture() {
  firstRotation = false;
  firstRotationCompleted = false;
  settlingRevsRemaining = NUM_SETTLING_REVS;

  loadSum = 0.0f;
  loadCount = 0;
  lastMs = 0;

  previousMagLevel = levelHyst(analogRead(MAG_PIN));
}

void clearCurrentMeasurementData() {
  for (uint8_t s = 0; s < NUM_SERIES; s++) {
    for (uint8_t c = 0; c < NUM_CYCLES; c++) {
      logData[s][c].rpm = 0.0f;
      logData[s][c].avgLoad = 0.0f;
      logData[s][c].tMs = 0;
    }
  }
}

void sendToBowl(const char* cmd) {
  Wire.beginTransmission(BOWL_I2C_ADDRESS);
  Wire.write((const uint8_t*)cmd, strlen(cmd));
  byte err = Wire.endTransmission();
  Serial.print(F("-> Bowl: "));
  Serial.print(cmd);
  if (err == 0) {
    Serial.println(F(" [OK]"));
  } else {
    Serial.print(F(" [ERR "));
    Serial.print(err);
    Serial.println(F("] (2=no ACK/not found, 3=data NACK, 4=other)"));
  }
}

void waitForBowl(unsigned long timeoutMs = 30000) {
  delay(100);   // short settle before polling
  unsigned long start = millis();
  Serial.print(F("Waiting for Bowl..."));
  while (millis() - start < timeoutMs) {
    Wire.requestFrom((uint8_t)BOWL_I2C_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
      byte status = Wire.read();
      if (status == 0x00) {
        Serial.println(F(" done."));
        return;
      }
    }
    delay(200);
  }
  Serial.println(F(" timed out."));
}

void waitForLinear(unsigned long timeoutMs = 120000) {
  delay(100);
  unsigned long start = millis();
  Serial.print(F("Waiting for Linear..."));
  while (millis() - start < timeoutMs) {
    Wire.requestFrom((uint8_t)LINEAR_I2C_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
      byte status = Wire.read();
      if (status == 0x00) {
        Serial.println(F(" done."));
        return;
      }
    }
    delay(200);
  }
  Serial.println(F(" timed out."));
}

void sendToLinear(const char* cmd) {
  Wire.beginTransmission(LINEAR_I2C_ADDRESS);
  Wire.write((const uint8_t*)cmd, strlen(cmd));
  byte err = Wire.endTransmission();
  Serial.print(F("-> Linear: "));
  Serial.print(cmd);
  if (err == 0) {
    Serial.println(F(" [OK]"));
  } else {
    Serial.print(F(" [ERR "));
    Serial.print(err);
    Serial.println(F("] (2=no ACK/not found, 3=data NACK, 4=other)"));
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();   // I2C master
  delay(30);

  pinMode(MAG_PIN, INPUT);
  pinMode(HEIGHT_HALL_PIN, INPUT);

  pinMode(MIXER_PIN, INPUT);

  pinMode(dirMixer, OUTPUT);
  pinMode(stepMixer, OUTPUT);
  digitalWrite(stepMixer, LOW);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;

  pinMode(stepperRelayPin, OUTPUT);
  digitalWrite(stepperRelayPin, RELAY_OFF);
  stepperPowerState = false;

  Serial.println(F("Commands:"));
  Serial.println(F("  mu    = MIXER up"));
  Serial.println(F("  md    = MIXER down"));
  Serial.println(F("  home  = home mixer height using A1 hall"));
  Serial.println(F("  home2 = home mixer, raise, then home bowl (sequence)"));
  Serial.println(F("  mix   = go to mix height"));
  Serial.println(F("  raise = go to raise height"));
  Serial.println(F("  pos   = print current height position"));
  Serial.println(F("  hall  = print A1 hall state"));
  Serial.println(F("  start = relay ON"));
  Serial.println(F("  end   = relay OFF"));
  Serial.println(F("  stop  = stop homing"));
  Serial.println(F("  cal?   = print HX711 calibration"));
  Serial.println(F("  setcal=x = set HX711 calibration"));
  Serial.println(F("  savecal = save HX711 calibration to EEPROM"));

  wiperIndex = 0;
  setWiper(WIPER_POSITIONS[wiperIndex]);

  previousMagLevel = levelHyst(analogRead(MAG_PIN));
  clearCurrentMeasurementData();

  // ---- HX711 init ----
  LoadCell.begin();
  loadCalibrationFromEEPROM();
}

void loop() {
  handleSerialCommands();
  updateHallState(heightHall);

  if (LoadCell.update()) {
    float load = LoadCell.getData();   // current reading (filtered by library)
    loadSum += load;
    loadCount++;
  }

  if (!relayState) {
    return;
  }

  int mag = analogRead(MAG_PIN);
  bool rise = magnetRisingEdge(mag);

  if (!firstRotation) {
    if (rise) {
      firstRotation = true;
      firstRotationCompleted = true;
      lastMs = millis();   // sync point
      loadSum = 0.0f;
      loadCount = 0;
      //Serial.println(F("Sync edge found"));
    }
    return;
  }

  if (rise && firstRotationCompleted) {
    unsigned long now = millis();
    unsigned long dt = now - lastMs;
    lastMs = now;

    float rpm = rpmCalculator(dt);
    float avgLoad = (loadCount > 0) ? (loadSum / (float)loadCount) : NAN;

    // Ignore the first 2 full timed revolutions after startup / speed change
    if (settlingRevsRemaining > 0) {
      settlingRevsRemaining--;
      loadSum = 0.0f;
      loadCount = 0;
      return;
    }

    // --- store using 0-based indices ---
    uint8_t sIdx = (uint8_t)(series - 1);
    uint8_t cIdx = (uint8_t)(cycle - 1);

    if (sIdx < NUM_SERIES && cIdx < NUM_CYCLES) {
      logData[sIdx][cIdx] = { rpm, avgLoad, now };
    }

    // --- print CSV: measurement,series,cycle,rpm,avgLoad ---
    Serial.print(measurement + 1);
    Serial.print(",");
    Serial.print(series);
    Serial.print(",");
    Serial.print(cycle);
    Serial.print(",");
    Serial.print(rpm, 3);
    Serial.print(",");
    Serial.println(avgLoad, 3);

    cycle += 1;
    loadSum = 0.0f;
    loadCount = 0;
  }

  // After 3 recorded cycles, advance to next series (next wiper position)
  if (cycle == (NUM_CYCLES + 1)) {
    cycle = 1;
    series += 1;
    wiperIndex += 1;

    if (wiperIndex >= NUM_SERIES) {
      // Finished this measurement
      printMeasurementSummaryAndFit(measurement);

      wiperIndex = 0;
      series = 1;

      if (measurement + 1 < MAX_MEASUREMENTS) {
        measurement += 1;
      }
      clearCurrentMeasurementData();
    }

    setWiper(WIPER_POSITIONS[wiperIndex]);
    resetRotationCapture();   // ignore first 2 revs after each speed change
  }
}
