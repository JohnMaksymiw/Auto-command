#include <Wire.h>

// Two NEMA 23 steppers (STEP/DIR drivers)
// Motor A: DIR=2, STEP=3
// Motor B: DIR=4, STEP=5
// Bowl NEMA 17:  DIR=6, STEP=7   (bb/bf)
// Relay: pin 8
// Bowl motor power relay: pin 10
// hello
// ===== Relay control =====
const int relayPin = 8;
// ===== Master power relay control =====
const int masterRelayPin = 9;
const int bowlRelayPin = 10;

// Change these if your relay board is active LOW
const int MASTER_RELAY_ON  = HIGH;
const int MASTER_RELAY_OFF = LOW;
const int BOWL_RELAY_ON    = HIGH;
const int BOWL_RELAY_OFF   = LOW;

bool masterRelayState = false;
bool bowlRelayState = false;
const unsigned long BOWL_RELAY_SETTLE_MS = 20;

void setupMasterRelayControl() {
  // FIX 1: pinMode must be set before digitalWrite
  pinMode(masterRelayPin, OUTPUT);
  digitalWrite(masterRelayPin, MASTER_RELAY_OFF);
  masterRelayState = false;
  Serial.println(F("Master power commands: poweron, poweroff"));
}

void masterPowerOn() {
  digitalWrite(masterRelayPin, MASTER_RELAY_ON);
  masterRelayState = true;
  Serial.println(F("Master power ON"));
}

void masterPowerOff() {
  digitalWrite(masterRelayPin, MASTER_RELAY_OFF);
  masterRelayState = false;
  Serial.println(F("Master power OFF"));
}

void setupBowlRelayControl() {
  // FIX 1: pinMode must be set before digitalWrite
  pinMode(bowlRelayPin, OUTPUT);
  digitalWrite(bowlRelayPin, BOWL_RELAY_OFF);
  bowlRelayState = false;
}

void bowlMotorPowerOn() {
  if (!bowlRelayState) {
    digitalWrite(bowlRelayPin, BOWL_RELAY_ON);
    bowlRelayState = true;
    delay(BOWL_RELAY_SETTLE_MS);
  }
}

void bowlMotorPowerOff() {
  if (bowlRelayState) {
    digitalWrite(bowlRelayPin, BOWL_RELAY_OFF);
    bowlRelayState = false;
  }
}

// ===== Magnet control =====
// ===== Hall sensor control =====
#define BOWL_MAG_PIN   A0
#define ANGLE_MAG_PIN  A1

const int BOWL_FAR_THRESHOLD  = 200;   // adjust if needed
const int ANGLE_FAR_THRESHOLD = 200;   // adjust if needed
const byte HALL_WINDOW = 100;          // number of clean reads needed for NEAR

struct HallMonitor {
  uint8_t pin;
  int farThreshold;
  byte window;
  bool isNear;
  byte samplesSinceLow;
};

HallMonitor bowlHall  = {BOWL_MAG_PIN,  BOWL_FAR_THRESHOLD,  HALL_WINDOW, false, 0};
HallMonitor angleHall = {ANGLE_MAG_PIN, ANGLE_FAR_THRESHOLD, HALL_WINDOW, false, 0};

void updateBowlHallState() {
  int mag = analogRead(bowlHall.pin);

  if (mag < bowlHall.farThreshold) {
    bowlHall.samplesSinceLow = 0;   // definite FAR
    bowlHall.isNear = false;
  } else {
    if (bowlHall.samplesSinceLow < bowlHall.window) {
      bowlHall.samplesSinceLow++;
    }
    bowlHall.isNear = (bowlHall.samplesSinceLow >= bowlHall.window);
  }
}

void updateAngleHallState() {
  int mag = analogRead(angleHall.pin);

  if (mag < angleHall.farThreshold) {
    angleHall.samplesSinceLow = 0;   // definite FAR
    angleHall.isNear = false;
  } else {
    if (angleHall.samplesSinceLow < angleHall.window) {
      angleHall.samplesSinceLow++;
    }
    angleHall.isNear = (angleHall.samplesSinceLow >= angleHall.window);
  }
}

void printHallStates() {
  int bowlRaw = analogRead(BOWL_MAG_PIN);
  int angleRaw = analogRead(ANGLE_MAG_PIN);

  Serial.print(F("Bowl A0: "));
  Serial.print(bowlRaw);
  Serial.print(F(" -> "));
  Serial.print(bowlHall.isNear ? F("NEAR") : F("FAR"));

  Serial.print(F(" | Angle A1: "));
  Serial.print(angleRaw);
  Serial.print(F(" -> "));
  Serial.println(angleHall.isNear ? F("NEAR") : F("FAR"));
}

// ===== Homing / preset settings =====
const bool HOME_MAIN_FORWARDS = false;   // uses command direction "f"
const bool HOME_BOWL_FORWARDS = false;   // uses command direction "bf"
const long HOME_MAX_CYCLES = 20000;      // safety stop if sensor not found
const long MAIN_UNLOCK_BOWL_STEPS = 3000;   // bowl must travel this many steps during homing before main axis begins

bool hallStreaming = false;
unsigned long lastHallPrintMs = 0;
const unsigned long HALL_PRINT_INTERVAL_MS = 100;   // print every 100 ms

// EDIT THESE POSITIONS IN STEPS FROM HOME
const long MIX_MAIN_POS      = 000;
const long MIX_BOWL_POS      = 8950;
const long DISPENSE_MAIN_POS = 4600;
const long DISPENSE_BOWL_POS = 0;

long mainPosSteps = 0;
long bowlPosSteps = 0;
bool isHomed = false;

bool homingActive = false;
bool stopRequested = false;

// Your relay is active HIGH
const int RELAY_ON = HIGH;
const int RELAY_OFF = LOW;

bool relayState = false;
// ===== Shared serial command buffer =====
char serialBuffer[20];
byte serialIndex = 0;
const uint8_t dirA  = 2;
const uint8_t stepA = 3;
const uint8_t dirB  = 4;
const uint8_t stepB = 5;

const uint8_t dirBowl  = 6;
const uint8_t stepBowl = 7;

const uint16_t STEPS_PER_REV  = 100;   // set for your driver microstep setting
// FIX 2: HALF_ROTATION should be half of STEPS_PER_REV, not a full rotation
const uint16_t HALF_ROTATION  = STEPS_PER_REV / 2;

const long ANGLE_HOME_F_TURNS       = 15;   // X turns in f direction
const long ANGLE_HOME_EXTRA_B_TURNS = 15;   // Y extra turns in b direction

const long ANGLE_HOME_F_STEPS = ANGLE_HOME_F_TURNS * STEPS_PER_REV;
const long ANGLE_HOME_B_STEPS = (ANGLE_HOME_F_TURNS + ANGLE_HOME_EXTRA_B_TURNS) * STEPS_PER_REV;

const uint16_t PULSE_US         = 5;     // step pulse width
const uint16_t INTERVAL_US      = 1000;  // time between steps during normal moves; lower = faster
const uint16_t HOME_INTERVAL_US = 500;   // faster interval during homing to avoid resonant frequency

const bool invertB      = true;         // flip Motor B direction if mechanics are mirrored
const bool invertBowl   = false;        // flip bowl direction if it's backwards

void setupRelayControl() {
  // FIX 1: pinMode must be set before digitalWrite
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;
  Serial.println(F("Relay commands: on, off, toggle, status"));
}

void relayOn() {
  digitalWrite(relayPin, RELAY_ON);
  relayState = true;
  Serial.println(F("Relay ON"));
}

void relayOff() {
  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;
  Serial.println(F("Relay OFF"));
}

void relayToggle() {
  if (relayState) relayOff();
  else relayOn();
}

// ===== I2C: send command to Mixer slave =====
#define BOWL_I2C_ADDRESS 0x09

// ===== I2C Slave receive =====
volatile char  i2cRxBuffer[32];
volatile byte  i2cRxIndex     = 0;
volatile bool  i2cCommandReady = false;
volatile bool  bowlBusy        = false;

void receiveI2C(int bytes) {
  i2cRxIndex = 0;
  while (Wire.available() && i2cRxIndex < (byte)(sizeof(i2cRxBuffer) - 1)) {
    i2cRxBuffer[i2cRxIndex++] = Wire.read();
  }
  i2cRxBuffer[i2cRxIndex] = '\0';
  i2cCommandReady = true;
  bowlBusy = true;
}

void requestI2C() {
  Wire.write(bowlBusy ? 0x01 : 0x00);
}

void handleI2CCommands() {
  if (!i2cCommandReady) return;
  i2cCommandReady = false;
  processCommand((const char*)i2cRxBuffer);
  bowlBusy = false;
}

void processCommand(const char* cmd) {

  if (strcmp(cmd, "stop") == 0) {
    if (homingActive) {
      stopRequested = true;
      Serial.println(F("STOP requested"));
    } else if (hallStreaming) {
      hallStreaming = false;
      Serial.println(F("Hall streaming OFF"));
    } else {
      Serial.println(F("Nothing is currently running"));
    }
    return;
  }

  if (strcmp(cmd, "poweron") == 0) {
    masterPowerOn();
    return;
  }
  else if (strcmp(cmd, "poweroff") == 0) {
    masterPowerOff();
    return;
  }

  if (homingActive) {
    Serial.println(F("Homing in progress - type STOP to cancel"));
    return;
  }

  if (strcmp(cmd, "on") == 0) {
    relayOn();
  }
  else if (strcmp(cmd, "off") == 0) {
    relayOff();
  }
  else if (strcmp(cmd, "toggle") == 0) {
    relayToggle();
  }
  else if (strcmp(cmd, "status") == 0) {
    Serial.print(F("Relay is "));
    Serial.println(relayState ? F("ON") : F("OFF"));
  }
  else if (strcmp(cmd, "b") == 0) {
    moveBoth(false, HALF_ROTATION);
    Serial.println(F("Main: BACK 1/2 turn (b)"));
    Serial.print(F("Main pos = "));
    Serial.println(mainPosSteps);
  }
  else if (strcmp(cmd, "f") == 0) {
    moveBoth(true, HALF_ROTATION);
    Serial.println(F("Main: FORWARDS 1/2 turn (f)"));
    Serial.print(F("Main pos = "));
    Serial.println(mainPosSteps);
  }
  else if (strcmp(cmd, "bb") == 0) {
    moveBowl(false, HALF_ROTATION);
    Serial.println(F("Bowl: BACK 1/2 turn (bb)"));
    Serial.print(F("Bowl pos = "));
    Serial.println(bowlPosSteps);
  }
  else if (strcmp(cmd, "bf") == 0) {
    moveBowl(true, HALF_ROTATION);
    Serial.println(F("Bowl: FORWARDS 1/2 turn (bf)"));
    Serial.print(F("Bowl pos = "));
    Serial.println(bowlPosSteps);
  }
  else if (strcmp(cmd, "home") == 0) {
    homeAxes();
  }
  else if (strcmp(cmd, "mix") == 0) {
    moveToPreset(MIX_MAIN_POS, MIX_BOWL_POS, "MIX");
  }
  else if (strcmp(cmd, "dispense") == 0) {
    if (!isHomed) { Serial.println(F("Run HOME first")); }
    else {
      Serial.println(F("Dispense: pre-move bowl to 2300..."));
      moveBowlTo(2300);
      Serial.println(F("Dispense: pre-move main to 950..."));
      moveBothTo(950);
      moveToPreset(DISPENSE_MAIN_POS, DISPENSE_BOWL_POS, "DISPENSE");
    }
  }
  else if (strcmp(cmd, "hall") == 0) {
    hallStreaming = true;
    Serial.println(F("Hall streaming ON"));
  }
  else if (strcmp(cmd, "halloff") == 0) {
    hallStreaming = false;
    Serial.println(F("Hall streaming OFF"));
  }
  else if (strcmp(cmd, "pos") == 0) {
    Serial.print(F("Main pos = "));
    Serial.print(mainPosSteps);
    Serial.print(F(" | Bowl pos = "));
    Serial.println(bowlPosSteps);
  }
  else if (strncmp(cmd, "moveto=", 7) == 0) {
    long target = atol(cmd + 7);
    moveBowlTo(target);
    Serial.print(F("Bowl axis moved to "));
    Serial.println(target);
  }
  else if (strncmp(cmd, "mainmoveto=", 11) == 0) {
    long target = atol(cmd + 11);
    moveBothTo(target);
    Serial.print(F("Main axis moved to "));
    Serial.println(target);
  }
  else if (strlen(cmd) > 0) {
    Serial.println(F("Unknown cmd (use b, f, bb, bf, home, mix, dispense, hall, halloff, pos, moveto=N, mainmoveto=N, stop, on, off, toggle, status, poweron, poweroff)"));
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // End of command
    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        processCommand(serialBuffer);
        serialIndex = 0;
      }
    }
    else if (c != ' ' && c != '\t') {
      // Convert to lowercase
      if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';

      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      }
    }
  }
}

void setup() {
  pinMode(dirA, OUTPUT);
  pinMode(stepA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(stepB, OUTPUT);

  pinMode(dirBowl, OUTPUT);
  pinMode(stepBowl, OUTPUT);

  digitalWrite(stepA, LOW);
  digitalWrite(stepB, LOW);
  digitalWrite(stepBowl, LOW);

  pinMode(BOWL_MAG_PIN, INPUT);
  pinMode(ANGLE_MAG_PIN, INPUT);

  Serial.begin(9600);
  Wire.begin(BOWL_I2C_ADDRESS);   // I2C slave
  Wire.onReceive(receiveI2C);
  Wire.onRequest(requestI2C);

  Serial.println(F("Commands:"));
  Serial.println(F("  b        = MAIN back (1/2 turn)"));
  Serial.println(F("  f        = MAIN forwards (1/2 turn)"));
  Serial.println(F("  bb       = BOWL back (1/2 turn)"));
  Serial.println(F("  bf       = BOWL forwards (1/2 turn)"));
  Serial.println(F("  home     = home both axes using hall sensors"));
  Serial.println(F("  mix      = go to mix position"));
  Serial.println(F("  dispense = go to dispense position"));
  Serial.println(F("  on       = Relay ON"));
  Serial.println(F("  off      = Relay OFF"));
  Serial.println(F("  stop     = stop homing"));
  Serial.println(F("  toggle   = Relay toggle"));
  Serial.println(F("  status   = Relay status"));
  Serial.println(F("  hall     = start hall streaming"));
  Serial.println(F("  halloff  = stop hall streaming"));
  Serial.println(F("  pos      = print current positions"));
  Serial.println(F("  poweron  = Master power ON"));
  Serial.println(F("  poweroff = Master power OFF"));


  setupRelayControl();
  setupMasterRelayControl();
  setupBowlRelayControl();
}

void loop() {
  handleI2CCommands();
  handleSerialCommands();

  updateBowlHallState();
  updateAngleHallState();

  if (hallStreaming) {
    unsigned long now = millis();
    if (now - lastHallPrintMs >= HALL_PRINT_INTERVAL_MS) {
      lastHallPrintMs = now;
      printHallStates();
    }
  }
  delay(5);
}

void stepBothOnce(bool forwards) {
  digitalWrite(dirA, forwards ? HIGH : LOW);
  digitalWrite(dirB, (forwards ^ invertB) ? HIGH : LOW);

  digitalWrite(stepA, HIGH);
  digitalWrite(stepB, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(stepA, LOW);
  digitalWrite(stepB, LOW);
  delayMicroseconds(INTERVAL_US);

  mainPosSteps += forwards ? 1 : -1;
}

void stepBowlOnce(bool forwards) {
  digitalWrite(dirBowl, (forwards ^ invertBowl) ? HIGH : LOW);

  digitalWrite(stepBowl, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(stepBowl, LOW);
  delayMicroseconds(INTERVAL_US);

  bowlPosSteps += forwards ? 1 : -1;
}

void stepBothOnceHoming(bool forwards) {
  digitalWrite(dirA, forwards ? HIGH : LOW);
  digitalWrite(dirB, (forwards ^ invertB) ? HIGH : LOW);

  digitalWrite(stepA, HIGH);
  digitalWrite(stepB, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(stepA, LOW);
  digitalWrite(stepB, LOW);
  delayMicroseconds(HOME_INTERVAL_US);

  mainPosSteps += forwards ? 1 : -1;
}

void stepBowlOnceHoming(bool forwards) {
  digitalWrite(dirBowl, (forwards ^ invertBowl) ? HIGH : LOW);

  digitalWrite(stepBowl, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(stepBowl, LOW);
  delayMicroseconds(HOME_INTERVAL_US);

  bowlPosSteps += forwards ? 1 : -1;
}

void moveBoth(bool forwards, long steps) {
  for (long i = 0; i < steps; i++) {
    stepBothOnce(forwards);
  }
}

void moveBowl(bool forwards, long steps) {
  if (steps <= 0) return;

  bowlMotorPowerOn();
  for (long i = 0; i < steps; i++) {
    stepBowlOnce(forwards);
  }
  bowlMotorPowerOff();
}

void moveBothTo(long targetSteps) {
  long delta = targetSteps - mainPosSteps;

  if (delta > 0) moveBoth(true, delta);
  else if (delta < 0) moveBoth(false, -delta);
}

void moveBowlTo(long targetSteps) {
  long delta = targetSteps - bowlPosSteps;

  if (delta > 0) moveBowl(true, delta);
  else if (delta < 0) moveBowl(false, -delta);
}

void homeAxes() {
  Serial.println(F("Homing started..."));
  Serial.println(F("Type STOP to cancel"));

  homingActive = true;
  stopRequested = false;

  // settle both hall monitors first
  for (byte i = 0; i < HALL_WINDOW + 1; i++) {
    updateBowlHallState();
    updateAngleHallState();
    delay(2);
  }

  bool angleHomed  = false;
  bool bowlHomed   = false;
  bool mainUnlocked = false;
  bool mainPreMoveDone = false;   // true once main axis has reached -500 pre-move position
  long bowlHomingStepCount = 0;   // steps taken by bowl since homing started
  long cycles = 0;

  byte angleSearchPhase = 0;   // 0 = searching in f, 1 = searching in b
  long angleStepsDone = 0;

  while (cycles < HOME_MAX_CYCLES) {
    handleSerialCommands();

    if (stopRequested) {
      bowlMotorPowerOff();
      homingActive = false;
      Serial.println(F("HOME stopped by user"));
      return;
    }

    // unlock main axis once bowl has traveled enough steps, OR if bowl finds home early
    if (!mainUnlocked && (bowlHomingStepCount >= MAIN_UNLOCK_BOWL_STEPS || bowlHomed)) {
      mainUnlocked = true;
      Serial.println(F("Main axis homing unlocked"));
    }

    // update sensors
    if (!bowlHomed) updateBowlHallState();
    if (!angleHomed && mainUnlocked && mainPreMoveDone) updateAngleHallState();

    // latch bowl home
    if (!bowlHomed && bowlHall.isNear) {
      bowlHomed = true;
      bowlMotorPowerOff();
      Serial.println(F("Bowl HOME detected"));
    }

    // latch angle home — only after pre-move to -500 is complete
    if (!angleHomed && mainUnlocked && mainPreMoveDone && angleHall.isNear) {
      angleHomed = true;
      Serial.println(F("Angle HOME detected"));
    }

    // bowl moves first until homed
    if (!bowlHomed) {
      bowlMotorPowerOn();
      stepBowlOnceHoming(HOME_BOWL_FORWARDS);
      bowlHomingStepCount++;
    }

    // main axis: once unlocked, move to -500 first, then begin angle search
    if (!angleHomed && mainUnlocked) {
      if (!mainPreMoveDone) {
        if (mainPosSteps > -500) {
          stepBothOnceHoming(false);   // move toward -500
        } else {
          mainPreMoveDone = true;
          Serial.println(F("Main axis at -500, starting angle search"));
        }
      }
      else {
        if (angleSearchPhase == 0) {
          if (angleStepsDone < ANGLE_HOME_F_STEPS) {
            stepBothOnceHoming(true);   // f direction
            angleStepsDone++;
          } else {
            angleSearchPhase = 1;
            angleStepsDone = 0;
            Serial.println(F("Angle not found in F search, reversing to B search"));
          }
        }
        else if (angleSearchPhase == 1) {
          if (angleStepsDone < ANGLE_HOME_B_STEPS) {
            stepBothOnceHoming(false);  // b direction
            angleStepsDone++;
          } else {
            bowlMotorPowerOff();
            homingActive = false;
            Serial.println(F("HOME failed: angle not found in search range"));
            return;
          }
        }
      }
    }

    if (angleHomed && bowlHomed) {
      mainPosSteps = 0;
      bowlPosSteps = 0;
      isHomed = true;
      bowlMotorPowerOff();
      homingActive = false;
      Serial.println(F("HOME complete"));
      return;
    }

    cycles++;
  }

  bowlMotorPowerOff();
  homingActive = false;
  Serial.println(F("HOME failed: max cycles reached"));
}

void moveToPreset(long targetMain, long targetBowl, const char* name) {
  if (!isHomed) {
    Serial.println(F("Run HOME first"));
    return;
  }

  moveBothTo(targetMain);
  moveBowlTo(targetBowl);

  Serial.print(name);
  Serial.print(F(" position reached. Main="));
  Serial.print(mainPosSteps);
  Serial.print(F(" Bowl="));
  Serial.println(bowlPosSteps);
}
