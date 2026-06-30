/*
  COMBINED: Powder dispenser + actuator stepper + 2 extra servos (o/c and o2/c2)
  ---------------------------------------------------------------------------
  Dispenser:
    - HX711 load cell: dout=5, sck=4
    - Auger TB6600: STEP=10, DIR=11 (runs ONLY after "start")
    - Gate servo (latch): pin 12, angles SERVO_GATE_OPEN/CLOSED
    - Actuator step/dir: STEP=2, DIR=3  (bl/br)

  Extra servos:
    - Mixer Extraction Servo: commands o / c   (pin 6)
    - Servo B: commands o2 / c2 (pin 7)

  Serial Commands (send with newline):
    start           -> begin dispensing logic (auger + AUTO gate logic)
    stop            -> stop dispensing + close gate (and return gate to AUTO mode)
    t               -> tare
    bl [rot]        -> actuator move one way (default ACT_JOG_ROTATIONS_DEFAULT)
    br [rot]        -> actuator move other way (default ACT_JOG_ROTATIONS_DEFAULT)

    open            -> MANUAL gate open (overrides auto)
    close           -> MANUAL gate close (overrides auto)
    auto            -> return gate to AUTO mode

    o               -> Mixer Extraction Servo open
    c               -> Mixer Extraction Servo close
    o2              -> Servo B open - pin 7
    c2              -> Servo B close pin 6o

*/

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include <Servo.h>

#include <stdlib.h>   // atof()
#include <ctype.h>    // isdigit()

// -------------------- Load cell --------------------
const int HX+711_dout = 5;
const int HX711_sck  = 4;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
float calibrationValue = 1939.037353f;

// IMPORTANT: this is now the ONE source of truth for weight
static float currentWeight = 0.0f;
static unsigned long lastWeightPrintMs = 0;

// Update currentWeight whenever new HX711 data arrives
static inline void serviceLoadCell()
{
  if (LoadCell.update()) {
    currentWeight = LoadCell.getData();
  }
}

// -------------------- Gate servo (latch) --------------------
const uint8_t SERVO_GATE_PIN = 12;
Servo gateServo;

const uint8_t SERVO_GATE_OPEN_ANGLE   = 100; // adjust for your latch
const uint8_t SERVO_GATE_CLOSED_ANGLE = 10;  // adjust for your latch
static bool gateManualOverride = false;

// -------------------- Extra servos --------------------
const uint8_t SERVO_A_PIN = 6;
const uint8_t SERVO_B_PIN = A3;

const uint8_t SERVO_A_OPEN_ANGLE   = 125;
const uint8_t SERVO_A_CLOSED_ANGLE = 35;

const uint8_t SERVO_B_OPEN_ANGLE   = 65;
const uint8_t SERVO_B_CLOSED_ANGLE = 168;

Servo servoA;
Servo servoB;

// -------------------- Auger power relay --------------------
const uint8_t AUGER_RELAY_PIN = 9;
static bool augerRelayState = false;

static void augerPowerOn() {
  if (!augerRelayState) {
    digitalWrite(AUGER_RELAY_PIN, HIGH);
    augerRelayState = true;
    delay(50);   // settle before stepping
  }
}

static void augerPowerOff() {
  if (augerRelayState) {
    digitalWrite(AUGER_RELAY_PIN, LOW);
    augerRelayState = false;
  }
}

// -------------------- Auger (TB6600) --------------------
const uint8_t AUGER_STEP_PIN = A0;
const uint8_t AUGER_DIR_PIN  = 13;

const uint8_t AUGER_EN_PIN = 255;
const bool    AUGER_EN_ACTIVE_LOW = true;

const bool AUGER_DIR_INVERT     = true;
const bool AUGER_DIR_ACTIVE_LOW = false;

const uint16_t AUGER_STEPS_PER_REV = 200;

// -------------------- Actuator (step/dir driver) --------------------
const uint8_t ACT_STEP_PIN = 2;
const uint8_t ACT_DIR_PIN  = 3;

const uint8_t ACT_EN_PIN = 255;
const bool    ACT_EN_ACTIVE_LOW = true;

const bool ACT_DIR_INVERT     = false;
const bool ACT_DIR_ACTIVE_LOW = false;

const uint16_t ACT_STEPS_PER_REV = 200;
const float    ACT_JOG_ROTATIONS_DEFAULT = 1.0f;

// -------------------- Step pulse timing --------------------
const uint16_t STEP_PULSE_HIGH_US = 20;
const uint16_t STEP_PULSE_LOW_US  = 20;
const uint16_t DIR_SETUP_US       = 50;

// -------------------- START gating --------------------
static bool dispensingEnabled = false;

// -------------------- Gate timing state (AUTO mode) --------------------
static bool gateOpen = false;
static unsigned long gateOpenedAt = 0;

// -------------------- Helpers --------------------
static inline void setEnablePin(uint8_t enPin, bool activeLow, bool enable)
{
  if (enPin == 255) return;
  bool activeLevel   = activeLow ? LOW : HIGH;
  bool inactiveLevel = activeLow ? HIGH : LOW;
  digitalWrite(enPin, enable ? activeLevel : inactiveLevel);
}

// Wait but keep load cell serviced
static inline void waitUsWhileServicingHX(uint32_t us)
{
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < us) {
    serviceLoadCell();
  }
}

static inline void pulseStep(uint8_t stepPin)
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_HIGH_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_PULSE_LOW_US);
}

static void stepDriverChunk(uint8_t stepPin,
                            uint8_t dirPin,
                            bool dirCmd,
                            uint16_t stepsPerRev,
                            uint32_t steps,
                            uint16_t rpm,
                            bool dirInvert,
                            bool dirActiveLow)
{
  if (steps == 0 || rpm == 0 || stepsPerRev == 0) return;

  bool actualDir = dirCmd ^ dirInvert;
  bool dirLevel  = actualDir ^ dirActiveLow;
  digitalWrite(dirPin, dirLevel ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);

  uint32_t intervalUs = (60000000UL) / ((uint32_t)rpm * (uint32_t)stepsPerRev);

  uint32_t pulseTotal = (uint32_t)STEP_PULSE_HIGH_US + (uint32_t)STEP_PULSE_LOW_US;
  if (intervalUs > pulseTotal) intervalUs -= pulseTotal;
  else intervalUs = 0;

  for (uint32_t i = 0; i < steps; i++) {
    pulseStep(stepPin);
    if (intervalUs) waitUsWhileServicingHX(intervalUs);
    else serviceLoadCell();
  }
}

// -------------------- Serial command parsing --------------------
static char lineBuf[32];
static uint8_t lineLen = 0;

static float parseFloatOrDefault(const char* s, float defVal)
{
  if (!s) return defVal;
  while (*s == ' ') s++;
  if (*s == '\0') return defVal;

  const char* p = s;
  if (*p == '+' || *p == '-') p++;

  bool hasDigit = false;
  bool hasDot = false;

  while (*p) {
    if (isdigit((unsigned char)*p)) hasDigit = true;
    else if (*p == '.' && !hasDot) hasDot = true;
    else break;
    p++;
  }

  if (!hasDigit) return defVal;
  return (float)atof(s);
}

static void stopDispensingNow()
{
  dispensingEnabled = false;
  augerPowerOff();
  setEnablePin(AUGER_EN_PIN, AUGER_EN_ACTIVE_LOW, false);

  gateManualOverride = false;
  gateServo.write(SERVO_GATE_CLOSED_ANGLE);
  gateOpen = false;
}

static void handleLine(char* line)
{
  char* cmd = strtok(line, " ");
  if (!cmd) return;

  if (strcmp(cmd, "t") == 0) {
    LoadCell.tareNoDelay();
    return;
  }

  if (strcmp(cmd, "start") == 0) {
    dispensingEnabled = true;
    augerPowerOn();
    Serial.println("Dispensing ENABLED (gate AUTO)");
    return;
  }

  if (strcmp(cmd, "stop") == 0) {
    stopDispensingNow();
    Serial.println("Dispensing STOPPED (gate closed, AUTO)");
    return;
  }

  if (strcmp(cmd, "open") == 0) {
    gateManualOverride = true;
    gateServo.write(SERVO_GATE_OPEN_ANGLE);
    gateOpen = true;
    Serial.println("Gate MANUAL OPEN");
    return;
  }

  if (strcmp(cmd, "close") == 0) {
    gateManualOverride = true;
    gateServo.write(SERVO_GATE_CLOSED_ANGLE);
    gateOpen = false;
    Serial.println("Gate MANUAL CLOSED");
    return;
  }

  if (strcmp(cmd, "auto") == 0) {
    gateManualOverride = false;
    Serial.println("Gate AUTO mode");
    return;
  }

  // Extra servos
  if (strcmp(cmd, "o") == 0) {
    servoA.write(SERVO_A_OPEN_ANGLE);
    Serial.println("Servo A -> OPEN");
    return;
  }
  if (strcmp(cmd, "c") == 0) {
    servoA.write(SERVO_A_CLOSED_ANGLE);
    Serial.println("Servo A -> CLOSED");
    return;
  }
  if (strcmp(cmd, "o2") == 0) {
    servoB.write(SERVO_B_OPEN_ANGLE);
    Serial.println("Servo B -> OPEN");
    return;
  }
  if (strcmp(cmd, "c2") == 0) {
    servoB.write(SERVO_B_CLOSED_ANGLE);
    Serial.println("Servo B -> CLOSED");
    return;
  }

  // Actuator jog
  if (strcmp(cmd, "bl") == 0 || strcmp(cmd, "br") == 0) {
    char* arg = strtok(nullptr, " ");
    float rotations = parseFloatOrDefault(arg, ACT_JOG_ROTATIONS_DEFAULT);
    if (rotations < 0) rotations = -rotations;

    uint32_t jogSteps = (uint32_t)(rotations * (float)ACT_STEPS_PER_REV + 0.5f);
    const uint16_t jogRpm = 120;

    setEnablePin(AUGER_EN_PIN, AUGER_EN_ACTIVE_LOW, false);
    setEnablePin(ACT_EN_PIN,   ACT_EN_ACTIVE_LOW,   true);

    bool dirCmd = (strcmp(cmd, "br") == 0);
    stepDriverChunk(ACT_STEP_PIN, ACT_DIR_PIN, dirCmd,
                    ACT_STEPS_PER_REV, jogSteps, jogRpm,
                    ACT_DIR_INVERT, ACT_DIR_ACTIVE_LOW);

    setEnablePin(ACT_EN_PIN, ACT_EN_ACTIVE_LOW, false);

    Serial.print("Actuator ");
    Serial.print(cmd);
    Serial.print(" rotations=");
    Serial.println(rotations, 3);
    return;
  }

  Serial.print("Unknown cmd: ");
  Serial.println(cmd);
}

static void pollSerialLines()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      }
    }
  }
}

// -------------------- Setup / Loop --------------------
void setup()
{
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  // Auger power relay
  pinMode(AUGER_RELAY_PIN, OUTPUT);
  digitalWrite(AUGER_RELAY_PIN, LOW);
  augerRelayState = false;

  // Stepper outputs
  pinMode(AUGER_STEP_PIN, OUTPUT);
  pinMode(AUGER_DIR_PIN,  OUTPUT);
  pinMode(ACT_STEP_PIN,   OUTPUT);
  pinMode(ACT_DIR_PIN,    OUTPUT);
  digitalWrite(AUGER_STEP_PIN, LOW);
  digitalWrite(AUGER_DIR_PIN,  LOW);
  digitalWrite(ACT_STEP_PIN,   LOW);
  digitalWrite(ACT_DIR_PIN,    LOW);

  if (AUGER_EN_PIN != 255) { pinMode(AUGER_EN_PIN, OUTPUT); setEnablePin(AUGER_EN_PIN, AUGER_EN_ACTIVE_LOW, false); }
  if (ACT_EN_PIN   != 255) { pinMode(ACT_EN_PIN,   OUTPUT); setEnablePin(ACT_EN_PIN,   ACT_EN_ACTIVE_LOW,   false); }

  // Load cell init
  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);

  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1) {;}
  } else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup is complete");
  }

  // Prime currentWeight with an initial read (helps avoid 0-stuck on startup)
  unsigned long primeStart = millis();
  while (millis() - primeStart < 300) {
    serviceLoadCell();
  }

  // Attach servos
  gateServo.attach(SERVO_GATE_PIN);
  gateServo.write(SERVO_GATE_CLOSED_ANGLE);
  gateOpen = false;

  servoA.attach(SERVO_A_PIN);
  servoB.attach(SERVO_B_PIN);
  servoA.write(SERVO_A_CLOSED_ANGLE);
  servoB.write(SERVO_B_CLOSED_ANGLE);

  Serial.println("Commands:");
  Serial.println("  start | stop | t | bl [rot] | br [rot]");
  Serial.println("  open | close | auto     (GATE SERVO)");
  Serial.println("  o | c | o2 | c2         (EXTRA SERVOS)");
}

void loop()
{
  // Always service load cell
  serviceLoadCell();

  // Print weight at ~5Hz (avoid spamming Serial)
  if (millis() - lastWeightPrintMs >= 200) {
    lastWeightPrintMs = millis();
    Serial.print("Weight: ");
    Serial.print(currentWeight);
    Serial.print(" | dispensing=");
    Serial.print(dispensingEnabled ? "YES" : "NO");
    Serial.print(" | gate=");
    Serial.print(gateManualOverride ? "MANUAL" : "AUTO");
    Serial.println();
  }

  // Serial control always available
  pollSerialLines();

  // ---- DISPENSER: only after "start" ----
  if (dispensingEnabled) {

    const uint16_t chunkFast  = 20;
    const uint16_t chunkMed   = 10;
    const uint16_t chunkSlow  = 2;
    const uint16_t chunkCreep = 1;

    setEnablePin(ACT_EN_PIN,   ACT_EN_ACTIVE_LOW,   false);
    setEnablePin(AUGER_EN_PIN, AUGER_EN_ACTIVE_LOW, true);
    augerPowerOn();

    // Use currentWeight (always updated), not a stale local variable
    if (currentWeight < 400) {
      stepDriverChunk(AUGER_STEP_PIN, AUGER_DIR_PIN, true,
                      AUGER_STEPS_PER_REV, chunkFast, 60,
                      AUGER_DIR_INVERT, AUGER_DIR_ACTIVE_LOW);
    } else if (currentWeight < 425) {
      stepDriverChunk(AUGER_STEP_PIN, AUGER_DIR_PIN, true,
                      AUGER_STEPS_PER_REV, chunkMed, 30,
                      AUGER_DIR_INVERT, AUGER_DIR_ACTIVE_LOW);
    } else if (currentWeight < 445) {
      stepDriverChunk(AUGER_STEP_PIN, AUGER_DIR_PIN, true,
                      AUGER_STEPS_PER_REV, chunkSlow, 10,
                      AUGER_DIR_INVERT, AUGER_DIR_ACTIVE_LOW);
    } else if (currentWeight < 450) {
      stepDriverChunk(AUGER_STEP_PIN, AUGER_DIR_PIN, true,
                      AUGER_STEPS_PER_REV, chunkCreep, 1,
                      AUGER_DIR_INVERT, AUGER_DIR_ACTIVE_LOW);
    } else {
      stopDispensingNow();
      Serial.println("Target reached - dispensing stopped");
      return;
    }

    setEnablePin(AUGER_EN_PIN, AUGER_EN_ACTIVE_LOW, false);
    augerPowerOff();

    // AUTO gate logic (ignored if manual override)
    if (!gateManualOverride) {
      if (!gateOpen && currentWeight >= 450) {
        gateServo.write(SERVO_GATE_OPEN_ANGLE);
        gateOpen = true;
        gateOpenedAt = millis();
      }
      if (gateOpen && (millis() - gateOpenedAt >= 30000UL)) {
        gateServo.write(SERVO_GATE_CLOSED_ANGLE);
        gateOpen = false;
      }
    }
  }

  if (LoadCell.getTareStatus()) {
    Serial.println("Tare complete");
  }
}