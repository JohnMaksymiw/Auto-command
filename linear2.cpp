// linear2.cpp — Two NEMA 17 steppers via TB6600, relay-switched power
//
// Motor 1: STEP=3  DIR=2  RELAY=5
// Motor 2: STEP=7  DIR=6  RELAY=4
//
// TB6600 at 1/32 microstep → 6400 steps/rev (NEMA 17, 200 base steps × 32)
// Relays are active-HIGH (HIGH = energised/on, LOW = off)
//
// Commands:
//   raise — both motors run 10 rotations in opposite directions
//   stop  — stop immediately

const int M1_STEP  = 3;
const int M1_DIR   = 2;
const int M1_RELAY = 5;

const int M2_STEP  = 7;
const int M2_DIR   = 6;
const int M2_RELAY = 4;

const unsigned long INTERVAL_US   = 500;    // µs between steps (~2000 steps/sec)
const unsigned int  PULSE_US      = 10;     // step pulse width µs
const long          STEPS_PER_REV = 6400;   // TB6600 1/32 microstep, NEMA 17
const int           RAISE_REVS    = 10;
const long          RAISE_STEPS   = (long)RAISE_REVS * STEPS_PER_REV; // 64000

bool running   = false;
long stepsLeft = 0;
unsigned long lastStep = 0;

void stopMotors() {
  running    = false;
  stepsLeft  = 0;
  digitalWrite(M1_RELAY, LOW);
  digitalWrite(M2_RELAY, LOW);
  Serial.println(F("Stopped"));
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_STEP,  OUTPUT);
  pinMode(M1_DIR,   OUTPUT);
  pinMode(M1_RELAY, OUTPUT);
  pinMode(M2_STEP,  OUTPUT);
  pinMode(M2_DIR,   OUTPUT);
  pinMode(M2_RELAY, OUTPUT);

  // Relays off at startup
  digitalWrite(M1_RELAY, LOW);
  digitalWrite(M2_RELAY, LOW);

  Serial.println(F("linear2 ready. Commands: raise, lower, stop"));
}

void startMove(bool raising) {
  if (running) { Serial.println(F("Already running")); return; }
  digitalWrite(M1_DIR, raising ? LOW  : HIGH);
  digitalWrite(M2_DIR, raising ? HIGH : LOW);
  digitalWrite(M1_RELAY, HIGH);
  digitalWrite(M2_RELAY, HIGH);
  stepsLeft = RAISE_STEPS;
  lastStep  = micros();
  running   = true;
  Serial.print(raising ? F("Raising: ") : F("Lowering: "));
  Serial.print(RAISE_STEPS);
  Serial.println(F(" steps each motor"));
}

void handleCommand(const char *cmd) {
  if      (strcmp(cmd, "raise") == 0) startMove(true);
  else if (strcmp(cmd, "lower") == 0) startMove(false);
  else if (strcmp(cmd, "stop")  == 0) stopMotors();
  else Serial.println(F("Unknown cmd. Use: raise, lower, stop"));
}

void loop() {
  // Serial command reader
  static char    buf[32];
  static uint8_t pos = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos > 0) { buf[pos] = '\0'; handleCommand(buf); pos = 0; }
    } else if (pos < sizeof(buf) - 1) {
      buf[pos++] = c;
    }
  }

  // Non-blocking stepping
  if (running && stepsLeft > 0) {
    unsigned long now = micros();
    if (now - lastStep >= INTERVAL_US) {
      lastStep = now;
      digitalWrite(M1_STEP, HIGH);
      digitalWrite(M2_STEP, HIGH);
      delayMicroseconds(PULSE_US);
      digitalWrite(M1_STEP, LOW);
      digitalWrite(M2_STEP, LOW);
      if (--stepsLeft == 0) stopMotors();
    }
  }
}
