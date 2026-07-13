// linear2.cpp — Two NEMA 17 stepper motors with relay control
//
// Motor 1: STEP=2  DIR=3  RELAY=5
// Motor 2: STEP=6  DIR=7  RELAY=4
//
// Relay modules are active-HIGH (HIGH = energised/on, LOW = off).
//
// Commands:
//   m1on   — relay on, motor 1 runs
//   m1off  — motor 1 stops, relay off
//   m2on   — relay on, motor 2 runs
//   m2off  — motor 2 stops, relay off
//   allon  — both motors on
//   alloff — both motors off
//   status — print current state

const int M1_STEP  = 3;
const int M1_DIR   = 2;
const int M1_RELAY = 5;

const int M2_STEP  = 7;
const int M2_DIR   = 6;
const int M2_RELAY = 4;

const unsigned long INTERVAL_US = 1000;  // µs between steps (~1000 steps/sec)
const unsigned int  PULSE_US    = 10;    // step pulse width µs

bool m1Running = false;
bool m2Running = false;

unsigned long m1LastStep = 0;
unsigned long m2LastStep = 0;

// ---- helpers ----

void motorOn(int relayPin, bool &running, unsigned long &lastStep) {
  digitalWrite(relayPin, HIGH);  // energise relay
  running  = true;
  lastStep = micros();
}

void motorOff(int relayPin, bool &running) {
  running = false;
  digitalWrite(relayPin, LOW);   // de-energise relay
}

// ---- setup ----

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

  // Direction (fixed for now)
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  Serial.println(F("linear2 ready. Commands: m1on m1off m2on m2off allon alloff status"));
}

// ---- command handler ----

void handleCommand(const char *cmd) {
  if (strcmp(cmd, "m1on") == 0) {
    motorOn(M1_RELAY, m1Running, m1LastStep);
    Serial.println(F("Motor 1 ON"));
  }
  else if (strcmp(cmd, "m1off") == 0) {
    motorOff(M1_RELAY, m1Running);
    Serial.println(F("Motor 1 OFF"));
  }
  else if (strcmp(cmd, "m2on") == 0) {
    motorOn(M2_RELAY, m2Running, m2LastStep);
    Serial.println(F("Motor 2 ON"));
  }
  else if (strcmp(cmd, "m2off") == 0) {
    motorOff(M2_RELAY, m2Running);
    Serial.println(F("Motor 2 OFF"));
  }
  else if (strcmp(cmd, "allon") == 0) {
    motorOn(M1_RELAY, m1Running, m1LastStep);
    motorOn(M2_RELAY, m2Running, m2LastStep);
    Serial.println(F("Both motors ON"));
  }
  else if (strcmp(cmd, "alloff") == 0) {
    motorOff(M1_RELAY, m1Running);
    motorOff(M2_RELAY, m2Running);
    Serial.println(F("Both motors OFF"));
  }
  else if (strcmp(cmd, "status") == 0) {
    Serial.print(F("Motor 1: ")); Serial.println(m1Running ? F("ON") : F("OFF"));
    Serial.print(F("Motor 2: ")); Serial.println(m2Running ? F("ON") : F("OFF"));
  }
  else {
    Serial.println(F("Unknown cmd. Use: m1on m1off m2on m2off allon alloff status"));
  }
}

// ---- loop ----

void loop() {
  // Serial command reader
  static char buf[32];
  static uint8_t pos = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos > 0) {
        buf[pos] = '\0';
        handleCommand(buf);
        pos = 0;
      }
    } else if (pos < sizeof(buf) - 1) {
      buf[pos++] = c;
    }
  }

  // Non-blocking stepping
  unsigned long now = micros();

  if (m1Running && (now - m1LastStep >= INTERVAL_US)) {
    m1LastStep = now;
    digitalWrite(M1_STEP, HIGH);
    delayMicroseconds(PULSE_US);
    digitalWrite(M1_STEP, LOW);
  }

  if (m2Running && (now - m2LastStep >= INTERVAL_US)) {
    m2LastStep = now;
    digitalWrite(M2_STEP, HIGH);
    delayMicroseconds(PULSE_US);
    digitalWrite(M2_STEP, LOW);
  }
}
