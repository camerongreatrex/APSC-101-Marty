#include <AFMotor.h>
#include <Wire.h>

// Marty Combined v2 with start/stop buttons
#define STOP_PIN A14
#define START_PIN A15

AF_DCMotor Dpump(2);
AF_DCMotor Ppump(1);
AF_DCMotor Impmotor(4);
AF_DCMotor Pressmotor(3);

const byte RELAY_PIN = 53;
const byte STEP_COUNT = 15;

// Durations for each step (ms)
const unsigned long stepTime[STEP_COUNT] = {
  10000, // diaphragm run
  0,     // diaphragm release
  2000,  // wait
  5000,  // peri run
  0,     // peri release
  2000,  // wait
  10000, // imp fast
  10000, // imp slow
  0,     // imp release
  2000,  // wait
  20000, // press run
  0,     // press release
  2000,  // wait
  8000,  // relay on
  0      // relay off
};

byte stepIndex = 255;      // current step (255 = idle)
bool running = false;      // true when sequence is active
unsigned long stepStart = 0;
unsigned long pauseOffset = 0;

void startSystem();       // start or resume sequence
void stopSystem();        // pause sequence
void updateProcess();     // timing engine
void enterStep(byte idx); // start a step
void applyStep(byte idx); // drive hardware for a step
void stopAllActuators();  // release everything

void setup() {
  Serial.begin(9600);
  Serial.println("Hi! I'm MARTY, hope you're thirsty");

  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  stopAllActuators();
}

void loop() {
  if (digitalRead(STOP_PIN) == LOW) {
    stopSystem();
  } else if (digitalRead(START_PIN) == LOW) {
    startSystem();
  }

  updateProcess();
}

void startSystem() { // starts or resumes the drink cycle
  if (stepIndex >= STEP_COUNT) {
    stepIndex = 0;
    pauseOffset = 0;
  }

  if (!running) {
    running = true;
    if (pauseOffset == 0) {
      if (stepIndex == 255) {
        stepIndex = 0;
      }
      Serial.println("System starting.");
      enterStep(stepIndex);
    } else {
      stepStart = millis() - pauseOffset;
      pauseOffset = 0;
      applyStep(stepIndex);
      Serial.println("System resuming.");
    }
  }
}

void stopSystem() { // pauses the cycle and releases hardware
  if (running) {
    pauseOffset = millis() - stepStart;
  }
  running = false;
  stopAllActuators();
  Serial.println("System stopped.");
}

void updateProcess() { // advances steps when time expires
  if (!running || stepIndex >= STEP_COUNT) {
    return;
  }

  unsigned long duration = stepTime[stepIndex];
  if (duration == 0 || millis() - stepStart >= duration) {
    stepIndex++;
    if (stepIndex >= STEP_COUNT) {
      running = false;
      stopAllActuators();
      Serial.println("All done! Drink up :)");
    } else {
      enterStep(stepIndex);
    }
  }
}

void enterStep(byte idx) { // initializes a step timer
  stepIndex = idx;
  stepStart = millis();
  pauseOffset = 0;
  applyStep(idx);
}

void applyStep(byte idx) { // performs the hardware action for a step
  switch (idx) {
    case 0:
      Dpump.setSpeed(255);
      Dpump.run(FORWARD);
      Serial.println("Diaphragm pump on.");
      break;
    case 1:
      Dpump.run(RELEASE);
      break;
    case 3:
      Ppump.setSpeed(255);
      Ppump.run(FORWARD);
      Serial.println("Peri pump on.");
      break;
    case 4:
      Ppump.run(RELEASE);
      break;
    case 6:
      Impmotor.setSpeed(180);
      Impmotor.run(FORWARD);
      Serial.println("Imp fast.");
      break;
    case 7:
      Impmotor.setSpeed(100);
      Impmotor.run(FORWARD);
      Serial.println("Imp slow.");
      break;
    case 8:
      Impmotor.run(RELEASE);
      break;
    case 10:
      Pressmotor.setSpeed(180);
      Pressmotor.run(FORWARD);
      Serial.println("Press motor on.");
      break;
    case 11:
      Pressmotor.run(RELEASE);
      break;
    case 13:
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay on.");
      break;
    case 14:
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay off.");
      break;
    default:
      stopAllActuators();
      break;
  }
}

void stopAllActuators() { // releases pumps, motors, and relay
  Dpump.run(RELEASE);
  Ppump.run(RELEASE);
  Impmotor.run(RELEASE);
  Pressmotor.run(RELEASE);
  digitalWrite(RELAY_PIN, LOW);
}
