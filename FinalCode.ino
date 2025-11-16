#include <AFMotor.h>
#include <Wire.h>

#define START_BUTTON A14
#define STOP_BUTTON A15

AF_DCMotor diaphragmPump(2);
AF_DCMotor peristalticPump(1);
AF_DCMotor impeller(4);
AF_DCMotor pressMotor(3);
AF_DCMotor spareMotor(8);

const byte RELAY_PIN = 53;

bool systemRunning = false;
bool lastStartState = HIGH;
bool lastStopState = HIGH;
unsigned long stepStartTime = 0;
int currentStep = -1;

void stopEverything() {
  diaphragmPump.run(RELEASE);
  peristalticPump.run(RELEASE);
  impeller.run(RELEASE);
  pressMotor.run(RELEASE);
  spareMotor.run(RELEASE);
  digitalWrite(RELAY_PIN, LOW);
}

unsigned long stepDuration(int step) {
  if (step == 0) return 10000;   // Diaphragm pump
  if (step == 1) return 2000;    // Pause
  if (step == 2) return 5000;    // Peristaltic pump
  if (step == 3) return 2000;    // Pause
  if (step == 4) return 10000;   // Impeller fast
  if (step == 5) return 10000;   // Impeller slow
  if (step == 6) return 2000;    // Pause
  if (step == 7) return 20000;   // Press motor
  if (step == 8) return 2000;    // Pause
  if (step == 9) return 8000;    // Relay pump
  return 0;
}

void startStepActions(int step) {
  stepStartTime = millis();
  if (step == 0) {
    Serial.println("Diaphragm pump");
    diaphragmPump.setSpeed(255);
    diaphragmPump.run(FORWARD);
  } else if (step == 1) {
    Serial.println("Pause");
    stopEverything();
  } else if (step == 2) {
    Serial.println("Peristaltic pump");
    peristalticPump.setSpeed(255);
    peristalticPump.run(FORWARD);
  } else if (step == 3) {
    Serial.println("Pause");
    stopEverything();
  } else if (step == 4) {
    Serial.println("Impeller fast");
    impeller.setSpeed(180);
    impeller.run(FORWARD);
  } else if (step == 5) {
    Serial.println("Impeller slow");
    impeller.setSpeed(100);
    impeller.run(FORWARD);
  } else if (step == 6) {
    Serial.println("Pause");
    stopEverything();
  } else if (step == 7) {
    Serial.println("Press motor");
    pressMotor.setSpeed(180);
    pressMotor.run(FORWARD);
  } else if (step == 8) {
    Serial.println("Pause");
    stopEverything();
  } else if (step == 9) {
    Serial.println("Horizontal sub pump");
    digitalWrite(RELAY_PIN, HIGH);
  }
}

void stopStepActions(int step) {
  if (step == 0) diaphragmPump.run(RELEASE);
  else if (step == 2) peristalticPump.run(RELEASE);
  else if (step == 4 || step == 5) impeller.run(RELEASE);
  else if (step == 7) pressMotor.run(RELEASE);
  else if (step == 9) digitalWrite(RELAY_PIN, LOW);
}

void beginNextStep() {
  currentStep++;
  if (currentStep > 9) {
    Serial.println("All done! Drink up :)");
    stopEverything();
    systemRunning = false;
    currentStep = -1;
    return;
  }
  startStepActions(currentStep);
}

void updateProcess() {
  if (!systemRunning) return;
  unsigned long elapsed = millis() - stepStartTime;
  if (elapsed >= stepDuration(currentStep)) {
    stopStepActions(currentStep);
    beginNextStep();
  }
}

void startProcess() {
  if (systemRunning) return;
  Serial.println("Starting MARTY's drink cycle.");
  systemRunning = true;
  currentStep = -1;
  beginNextStep();
}

void stopProcess() {
  if (!systemRunning) return;
  Serial.println("Emergency stop! Releasing all hardware.");
  systemRunning = false;
  currentStep = -1;
  stopEverything();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Hi! I'm MARTY, hope you're thirsty.");
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  stopEverything();
  Serial.println("Press START to begin. Press STOP anytime to halt.");
}

void loop() {
  bool startReading = digitalRead(START_BUTTON);
  bool stopReading = digitalRead(STOP_BUTTON);

  if (startReading == LOW && lastStartState == HIGH) startProcess();
  if (stopReading == LOW && lastStopState == HIGH) stopProcess();

  lastStartState = startReading;
  lastStopState = stopReading;

  updateProcess();
}
