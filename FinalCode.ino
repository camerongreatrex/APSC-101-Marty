#include <AFMotor.h>  // Motor shield control library
#include <Wire.h>      // Required by AFMotor even if I2C is unused here

// === Button wiring ===
// START button is soldered to analog pin A14 (digital pin 68 on Mega) and
// STOP button is on A15. Both use INPUT_PULLUP, so HIGH = idle, LOW = pressed.
#define START_BUTTON A14
#define STOP_BUTTON A15

// === Motor assignments on the Adafruit shield ===
// Motor numbers match the shield terminals so wiring is easy to trace later.
AF_DCMotor diaphragmPump(2);   // M2 terminal drives the diaphragm pump
AF_DCMotor peristalticPump(1); // M1 terminal drives the peristaltic pump
AF_DCMotor impeller(4);        // M4 terminal drives the impeller
AF_DCMotor pressMotor(3);      // M3 terminal drives the French press motor
AF_DCMotor spareMotor(8);      // Spare port kept here for future ideas

// === Relay for the horizontal sub pump ===
// Pin 53 is convenient on the Mega and keeps SPI pins free. Relay is active-HIGH.
const byte RELAY_PIN = 53;

// === Run/stop state tracking ===
bool systemRunning = false;  // True when the drink sequence is executing
bool lastStartState = HIGH;  // Previous raw reading from START button
bool lastStopState = HIGH;   // Previous raw reading from STOP button
unsigned long stepStartTime = 0;  // Time stamp when the current step began
int currentStep = -1;             // Index of the active step (-1 means idle)

// ----------------------------------------------------------------------------
// Utility helpers
// ----------------------------------------------------------------------------

// Releases every motor and makes sure the relay is off. Called at power-up,
// between steps that need a pause, and whenever STOP is pressed.
void stopEverything() {
  diaphragmPump.run(RELEASE);
  peristalticPump.run(RELEASE);
  impeller.run(RELEASE);
  pressMotor.run(RELEASE);
  spareMotor.run(RELEASE);
  digitalWrite(RELAY_PIN, LOW);
}

// Returns how long the numbered step should last (in milliseconds).
// Using simple if statements keeps the code beginner-friendly and matches
// the request to avoid advanced C++ features.
unsigned long stepDuration(int step) {
  if (step == 0) return 10000;   // 10 s diaphragm pump
  if (step == 1) return 2000;    // 2 s pause
  if (step == 2) return 5000;    // 5 s peristaltic pump
  if (step == 3) return 2000;    // 2 s pause
  if (step == 4) return 10000;   // 10 s impeller fast
  if (step == 5) return 10000;   // 10 s impeller slow
  if (step == 6) return 2000;    // 2 s pause
  if (step == 7) return 20000;   // 20 s French press motor
  if (step == 8) return 2000;    // 2 s pause
  if (step == 9) return 8000;    // 8 s relay-driven sub pump
  return 0;                      // Safety fallback (should never trigger)
}

// Starts the hardware associated with the specified step and prints a message
// so anyone watching Serial Monitor knows what MARTY is doing.
void startStepActions(int step) {
  stepStartTime = millis();  // Remember when this step kicked off

  if (step == 0) {
    Serial.println("Stage 1: Diaphragm pump ON");
    diaphragmPump.setSpeed(255);
    diaphragmPump.run(FORWARD);
  } else if (step == 1) {
    Serial.println("Stage 2: Pause before peristaltic pump");
    stopEverything();
  } else if (step == 2) {
    Serial.println("Stage 3: Peristaltic pump ON");
    peristalticPump.setSpeed(255);
    peristalticPump.run(FORWARD);
  } else if (step == 3) {
    Serial.println("Stage 4: Pause before impeller");
    stopEverything();
  } else if (step == 4) {
    Serial.println("Stage 5: Impeller FAST");
    impeller.setSpeed(180);
    impeller.run(FORWARD);
  } else if (step == 5) {
    Serial.println("Stage 6: Impeller SLOW");
    impeller.setSpeed(100);
    impeller.run(FORWARD);
  } else if (step == 6) {
    Serial.println("Stage 7: Pause before press motor");
    stopEverything();
  } else if (step == 7) {
    Serial.println("Stage 8: Press motor ON");
    pressMotor.setSpeed(180);
    pressMotor.run(FORWARD);
  } else if (step == 8) {
    Serial.println("Stage 9: Pause before sub pump");
    stopEverything();
  } else if (step == 9) {
    Serial.println("Stage 10: Horizontal sub pump ON");
    digitalWrite(RELAY_PIN, HIGH);  // Relay energizes the pump
  }
}

// Turns off only the hardware owned by the completed step. Pauses already
// call stopEverything(), so they fall through without any extra code.
void stopStepActions(int step) {
  if (step == 0) {
    diaphragmPump.run(RELEASE);
  } else if (step == 2) {
    peristalticPump.run(RELEASE);
  } else if (step == 4 || step == 5) {
    impeller.run(RELEASE);
  } else if (step == 7) {
    pressMotor.run(RELEASE);
  } else if (step == 9) {
    digitalWrite(RELAY_PIN, LOW);
  }
}

// Increments the step counter, runs the matching hardware, and gracefully
// exits when the sequence finishes.
void beginNextStep() {
  currentStep++;

  if (currentStep > 9) {
    Serial.println("All done! Drink up :)");
    stopEverything();
    systemRunning = false;
    currentStep = -1;  // Reset so START restarts from the top
    return;
  }

  startStepActions(currentStep);
}

// Checks whether the current step finished its time slice. If so, stop the
// hardware for that step and advance to the next one.
void updateProcess() {
  if (!systemRunning) {
    return;  // Nothing to do while idle
  }

  unsigned long elapsed = millis() - stepStartTime;
  if (elapsed >= stepDuration(currentStep)) {
    stopStepActions(currentStep);
    beginNextStep();
  }
}

// Called when START is pressed (button transitions from HIGH to LOW).
// Ignores repeated presses while already running.
void startProcess() {
  if (systemRunning) {
    return;
  }

  Serial.println("Starting MARTY's drink cycle.");
  systemRunning = true;
  currentStep = -1;   // Ensures beginNextStep() starts at step 0
  beginNextStep();
}

// Called when STOP is pressed. All motors shut down immediately and the
// sequence returns to idle so START can be pressed again later.
void stopProcess() {
  if (!systemRunning) {
    return;
  }

  Serial.println("Emergency stop! Releasing all hardware.");
  systemRunning = false;
  currentStep = -1;
  stopEverything();
}

// ----------------------------------------------------------------------------
// Arduino entry points
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Hi! I'm MARTY, hope you're thirsty.");

  pinMode(START_BUTTON, INPUT_PULLUP);  // Enable internal resistor on START
  pinMode(STOP_BUTTON, INPUT_PULLUP);   // Enable internal resistor on STOP
  pinMode(RELAY_PIN, OUTPUT);           // Prepare relay output pin

  stopEverything();  // Make sure everything is idle before the first START

  Serial.println("Press START to begin. Press STOP anytime to halt.");
}

void loop() {
  bool startReading = digitalRead(START_BUTTON);  // Current START state
  bool stopReading = digitalRead(STOP_BUTTON);    // Current STOP state

  // Look for button edges (HIGH -> LOW) because INPUT_PULLUP idles HIGH.
  if (startReading == LOW && lastStartState == HIGH) {
    startProcess();
  }
  if (stopReading == LOW && lastStopState == HIGH) {
    stopProcess();
  }

  lastStartState = startReading;  // Store readings for the next loop
  lastStopState = stopReading;

  updateProcess();  // Keep the sequence moving when running
}
