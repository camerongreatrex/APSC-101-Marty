#include <AFMotor.h>  // motor shield support library so pumps can run
#include <Wire.h>     // wire library required by the motor shield firmware

const byte STOP_BUTTON_PIN = A14;   // wired stop button that pauses everything
const byte START_BUTTON_PIN = A15;  // wired start button that resumes the cycle

// Initialize motors and pumps
AF_DCMotor diaphragmPumpMotor(2);
AF_DCMotor peristalticPumpMotor(1);
AF_DCMotor impellerMotor(4);
AF_DCMotor pressMotor(3);

const byte RELAY_PIN = 53;   // relay that toggles the submersible pump
const byte STEP_COUNT = 17;  // total number of steps in the system (including turbidity)

//========Turbidity=========//
unsigned long currentMillis = 0;   // Stores the the current time
unsigned long previousMillis = 0;  // Stores the last time the serial monitor was updated
const unsigned long interval = 1000;
int seconds = 0;                 // Counter for seconds
float tSensor1Adj = 4.1 / 3.98;  // A scaling factor to adjust the current voltage readings to the expected reading of 4.1v.
float tSensor2Adj = 4.1 / 4.70;  // A scaling factor for sensor 2
bool turbidityLoggingActive = false;
bool turbiditySessionIsFinal = false;

const unsigned long STEP_STATUS_INTERVAL = 1000;  // how often to print step + elapsed info
unsigned long lastStepStatusMillis = 0;

// Durations for each numbered step (milliseconds) listed in execution order constant (const) and can't be negative to allow greater values (unsigned)
const unsigned long stepTime[STEP_COUNT] = {
  60000,  // Step 0  - turbidity readings at the start of the run
  30000,  // Step 1  - diaphragm pump runs // s/b 29.55
  0,      // Step 2  - diaphragm pump releases
  2000,   // Step 3  - wait so fluid settles
  53000,  // Step 4  - peristaltic pump runs // s/b 51.35
  0,      // Step 5  - peristaltic pump releases
  10000,   // Step 6  - wait before impeller runs to let alum settle
  10000,  // Step 7  - impeller fast mix
  30000,  // Step 8  - impeller slow mix
  0,      // Step 9  - impeller releases
  2000,   // Step 10  - wait so flocs can drop
  40000,  // Step 11 - press motor runs
  0,      // Step 12 - press motor releases
  2000,   // Step 13 - wait before relay action
  12000,  // Step 14 - relay turns on // s/b 11.25s REDUNDENT B/C BUTTON CODE IS DIFF FOR RELAY SO CHANGE CODE IN SWITCH
  0,      // Step 15 - relay turns off
  60000   // Step 16 - turbidity readings at the end of the run
};

byte currentStepIndex = 255;          // 255 means idle before the first run
bool processRunning = false;          // true while the drink sequence advances
unsigned long stepStartTime = 0;      // timestamp when the current step began
unsigned long pausedElapsedTime = 0;  // time already spent in a paused step

// NEW: variables needed for edge-trigger button behavior
bool stopButtonPreviouslyPressed = false;
bool startButtonPreviouslyPressed = false;

void startSystem();        // starts or resumes the drink sequence
void stopSystem();         // pauses the sequence and releases hardware
void updateProcess();      // checks timers and advances steps
void enterStep(byte idx);  // sets the timer for a new step
void applyStep(byte idx);  // drives the hardware for a specific step
void stopAllActuators();   // turns every motor and relay off
void handleTurbidityLogging();
void startTurbidityLogging(bool finalSession);
void stopTurbidityLogging();
void printStepStatus();
float voltageToNTU(float voltageReading);

void setup() {
  Serial.begin(9600);  // open the USB serial log port
  Serial.println("Hi! I'm MARTY, hope you're thirsty");

  // Configure buttons as pull-ups so they read LOW when pressed.
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  // Prep the relay output so the accessory stays off until needed.
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Begin with every actuator released for safety.
  stopAllActuators();
}

void loop() {

  // === NEW EDGE-TRIGGER BUTTON LOGIC ===
  bool stopPressedNow = (digitalRead(STOP_BUTTON_PIN) == LOW);
  bool startPressedNow = (digitalRead(START_BUTTON_PIN) == LOW);

  if (stopPressedNow && !stopButtonPreviouslyPressed) {
    stopSystem();  // stop only once per press
  } else if (startPressedNow && !startButtonPreviouslyPressed) {
    startSystem();  // start only once per press
  }

  // update stored states
  stopButtonPreviouslyPressed = stopPressedNow;
  startButtonPreviouslyPressed = startPressedNow;

  updateProcess();  // continue
  printStepStatus();
  handleTurbidityLogging();
}


// startSystem starts or resumes the drink cycle at the stored step index.
void startSystem() {
  if (currentStepIndex >= STEP_COUNT) {
    currentStepIndex = 0;   // restart from the first step
    pausedElapsedTime = 0;  // clear any stale pause timing
  }

  if (!processRunning) {
    processRunning = true;  // flag indicates timing should advance
    if (pausedElapsedTime == 0) {
      if (currentStepIndex == 255) {
        currentStepIndex = 0;  // move out of idle so first step runs
      }
      Serial.println("System starting.");
      enterStep(currentStepIndex);  // begin step timer and hardware actions
    } else {
      stepStartTime = millis() - pausedElapsedTime;
      pausedElapsedTime = 0;
      applyStep(currentStepIndex);
      Serial.println("System resuming.");
    }
  }
}

void stopSystem() {
  if (processRunning) {
    pausedElapsedTime = millis() - stepStartTime;
  }
  processRunning = false;
  stopAllActuators();
  Serial.println("System stopped.");
}

void updateProcess() {
  if (!processRunning || currentStepIndex >= STEP_COUNT) {
    return;
  }

  unsigned long duration = stepTime[currentStepIndex];

  if (duration == 0 || millis() - stepStartTime >= duration) {
    currentStepIndex++;
    if (currentStepIndex >= STEP_COUNT) {
      processRunning = false;
      stopAllActuators();
      stopTurbidityLogging();
      Serial.println("All done! Drink up :)");
    } else {
      enterStep(currentStepIndex);
    }
  }
}

void enterStep(byte idx) {
  currentStepIndex = idx;
  stepStartTime = millis();
  pausedElapsedTime = 0;
  if (idx == 0) {
    startTurbidityLogging(false);
  } else if (idx == 16) {
    startTurbidityLogging(true);
  } else {
    stopTurbidityLogging();
  }
  lastStepStatusMillis = millis() - STEP_STATUS_INTERVAL;
  applyStep(idx);
}

void applyStep(byte idx) {
  switch (idx) {
    case 0:
      Serial.println("Initial turbidity readings running.");
      break;
    case 1:
      diaphragmPumpMotor.setSpeed(255);
      diaphragmPumpMotor.run(FORWARD);
      Serial.println("Diaphragm pump on.");
      break;
    case 2:
      diaphragmPumpMotor.run(RELEASE);
      break;
    case 4:
      peristalticPumpMotor.setSpeed(255);
      peristalticPumpMotor.run(FORWARD);
      Serial.println("Peri pump on.");
      break;
    case 5:
      peristalticPumpMotor.run(RELEASE);
      break;
    case 7:
      impellerMotor.setSpeed(180);
      impellerMotor.run(FORWARD);
      Serial.println("Imp fast.");
      break;
    case 8:
      impellerMotor.setSpeed(100);
      impellerMotor.run(FORWARD);
      Serial.println("Imp slow.");
      break;
    case 9:
      impellerMotor.run(RELEASE);
      break;
    case 11:
      pressMotor.setSpeed(180);
      pressMotor.run(FORWARD);
      Serial.println("Press motor on.");
      break;
    case 12:
      pressMotor.run(RELEASE);
      break;
    case 14:
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay on.");
      // workaround relay only on when button is held
      delay(12000); // REAL time for relay to run (will not stop for button press here)
      break;
    case 15:
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay off.");
      break;
    case 16:
      Serial.println("Final turbidity readings running.");
      break;
    default:
      stopAllActuators();
      break;
  }
}

void stopAllActuators() {
  diaphragmPumpMotor.run(RELEASE);
  peristalticPumpMotor.run(RELEASE);
  impellerMotor.run(RELEASE);
  pressMotor.run(RELEASE);
  digitalWrite(RELAY_PIN, LOW);
}

void handleTurbidityLogging() {
  if (!turbidityLoggingActive) {
    return;
  }

  if (!processRunning) {
    previousMillis = millis();
    return;
  }

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    seconds++;

    int sensor1 = analogRead(A0);// read the input on analog pin 0
    float voltage1 = sensor1 * (5.0 / 1024.0) * tSensor1Adj; // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
    int sensor2 = analogRead(A1);
    float voltage2 = sensor2 * (5.0 / 1024.0) * tSensor2Adj;

    float turbidity1 = voltageToNTU(voltage1);
    float turbidity2 = voltageToNTU(voltage2);

    Serial.print(turbiditySessionIsFinal ? "Final" : "Initial");
    Serial.print(" turbidity s=");
    Serial.print(seconds);
    Serial.print(" | Sensor 1: ");
    Serial.print(turbidity1);
    Serial.print(" NTU | Sensor 2: ");
    Serial.print(turbidity2);
    Serial.println(" NTU");
  }
}

void startTurbidityLogging(bool finalSession) {
  turbidityLoggingActive = true;
  turbiditySessionIsFinal = finalSession;
  seconds = 0;
  previousMillis = millis();
  Serial.println(finalSession ? "Starting final turbidity readings." : "Starting initial turbidity readings.");
}

void stopTurbidityLogging() {
  turbidityLoggingActive = false;
}

void printStepStatus() {
  if (!processRunning || currentStepIndex >= STEP_COUNT) {
    return;
  }

  unsigned long now = millis();
  if (now - lastStepStatusMillis >= STEP_STATUS_INTERVAL) {
    lastStepStatusMillis = now;
    unsigned long elapsed = now - stepStartTime;
    Serial.print("Step ");
    Serial.print(currentStepIndex);
    Serial.print(" | Elapsed (ms): ");
    Serial.println(elapsed);
  }
}

float voltageToNTU(float voltageReading) {
  return (-1120.4 * voltageReading * voltageReading) + (5742.3 * voltageReading) - 4352.9;
}
