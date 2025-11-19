#include <AFMotor.h>  // motor shield support library so pumps can run
#include <Wire.h>     // wire library required by the motor shield firmware

const byte RELAY_PIN = 53;   // relay that toggles the submersible pump
const byte STEP_COUNT = 15;  // total number of steps in the system

// Initialize motors and pumps
AF_DCMotor diaphragmPumpMotor(2);    // diaphragm pump motor on M2
AF_DCMotor peristalticPumpMotor(1);  // peristaltic pump motor on M1
AF_DCMotor impellerMotor(4);         // impeller motor on M4
AF_DCMotor pressMotor(3);            // press motor on M3

const byte STOP_BUTTON_PIN = A14;   // wired stop button that pauses everything
const byte START_BUTTON_PIN = A15;  // wired start button that resumes the cycle

// timing variables for turbidity
unsigned long currentMillis = 0;      // stores current time for turbidity interval
unsigned long previousMillis = 0;     // last time turbidity was sampled
const unsigned long interval = 1000;  // 1 second sampling period for turbidity
// voltage calibration factors
float tSensor1Adj = 4.1 / 4.29;  // calibration multiplier for sensor 1
float tSensor2Adj = 4.1 / 4.70;  // calibration multiplier for sensor 2

// step tracking variables
byte currentStepIndex = 255;          // 255 means system has not started yet
bool processRunning = false;          // tracks if system is executing steps
unsigned long stepStartTime = 0;      // timestamp when current step began
unsigned long pausedElapsedTime = 0;  // time stored when paused

// tracks total run time excluding stops
unsigned long totalRunTime = 0;       // accumulated run time
unsigned long runtimeStartStamp = 0;  // timestamp when runtime resumed
bool runtimeStarted = false;          // ensures runtime only starts once

// durations for each numbered step (milliseconds) listed in execution order
const unsigned long stepTime[STEP_COUNT] = {
  27000,   // Step 0  - diaphragm pump runs // s/b 29.55
  0,       // Step 1  - diaphragm pump releases
  2000,    // Step 2  - wait so fluid settles
  38000,   // Step 3  - peristaltic pump runs // s/b 51.35
  0,       // Step 4  - peristaltic pump releases
  2000,    // Step 5  - wait before impeller runs to let alum settle
  10000,   // Step 6  - impeller fast mix
  30000,   // Step 7  - impeller slow mix
  0,       // Step 8  - impeller releases
  120000,  // Step 9  - wait so flocs can drop (2 minutes)
  39000,   // Step 10 - press motor runs
  0,       // Step 11 - press motor releases
  2000,    // Step 12 - wait before relay action
  23000,   // Step 13 - relay turns ON (workaround because relay was not holding state)
  0        // Step 14 - relay turns OFF
};

// variables for button behavior
bool stopButtonPreviouslyPressed = false;   // stores previous stop button state
bool startButtonPreviouslyPressed = false;  // stores previous start button state

// call all functions (void b/c no need to return anything)
void startSystem();        // starts or resumes the drink sequence
void stopSystem();         // pauses the sequence and releases hardware
void updateProcess();      // checks timers and advances steps
void enterStep(byte idx);  // sets timer + activates hardware for step
void applyStep(byte idx);  // drives hardware according to step number
void stopAllActuators();   // turns every motor, pump, and relay off

//=======TURBIDITY=======//
// measures turbidity every second for 60 seconds and prints voltages + NTU results.
void turbidityMeasurement() {
  Serial.println("------ TURBIDITY MEASUREMENT (60s) ------");
  Serial.println("Time\tV1\tV2\tNTU1\tNTU2");

  unsigned long turbidityStart = millis();  // mark start time of 60s window
  int seconds = 0;                          // local second counter

  while (millis() - turbidityStart < 60000) {      // sample for 60 seconds
    int s1 = analogRead(A0);                       // read raw sensor 1 value
    float v1 = s1 * (5.0 / 1024.0) * tSensor1Adj;  // convert to voltage with calibration
    int s2 = analogRead(A1);                       // read raw sensor 2 value
    float v2 = s2 * (5.0 / 1024.0) * tSensor2Adj;  // convert to voltage with calibration

    // apply turbidity to NTU formula to both sensors
    float NTU1 = -1120.4 * v1 * v1 + 5742.3 * v1 - 4352.9;
    float NTU2 = -1120.4 * v2 * v2 + 5742.3 * v2 - 4352.9;

    Serial.print(seconds);
    Serial.print("\t");
    Serial.print(v1);
    Serial.print("\t");
    Serial.print(v2);
    Serial.print("\t");
    Serial.print(NTU1);
    Serial.print("\t");
    Serial.println(NTU2);

    seconds++;
    delay(1000);  // wait exactly 1 second between readings
  }
  Serial.println("------ TURBIDITY DONE ------");
}

//=======SETUP=======//
// runs once when the Arduino turns on and sets up all pins, motors, serial output, and makes sure everything starts in the off position
void setup() {
  Serial.begin(9600);  // start serial monitor
  Serial.println("Hi! I'm MARTY, hope you're thirsty");

  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);   // configure stop button with pull-up
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);  // configure start button with pull-up

  pinMode(RELAY_PIN, OUTPUT);    // relay output setup
  digitalWrite(RELAY_PIN, LOW);  // ensure relay off on boot

  stopAllActuators();  // ensure motors/relay off at startup
}

//=======SYSTEM LOOP=======//
// constantly checks the start and stop buttons and runs the main system logic that controls when steps run or pause
void loop() {
  bool stopPressedNow = (digitalRead(STOP_BUTTON_PIN) == LOW);    // detect stop button press
  bool startPressedNow = (digitalRead(START_BUTTON_PIN) == LOW);  // detect start button press

  // if stop button is pressed now and wasn’t pressed before, stop the system
  if (stopPressedNow && !stopButtonPreviouslyPressed) {
    stopSystem();  //triggers when button becomes pressed
  }

  // if the start button is pressed and wasn't previously pressed, start the system
  else if (startPressedNow && !startButtonPreviouslyPressed) {
    // start runtime on first start press
    if (!runtimeStarted) {
      runtimeStarted = true;
      runtimeStartStamp = millis();  // mark runtime beginning
    }
    // resume runtime if restarting after pause
    else if (runtimeStarted && !processRunning) {
      runtimeStartStamp = millis();
    }
    // run turbidity BEFORE beginning the first step
    if (currentStepIndex == 255) {
      turbidityMeasurement();
    }
    startSystem();  // start or resume the system
  }

  // update stored button states for edge detection
  stopButtonPreviouslyPressed = stopPressedNow;
  startButtonPreviouslyPressed = startPressedNow;

  updateProcess();  // handle step timing + transitions
}

//=======START SYSTEM=======//
// starts the whole filtering sequence or resumes it from where it stopped
void startSystem() {
  // reset index if beyond last step
  if (currentStepIndex >= STEP_COUNT) {
    currentStepIndex = 0;
    pausedElapsedTime = 0;  // clear pause time so step restarts cleanly
  }

  // only start if currently stopped
  if (!processRunning) {
    processRunning = true;         // mark system as running
    if (pausedElapsedTime == 0) {  // system was fresh (first start)

      if (currentStepIndex == 255) currentStepIndex = 0;  // go to step 0
      Serial.println("System starting.");
      enterStep(currentStepIndex);                   // begin first step
    } else {                                         // system was paused, resume same step
      stepStartTime = millis() - pausedElapsedTime;  // rebuild elapsed time
      pausedElapsedTime = 0;
      applyStep(currentStepIndex);  // reapply motor state
      Serial.println("System resuming.");
    }
  }
}

//=======STOP SYSTEM=======//
// pauses the current step, records how long it was running, and turns everything off
void stopSystem() {
  if (processRunning) {
    pausedElapsedTime = millis() - stepStartTime;  // store how long step was running
    totalRunTime += millis() - runtimeStartStamp;  // accumulate runtime
  }

  processRunning = false;  // mark system as paused
  stopAllActuators();      // ensure all motors/relay stop immediately
  Serial.println("System stopped.");
}

//=======UPDATE SERIAL MONITOR + MANAGE STEPS=======//
// checks how long the current step has been running and moves to the next step when time is up
void updateProcess() {
  if (!processRunning || currentStepIndex >= STEP_COUNT) return;  // no action if paused

  unsigned long duration = stepTime[currentStepIndex];  // fetch duration for current step

  // Print step + elapsed time once per second
  static unsigned long lastStepPrint = 0;  // store last print time
  if (millis() - lastStepPrint >= 1000) {  // print every 1 second
    lastStepPrint = millis();
    Serial.print("Step ");
    Serial.print(currentStepIndex);
    Serial.print("   Elapsed(s): ");
    Serial.println((millis() - stepStartTime) / 1000);
  }

  // check if time for the current step has finished
  if (duration == 0 || millis() - stepStartTime >= duration) {
    currentStepIndex++;  // move to next step

    // check if sequence is complete
    if (currentStepIndex >= STEP_COUNT) {
      processRunning = false;                        // mark as finished
      stopAllActuators();                            // ensure all hardware is off
      totalRunTime += millis() - runtimeStartStamp;  // add remaining runtime
      turbidityMeasurement();                        // take final turbidity reading

      // print total runtime at end of process
      Serial.print("Total runtime excluding turbidity: ");
      Serial.print(totalRunTime / 1000);
      Serial.println(" seconds");

      Serial.println("Marty is all done! Drink up :)");  // Marty sign-off
    } else {
      enterStep(currentStepIndex);  // start the next step
    }
  }
}

//=======ENTER EACH STEP (COMPUTER)=======//
// sets the new active step, resets its timer, and applies the hardware actions for that step
void enterStep(byte idx) {
  currentStepIndex = idx;    // record which step we're moving into
  stepStartTime = millis();  // mark start time for the step
  pausedElapsedTime = 0;     // clear pause tracking
  applyStep(idx);            // activate hardware for the step
}

//=======APPLY EACH STEP (HARDWARE)=======//
// turns on/off the correct pump, motor, or relay depending on which step number is running
void applyStep(byte idx) {
  switch (idx) {
    case 0:
      diaphragmPumpMotor.setSpeed(255);  // max diaphragm pump speed
      diaphragmPumpMotor.run(FORWARD);   // pump water in
      Serial.println("Diaphragm pump on.");
      break;
    case 1:
      diaphragmPumpMotor.run(RELEASE);  // stop diaphragm pump
      break;
    case 3:
      peristalticPumpMotor.setSpeed(255);  // max peri pump speed
      peristalticPumpMotor.run(FORWARD);   // push alum through
      Serial.println("Peristaltic pump on.");
      break;
    case 4:
      peristalticPumpMotor.run(RELEASE);  // stop peri pump
      break;
    case 6:
      impellerMotor.setSpeed(110);  // mixing speed fast
      impellerMotor.run(FORWARD);
      Serial.println("Impeller fast.");
      break;
    case 7:
      impellerMotor.setSpeed(70);  // mixing speed slow
      impellerMotor.run(FORWARD);
      Serial.println("Impeller slow.");
      break;
    case 8:
      impellerMotor.run(RELEASE);  // stop impeller
      break;
    case 10:
      pressMotor.setSpeed(180);  // set speed for pressing
      pressMotor.run(FORWARD);   // move press down
      Serial.println("Press motor on.");
      break;
    case 11:
      pressMotor.run(RELEASE);  // stop press motor
      break;
    case 13:
      digitalWrite(RELAY_PIN, HIGH);  // turn relay ON
      Serial.println("Relay on.");
      delay(14000);  // hold for required time (relay workaround)
      break;
    case 14:
      digitalWrite(RELAY_PIN, LOW);  // turn relay OFF
      Serial.println("Relay off.");
      break;
    default:
      stopAllActuators();  // safety stop for unexpected index
      break;
  }
}

//=======EMERGENCY STOP=======//
// immediately shuts off every motor and the relay for safety
void stopAllActuators() {
  diaphragmPumpMotor.run(RELEASE);    // stop diaphragm pump
  peristalticPumpMotor.run(RELEASE);  // stop peri pump
  impellerMotor.run(RELEASE);         // stop impeller
  pressMotor.run(RELEASE);            // stop press motor
  digitalWrite(RELAY_PIN, LOW);       // ensure relay off
}
