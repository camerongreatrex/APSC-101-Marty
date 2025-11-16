#include <AFMotor.h>  // motor shield support library so pumps can run
#include <Wire.h>      // wire library required by the motor shield firmware


const byte STOP_BUTTON_PIN = A14;  // wired stop button that pauses everything
const byte START_BUTTON_PIN = A15; // wired start button that resumes the cycle

// Initialize motors and pumps
AF_DCMotor diaphragmPumpMotor(2); 
AF_DCMotor peristalticPumpMotor(1);
AF_DCMotor impellerMotor(4);
AF_DCMotor pressMotor(3);

const byte RELAY_PIN = 53;     // relay that toggles the submersible pump
const byte STEP_COUNT = 15;    // total number of steps in the system

// Durations for each numbered step (milliseconds) listed in execution order
const unsigned long stepTime[STEP_COUNT] = {
  10000, // Step 0  - diaphragm pump runs
  0,     // Step 1  - diaphragm pump releases
  2000,  // Step 2  - wait so fluid settles
  5000,  // Step 3  - peristaltic pump runs
  0,     // Step 4  - peristaltic pump releases
  2000,  // Step 5  - wait before impeller work
  10000, // Step 6  - impeller fast mix
  10000, // Step 7  - impeller slow mix
  0,     // Step 8  - impeller releases
  2000,  // Step 9  - wait so foam can drop
  20000, // Step 10 - press motor runs
  0,     // Step 11 - press motor releases
  2000,  // Step 12 - wait before relay action
  8000,  // Step 13 - relay energizes
  0      // Step 14 - relay turns off
};

byte currentStepIndex = 255;        // 255 means idle before the first run
bool processRunning = false;        // true while the drink sequence advances
unsigned long stepStartTime = 0;    // timestamp when the current step began
unsigned long pausedElapsedTime = 0;// time already spent in a paused step

void startSystem();        // starts or resumes the drink sequence
void stopSystem();         // pauses the sequence and releases hardware
void updateProcess();      // checks timers and advances steps
void enterStep(byte idx);  // sets the timer for a new step
void applyStep(byte idx);  // drives the hardware for a specific step
void stopAllActuators();   // turns every motor and relay off

void setup() {
  Serial.begin(9600);                         // open the USB serial log port
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
  // Buttons are checked every pass; pressing Stop has priority.
  if (digitalRead(STOP_BUTTON_PIN) == LOW) {
    stopSystem();                        // halt immediately whenever Stop is LOW
  } else if (digitalRead(START_BUTTON_PIN) == LOW) {
    startSystem();                       // resume or begin when Start is LOW
  }

  // Advance the routine only when timing allows.
  updateProcess();                       // run timing logic continuously
}

// startSystem starts or resumes the drink cycle at the stored step index.
void startSystem() {
  if (currentStepIndex >= STEP_COUNT) {
    currentStepIndex = 0;                // restart from the first step
    pausedElapsedTime = 0;               // clear any stale pause timing
  }

  if (!processRunning) {
    processRunning = true;               // flag indicates timing should advance
    if (pausedElapsedTime == 0) {
      if (currentStepIndex == 255) {
        currentStepIndex = 0;            // move out of idle so first step runs
      }
      Serial.println("System starting.");
      enterStep(currentStepIndex);       // begin step timer and hardware actions
    } else {
      // Recreate the partially completed timer so the step finishes naturally.
      stepStartTime = millis() - pausedElapsedTime; // offset timer by work done
      pausedElapsedTime = 0;             // reset pause storage now that used
      applyStep(currentStepIndex);       // reapply outputs for the stored step
      Serial.println("System resuming.");
    }
  }
}

// stopSystem freezes the timer and drops power to every actuator
void stopSystem() {
  if (processRunning) {
    pausedElapsedTime = millis() - stepStartTime; // capture time spent so far
  }
  processRunning = false;                // prevent loop from advancing timers
  stopAllActuators();                    // guarantee everything is released
  Serial.println("System stopped.");
}

// updateProcess compares elapsed time to the configured duration and advances.
void updateProcess() {
  if (!processRunning || currentStepIndex >= STEP_COUNT) {
    return;                              // bail if paused or sequence finished
  }

  unsigned long duration = stepTime[currentStepIndex];
  // Duration of zero means "instant" steps such as motor releases.
  if (duration == 0 || millis() - stepStartTime >= duration) {
    currentStepIndex++;                  // move to the next entry in the table
    if (currentStepIndex >= STEP_COUNT) {
      processRunning = false;            // stay idle after the last step
      stopAllActuators();                // make sure nothing stays powered
      Serial.println("All done! Drink up :)");
    } else {
      enterStep(currentStepIndex);       // begin timing for the new step
    }
  }
}

// enterStep records the new step index and restarts the timer reference.
void enterStep(byte idx) {
  currentStepIndex = idx;                // remember which table entry is active
  stepStartTime = millis();              // timestamp the start of the action
  pausedElapsedTime = 0;                 // clear pause accumulator when running
  applyStep(idx);                        // energize hardware for the step
}

// applyStep energizes or releases the actuator assigned to the provided step.
void applyStep(byte idx) {
  switch (idx) {
    case 0:
      diaphragmPumpMotor.setSpeed(255);  // full speed for diaphragm pump
      diaphragmPumpMotor.run(FORWARD);   // spin pump forward to fill
      Serial.println("Diaphragm pump on.");
      break;
    case 1:
      diaphragmPumpMotor.run(RELEASE);   // stop diaphragm pump motion
      break;
    case 3:
      peristalticPumpMotor.setSpeed(255);// max speed for peri pump
      peristalticPumpMotor.run(FORWARD); // push fluid forward
      Serial.println("Peri pump on.");
      break;
    case 4:
      peristalticPumpMotor.run(RELEASE); // stop peristaltic pump rotation
      break;
    case 6:
      impellerMotor.setSpeed(180);       // fast stir speed for mixing
      impellerMotor.run(FORWARD);        // spin impeller forward
      Serial.println("Imp fast.");
      break;
    case 7:
      impellerMotor.setSpeed(100);       // slow stir to finish blending
      impellerMotor.run(FORWARD);        // keep impeller direction consistent
      Serial.println("Imp slow.");
      break;
    case 8:
      impellerMotor.run(RELEASE);        // release impeller so fluid rests
      break;
    case 10:
      pressMotor.setSpeed(180);          // speed for press stage
      pressMotor.run(FORWARD);           // drive press downward
      Serial.println("Press motor on.");
      break;
    case 11:
      pressMotor.run(RELEASE);           // release pressure motor
      break;
    case 13:
      digitalWrite(RELAY_PIN, HIGH);     // give power to relay 
      Serial.println("Relay on.");
      break;
    case 14:
      digitalWrite(RELAY_PIN, LOW);      // drop relay
      Serial.println("Relay off.");
      break;
    default:
      // Waiting steps fall through here so everything remains released.
      stopAllActuators();
      break;
  }
}

// stopAllActuators guarantees every motor and the relay are released.
void stopAllActuators() {
  diaphragmPumpMotor.run(RELEASE);      // diaphragm pump off
  peristalticPumpMotor.run(RELEASE);    // peristaltic pump off
  impellerMotor.run(RELEASE);           // impeller off
  pressMotor.run(RELEASE);              // press motor off
  digitalWrite(RELAY_PIN, LOW);         // relay cut off from power
}
