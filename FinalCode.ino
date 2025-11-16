#include <AFMotor.h>
#include <Servo.h>
#include <Wire.h>

#define pushButton1 A14
#define pushButton2 A15
AF_DCMotor Dpump(2);   //M3 port
AF_DCMotor Ppump(1);   //M4 port
AF_DCMotor Impmotor(4);   //M2 port
AF_DCMotor Pressmotor(3); //M1 port
AF_DCMotor HSubpump(8);   //M8 port

bool buttonState1;    // Button 1 boolean set to true 
bool buttonState2;    // Button 2 boolean set to true 
bool systemstate = LOW;

void setup() {
  Serial.begin(9600);
  Serial.println("Hi! Im MARTY, hope you're thirsty");

  // Sets the buttons as inputs
  // Uses the internal pullup resistor to prevent electrical noise from the environment by keeping the button voltage at HIGH.
  // Button voltage is HIGH when not pressed and LOW when pressed. 
  pinMode(pushButton1, INPUT_PULLUP);
  pinMode(pushButton2, INPUT_PULLUP);
}

void loop() {
  // read buttons (LOW means pressed because of INPUT_PULLUP)
  buttonState1 = digitalRead(pushButton1);
  buttonState2 = digitalRead(pushButton2);

  if (buttonState1 == LOW) {
    Serial.println("Button 1 Pressed");
    systemstate = LOW;   // set false
  }
  else if (buttonState2 == LOW) {
    Serial.println("Button 2 Pressed");
    systemstate = HIGH;  // set true
  }

  if (systemstate == HIGH) {
    
  }

  // print state
  Serial.print("System state: ");
  Serial.println(systemstate);

  delay(200);
}
