#include <AFMotor.h>
#include <Servo.h>

#define pushButton1 A14
#define pushButton2 A15


bool buttonState1 = HIGH;
bool buttonState2 = LOW;
bool systemstate = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(pushButton1, INPUT_PULLUP);
  pinMode(pushButton2, INPUT_PULLUP);

}

void loop() {
  buttonState1 = digitalRead(pushButton1);
  buttonState2 = digitalRead(pushButton2);

if (buttonState1 == LOW) {
  Serial.println("Button 1 Pressed");
}
else if (buttonState2 == LOW) {
  Serial.println("Button 2 Pressed");
}


  delay(200);
}

