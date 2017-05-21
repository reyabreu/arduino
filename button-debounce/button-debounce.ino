/*
  pause and upause servo sweep with pushbutton
*/
#include <Servo.h>

//consts remain static
const int buttonPin = 2;    // the number of the pushbutton pin
const int ledPin = LED_BUILTIN;      // the number of the LED pin
const int servoPin = 9;

// Variables will change:
Servo myServo;
int servoPos = 0;
int buttonState;
int lastButtonState = HIGH;
boolean paused = false; 

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  myServo.attach(servoPin);
  // set initial LED state
  digitalWrite(ledPin, LOW);
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        paused = !paused;
      }
    }
  }
  if (paused) {
    digitalWrite(ledPin, HIGH);  
  } else {
    digitalWrite(ledPin, LOW);  
  }
  lastButtonState = reading;
  for (servoPos = 0; servoPos <= 180; servoPos += 1) {
    if (paused) { break; }      
    myServo.write(servoPos);
    delay(15);
  }
  for (servoPos = 180; servoPos >= 0; servoPos -= 1) {
    if (paused) { break; }      
    myServo.write(servoPos);
    delay(15);
  }
}
