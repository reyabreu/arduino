/*
  pause and upause servo sweep with pushbutton
*/
#include <Servo.h>
#include <SoftwareSerial.h>

//consts remain static
const int buttonPin = 2;    // the number of the pushbutton pin
const int ledPin = LED_BUILTIN;      // the number of the LED pin
const int SERVO_PIN_ARM = 9;

// Variables will change:
Servo armServo;
int armPosition = 0;
int delta = 45;
unsigned long lastCommandTime = 0;
unsigned long servoDelay = 1000;

//button state variables
int buttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

//loop control variables
boolean paused = true;

void setup() {
  //initialize Serial
  Serial.begin(57600);
  
  //attach arm servo
  armServo.attach(SERVO_PIN_ARM);

  //attach button
  pinMode(buttonPin, INPUT);

  //attach led - set state
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  //wait until serial instantiated
  initSerial;
}

//for duemilanove only i guess
void initSerial() {  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  //just to let know we're good to go
  if (millis() < 5) {
    Serial.println(F("Serial port is active"));
    armServo.write(armPosition);
  }

  //button debouncing
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        paused = !paused;
        Serial.print(F("Pause state changed: "));
        Serial.println(paused);
      }
    }
  }
  digitalWrite(ledPin, paused);  
  lastButtonState = reading;

  //move servo
  if (!paused) {
    
    if (millis() - lastCommandTime > servoDelay) {
      
      Serial.print(F("arm at: "));
      Serial.print(armServo.read());
      Serial.println(F(" degrees"));

      armPosition += delta;
      if (armPosition <= 0 || armPosition >= 180){
        delta = -1 * delta;
        if (armPosition < 0) {
          armPosition = 0; 
        } else if (armPosition > 180) {
          armPosition = 180; 
        };
      }
      armServo.write(armPosition);                  
      lastCommandTime = millis();
    }
  }
}
