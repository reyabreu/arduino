/*
  pause and upause servo sweep with pushbutton
*/
#include <Servo.h>
#include <SoftwareSerial.h>

//consts remain static
const int DIGITAL_BTN_PAUSE_PIN = 2;    // the number of the pushbutton pin
const int DIGITAL_LED_PAUSE_PIN = LED_BUILTIN;      // the number of the LED pin
const int DIGITAL_SERVO1_PIN = 9;
const int ANALOG_POT_PIN = 0;

// Variables will change:
Servo servo1;

int servo1Position = 0;
int delta = 45;
unsigned long lastCommandTime = 0;
unsigned long servoDelay = 1000;

//button state variables
int buttonState;
int lastButtonState = HIGH;
int lastPotReading = 0;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

//loop control variables
boolean paused = true;

void setup() {
  //initialize Serial
  Serial.begin(57600);
  
  //attach arm servo
  servo1.attach(DIGITAL_SERVO1_PIN);

  //attach button
  pinMode(DIGITAL_BTN_PAUSE_PIN, INPUT);

  //attach led - set state
  pinMode(DIGITAL_LED_PAUSE_PIN, OUTPUT);
  digitalWrite(DIGITAL_LED_PAUSE_PIN, LOW);

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
    servo1.write(servo1Position);
  }

  //button debouncing
  int reading = digitalRead(DIGITAL_BTN_PAUSE_PIN);
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
  digitalWrite(DIGITAL_LED_PAUSE_PIN, paused);  
  lastButtonState = reading;

  //move servo
  if (!paused) {
    
    if (millis() - lastCommandTime > servoDelay) {
      
      Serial.print(F("arm at: "));
      Serial.print(servo1.read());
      Serial.println(F(" degrees"));

      servo1Position += delta;
      if (servo1Position <= 0 || servo1Position >= 180){
        delta = -1 * delta;
        if (servo1Position < 0) {
          servo1Position = 0; 
        } else if (servo1Position > 180) {
          servo1Position = 180; 
        };
      }
      servo1.write(servo1Position);                  
      lastCommandTime = millis();
    }
  } else {
    int potReading = analogRead(ANALOG_POT_PIN);
    //reading = map(reading, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    if (abs(potReading - lastPotReading) > 10) {
      Serial.print(F("Pot reading: "));
      Serial.println(potReading);
      lastPotReading = potReading;
    }
    
  }
}
