/*
  pause and upause micro9g servo sweep with pushbutton. In paused mode set angle of servo by using 22K pot
*/
#include <Servo.h>
#include <SoftwareSerial.h>

//consts remain static
const int DIGITAL_BTN_PAUSE_PIN = 2;    // the number of the pushbutton pin
const int DIGITAL_LED_PAUSE_PIN = LED_BUILTIN;      // the number of the LED pin
const int DIGITAL_SERVO1_PIN = 9;
const int ANALOG_POT_PIN = 0;

const float SAMPLE_WEIGHT = 0.6;  //initialization of EMA alpha

// Variables will change:
Servo servo1;

int servo1Position = 0;
int lastServo1Position = 0;

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

//exponential moving average low pass filtering for pot
int filteredPotReading = 0;      //filtered pot reading  

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

  //initialise first reading
  filteredPotReading = analogRead(ANALOG_POT_PIN);

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
        Serial.println(paused == 1 ? F("TRUE") : F("FALSE"));
      }
    }
  }
  digitalWrite(DIGITAL_LED_PAUSE_PIN, paused);  
  lastButtonState = reading;

 //only bother if enough time has passed since last move   
  //sweep servo
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
      }//end servo boundary check
      
      servo1.write(servo1Position);
      lastCommandTime = millis();
    } //end last servo command check
    
    } // end not paused option
    
   // read pot and move servo
   else {      
      int potReading = analogRead(ANALOG_POT_PIN);
      filteredPotReading = ( SAMPLE_WEIGHT * potReading ) + ( ( 1 - SAMPLE_WEIGHT ) * filteredPotReading );

      servo1Position = map(filteredPotReading, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
      servo1.write(servo1Position);
      
      if (millis() - lastCommandTime > servoDelay) {
        if (abs(lastServo1Position - servo1Position) >= 1){
          Serial.print(F("Filtered Pot reading: "));
          Serial.print(filteredPotReading);
          Serial.print(F(" servo angle mapping: "));
          Serial.println(servo1Position);
        }
        lastCommandTime = millis();
        lastServo1Position = servo1Position;
      } //end last servo command check
    } //end paused option 

}
