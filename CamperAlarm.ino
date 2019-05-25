/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital
  pin 13, when pressing a pushbutton attached to pin 2.


  The circuit:
   LED attached from pin 13 to ground
   pushbutton attached to pin 2 from +3.3V
   10K resistor attached to pin 2 from ground

   Note: on most Arduinos there is already an LED on the board
  attached to pin 13.


  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe
  modified Apr 27 2012
  by Robert Wessels

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Button
*/

// constants won't change. They're used here to
// set pin numbers:
const int redLed = RED_LED;
const int buttonPin = 8;     // the number of the pushbutton pin(was PUSH2).
const int ledPin =  GREEN_LED;      // the number of the LED pin
const int ambientLight = 7;
const int alarmPin = 9;
const int lightPin = 18; //hook this to a transistor
const int interruptPin1 = P1_3; //opening door
//const int interruptPin2 = P1_4; //closing door
const bool OFF = false;
const bool ON = true;
bool on = true;
//********** variables **********

int light = 0;
bool doorOpen = false;
bool alarmState = OFF;
bool firstTimeThru = true;
int startTime = 0;
int elapsed = 0;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
void buttonISR() {
  wakeup();
  delay(50);
  Serial.println("waking up");
  doorOpen=true;
}
//set alarm triggered in On or Off state.
void SetAlarmTrigger(bool state) {

  alarmState = state;
  //Serial.print("Alarm state is now ");
  //Serial.println(alarmState);
}
void toggleAlarm() {
  digitalWrite(alarmPin, alarmState); //bool true=1...same as HIGH
  if (alarmState) {
    Serial.println("alarming");
    startTime = millis() / 1000;
    delay(10);
    while (digitalRead(buttonPin) == HIGH ) {//while the alarm reset has not been pushed
      delay(10);
      //keep sounding alarm for 10 min.
      elapsed = millis() / 1000 - startTime;
      if (elapsed > 600  ) { //600 sec. is 10 min, don't run the battery down.
        break;
      }

    }
    alarmState = OFF;
    Serial.println("Alarm OFF");
    toggleAlarm();
  }
}

void setup() {

  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(ambientLight, INPUT);
  pinMode(interruptPin1, INPUT_PULLUP);
  // pinMode(interruptPin2, INPUT);
  pinMode(alarmPin, OUTPUT);
  pinMode(lightPin, OUTPUT);

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(P1_3, buttonISR, RISING);
 // attachInterrupt(P1_4, doorClosed, FALLING);

}
void doorClosed(){
  Serial.println("Someone just closed all doors.");
  doorOpen=false;
}
void loop() {
  digitalWrite(ledPin, HIGH);
  Serial.println("Enter Loop");
  delay(50);
  doorOpen = digitalRead(interruptPin1);
  
  Serial.println(doorOpen);
  if (firstTimeThru && doorOpen) {
    SetAlarmTrigger(ON);
    startTime = millis() / 1000; //time in seconds
    delay(10);
    elapsed = 0;
    //person has 15 seconds to disarm the alarm
    elapsed = millis() / 1000 - startTime;
    while (elapsed < 15) {
      elapsed = millis() / 1000 - startTime;
      //Serial.println(elapsed);
      //if button is not pushed in 15 sec, the alarm will stay triggered and will sound.
      if (digitalRead(buttonPin) == LOW) {
        SetAlarmTrigger(OFF);
      }

    }
  }
  firstTimeThru = false;
  //the alarm state is alway ON when door is opened and micro comes back to life, but
  // button(see "while loop") will change alarmState to OFF.
  toggleAlarm();
  delay(500);


  //Serial.println(light);
  delay(10);
  if (!doorOpen) {
    firstTimeThru = true; //set up for the awake function to set the alarm again.
    Serial.println("Doors all closed, going to sleep.");
    digitalWrite(ledPin, LOW);
    digitalWrite(lightPin, LOW);
    suspend();
  } else {
    light = analogRead(ambientLight);
    NeedLight();
  }
  delay(2000);//take this out!!!!!!!!!!!!!!
}
void NeedLight() {
  if (doorOpen) {
    digitalWrite(redLed, HIGH);
    Serial.println("A door is open");
    Serial.println(light);
    if (light < 700 ) {
      Serial.println("Light On");
      //digitalWrite(redLed, HIGH);
      digitalWrite(lightPin,HIGH);
    } else {
      Serial.println("Light OFF");
      //digitalWrite(redLed, LOW);
      digitalWrite(lightPin,LOW);
    }
  } else {
    Serial.println("Light OFF");
    digitalWrite(redLed, LOW);
    digitalWrite(lightPin, LOW);
  }
}

