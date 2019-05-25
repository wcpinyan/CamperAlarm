/*
 Author:      WCPinyan
 Date:        5/25/2019
 Micro:       TI MSP-EXP430G2-2553

 Description: Automatic light and alarm for
 truck camper top (ARE) with 3 doors.
 Chip is put to sleep once all doors are closed and
 is triggered awake by opening any door.
 Operator has 15 sec. to push out-of-sight button to
 disable alarm.
 Alarm will sound for 10 minutes unless button 
 pressed or time-out occurs.  Time-out prevents
 truck battery from being drained.
 Ambient sensor prob not necessary since the inside of 
 the shell always needs light (tinted windows.)

 The launchpad was only used during development. The 2553 was then
 removed and installed on protoboard (via removable socket to accommodate
 further programming.) Thus, LED code below only applicable for dev and testing.

 3 voltage levels: 12V  from vehicle battery(main power and alarm siren)
                    6V  for LED light and feed for conversion to 3.3V
                    3V3 for 2553 chip.
    12V-6V  SMAKN DC-DC Buck Power Converter. Amazon
    6V-3V2: https://www.youtube.com/watch?v=J66_8P043ko&t=621s Thanks to
            ITKindaWorks.

 
*/

const int redLed = RED_LED;
const int buttonPin = 8;     // the number of the pushbutton pin(was PUSH2).
const int ledPin =  GREEN_LED;      // the number of the LED pin
const int ambientLight = 7;
const int alarmPin = 9;
const int lightPin = 18; // 6V controlled by TIP120
const int interruptPin1 = P1_3; // 12V controlled by TIP120
const bool OFF = false;
const bool ON = true;

//********** variables **********

int light = 0;
bool doorOpen = false;
bool alarmState = OFF;
bool firstTimeThru = true;
int startTime = 0;
int elapsed = 0;
int buttonState = 0; // kills alarm after door open

// handles interrupt from sleep
void buttonISR() {
  wakeup();
  delay(50); //give time for chip warmup.
  Serial.println("waking up");
  doorOpen=true;
}
//set alarm triggered for On or Off state.
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
 // Serial.println("Enter Loop");
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


  //Serial.println(light); //test ambient light level
  delay(10);
  if (!doorOpen) {
    firstTimeThru = true; //set up for the awake function to set the alarm again.
    //Serial.println("Doors all closed, going to sleep.");
    digitalWrite(ledPin, LOW);
    digitalWrite(lightPin, LOW);
    suspend();
  } else {
    light = analogRead(ambientLight);
    NeedLight();
  }
  delay(2000);//this prob not needed
}
void NeedLight() {
  if (doorOpen) {
    digitalWrite(redLed, HIGH);
    //Serial.println("A door is open");
    //Serial.println(light);
    if (light < 700 ) {
      // Serial.println("Light On");
      //digitalWrite(redLed, HIGH);
      digitalWrite(lightPin,HIGH);
    } else {
      //Serial.println("Light OFF");
      //digitalWrite(redLed, LOW);
      digitalWrite(lightPin,LOW);
    }
  } else {
    // Serial.println("Light OFF");
    digitalWrite(redLed, LOW);
    digitalWrite(lightPin, LOW);
  }
}

