#include <WiFiNINA.h>
#include <Adafruit_Fingerprint.h>
#include <Wire.h>
#include "secrets.h"

// setup serial for fingerprint sensor
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(2, 3);
#else
#define mySerial Serial1
#endif


char ssid[] = SECRET_SSID;      // your network SSID (name) 
char pass[] = SECRET_PASS;      // your network password
int status = WL_IDLE_STATUS;    // the WiFi radio's status
int keyIndex = 0;               // your network key Index number (needed only for WEP)
WiFiClient  client;

// IFTTT setup
char   HOST_NAME[] = "maker.ifttt.com";
String PATH_ONE = "/trigger/";
String PATH_TWO = "/with/key/bdPJr_muRI-qiKcQswflfT";
String pathString = "";
//========= 
// WEBHOOKS
//=========
// forced_entry - https://maker.ifttt.com/trigger/forced_entry/with/key/bdPJr_muRI-qiKcQswflfT
// access_granted - https://maker.ifttt.com/trigger/access_granted/with/key/bdPJr_muRI-qiKcQswflfT
// open_to_long - https://maker.ifttt.com/trigger/open_to_long/with/key/bdPJr_muRI-qiKcQswflfT
// lockout - https://maker.ifttt.com/trigger/lockout/with/key/bdPJr_muRI-qiKcQswflfT
// failed_attempt - https://maker.ifttt.com/trigger/failed_attempt/with/key/bdPJr_muRI-qiKcQswflfT


// state setup
const int OPEN = 0, LOCKED = 1, WAITING = 2, SCANNING = 3, UNLOCKED = 4, LOCKOUT = 5, ALERT = 6;
int state;                                  // current state

// door state setup
const int DOOR_CLOSED = 0, DOOR_OPEN = 1;
int doorState;                              // current door state

// alert state setup for IFTTT
const int FORCED_ENTRY = 0, ACCESS_GRANTED = 1, OPEN_TO_LONG = 2, LOCKOUT_ALERT = 3, FAILED_ATTEMPT = 4;
int alertState;                              // current door state

// other variables
const int ATTEMPTS_ALLOWED = 3;             // allowed attempts before lockout
const int CONFIDENCE_LEVEL = 50;            // fingerprints need a confidence level above this to have a chance of gaining access
int declinedAttempts = 0;                   // current attempts that have failed to gain access
int flashCount = 0;                         // used to track the number of flashes in LOCKOUT
int red = LOW;                              // state of red LED, 0 = LOW, 1 = HIGH
int green = LOW;                            // state of green LED, 0 = LOW, 1 = HIGH
int blue = LOW;                             // state of blue LED, 0 = LOW, 1 = HIGH
int buttonState = 0;                        // changes when a button is pressed
int scanState = 2;                          // the next state after a finger is scanned
bool hasBeenNotified = false;               // is true when an alert message is sent
bool doorForced = false;                    // changes if door is forced open

// timer parameters in milliseconds
const unsigned long INTERVAL_TIME = 250;    // 0.25 second
const double LOCK_LED_TIME = 2000;          // 2 seconds; time showing the system was recently locked
const double SCAN_LED_TIME = 2000;          // 5 seconds; time allowed during scanning
const double UNLOCKED_LED_TIME = 5000;      // 5 seconds; time showing the system was recently unlocked
const double OPEN_TIME = 10000;             // 10 seconds; time allowed for door to be open before alerts start
const double ALERT_INTERVAL = 1000;         // 1 second; intervals of flashing lights while in alert state
const double LOCKOUT_TIME = 10000;          // 10 seconds; time in lockout after 3 denied/failed attempts in succession

// timers
double timer = 0;
double flashTimer = 0;
double timeInState = 0;
int seconds = 0;
int minutes = 0;
int hours = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

// fingerprint sensor setup
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial); // assign serial to finger
const int FINGERPRINT_SLOTS = 128;             // allowed attempts before lockout
bool fingerScanned = false;
bool access = false;
bool fingersWithAccess[FINGERPRINT_SLOTS];

// Arduino Nano Pin Information
// 5V = Red wire from Fingerprint Sensor
// GND = Black wire from Fingerprint Sensor
// TX = White wire from Fingerprint Sensor
// RX = Green wire from Fingerprint Sensor
// D2 = Yellow wire from Fingerprint Sensor - detects when a finger is on the scanner
// D4 = Release button
// D9 = Solenoid Lock
// D10 = Red LED
// D11 = Green LED
// D12 = Blue LED
// D16 = Door switch

// inputs
const int fingerprintPin = 2; // detects finger on fingerprint sensor
const int buttonPin = 4;      // release button
const int doorSwitchPin = 16; // door switch sensor

// outputs
const int solenoidPin = 9;    // solenoid lock
const int redLedPin = 10;     // red led
const int greenLedPin = 11;   // green led
const int blueLedPin = 12;    // blue led

void setup() {

  // WiFi setup
  Serial.begin(115200);  
  while (!Serial) {
    ;
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware.");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    loadingDelay(10);
  }
  // you're connected now, so print out the data:
  Serial.println("You're connected to the network.");
  
  // fingerprint sensor setup
  Serial.begin(9600);
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }
  // reset access array
  for (int i = 0; i < FINGERPRINT_SLOTS; i++) fingersWithAccess[i] = false;
  // fingers to  test
  fingersWithAccess[1] = true;  // left index
  fingersWithAccess[2] = false; // left middle
  fingersWithAccess[3] = true;  // left ring
  fingersWithAccess[4] = true;  // left thumb
  fingersWithAccess[5] = false; // right thumb

  pinMode(doorSwitchPin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT);
  pinMode(fingerprintPin, INPUT);

  pinMode(solenoidPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  nextDoorState(DOOR_CLOSED);
  next(LOCKED);
}

// this method prints to serial during delays to show that things are still working
void loadingDelay(int time){
  for (int i = 0; i < time; i++){
    Serial.print(". ");
    delay(1000);
  }
  Serial.println(". ");
}

// this method builds the path for the webhooks
void buildPath(){
  pathString = PATH_ONE;
  switch (alertState){
    case FORCED_ENTRY:
      pathString += "forced_entry";
      break;
    case ACCESS_GRANTED:
      pathString += "access_granted";
      break;
    case OPEN_TO_LONG:
      pathString += "open_to_long";
      break;
    case LOCKOUT_ALERT:
      pathString += "lockout";
      break;
    case FAILED_ATTEMPT:
      pathString += "failed_attempt";
      break;
  }
  pathString += PATH_TWO;
}

// this method connects to the webhook server and sends the webhook
void sendWebhook(){
  Serial.println("Connecting to Server.");
  
  // connect to web server on port 80:
  if (client.connect(HOST_NAME, 80)) {
    // if connected:
    Serial.println("Connected to server");
  }
  else {// if not connected:
    Serial.println("Connection failed");
  }
  buildPath();
  Serial.println(pathString);
  // make a HTTP request:
  // send HTTP header
  client.println("POST " + pathString + " HTTP/1.1");
  client.println("Host: " + String(HOST_NAME));
  client.println("Connection: close");
  client.println(); // end HTTP header

  // the server's disconnected, stop the client:
  client.stop();
  Serial.println("Disconnected from Server.");
}

// this method handles every tick for every state
void tick()
{
  timer -= INTERVAL_TIME;
  flashTimer -= INTERVAL_TIME;
  updateTimeInState();
  switch(state)
  {
    case OPEN:
      if (doorState == DOOR_CLOSED) {
        next(LOCKED);
        return;
      }
      if (flashTimer < 0) {
        flashTimer = ALERT_INTERVAL;
        green = LOW;
        flashCount++;
        if (flashCount > 1) flashCount = 0;
        if (flashCount > 0) green = HIGH;
        updateLEDs();
      }
      if (timer > 0) return;
      alertState = OPEN_TO_LONG;
      Serial.println("OPEN TO LONG: The system has detected that access has been open for more than "  + String(OPEN_TIME/1000) + " seconds! Please check system access.");
      hasBeenNotified = true;
      sendWebhook();
      next(ALERT);
      return;
    case LOCKED:
      if (doorState == DOOR_OPEN) {
        doorForced = true;
        next(ALERT);
      }
      if (timer > 0) return;
      next(WAITING);
      return;
    case WAITING:
      if (doorState == DOOR_OPEN) {
        doorForced = true;
        next(ALERT);
      }
      return;
    case SCANNING:
      if (doorState == DOOR_OPEN) {
        doorForced = true;
        next(ALERT);
      }
      if(!fingerScanned) scanState = scanningFinger();
      if (timer > 0 && !access) return;
      fingerScanned = false;
      next(scanState);
      return;
    case UNLOCKED:
      if (doorState == DOOR_OPEN) next(OPEN);
      if (timer > 0) return;
      next(LOCKED);
      return;
    case LOCKOUT:
      if (flashTimer < 0) {
        flashTimer = ALERT_INTERVAL;
        flashCount++;
        if (flashCount > 3) flashCount = 0;
        flashingLEDs();
        updateLEDs();
      }
      if (doorState == DOOR_OPEN) {
        doorForced = true;
        next(ALERT);
      }
      if (timer > 0) return;
      next(LOCKED);
      return;
    case ALERT:
      if (doorState == DOOR_CLOSED) {
        if (doorForced){
          doorForced = false;
          next (LOCKOUT);
          return;
        } else {
          next(LOCKED);
          return;
        }
      }
      if (doorState == DOOR_OPEN && !hasBeenNotified) {
        alertState = FORCED_ENTRY;
        Serial.println("FORCED_ENTRY: he system has been forced open! Please check system access.");
        hasBeenNotified = true;
        sendWebhook();
      }
      if (flashTimer < 0) {
        flashTimer = ALERT_INTERVAL;
        flashCount++;
        if (flashCount > 1) flashCount = 0;
        flashingLEDs();
        updateLEDs();
      }
      return;
    default:
      next(OPEN);
  }
}

// this method handles a release button press during the states
void releaseButtonPressed()
{
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) return;

  buttonState = LOW;
  switch(state)
  {
    case OPEN:
      return;
    case LOCKED:
      next(UNLOCKED);
      return;
    case WAITING:
      next(UNLOCKED);
      return;
    case SCANNING:
      next(UNLOCKED);
      return;
    case UNLOCKED:
      return;
    case LOCKOUT:
      next(UNLOCKED);
      return;
    case ALERT:
      return;
    default:
      next(OPEN);
  }
}

// this method handles scanning the fingerprint
int scanningFinger()
{
  // scanned finger
  uint8_t scannedFinger = getFingerprintID();
  fingerScanned = true;
  // handle fingerprint scanner 
  if (!access || !fingersWithAccess[scannedFinger]) 
  {
    access = false;
    declinedAttempts++;
    if (declinedAttempts == ATTEMPTS_ALLOWED) 
    {
      alertState = LOCKOUT_ALERT;
      Serial.println("LOCKOUT: The system detected multiple failed or declined attempts. System is in LOUTOUT for " + String(LOCKOUT_TIME/1000) + " seconds!");
      sendWebhook();
      return LOCKOUT;
    }
    alertState = FAILED_ATTEMPT;
    Serial.println("FAILED ATTEMPT: A scan has failed to gain access. Remaining Attempts: " + String(ATTEMPTS_ALLOWED - declinedAttempts) + ".");
    sendWebhook();
    return WAITING;
  }
  alertState = ACCESS_GRANTED;
  Serial.println("ACCESS APPROVED! Door is UNLOCKED.");
  sendWebhook();
  return UNLOCKED;
}

// this method handles activation of the fingerprint sensor and determines what to do by looking at the current state
void readFingerprintSensor()
{
  // read the state of the fingerprint value:
  if(digitalRead(fingerprintPin) == LOW) return;

  bool access = false;
  switch(state)
  {
    case OPEN:
      return;
    case LOCKED:
      Serial.println("Door has just locked. Please wait.");
      return;
    case WAITING:
      next(SCANNING);
      return;
    case SCANNING:
      return;
    case UNLOCKED:
      return;
    case LOCKOUT:
      Serial.println("System in LOCKOUT. " + printTimeInState() + "/0:10:00 LOCKOUT time.");
      return;
    case ALERT:
      return;
    default:
      next(SCANNING);
  }
}

// this method handles door states
// this is part of a second FSM within the first but focuses on the state of the door
void nextDoorState(int newDoorState){
  doorState = newDoorState;
  // 0 is closed
  // 1 is open
  switch (doorState)  {
    case DOOR_CLOSED: // closed
      //Serial.println("Door switch is CLOSED");
      return;
    case DOOR_OPEN: // open
      //Serial.println("Door switch is OPEN");
      return;
  }
}

// this method reads the door switch sensor every tick
void readDoorSensor()
{
  int readDoorState = digitalRead(doorSwitchPin);
  if(doorState != readDoorState) nextDoorState(readDoorState);
}

// this method handles state changes and sets once off events
void next(int newState)
{
  flashCount = 0;
  flashingLEDs();
  resetTimeInState();
  state = newState;
  switch(state)
  {
    case OPEN:
      timer = OPEN_TIME;
      updateLEDs();
      digitalWrite(solenoidPin, LOW);
      Serial.println("STATE: OPEN");
      return;
    case LOCKED:
      hasBeenNotified = false;
      digitalWrite(solenoidPin, LOW);
      timer = LOCK_LED_TIME;
      red = HIGH;
      updateLEDs();
      Serial.println("STATE: LOCKED");
      return;
    case WAITING:
      updateLEDs();
      Serial.println("STATE: WAITING");
      return;
    case SCANNING:
      timer = SCAN_LED_TIME;
      blue = HIGH;
      updateLEDs();
      Serial.println("STATE: SCANNING");
      return;
    case UNLOCKED:
      digitalWrite(solenoidPin, HIGH);
      declinedAttempts = 0;
      timer = UNLOCKED_LED_TIME;
      green = HIGH;
      updateLEDs();
      Serial.println("STATE: UNLOCKED");
      return;
    case LOCKOUT:
      hasBeenNotified = false;
      timer = LOCKOUT_TIME;
      flashCount = 1;
      flashTimer = ALERT_INTERVAL;
      declinedAttempts = 0;
      flashingLEDs();
      updateLEDs();
      Serial.println("STATE: LOCKOUT");
      return;
    case ALERT:
      timer = ALERT_INTERVAL;
      flashCount = 0;
      flashingLEDs();
      updateLEDs();
      Serial.println("STATE: ALERT");
      return;
    default:
      next(OPEN);
  }
}

// this method sets the LED on and off according to flash count
void flashingLEDs()
{
  // red turns on when above 0
  // green turns on when above 1
  // blue turns on when above 2
  switch(flashCount){
    case 1:
      red = HIGH;
      green = LOW;
      blue = LOW;
      return;
    case 2:
      red = HIGH;
      green = HIGH;
      blue = LOW;
      return;
    case 3:
      red = HIGH;
      green = HIGH;
      blue = HIGH;
      return;
    default:
      red = LOW;
      green = LOW;
      blue = LOW;
  }
}

// this method updates the LEDs 
void updateLEDs()
{
  digitalWrite(redLedPin, red);
  digitalWrite(greenLedPin, green);
  digitalWrite(blueLedPin, blue);
}

// this method resets the time in state
void resetTimeInState()
{
  seconds = 0;
  minutes = 0;
  hours = 0;
}

// updates the time in a state
// converts milliseconds to seconds, seconds to minutes, minutes to hours
// used to send in a webhook for alerts about lockout and attempts
void updateTimeInState(){
  timeInState += INTERVAL_TIME;
  
  if(timeInState > 1000)
  {
    timeInState -= 1000;
    seconds++;
  }

  if(seconds == 60)
  {
    seconds = 0;
    minutes++;
  }

  if (minutes == 60) 
  {
    minutes = 0;
    hours++;
  }

  // it should never reach 1000 hours
  if (hours > 1000) resetTimeInState();
}

// prints the time in state to the serial
String printTimeInState(){
  String totalTime = String(hours) + ":" + String(minutes) + ":" + String(seconds);
  return totalTime;
}

// this method scans the finger on the fingerprint sensor
// uses the library to serach if there is a matching fingerprint
// grants access if the confidence is above the confidence level
// returns the finger ID
uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // Image OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // Image OK converted!

  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
    
  if (finger.confidence > CONFIDENCE_LEVEL) access = true;

  return finger.fingerID;
}

void loop() {
  // timer to run on INTERVAL_TIME - set to 0.25 seconds
  currentMillis = millis();
  if (currentMillis < INTERVAL_TIME + previousMillis) return;
  previousMillis = currentMillis;
  
  // read inputs
  releaseButtonPressed();
  readDoorSensor();
  readFingerprintSensor();
  tick();
  
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      loadingDelay(5);
    } 
    Serial.println("\nConnected.");
  }
}

