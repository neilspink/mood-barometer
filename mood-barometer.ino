/*
* Mood baramoter dial angles are {144, 72, 0, 216, 288}:
*
* 1 = Super Happy
* 2 = Happy
* 3 = OK
* 4 = Not Happy
* 5 = Angry
*
*/

#include <Stepper.h>        // by Arduino
#include <WiFi.h>           // by Arduino
#include <HTTPClient.h>

// WiFi
const char* ssid = "Moody";
const char* password =  "firstclass";

const char* serverNameMood = "http://192.168.4.1/mood";

const unsigned int ReconnectEveryMillis = 1000;
unsigned long timeLastMsg = 0;

// The fastest hand/arm movements are precalculated in the route vector {start pos, target pos, angle to move}
int route[20][3] {
    {1,2,-72},
    {1,3,-144},
    {1,4,144},
    {1,5,72},
    {2,1,72},
    {2,3,-72},
    {2,4,-144},
    {2,5,144},
    {3,1,144},
    {3,2,72},
    {3,4,-72},
    {3,5,-144},
    {4,1,-144},
    {4,2,144},
    {4,3,72},
    {4,5,-72},
    {5,1,-72},
    {5,2,-144},
    {5,3,144},
    {5,4,72}
  };

const unsigned int MOTOR_SPEED = 6;
const unsigned int MOTOR_STEPS = 2038;   // how many steps for one revolution 360°
const unsigned int RECALIB_EVERY = 10;   // deal with drift

const int hallSensorPin = 17;
int onHall = 0;

unsigned int currentPos = 0;
unsigned int moves = 0;                  // track how many times hand/arm moved for recalibration

// Power save position
unsigned char IN1 = 0;
unsigned char IN2 = 0;
unsigned char IN3 = 0;
unsigned char IN4 = 0;

const unsigned int motorPin1 = 12;
const unsigned int motorPin2 = 27;
const unsigned int motorPin3 = 14;
const unsigned int motorPin4 = 26;

Stepper stepper(MOTOR_STEPS, motorPin1, motorPin2, motorPin3, motorPin4);

void setup() {
  Serial.begin(115200);
  Serial.println("setup starting");

  pinMode(hallSensorPin, INPUT); 
  
  Serial.println("Wifi");
  SetupWifi();

  Serial.println("Calibration");
  stepper.setSpeed(MOTOR_SPEED);
  Calibrate();

  Serial.println("setup complete");
}

void loop() {
  
  if (Serial.available() > 0) {
    SerialCommand();
  }

  unsigned long now = millis();
  if (now - timeLastMsg >= ReconnectEveryMillis) {
    timeLastMsg = now;

    Serial.println("Checking WiFi.status");  
    if(WiFi.status()== WL_CONNECTED ){ 
      Serial.println("Checking webservice");
      String mood = httpGETRequest(serverNameMood);
      Serial.println("The mood is "+(String)mood);
      unsigned int m = strtoul(mood.c_str(), NULL, 10);
      SetPos(m);
    } else {
      Serial.println("NO WiFi :("); 
    }
  }
}

void SerialCommand() {
  int command = Serial.parseInt();

  if (command == 8) {
    Calibrate();
  } else if (command > 0 && command < 6) {
    Serial.println("Serial command to set mood to "+(String)command);
    SetPos(command);
  }
}

boolean OnHallSensor() {
  onHall = digitalRead(hallSensorPin);
  return (onHall == LOW);
}

void Calibrate() {
  Serial.println("Calibrating");

  StepperOn();
  stepper.setSpeed(MOTOR_SPEED);

  if (OnHallSensor()) {
    Serial.println("On hall sensor, moving a bit off.");

    while (OnHallSensor()) {
      stepper.step(200);
    }
  }

  Serial.println("finding hall sensor.");
  while (OnHallSensor() == false) {
    stepper.step(100);
  }
  Serial.println("found");

  currentPos = 3;

  StepperOff();
}

void SetPos(const unsigned int target) {
  Serial.println("SetPos() Current position " + (String)currentPos + " New position " + (String)target);

  if (target < 1 || target > 5) {
    Serial.println("Invalid Mood");
    return;
  }

  if (currentPos != target) {
    if (moves > RECALIB_EVERY) {
      moves = 0;
      Calibrate();
    }
    moves++;

    StepperOn();
    stepper.setSpeed(MOTOR_SPEED);

    int i, endPos;
    for (i = 0; i < 20; i++) {
      if (route[i][0] == currentPos && route[i][1] == target) {
        Serial.println("currentPos " + (String)currentPos + " target " + (String)target + " and moving angle of " + (String)route[i][2] + "'");
        endPos = map(route[i][2], 0, 360, 0, MOTOR_STEPS);
        break;
      }
    }

    Serial.println("Moving " + (String)endPos + " steps.");

    stepper.step(endPos);

    currentPos = target;
    StepperOff();
  }
}

void StepperOff() {
  delay(100);     // wait incase motor is still moving
  StepperSave();

  digitalWrite(motorPin1, LOW);  // turning motor off saves power which also stops unnecessary heat
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

void StepperOn() {
  digitalWrite(motorPin1, IN1);  // return positional settings
  digitalWrite(motorPin2, IN2);
  digitalWrite(motorPin3, IN3);
  digitalWrite(motorPin4, IN4);
  delay(100);
}

void StepperSave() {
  IN1 = digitalRead(motorPin1);
  IN2 = digitalRead(motorPin2);
  IN3 = digitalRead(motorPin3);
  IN4 = digitalRead(motorPin4);
}

void SetupWifi() {
  delay(100);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}


String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}