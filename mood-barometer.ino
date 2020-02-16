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
#include <PubSubClient.h>   // by Nick O'Leary
#include <WiFi.h>           // by Arduino

// WiFi
WiFiClient espClient;
const char* ssid = "";
const char* password =  "";

// MQTT
PubSubClient client(espClient);
const char* mqttPublishTopic = "team1/moodometer";
const char* mqttSubscribeTopic = "team1/mood";
const char* mqttServer = "192.168.1.13";
const int mqttPort = 1883;
const char* mqttClientId = "team1";
const char* mqttUser = "sammy";
const char* mqttPassword = "1234";
const int ReconnectEveryMillis = 5000;
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

const int DETECT_HALL = 18;     // sensor value indicating hand/arm is at 0°
const int MOTOR_SPEED = 8;
const int MOTOR_STEPS = 2038;   // how many steps for one revolution 360°
const int RECALIB_EVERY = 10;   // deal with drift

int currentPos = 0;
int moves = 0;                  // track how many times hand/arm moved for recalibration

// Power save position
unsigned char IN1 = 0;
unsigned char IN2 = 0;
unsigned char IN3 = 0;
unsigned char IN4 = 0;

const int motorPin1 = 12;
const int motorPin2 = 27;
const int motorPin3 = 14;
const int motorPin4 = 26;

Stepper stepper(MOTOR_STEPS, motorPin1, motorPin2, motorPin3, motorPin4);

void setup() {
  Serial.begin(115200);
  Serial.println("setup starting");

  stepper.setSpeed(MOTOR_SPEED);
  Calibrate();

  SetupWifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(Callback);

  Serial.println("setup complete");
}

void loop() {
  if (Serial.available() > 0) {
    SerialCommand();
  }

  unsigned long now = millis();
  if (now - timeLastMsg >= ReconnectEveryMillis) {
    timeLastMsg = now;

    if (!client.connected()) {
      Reconnect();
    }
    client.loop();

    PublishCurrentMood();
  }
}

void SerialCommand() {
  int command = Serial.parseInt();

  if (command > 0 && command < 6) {
    Serial.println("Serial command to set mood to "+(String)command);
    SetPos(command);
  }
}

void PublishCurrentMood() {
  String currentMood = "Current Mood "+(String)currentPos;
  char charBuf[15];
  currentMood.toCharArray(charBuf, 15);

  Serial.println(currentMood);
  client.publish(mqttPublishTopic, charBuf);
}

boolean OnHallSensor() {
  int val = abs(hallRead());
  return (val > DETECT_HALL);
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
        Serial.println("Moving angle of " + (String)route[i][2] + "'");
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
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void Callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message of " + (String)length + " chars arrived on topic: ");
  Serial.print(topic);

  if (length == 1) {
    int mood = atoi(reinterpret_cast<char *>(message));  // if non integer number a zero is returned

    if (mood == 0) {
      Serial.print(" (message = " + (String)static_cast<char>(message[0]) + ")");
    }

    Serial.print(". Set mood to " + (String)mood);
    Serial.println();
    SetPos(mood);
  } else {
    Serial.print(". Invalid message: ");
    String messageTemp;

    for (int i = 0; i < length; i++) {
      messageTemp += static_cast<char>(message[i]);
    }
    Serial.print(messageTemp);
    Serial.println();
  }
}

void Reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect(mqttClientId, mqttUser, mqttPassword)) {
      Serial.println("connected");
      Serial.println("publishing mood to topic " + (String)mqttPublishTopic);
      Serial.println("subscribing to topic " + (String)mqttSubscribeTopic);
      client.subscribe(mqttSubscribeTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);  // Wait before retrying
    }
  }
}
