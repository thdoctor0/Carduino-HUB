#include <WiFiNINA.h>
#include <Servo.h>
#include <EEPROM.h>

#define CARDUINO_HUB_LINK "doctoru.github.io/Carduino-HUB"
#define WIFI_SSID "DoctorNet"
#define WIFI_PASS "SecurePass2024"
#define API_KEY "CDN-X5F9-7T2W-BZM8"
#define DEVICE_ID "Doctor_s_CarduinoV2"

const int MOTORS[8] = {2,3,4,5,6,7,8,9};
const int TRIG = 10, ECHO = 11, SERVO_PIN = 12;
const float WHEEL_CIRC = 20.0; // cm

Servo navServo;
WiFiServer server(80);
float posX, posY, heading;
byte routeMemory[50];
unsigned long autoTimer;

struct NavData {
  float speed;
  int distance;
  byte autoMode;
  int targetDeg;
} currentState;

void setup() {
  initHardware();
  connectNetwork();
  server.begin();
  loadRouteMemory();
}

void loop() {
  handleClientRequests();
  runAutonomousLogic();
  updatePositionData();
  if(millis() % 5000 < 100) sendTelemetry();
}

void initHardware() {
  Serial.begin(115200);
  for(int i=0; i<8; i++) pinMode(MOTORS[i], OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  navServo.attach(SERVO_PIN);
  navServo.write(90);
}

void connectNetwork() {
  while(WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(10000);
  }
}

void handleClientRequests() {
  WiFiClient client = server.available();
  if(client) {
    String req = client.readStringUntil('\n');
    if(validateRequest(req)) {
      processCommand(parseCommand(req), client);
      client.stop();
    }
  }
}

bool validateRequest(String &req) {
  return req.indexOf(API_KEY) != -1 && 
         req.indexOf(DEVICE_ID) != -1 &&
         req.indexOf(CARDUINO_HUB_LINK) != -1;
}

String parseCommand(String &req) {
  int cmdStart = req.indexOf("cmd=");
  return req.substring(cmdStart+4, req.indexOf(' ', cmdStart));
}

void processCommand(String cmd, WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  
  if(cmd == "auto") toggleAutonomous();
  else if(cmd.startsWith("rotate")) handleRotation(cmd);
  else executeMovement(cmd);

  client.print("{\"status\":\"OK\",\"x\":");
  client.print(posX);
  client.print(",\"y\":");
  client.print(posY);
  client.print(",\"heading\":");
  client.print(heading);
  client.print("}");
}

void executeMovement(String cmd) {
  if(cmd == "forward") moveForward();
  if(cmd == "back") moveBackward();
  if(cmd == "left") spinLeft();
  if(cmd == "right") spinRight();
  if(cmd == "stop") stopMotors();
}

void handleRotation(String cmd) {
  int degrees = cmd.substring(7).toInt();
  rotateToDegree(degrees);
}

void rotateToDegree(int target) {
  int current = heading;
  int dir = target > current ? 1 : -1;
  
  while(abs(current - target) > 2) {
    dir > 0 ? spinRight() : spinLeft();
    delay(15);
    current += dir * (millis() - autoTimer)/1000.0 * 90;
    autoTimer = millis();
    stopMotors();
  }
}

void runAutonomousLogic() {
  if(currentState.autoMode) {
    scanEnvironment();
    avoidObstacles();
    followRouteMemory();
  }
}

void scanEnvironment() {
  for(int angle=0; angle<=180; angle+=10) {
    navServo.write(angle);
    delay(50);
    currentState.distance = getDistance();
    updateRouteMemory(angle, currentState.distance);
  }
}

void updatePositionData() {
  static unsigned long lastUpdate;
  if(millis() - lastUpdate > 100) {
    posX += currentState.speed * cos(heading * PI / 180);
    posY += currentState.speed * sin(heading * PI / 180);
    lastUpdate = millis();
  }
}

// Motor Control Core Functions
void moveForward() { 
  digitalWrite(MOTORS[0], LOW); digitalWrite(MOTORS[1], HIGH);
  digitalWrite(MOTORS[2], HIGH); digitalWrite(MOTORS[3], LOW);
  digitalWrite(MOTORS[4], LOW); digitalWrite(MOTORS[5], HIGH);
  digitalWrite(MOTORS[6], HIGH); digitalWrite(MOTORS[7], LOW);
  currentState.speed = 15.0;
}

void spinLeft() {
  digitalWrite(MOTORS[0], HIGH); digitalWrite(MOTORS[1], LOW);
  digitalWrite(MOTORS[2], LOW); digitalWrite(MOTORS[3], HIGH);
  digitalWrite(MOTORS[4], HIGH); digitalWrite(MOTORS[5], LOW);
  digitalWrite(MOTORS[6], LOW); digitalWrite(MOTORS[7], HIGH);
  currentState.speed = 30.0;
}

void stopMotors() { 
  for(int i=0; i<8; i++) digitalWrite(MOTORS[i], LOW);
  currentState.speed = 0;
}

// Additional motor functions remain consistent with previous implementation
