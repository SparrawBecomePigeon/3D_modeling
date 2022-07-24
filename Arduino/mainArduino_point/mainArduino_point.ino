#include <StackArray.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <TFMPlus.h>  // Include TFMini Plus Library v1.5.0
TFMPlus tfmP;         // Create a TFMini Plus object
#include <ArduinoJson.h>
#include "WiFiEsp.h"

#define BLUE_PIN 22
#define GREEN_PIN 24
#define RED_PIN 26

#define sm1_pin1 2
#define sm1_pin2 3
#define sm1_pin3 4
#define sm1_pin4 5

#define sm2_pin1 6
#define sm2_pin2 7
#define sm2_pin3 8
#define sm2_pin4 9

#define aroundInterval 13000  // 한바퀴 도는 시간
#define LIDARDATASIZE 72      // 한바퀴 동안 측정할 데이터 개수 현재 360 / 72 = 5도 마다 측정

char ssid[] = "AndroidHotspot2409";            // your network SSID (name)
char pass[] = "12345678";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
char server[] = "13.125.205.44";  // server IP address
int server_port = 3000;           // server port number

// Initialize the Ethernet client object
WiFiEspClient client;

AccelStepper stepper1(4, sm1_pin1, sm1_pin3, sm1_pin2, sm1_pin4);
AccelStepper stepper2(4, sm2_pin1, sm2_pin3, sm2_pin2, sm2_pin4);

int16_t tfDist = 0;    // Distance to object in centimeters
unsigned long currentMillis;
unsigned long previousMillis;
int start_count;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // initialize serial for ESP module
  Serial2.begin(115200);  // Initialize TFMPLus device serial port.
  delay(20);               // Give port time to initalize
  tfmP.begin(&Serial2);   // Initialize device library object and...
  // pass device serial port to the object.

  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);

  LedBlue();
  delay(50);
  LedStop();
  delay(50);
  LedBlue();
  delay(50);
  LedStop();

  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  
  Serial.println("You're connected to the network");
  printWifiStatus();

  // Send some example commands to the TFMini-Plus
  // - - Perform a system reset - - - - - - - - - - -
  Serial.print( "Soft reset: ");
  if( tfmP.sendCommand( SOFT_RESET, 0))
  {
    Serial.print( "passed.\r\n");
  }
  else tfmP.printReply();

  delay(500);  // added to allow the System Rest enough time to complete

  // - - Display the firmware version - - - - - - - - -
  Serial.print( "Firmware version: ");
  if( tfmP.sendCommand( GET_FIRMWARE_VERSION, 0))
  {
    Serial.print(tfmP.version[ 0]); // print three single numbers
    Serial.print(tfmP.version[ 1]); // each separated by a dot
    Serial.print(tfmP.version[ 2]);
    Serial.println();
  }
  else tfmP.printReply();
  // - - Set the data frame-rate to 20Hz - - - - - - - -
  Serial.print( "Data-Frame rate: ");
  if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_20))
  {
    Serial.print(FRAME_20);
    Serial.println();
  }
  else tfmP.printReply();

  start_count = 0;
  delay(1000);
}

int cur_x = 0; int cur_y = 0;
int cur_rot = 90;

StackArray<unsigned long> st_rot;       // 다음 이동을 위한 방위각
StackArray<unsigned long> st_pre_rot;   // 이전 위치로 돌아갈 방위각
StackArray<unsigned long> st_pre_dist;  // 이전 위치로 돌아갈 거리

void loop() {
  String jsonstr = ""; // json string 선언
  LidarDetectAround(jsonstr); // 반시계방향으로 360도 돌면서 거리 측정 및 좌표로 환산
  SendDataWeb(jsonstr); // 측정된 좌표 웹에 전송
    
  if(st_rot.isEmpty()){
    Serial.println("FINISH!!!");
    LedStop();
    while(true) {}
  }
  
  returnPreviousPoint();
  goNextPoint();
  
  Serial.print("Current Point [x, y] = ");
  Serial.print(cur_x);
  Serial.print(" , ");
  Serial.println(cur_y);
  delay(2000);
}

void getDataWeb(){
  LedGreen();
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect(server, server_port)) {
    Serial.println("Connected to server");
    
    // Make a HTTP request
    client.print(F("GET /send HTTP/1.1\r\n"));;
    client.print(F("Host: 13.125.205.44:3000\r\n"));
    client.println();
  }
  else {
    // if you couldn't make a connection
    Serial.println("Connection failed");
    LedRed();
    delay(500);
  }

  client.flush();
  client.stop(); // 클라이언트 접속 종료
  // close the connection:
  Serial.println("Client disconnected");
}

void SendDataWeb(const String& jsonstr){
  LedGreen();
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect(server, server_port)) {
    Serial.println("Connected to server");
    
    // Make a HTTP request
    client.print(F("POST /send HTTP/1.1\r\n"));
    client.print(F("Content-Type: application/json\r\n"));
    client.print(F("Host: 13.125.205.44:3000\r\n" ));
    client.print(F("Content-Length: "));
    client.println(jsonstr.length());
    client.println();
    client.println(jsonstr);
  }
  else {
    // if you couldn't make a connection
    LedRed();
    Serial.println("Connection failed");
    delay(500);
  }
  client.flush();
  client.stop(); // 클라이언트 접속 종료
  // close the connection:
  Serial.println("Client disconnected");
}

int getDataLidar(){
  tfmP.getData( tfDist); // Get data from the device.
  Serial.print( "Dist : ");   // display distance,
  Serial.print(tfDist);
  Serial.println( "cm");
  return tfDist;
}

void LidarDetectAround(String& jsonstr){
  LedBlue();
  unsigned long lidarInterval = aroundInterval / LIDARDATASIZE;
  unsigned long lidarRunMillis = millis();
  
  DynamicJsonDocument doc(1024);
  JsonObject root = doc.to<JsonObject>();
  JsonArray Finish = root.createNestedArray("Finish");
  JsonArray Start = root.createNestedArray("Start");
  JsonArray Lidar_Location = root.createNestedArray("Lidar_Location");
  Lidar_Location[0] = cur_x;
  Lidar_Location[1] = cur_y;
  JsonArray LidarData_x = root.createNestedArray("LidarData_x");
  JsonArray LidarData_y = root.createNestedArray("LidarData_y");
  int dataCount = 0; int previousDist = -1;
  previousMillis = millis();
  while(true){
    currentMillis = millis();
    if(currentMillis - previousMillis >= aroundInterval) break;
    if((currentMillis - lidarRunMillis)>= lidarInterval){
      lidarRunMillis += lidarInterval;
      if(dataCount > LIDARDATASIZE - 1){
        Serial.println("ERROR: Lidar data Overflow!!!");
        break;
      }
      int currentDist = getDataLidar();
      if(previousDist < 0) { previousDist = currentDist; }
      if(currentDist == 0){
        st_rot.push(currentMillis - previousMillis);
      }
      if(currentDist - previousDist > previousDist * 2 && previousDist > 50){
        st_rot.push(currentMillis - previousMillis + lidarInterval);
      }
      previousDist = currentDist;
      LidarData_x[dataCount] = cur_x + int(currentDist * (cos(radians(cur_rot + (360 * (currentMillis - previousMillis) / aroundInterval)))));
      LidarData_y[dataCount] = cur_y + int(currentDist * (sin(radians(cur_rot + (360 * (currentMillis - previousMillis) / aroundInterval)))));
      dataCount++;
    }
    LeftRound();
  }
  if(st_rot.isEmpty()){
    Finish[0] = 0;
  }
  else{
    Finish[0] = 1;
  }
  Start[0] = start_count++;
  Stop();
  Serial.println("==============");
  serializeJson(doc, jsonstr);
  Serial.println(jsonstr);
  Serial.println("==============");
  return jsonstr;
}

void goNextPoint(){
  LedPupple();
  // 다음 측정 위치로 이동을 위한 방위각 설정
  unsigned long nt_rot = st_rot.pop();
  int nt_dist, cur_dist;

  st_pre_rot.push(nt_rot);

  Serial.print("Rotation : ");
  Serial.println(360 * nt_rot / aroundInterval);

  // 이동 방위각 회전
  previousMillis = millis();
  while(true){
    currentMillis = millis();
    if(currentMillis - previousMillis >= nt_rot) break;
    LeftRound();
  }
  Stop();
  
  nt_dist = getDataLidar();
  Serial.print("next_dist = ");
  Serial.println(nt_dist);
  delay(1000);

  // 이동
  unsigned long lidarInterval = aroundInterval / LIDARDATASIZE;
  previousMillis = millis();
  unsigned long startMillis = previousMillis;
  int previousLidar = -1;
  while(true){
    currentMillis = millis();
    if(currentMillis - previousMillis >= lidarInterval){
      previousMillis = currentMillis;
      int currentLidar = getDataLidar();
      if(currentLidar < 30 && currentLidar != 0) break;
      //if(currentLidar == previousLidar && currentLidar != 0) break;
      previousLidar = currentLidar;
    }
    Go();
  }
  Stop();
  st_pre_dist.push(currentMillis - startMillis);
  Serial.print("Go Distance : ");
  Serial.print((currentMillis - startMillis) / 200);
  Serial.println(" cm");

  if(nt_dist == 0) nt_dist = (currentMillis - startMillis) / 200;
  cur_dist = getDataLidar();
  cur_x += int((nt_dist - cur_dist) * (cos(radians(cur_rot + 360 * (nt_rot / aroundInterval)))));
  cur_y += int((nt_dist - cur_dist) * (sin(radians(cur_rot + 360 * (nt_rot / aroundInterval)))));

  // 이동 후 방위각 90도로 다시 돌아옴
  previousMillis = millis();
  while(true){
    currentMillis = millis();
    if(currentMillis - previousMillis >= nt_rot) break;
    RightRound();
  }
}

void returnPreviousPoint(){
  LedPupple();
  if(!st_pre_rot.isEmpty()){
    unsigned long point_rot = (st_pre_rot.pop() + aroundInterval / 2) % aroundInterval;
    // 방위각 회전
    previousMillis = millis();
    while(true){
      currentMillis = millis();
      if(currentMillis - previousMillis >= point_rot) break;
      LeftRound();
    }
    Stop();

    // 이동
    unsigned long point_dist = st_pre_dist.pop();
    previousMillis = millis();
    while(true){
      currentMillis = millis();
      if(currentMillis - previousMillis >= point_dist){
        break;
      }
      Go();
    }
    Stop();

    cur_x += int(point_dist / 200 * (cos(radians(cur_rot + (360 * point_rot / aroundInterval)))));
    cur_y += int(point_dist / 200 * (sin(radians(cur_rot + (360 * point_rot / aroundInterval)))));
      
    // 이동 후 방위각 90도로 다시 돌아옴
    previousMillis = millis();
    while(true){
      currentMillis = millis();
      if(currentMillis - previousMillis >= point_rot) break;
      RightRound();
    }
    Stop();
  }
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void Stop(){
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
}

void Go(){
  stepper1.setSpeed(-1000);
  stepper2.setSpeed(1000);
  stepper1.runSpeed();
  stepper2.runSpeed();
}

void RightRound(){
  stepper1.setSpeed(500);
  stepper2.setSpeed(500);
  stepper1.runSpeed();
  stepper2.runSpeed();
}

void LeftRound(){
  stepper1.setSpeed(-500);
  stepper2.setSpeed(-500);
  stepper1.runSpeed();
  stepper2.runSpeed();
}

void LedBlue(){
  digitalWrite(BLUE_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, LOW);
}
void LedGreen(){
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(RED_PIN, LOW);
}
void LedRed(){
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
}
void LedPupple(){
  digitalWrite(BLUE_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
}
void LedStop(){
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, LOW);
}
