/*
  RC Car Motor Controller — Arduino Uno R4 WiFi
  ESC      -> pin 9
  Servo    -> pin 10
  Jetson   -> pin 2 (SoftwareSerial RX, GPIO serial)
  Laptop   -> WiFi UDP port 5005

  Jetson autonomous = serial on pin 2 (fast, low latency)
  Laptop manual     = WiFi UDP (wireless override)
*/

#include <Servo.h>
#include <SoftwareSerial.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

Servo esc;
Servo steering;

// Jetson serial on pin 2
SoftwareSerial jetson(2, 3);  // RX=2, TX=3 (TX unused)

// WiFi for laptop control
const char* ssid     = "Raffay";
const char* password = "22997788";
const int   UDP_PORT = 5005;
WiFiUDP udp;

// Reverse state machine
enum ReverseState { IDLE, BRAKE, NEUTRAL_WAIT, REVERSING };
ReverseState revState  = IDLE;
unsigned long revTimer = 0;

void startReverse() {
  revState = BRAKE;
  revTimer = millis();
  esc.writeMicroseconds(1300);
}

void updateReverse() {
  if (revState == IDLE) return;
  unsigned long now = millis();
  if (revState == BRAKE && now - revTimer > 200) {
    esc.writeMicroseconds(1500);
    revState = NEUTRAL_WAIT;
    revTimer = now;
  }
  else if (revState == NEUTRAL_WAIT && now - revTimer > 100) {
    esc.writeMicroseconds(1300);
    revState = REVERSING;
  }
}

void handleCmd(char cmd) {
  switch (cmd) {
    case 'f': revState = IDLE; esc.writeMicroseconds(1750);      break;
    case 'm': revState = IDLE; esc.writeMicroseconds(1580);      break;
    case 'r': startReverse();                                     break;
    case 's': revState = IDLE; esc.writeMicroseconds(1500);      break;
    case 'a': steering.writeMicroseconds(1200);                  break;
    case 'd': steering.writeMicroseconds(1800);                  break;
    case 'c': steering.writeMicroseconds(1500);                  break;
  }
}

void setup() {
  Serial.begin(9600);
  jetson.begin(9600);

  esc.attach(9, 1000, 2000);
  steering.attach(10, 1000, 2000);
  esc.writeMicroseconds(1500);
  steering.writeMicroseconds(1500);
  Serial.println("Holding neutral - plug in battery NOW");
  delay(5000);
  Serial.println("ESC armed");

  // Connect WiFi
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nConnected!");
  IPAddress ip = WiFi.localIP();
  Serial.print(ip[0]); Serial.print(".");
  Serial.print(ip[1]); Serial.print(".");
  Serial.print(ip[2]); Serial.print(".");
  Serial.println(ip[3]);
  udp.begin(UDP_PORT);
  Serial.println("Ready - Serial(pin2) + WiFi UDP(5005)");
}

void loop() {
  updateReverse();

  // Jetson serial (autonomous - fast)
  if (jetson.available()) {
    char cmd = jetson.read();
    handleCmd(cmd);
  }

  // Laptop WiFi UDP (manual override)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char cmd = udp.read();
    handleCmd(cmd);
  }
}
