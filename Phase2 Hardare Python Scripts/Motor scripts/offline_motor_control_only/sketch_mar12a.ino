#include <Servo.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

Servo esc;
Servo steering;

const char* ssid     = "Raffay";
const char* password = "22997788";
const int   UDP_PORT = 5005;

WiFiUDP udp;

void setup() {
  Serial.begin(9600);
  delay(2000);

  esc.attach(9, 1000, 2000);
  steering.attach(10, 1000, 2000);
  esc.writeMicroseconds(1500);
  steering.writeMicroseconds(1500);
  Serial.println("Holding neutral - plug in battery NOW");
  delay(5000);

  Serial.print("Connecting to ");
  Serial.println(ssid);
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
  Serial.println("UDP ready on port 5005");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char cmd = udp.read();
    if      (cmd == 'f') { esc.writeMicroseconds(1700); Serial.println("FORWARD"); }
    else if (cmd == 'r') { esc.writeMicroseconds(1250); Serial.println("REVERSE"); }
    else if (cmd == 's') { esc.writeMicroseconds(1500); Serial.println("STOP"); }
    else if (cmd == 'a') { steering.writeMicroseconds(1200); Serial.println("LEFT"); }
    else if (cmd == 'd') { steering.writeMicroseconds(1800); Serial.println("RIGHT"); }
    else if (cmd == 'c') { steering.writeMicroseconds(1500); Serial.println("CENTRE"); }
  }
}