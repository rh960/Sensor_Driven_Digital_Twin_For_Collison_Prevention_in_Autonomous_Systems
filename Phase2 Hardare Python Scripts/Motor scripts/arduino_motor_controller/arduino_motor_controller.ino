/*
  RC Car Motor Controller — Arduino Uno R4 WiFi
  ESC   -> pin 9
  Servo -> pin 10

  Maverick MSC-25RC reverse sequence:
    1. Send brake pulse (1300us)
    2. Neutral (1500us) briefly
    3. Send reverse pulse (1300us) again
*/

#include <Servo.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

Servo esc;
Servo steering;

const char* ssid     = "Raffay";
const char* password = "22997788";
const int   UDP_PORT = 5005;

WiFiUDP udp;

void doReverse() {
  // Maverick brake+reverse sequence
  esc.writeMicroseconds(1300);  // brake
  delay(250);
  esc.writeMicroseconds(1500);  // neutral
  delay(100);
  esc.writeMicroseconds(1300);  // reverse
  Serial.println("REVERSE");
}

void handleCmd(char cmd) {
  switch (cmd) {
    case 'f': esc.writeMicroseconds(1750);      Serial.println("FORWARD FAST");  break;
    case 'm': esc.writeMicroseconds(1580);      Serial.println("FORWARD SLOW");  break;
    case 'r': doReverse();                                                        break;
    case 's': esc.writeMicroseconds(1500);      Serial.println("STOP");          break;
    case 'a': steering.writeMicroseconds(1200); Serial.println("LEFT");          break;
    case 'd': steering.writeMicroseconds(1800); Serial.println("RIGHT");         break;
    case 'c': steering.writeMicroseconds(1500); Serial.println("CENTRE");        break;
  }
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  esc.attach(9, 1000, 2000);
  steering.attach(10, 1000, 2000);
  esc.writeMicroseconds(1500);
  steering.writeMicroseconds(1500);
  Serial.println("Holding neutral - plug in battery NOW");
  delay(5000);
  Serial.println("ESC armed");

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
  // WiFi UDP (Jetson autonomous)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char cmd = udp.read();
    handleCmd(cmd);
  }

  // Serial USB (laptop keyboard)
  if (Serial.available()) {
    char cmd = Serial.read();
    handleCmd(cmd);
  }
}
