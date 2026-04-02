/*
  RC Car Motor Controller — Arduino Uno R4 WiFi
  ESC      -> pin 9
  Servo    -> pin 10
  WiFi UDP -> port 5005
  Static IP: 172.20.10.3

  Reverse sequence:
    BRAKE (1300us, 200ms) -> NEUTRAL (1500us, 300ms) -> REVERSE (1300us held)
  Once armed, reverse is held until any other throttle command cancels it.
  Incoming 'r' commands while already reversing are ignored safely.
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

// ── Static IP configuration ───────────────────────────────────
IPAddress staticIP (172, 20, 10, 3);
IPAddress gateway  (172, 20, 10, 1);
IPAddress subnet   (255, 255, 255, 0);
IPAddress dns      (8,   8,   8,  8);
// ─────────────────────────────────────────────────────────────

enum ReverseState { IDLE, BRAKE, NEUTRAL_WAIT, REVERSING };
ReverseState  revState    = IDLE;
unsigned long revTimer    = 0;
bool          reverseArmed = false;

void startReverse() {
  if (revState != IDLE) return;
  reverseArmed = false;
  revState     = BRAKE;
  revTimer     = millis();
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
  else if (revState == NEUTRAL_WAIT && now - revTimer > 300) {
    esc.writeMicroseconds(1300);
    revState     = REVERSING;
    reverseArmed = true;
    Serial.println("ESC REVERSE ARMED");
  }
}

void cancelReverse() {
  revState     = IDLE;
  reverseArmed = false;
}

void handleCmd(char cmd) {
  switch (cmd) {

    case 'f':
      cancelReverse();
      esc.writeMicroseconds(1750);
      Serial.println("FWD");
      break;

    case 'm':
      cancelReverse();
      esc.writeMicroseconds(1580);
      Serial.println("SLOW");
      break;

    case 's':
      cancelReverse();
      esc.writeMicroseconds(1500);
      Serial.println("STOP");
      break;

    case 'r':
      if (!reverseArmed) startReverse();
      Serial.println("REV");
      break;

    case 'a':
      steering.writeMicroseconds(1200);
      Serial.println("LEFT");
      break;

    case 'd':
      steering.writeMicroseconds(1800);
      Serial.println("RIGHT");
      break;

    case 'c':
      steering.writeMicroseconds(1500);
      Serial.println("CTR");
      break;
  }
}

void setup() {
  Serial.begin(9600);

  esc.attach(9, 1000, 2000);
  steering.attach(10, 1000, 2000);
  esc.writeMicroseconds(1500);
  steering.writeMicroseconds(1500);

  Serial.println("Holding neutral - plug in battery NOW");
  delay(5000);
  Serial.println("ESC armed - connecting WiFi");

  // Configure static IP before connecting
  WiFi.config(staticIP, dns, gateway, subnet);

  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500); Serial.print("."); attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
    Serial.println("Ready - listening on UDP 5005");
  } else {
    Serial.println("\nWiFi FAILED - check SSID/password");
  }
}

void loop() {
  updateReverse();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char cmd = udp.read();
    handleCmd(cmd);
  }
}
