/*
  Rover Drive Controller — plain ESP32 + L298N
  ---------------------------------------------
  Polls the `car_commands` Supabase table (row id=1) and drives an
  L298N-controlled differential-drive rover.

  Matches the web control panel, which does:
    PATCH {SUPA_URL}/rest/v1/car_commands?id=eq.1
    body: { direction: "forward"|"backward"|"left"|"right"|"stop", speed: 0-255, updated_at }

  REQUIRED LIBRARIES (install via Library Manager):
    - ArduinoJson (by Benoit Blanchon), v6.x or v7.x

  REQUIRED SUPABASE SETUP:
    - Table car_commands(id int primary key, direction text, speed int, updated_at timestamptz)
      with a row already inserted where id = 1
    - RLS policy allowing the anon role to SELECT car_commands (id=1)

  !! PIN ASSIGNMENTS ARE PLACEHOLDERS — verify against your own wiring before flashing. !!
*/

#include <WiFi.h>
#include <NetworkClientSecure.h>   // core 3.x: use this instead of WiFiClientSecure.h
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ---------- CONFIGURE ME ----------
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* SUPA_URL = "https://xxxxx.supabase.co";   // no trailing slash
const char* SUPA_KEY = "YOUR_ANON_PUBLISHABLE_KEY";   // same anon key the webpage uses

const unsigned long POLL_INTERVAL_MS = 200;  // how often to check for new commands

// L298N pin mapping — CHECK THIS AGAINST YOUR WIRING
const int LEFT_IN1  = 26;
const int LEFT_IN2  = 27;
const int LEFT_EN   = 14;   // PWM (ENA)
const int RIGHT_IN1 = 25;
const int RIGHT_IN2 = 33;
const int RIGHT_EN  = 32;   // PWM (ENB)

// LEDC (PWM) config — core 3.x LEDC API is pin-based, not channel-based
const int PWM_FREQ       = 5000;
const int PWM_RESOLUTION = 8;      // 8-bit -> 0-255, matches the webpage's speed slider
// -----------------------------------

String lastUpdatedAt = "";

void setMotor(int in1, int in2, int enPin, int speed) {
  // speed: -255..255, sign = direction, magnitude = PWM duty
  int duty = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(enPin, duty); // core 3.x: ledcWrite takes the pin number, not a channel
}

void drive(const String& direction, int speed) {
  speed = constrain(speed, 0, 255);
  if (direction == "forward") {
    setMotor(LEFT_IN1, LEFT_IN2, LEFT_EN, speed);
    setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, speed);
  } else if (direction == "backward") {
    setMotor(LEFT_IN1, LEFT_IN2, LEFT_EN, -speed);
    setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, -speed);
  } else if (direction == "left") {
    // pivot turn: left wheel back, right wheel forward
    setMotor(LEFT_IN1, LEFT_IN2, LEFT_EN, -speed);
    setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, speed);
  } else if (direction == "right") {
    setMotor(LEFT_IN1, LEFT_IN2, LEFT_EN, speed);
    setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, -speed);
  } else { // "stop" or unrecognized
    setMotor(LEFT_IN1, LEFT_IN2, LEFT_EN, 0);
    setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, 0);
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());
}

void pollAndDrive() {
  if (WiFi.status() != WL_CONNECTED) return;

  NetworkClientSecure client;
  client.setInsecure(); // skips TLS cert verification — acceptable for prototyping, not for production

  HTTPClient http;
  String url = String(SUPA_URL) + "/rest/v1/car_commands?id=eq.1&select=direction,speed,updated_at";

  if (!http.begin(client, url)) {
    Serial.println("http.begin failed");
    return;
  }
  http.addHeader("apikey", SUPA_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPA_KEY);

  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    JsonDocument doc; // ArduinoJson v7 style; use DynamicJsonDocument<256> doc; on v6
    DeserializationError err = deserializeJson(doc, payload);
    if (!err && doc.is<JsonArray>() && doc.size() > 0) {
      JsonObject row = doc[0];
      String direction = row["direction"] | "stop";
      int speed = row["speed"] | 0;
      String updatedAt = row["updated_at"] | "";

      if (updatedAt != lastUpdatedAt) {
        lastUpdatedAt = updatedAt;
        Serial.printf("New command: %s @ %d\n", direction.c_str(), speed);
        drive(direction, speed);
      }
    } else {
      Serial.println("JSON parse error or empty row");
    }
  } else {
    Serial.printf("GET failed, HTTP code: %d\n", code);
    if (code == 401 || code == 403) {
      Serial.println("-> check anon key and RLS SELECT policy on car_commands");
    }
  }
  http.end();
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  ledcAttach(LEFT_EN, PWM_FREQ, PWM_RESOLUTION);   // core 3.x: combines old setup+attach, keyed by pin
  ledcAttach(RIGHT_EN, PWM_FREQ, PWM_RESOLUTION);

  drive("stop", 0);

  connectWiFi();
}

void loop() {
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    pollAndDrive();
  }

  if (WiFi.status() != WL_CONNECTED) {
    drive("stop", 0); // fail-safe: stop if WiFi drops
    connectWiFi();
  }
}
