/*
  AGRIBOT-01 — Main Rover Firmware
  Board: ESP32 (any Dev Module variant)

  Talks to Supabase via REST (PostgREST) over HTTPS:
    - Polls  robot_commands  for the newest command and executes it
    - Pushes robot_status    (heartbeat, telemetry) on an interval
    - Pushes sensor_data     (DHT22, soil, ultrasonic, battery, GPS) on an interval

  Matches the schema used by your index.html dashboard:
    robot_status(robot_id, name, online, mode, motor_state, speed_value,
                 pump_status, gps_fix, gps_satellites, safety_stopped,
                 last_fault, camera_ip, updated_at)
    sensor_data(robot_id, battery_percent, battery_voltage, soil_moisture,
                temperature, humidity, distance_cm, created_at)
    robot_commands(robot_id, command, value, created_at)
      command values in use: forward, backward, left, right, stop, set_speed,
                              pump_on, pump_off

  NOTE ON SECURITY:
  Uses the SUPABASE_ANON_KEY (same one embedded in your dashboard), relying on
  RLS policies for robot_status/sensor_data/robot_commands to permit the
  ESP32's inserts/updates. Do NOT hardcode a service_role key on the device —
  it bypasses RLS entirely and anyone who dumps the firmware gets full DB access.

  Libraries required (Library Manager):
    - ArduinoJson (by Benoit Blanchon)
    - DHT sensor library (by Adafruit) + Adafruit Unified Sensor
    - TinyGPSPlus (by Mikal Hart)          [optional GPS]
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <TinyGPSPlus.h>

// ----------------------------------------------------------------------------------
// CONFIG — WiFi
// ----------------------------------------------------------------------------------
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ----------------------------------------------------------------------------------
// CONFIG — Supabase (same project/key your dashboard uses)
// ----------------------------------------------------------------------------------
const char* SUPABASE_URL      = "https://hvnasippwadzygnaodpp.supabase.co";
const char* SUPABASE_ANON_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imh2bmFzaXBwd2FkenlnbmFvZHBwIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzU5Mjg3NDMsImV4cCI6MjA5MTUwNDc0M30.dcS0J77idvjkwNesRJS7C-LfmhSDlILASMK65AesRaM";
const char* ROBOT_ID          = "agribot-01";
const char* ROBOT_NAME        = "AGRIBOT-01";

// If you're running the ESP32-CAM companion sketch, put its stream URL here so
// robot_status.camera_ip gets populated (leave blank if no camera yet).
const char* CAMERA_STREAM_URL = "";

// ----------------------------------------------------------------------------------
// CONFIG — Pin map (edit to match your wiring)
// ----------------------------------------------------------------------------------
// L298N / TB6612-style dual motor driver
#define PIN_IN1   26   // left motor forward
#define PIN_IN2   27   // left motor backward
#define PIN_IN3   25   // right motor forward
#define PIN_IN4   33   // right motor backward
#define PIN_ENA   14   // left motor PWM enable
#define PIN_ENB   32   // right motor PWM enable

// Relay for irrigation pump
#define PIN_PUMP  13

// HC-SR04 ultrasonic
#define PIN_TRIG  5
#define PIN_ECHO  18

// DHT22
#define PIN_DHT   4
#define DHTTYPE   DHT22

// Capacitive soil moisture sensor (analog)
#define PIN_SOIL  34

// Battery sense (voltage divider into ADC, e.g. 2x 100k for 2S/3S packs)
#define PIN_BATT  35
#define BATT_DIVIDER_RATIO 2.0f   // (R1+R2)/R2 — calibrate to your divider
#define BATT_ADC_VREF      3.3f
#define BATT_FULL_VOLTAGE  8.4f   // adjust to your pack chemistry/cell count
#define BATT_EMPTY_VOLTAGE 6.0f

// GPS (NEO-6M) on UART2
#define GPS_RX_PIN 16   // ESP32 RX2 <- GPS TX
#define GPS_TX_PIN 17   // ESP32 TX2 -> GPS RX
#define GPS_BAUD   9600

// Safety stop distance (cm) — obstacle inside this range forces a stop
#define SAFETY_DISTANCE_CM 15

// ----------------------------------------------------------------------------------
// Timing
// ----------------------------------------------------------------------------------
const unsigned long CMD_POLL_INTERVAL_MS    = 300;    // how often to check for new drive commands
const unsigned long STATUS_PUSH_INTERVAL_MS = 2000;   // robot_status heartbeat
const unsigned long SENSOR_PUSH_INTERVAL_MS = 5000;   // sensor_data insert
const unsigned long CMD_TIMEOUT_MS          = 800;    // stop driving if no command refresh within this window

// ----------------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------------
DHT dht(PIN_DHT, DHTTYPE);
HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

String lastCommandId = "";      // last robot_commands row id we've already executed
String currentMode   = "manual";
String currentMotorState = "stopped";
int    currentSpeed  = 180;
bool   pumpOn        = false;
bool   safetyStopped = false;
String lastFault     = "";
unsigned long lastCmdReceivedAt = 0;

unsigned long lastCmdPoll   = 0;
unsigned long lastStatusPush = 0;
unsigned long lastSensorPush = 0;

// Sensor cache (updated each sensor cycle, also used for self-check)
bool dhtOk = false, ultrasonicOk = false, gpsOk = false, soilOk = false;

// ====================================================================================
// SETUP
// ====================================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[AGRIBOT] Booting...");

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  dht.begin();
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  stopMotors();
  connectWiFi();
  systemSelfCheck();

  lastCmdReceivedAt = millis();
  pushStatus(true /*forceOnline*/);
}

// ====================================================================================
// LOOP
// ====================================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  unsigned long now = millis();

  // Safety: obstacle check happens every loop, independent of network timing
  float distCm = readUltrasonicCm();
  if (distCm > 0 && distCm < SAFETY_DISTANCE_CM && currentMotorState == "forward") {
    if (!safetyStopped) {
      safetyStopped = true;
      lastFault = "obstacle_detected";
      stopMotors();
      pushStatus(true);
      Serial.println("[SAFETY] Obstacle detected, forcing stop.");
    }
  } else if (distCm >= SAFETY_DISTANCE_CM || currentMotorState != "forward") {
    if (safetyStopped) {
      safetyStopped = false;
      pushStatus(true);
    }
  }

  // Command watchdog: if we haven't heard from Supabase in a while, stop driving
  if (currentMotorState != "stopped" && (now - lastCmdReceivedAt > CMD_TIMEOUT_MS)) {
    Serial.println("[WATCHDOG] Command timeout, stopping.");
    stopMotors();
  }

  if (now - lastCmdPoll >= CMD_POLL_INTERVAL_MS) {
    lastCmdPoll = now;
    pollCommands();
  }

  if (now - lastStatusPush >= STATUS_PUSH_INTERVAL_MS) {
    lastStatusPush = now;
    pushStatus(false);
  }

  if (now - lastSensorPush >= SENSOR_PUSH_INTERVAL_MS) {
    lastSensorPush = now;
    pushSensorData();
  }
}

// ====================================================================================
// WiFi
// ====================================================================================
void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Failed to connect, will retry in loop.");
  }
}

// ====================================================================================
// Motor control
// ====================================================================================
void setMotors(bool leftFwd, bool leftRev, bool rightFwd, bool rightRev, int speed) {
  digitalWrite(PIN_IN1, leftFwd);
  digitalWrite(PIN_IN2, leftRev);
  digitalWrite(PIN_IN3, rightFwd);
  digitalWrite(PIN_IN4, rightRev);
  analogWrite(PIN_ENA, speed);
  analogWrite(PIN_ENB, speed);
}

void stopMotors() {
  setMotors(false, false, false, false, 0);
  currentMotorState = "stopped";
}

void driveForward()  { setMotors(true, false, true, false, currentSpeed);  currentMotorState = "forward";  }
void driveBackward() { setMotors(false, true, false, true, currentSpeed);  currentMotorState = "backward"; }
void driveLeft()      { setMotors(false, true, true, false, currentSpeed); currentMotorState = "left";     }
void driveRight()     { setMotors(true, false, false, true, currentSpeed); currentMotorState = "right";    }

// ====================================================================================
// Sensors
// ====================================================================================
float readUltrasonicCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 25000); // 25ms timeout (~4m range)
  if (duration == 0) {
    ultrasonicOk = false;
    return -1; // sensor not responding / out of range
  }
  ultrasonicOk = true;
  return duration * 0.0343f / 2.0f;
}

int readSoilPercent() {
  int raw = analogRead(PIN_SOIL); // 0-4095 on ESP32
  // Calibrate these bounds to your sensor: dry ~= high raw, wet ~= low raw (typical capacitive sensor)
  const int DRY_RAW = 3000;
  const int WET_RAW = 1200;
  if (raw <= 0 || raw >= 4095) { soilOk = false; return -1; }
  soilOk = true;
  int pct = map(raw, DRY_RAW, WET_RAW, 0, 100);
  pct = constrain(pct, 0, 100);
  return pct;
}

float batteryVoltage() {
  int raw = analogRead(PIN_BATT);
  float v = (raw / 4095.0f) * BATT_ADC_VREF * BATT_DIVIDER_RATIO;
  return v;
}

int batteryPercent(float v) {
  float pct = (v - BATT_EMPTY_VOLTAGE) / (BATT_FULL_VOLTAGE - BATT_EMPTY_VOLTAGE) * 100.0f;
  return constrain((int)pct, 0, 100);
}

void updateGPS() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }
  gpsOk = gps.location.isValid();
}

// ====================================================================================
// Boot-time self check
// ====================================================================================
void systemSelfCheck() {
  Serial.println("[SELFCHECK] Running...");

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  dhtOk = !isnan(t) && !isnan(h);
  Serial.printf("  DHT22: %s\n", dhtOk ? "OK" : "FAIL (null reading)");

  float d = readUltrasonicCm();
  Serial.printf("  Ultrasonic: %s\n", ultrasonicOk ? "OK" : "FAIL (no echo)");

  int soil = readSoilPercent();
  Serial.printf("  Soil sensor: %s\n", soilOk ? "OK" : "FAIL / pinned");

  unsigned long gpsCheckStart = millis();
  while (millis() - gpsCheckStart < 1500) { // brief window to catch NMEA sentences
    while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());
  }
  gpsOk = gps.charsProcessed() > 10; // module is at least talking, even w/o fix
  Serial.printf("  GPS UART: %s\n", gpsOk ? "OK (data received)" : "FAIL (no NMEA data)");

  if (!dhtOk || !ultrasonicOk || !soilOk || !gpsOk) {
    lastFault = "selfcheck_sensor_fault";
  }
  Serial.println("[SELFCHECK] Done.");
}

// ====================================================================================
// Supabase — poll robot_commands
// ====================================================================================
void pollCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure(); // TLS without cert validation - fine for PostgREST, swap for setCACert() if you want strict validation

  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/robot_commands?robot_id=eq." + ROBOT_ID +
               "&order=created_at.desc&limit=1&select=id,command,value,created_at";

  http.begin(client, url);
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);

  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err && doc.is<JsonArray>() && doc.size() > 0) {
      JsonObject row = doc[0];
      String id = row["id"].as<String>();
      String command = row["command"].as<String>();

      if (id != lastCommandId) {
        lastCommandId = id;
        lastCmdReceivedAt = millis();
        executeCommand(command, row["value"]);
      }
    }
  } else if (code > 0) {
    Serial.printf("[CMD] GET failed, HTTP %d\n", code);
  }
  http.end();
}

void executeCommand(const String& command, JsonVariant value) {
  Serial.printf("[CMD] %s\n", command.c_str());

  if (safetyStopped && command == "forward") {
    Serial.println("[CMD] Ignored forward — safety stop active.");
    return;
  }

  if (command == "forward")       driveForward();
  else if (command == "backward") driveBackward();
  else if (command == "left")     driveLeft();
  else if (command == "right")    driveRight();
  else if (command == "stop")     stopMotors();
  else if (command == "set_speed") {
    if (!value.isNull()) {
      currentSpeed = constrain(value.as<int>(), 0, 255);
      // re-apply to motors if currently moving so speed change takes effect immediately
      if (currentMotorState == "forward")  driveForward();
      else if (currentMotorState == "backward") driveBackward();
      else if (currentMotorState == "left")     driveLeft();
      else if (currentMotorState == "right")    driveRight();
    }
  }
  else if (command == "pump_on") {
    pumpOn = true;
    digitalWrite(PIN_PUMP, HIGH);
  }
  else if (command == "pump_off") {
    pumpOn = false;
    digitalWrite(PIN_PUMP, LOW);
  }

  pushStatus(false); // reflect the new state promptly
}

// ====================================================================================
// Supabase — upsert robot_status
// ====================================================================================
void pushStatus(bool force) {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String url = String(SUPABASE_URL) + "/rest/v1/robot_status?on_conflict=robot_id";
  http.begin(client, url);
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "resolution=merge-duplicates,return=minimal");

  StaticJsonDocument<512> doc;
  doc["robot_id"]        = ROBOT_ID;
  doc["name"]             = ROBOT_NAME;
  doc["online"]           = true;
  doc["mode"]             = currentMode;
  doc["motor_state"]      = currentMotorState;
  doc["speed_value"]      = currentSpeed;
  doc["pump_status"]      = pumpOn;
  doc["gps_fix"]          = gpsOk && gps.location.isValid();
  doc["gps_satellites"]   = gps.satellites.isValid() ? gps.satellites.value() : 0;
  doc["safety_stopped"]   = safetyStopped;
  doc["last_fault"]       = lastFault;
  if (strlen(CAMERA_STREAM_URL) > 0) doc["camera_ip"] = CAMERA_STREAM_URL;

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  if (code != 200 && code != 201 && code != 204) {
    Serial.printf("[STATUS] Push failed, HTTP %d: %s\n", code, http.getString().c_str());
  }
  http.end();
}

// ====================================================================================
// Supabase — insert sensor_data
// ====================================================================================
void pushSensorData() {
  if (WiFi.status() != WL_CONNECTED) return;

  updateGPS();
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  dhtOk = !isnan(temp) && !isnan(hum);

  int soil = readSoilPercent();
  float dist = readUltrasonicCm();
  float vbatt = batteryVoltage();
  int battPct = batteryPercent(vbatt);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String url = String(SUPABASE_URL) + "/rest/v1/sensor_data";
  http.begin(client, url);
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");

  StaticJsonDocument<512> doc;
  doc["robot_id"]         = ROBOT_ID;
  doc["battery_percent"]  = battPct;
  doc["battery_voltage"]  = vbatt;
  if (soilOk) doc["soil_moisture"] = soil;
  if (dhtOk) {
    doc["temperature"] = temp;
    doc["humidity"]    = hum;
  }
  if (ultrasonicOk && dist > 0) doc["distance_cm"] = dist;

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  if (code != 200 && code != 201 && code != 204) {
    Serial.printf("[SENSOR] Push failed, HTTP %d: %s\n", code, http.getString().c_str());
  }
  http.end();
}
