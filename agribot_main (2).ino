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
#include <WebServer.h>
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

// Motor current sensing — wire to your L298N's "Current Sensing A / B" pins
// (if your board breaks them out). Used to detect a disconnected/dead motor
// channel: ENA/IN1/IN2/left-motor wiring -> Motor A, ENB/IN3/IN4/right-motor -> Motor B.
#define PIN_CURR_A 36   // ADC1_CH0, input-only pin, safe to read alongside WiFi
#define PIN_CURR_B 39   // ADC1_CH3, input-only pin, safe to read alongside WiFi
#define CURRENT_FAULT_ADC_THRESHOLD 80    // raw ADC (0-4095) below this while driving = "no current"
#define CURRENT_CHECK_SETTLE_MS     150   // wait this long after a drive command before judging current

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
WebServer localServer(80);   // on-board control page — open the ESP32's IP in a browser

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

// Motor channel fault state (current-sense based)
bool motorAFault = false;   // true = "ENA/IN1/IN2/left motor: not connected or not moving"
bool motorBFault = false;   // true = "ENB/IN3/IN4/right motor: not connected or not moving"
unsigned long motorStateChangedAt = 0;   // when currentMotorState last changed, for settle-time gating

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
  startLocalServer();

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

  localServer.handleClient();

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

  // Motor channel fault check (current-sense based, e.g. ENB unplugged)
  checkMotorCurrent();

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
    Serial.println("========================================");
    Serial.print("  AGRIBOT CONTROL PAGE: http://");
    Serial.println(WiFi.localIP());
    Serial.println("  Open that address in a browser to drive.");
    Serial.println("========================================");
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
  motorStateChangedAt = millis();
}

void stopMotors() {
  setMotors(false, false, false, false, 0);
  currentMotorState = "stopped";
  motorAFault = false;   // no fault when intentionally stopped
  motorBFault = false;
}

void driveForward()  { setMotors(true, false, true, false, currentSpeed);  currentMotorState = "forward";  }
void driveBackward() { setMotors(false, true, false, true, currentSpeed);  currentMotorState = "backward"; }
void driveLeft()      { setMotors(false, true, true, false, currentSpeed); currentMotorState = "left";     }
void driveRight()     { setMotors(true, false, false, true, currentSpeed); currentMotorState = "right";    }

// ====================================================================================
// Local web control page — served directly by the ESP32, no Supabase needed
// Open the IP printed on boot (e.g. http://192.168.1.50) in a browser.
// ====================================================================================
const char LOCAL_CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AGRIBOT Control</title>
  <style>
    body { font-family: sans-serif; text-align: center; background: #111; color: #eee; margin: 0; padding: 20px; }
    h2 { margin-bottom: 4px; }
    .status { color: #8f8; font-size: 14px; margin-bottom: 20px; }
    .pad { display: grid; grid-template-columns: 80px 80px 80px; grid-gap: 10px; justify-content: center; margin-bottom: 20px; }
    button { font-size: 20px; padding: 20px 0; border-radius: 10px; border: none; background: #2a2a2a; color: #fff; }
    button:active { background: #4caf50; }
    .stop { background: #a33; }
    .pump { font-size: 16px; padding: 15px 25px; border-radius: 10px; border: none; margin: 5px; }
    .pump-on { background: #2a7; color: #fff; }
    .pump-off { background: #555; color: #fff; }
  </style>
</head>
<body>
  <h2>AGRIBOT-01</h2>
  <div class="status" id="st">loading...</div>
  <div class="pad">
    <div></div><button onclick="cmd('forward')">&#8593;</button><div></div>
    <button onclick="cmd('left')">&#8592;</button>
    <button class="stop" onclick="cmd('stop')">STOP</button>
    <button onclick="cmd('right')">&#8594;</button>
    <div></div><button onclick="cmd('backward')">&#8595;</button><div></div>
  </div>
  <div>
    <button class="pump pump-on" onclick="cmd('pump_on')">Pump ON</button>
    <button class="pump pump-off" onclick="cmd('pump_off')">Pump OFF</button>
  </div>
  <script>
    function cmd(c) {
      fetch('/cmd?action=' + c).then(r => r.text()).then(t => document.getElementById('st').innerText = t);
    }
    document.getElementById('st').innerText = 'ready';
  </script>
</body>
</html>
)HTML";

void handleLocalRoot() {
  localServer.send(200, "text/html", LOCAL_CONTROL_PAGE);
}

void handleLocalCmd() {
  if (!localServer.hasArg("action")) {
    localServer.send(400, "text/plain", "missing action");
    return;
  }
  String action = localServer.arg("action");
  lastCmdReceivedAt = millis(); // counts as a live command, resets the watchdog

  if (safetyStopped && action == "forward") {
    localServer.send(200, "text/plain", "blocked: obstacle");
    return;
  }

  if (action == "forward")       driveForward();
  else if (action == "backward") driveBackward();
  else if (action == "left")     driveLeft();
  else if (action == "right")    driveRight();
  else if (action == "stop")     stopMotors();
  else if (action == "pump_on")  { pumpOn = true;  digitalWrite(PIN_PUMP, HIGH); }
  else if (action == "pump_off") { pumpOn = false; digitalWrite(PIN_PUMP, LOW);  }
  else { localServer.send(400, "text/plain", "unknown action"); return; }

  pushStatus(false); // keep the Supabase dashboard in sync too
  localServer.send(200, "text/plain", "ok: " + action + " | mode: " + currentMotorState);
}

void startLocalServer() {
  localServer.on("/", handleLocalRoot);
  localServer.on("/cmd", handleLocalCmd);
  localServer.begin();
  Serial.println("[WEB] Local control server started on port 80.");
}

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

// Reads the L298N current-sense pins and flags a channel as faulted if we
// commanded it to drive but it's pulling ~no current — almost always means
// ENA/IN1/IN2 (Motor A) or ENB/IN3/IN4 (Motor B) is unplugged, or the motor
// itself is disconnected/dead. Only judges after a short settle window so we
// don't false-trigger on the instant a command lands.
void checkMotorCurrent() {
  if (currentMotorState == "stopped") return;
  if (millis() - motorStateChangedAt < CURRENT_CHECK_SETTLE_MS) return;

  int currA = analogRead(PIN_CURR_A);
  int currB = analogRead(PIN_CURR_B);

  bool aFaultNow = currA < CURRENT_FAULT_ADC_THRESHOLD;
  bool aChanged  = aFaultNow != motorAFault;
  motorAFault = aFaultNow;

  bool bFaultNow = currB < CURRENT_FAULT_ADC_THRESHOLD;
  bool bChanged  = bFaultNow != motorBFault;
  motorBFault = bFaultNow;

  if (aChanged || bChanged) {
    if (motorAFault && motorBFault) lastFault = "motor_a_and_b_disconnected";
    else if (motorAFault)           lastFault = "motor_a_disconnected"; // ENA/IN1/IN2/left motor
    else if (motorBFault)           lastFault = "motor_b_disconnected"; // ENB/IN3/IN4/right motor
    else                             lastFault = "";

    Serial.printf("[MOTOR] A:%s (adc=%d)  B:%s (adc=%d)\n",
                  motorAFault ? "FAULT-NOT MOVING" : "ok", currA,
                  motorBFault ? "FAULT-NOT MOVING" : "ok", currB);
    pushStatus(true); // push immediately so the dashboard reflects it right away
  }
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
  doc["motor_a_fault"]    = motorAFault;   // ENA / IN1 / IN2 / left motor path
  doc["motor_b_fault"]    = motorBFault;   // ENB / IN3 / IN4 / right motor path
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
