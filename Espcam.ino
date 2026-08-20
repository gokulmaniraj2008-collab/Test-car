/*
  ESP32-CAM -> Supabase Live Feed

  Captures frames continuously and pushes each one to a Supabase table
  (car_camera, single row id=1). The webpage holds a Realtime websocket
  subscription to that row, so as soon as this board writes a new frame
  Supabase pushes it straight to the browser — no polling, no local
  network requirement. Frame rate depends on JPEG size + your WiFi/
  internet round-trip, typically a few fps.

  Board: AI-Thinker ESP32-CAM

  SETUP (run once, in Supabase SQL Editor):
    See supabase_setup.sql in this project — creates the car_camera
    table, RLS policies, and enables Realtime on it.

  SETUP (this board):
  1. Fill in WIFI_SSID / WIFI_PASSWORD / supabaseUrl / apiKey below
     (same Supabase project + key as Esp32.ino)
  2. Flash this sketch (GPIO0 -> GND while flashing, remove after, RST)
  3. Open Serial Monitor (115200 baud) to confirm it's uploading frames
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "base64.h"

// ---------- CONFIG ----------
const char* WIFI_SSID     = "AGRIBOT_WIFI";
const char* WIFI_PASSWORD = "12345678";

// Same Supabase project + key as Esp32.ino
const char* supabaseUrl = "https://hvnasippwadzygnaodpp.supabase.co/rest/v1/car_camera?id=eq.1";
const char* apiKey      = "sb_publishable_ZFFW9ifODSHTwOPlOBWhqw_G8H7FACO";
// -----------------------------

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Small frames = faster uploads = smoother "live" feel over the network.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 14;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_QQVGA; // 160x120 - keeps payloads small on boards without PSRAM
    config.jpeg_quality = 16;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  initCamera();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  Serial.println("Pushing frames to Supabase...");
}

void pushFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  String b64 = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  String body = "{\"frame\":\"" + b64 + "\"}";

  HTTPClient http;
  http.begin(supabaseUrl);
  http.addHeader("apikey", apiKey);
  http.addHeader("Authorization", String("Bearer ") + apiKey);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");

  int code = http.PATCH(body);
  if (code != 204 && code != 200) {
    Serial.println("Upload failed, HTTP code: " + String(code));
  }
  http.end();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    pushFrame();
  } else {
    Serial.println("WiFi disconnected, retrying...");
    delay(1000);
  }
  // No extra delay: next capture starts immediately after upload completes.
  // Real-world fps is limited by JPEG size + network round-trip.
}
