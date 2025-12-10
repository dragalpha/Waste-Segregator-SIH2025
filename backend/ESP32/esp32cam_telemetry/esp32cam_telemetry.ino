// ESP32-CAM capture + POST image to backend /image endpoint
// Receives JSON response with fields including prediction and optionally waste_state/hazard
// Update WiFi credentials and server URL before use.

#include "WiFi.h"
#include "HTTPClient.h"
#include "esp_camera.h"

// === EDIT THESE ===
const char* ssid = "Ratul 2";
const char* password = "12345678";
const char* serverUrl = "http://10.160.138.53:8000/image"; // change to your PC IP

// Onboard LED pin (change for your board)
const int HAZARD_LED_PIN = 33; // Example GPIO for ESP32-CAM dev boards

// Camera pin config for AI-Thinker module (common ESP32-CAM)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


void startCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
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
  // init with high specs to capture better images
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
}

String httpPOSTImage(String url, uint8_t * buf, size_t bufLen) {
  HTTPClient http;
  String boundary = "----ESP32CAMBoundary";
  String head = "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"image\"; filename=\"frame.jpg\"\r\n";
  head += "Content-Type: image/jpeg\r\n\r\n";

  http.begin(url);
  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
  int contentLength = head.length() + bufLen + 4 + boundary.length() + 8;

  WiFiClient * stream = http.getStreamPtr();
  int httpCode = http.sendRequest("POST", (uint8_t*)NULL, 0); // trigger headers
  // Note: ArduinoHTTPClient doesn't expose low-level multipart send easily; we'll use alternative
  // Instead, use HTTPClient::beginStream to write raw body if platform supports it.

  // Fallback: use simple POST with image as body and Content-Type image/jpeg
  http.end();
  http.begin(url);
  http.addHeader("Content-Type", "image/jpeg");
  int code = http.POST(buf, bufLen);
  String payload = "";
  if (code > 0) {
    payload = http.getString();
  } else {
    Serial.printf("POST failed, error: %d\n", code);
  }
  http.end();
  return payload;
}

void setup(){
  Serial.begin(115200);
  pinMode(HAZARD_LED_PIN, OUTPUT);
  digitalWrite(HAZARD_LED_PIN, LOW);

  WiFi.begin(ssid, password);
  Serial.printf("Connecting to %s", ssid);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tries++;
    if (tries > 60) {
      Serial.println("\nFailed to connect to WiFi");
      // restart or decide to continue trying
      tries = 0;
    }
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  startCamera();
  Serial.println("Camera initialized");
}

void loop(){
  // capture frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    return;
  }

  // send to server
  String resp = httpPOSTImage(String(serverUrl), fb->buf, fb->len);
  esp_camera_fb_return(fb);

  if (resp.length() > 0) {
    Serial.println("Server response:");
    Serial.println(resp);

    // look for simple JSON fields: prediction, waste_state, hazard
    // naive parsing to avoid adding ArduinoJson dependency here
    if (resp.indexOf("waste_state") >= 0 || resp.indexOf("soil_state") >= 0) {
      if (resp.indexOf("\"wet\"") >= 0) {
        Serial.println("Waste: WET");
      } else if (resp.indexOf("\"dry\"") >= 0) {
        Serial.println("Waste: DRY");
      }
    }
    if (resp.indexOf("\"hazard\": 1") >= 0 || resp.indexOf("\"hazard\":1") >= 0) {
      Serial.println("HAZARD detected -> Turning on LED");
      digitalWrite(HAZARD_LED_PIN, HIGH);
    } else {
      digitalWrite(HAZARD_LED_PIN, LOW);
    }
  } else {
    Serial.println("Empty response from server");
  }

  // wait a while between captures
  delay(1500);
}
