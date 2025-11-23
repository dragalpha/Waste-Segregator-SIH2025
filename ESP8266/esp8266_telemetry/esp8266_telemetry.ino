// ESP8266 Full SIH Mock Version (no sensors required) - HTTPClient fixed
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// ======== CHANGE THESE ========
const char* ssid     = "Akash";
const char* password = "987654321";
String server        = "http://10.64.159.159:8000";
// ==============================

// Generate mock sensor values
float mock_mq135() { return random(200, 800); }
float mock_mq2()   { return random(150, 900); }
float mock_soil_dry() { return random(10, 80); }
float mock_soil_wet() { return random(20, 95); }
float mock_loadcell() { return random(200, 5000); }
float mock_tds() { return random(300, 700); }
float mock_ultrasonic() { return random(5, 200); }
int   mock_proximity() { return random(0, 2); }
float mock_heading() { return random(0, 359); }
float mock_battery() { return 3.7; }
int   mock_rssi() { return WiFi.RSSI(); }

// MOCK GPS (fixed location)
String mock_lat() { return "22.5726"; }
String mock_lon() { return "88.3639"; }

// Poll YOLO detection from backend (fixed to use WiFiClient)
String get_detection() {
  if (WiFi.status() != WL_CONNECTED) return "-";
  WiFiClient client;
  HTTPClient http;
  String url = server + "/detected";
  http.begin(client, url.c_str());
  int code = http.GET();
  String result = "-";

  if (code == 200) {
    String payload = http.getString(); // {"prediction":"bottle:0.78"}
    int i = payload.indexOf(":");
    int j = payload.lastIndexOf("}");
    if (i > 0 && j > i) {
      result = payload.substring(i + 2, j - 1);
    }
  }

  http.end();
  return result;
}

// POST telemetry JSON to backend (uses WiFiClient too)
void sendTelemetry(String detected) {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClient client;
  HTTPClient http;
  String url = server + "/telemetry";
  http.begin(client, url.c_str());
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<1024> doc;

  doc["device_id"] = WiFi.macAddress();
  doc["boat_id"] = "boat1";

  doc["lat"] = mock_lat();
  doc["lon"] = mock_lon();
  doc["heading_deg"] = mock_heading();

  doc["mq135_ppm"] = mock_mq135();
  doc["mq2_ppm"] = mock_mq2();
  doc["soil_dry_belt_pct"] = mock_soil_dry();
  doc["soil_wet_belt_pct"] = mock_soil_wet();

  doc["loadcell_grams"] = mock_loadcell();
  doc["tds_ppm"] = mock_tds();
  doc["ultrasonic_cm"] = mock_ultrasonic();
  doc["proximity_inductive"] = mock_proximity();

  doc["yolo_raw"] = detected;
  doc["waste_category"] = "-";
  doc["waste_subtype"] = "-";

  doc["collection_event"] = 0;
  doc["collection_bin_id"] = "-";

  doc["battery_volt"] = mock_battery();
  doc["rssi"] = mock_rssi();

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.printf("POST /telemetry = %d\n", code);
  if (code > 0) {
    Serial.println(http.getString());
  } else {
    Serial.println("POST failed");
  }

  http.end();
}

// Setup
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Booting mock SIH telemetry system...");
  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 60) {
    delay(300);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi FAILED.");
  }
}

// Loop
void loop() {
  String det = get_detection();
  Serial.println("Detected: " + det);

  sendTelemetry(det);

  delay(8000);
}
