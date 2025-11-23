/*
  ESP8266 telemetry poster (mock)
  - Connects to WiFi
  - Periodically POSTs telemetry JSON to backend /telemetry
  - Polls /detected for latest detection and prints to Serial
  Requirements:
    Install ESP8266 board support in Arduino IDE
    Set SSID and PASSWORD below
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// set your backend server IP (use PC LAN IP) and port
const char* server = "http://192.168.1.100:8000"; // change to your laptop IP

unsigned long lastPost = 0;
const unsigned long POST_INTERVAL = 15000; // 15s

void setup() {
  Serial.begin(115200);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 20000) {
      Serial.println();
      Serial.println("WiFi connect timeout — restarting WiFi");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      start = millis();
    }
  }
  Serial.println();
  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Server configured as: ");
  Serial.println(server);
}

String makeTelemetryJson(){
  // minimal mock telemetry fields; expand as needed
  String json = "{";
  json += "\"device_id\": \"esp8266_1\",";
  json += "\"boat_id\": \"boat_1\",";
  json += "\"lat\": \"22.57\",";
  json += "\"lon\": \"88.36\",";
  json += "\"heading_deg\": \"90\",";
  json += "\"mq135_ppm\": \"0\",";
  json += "\"mq2_ppm\": \"0\",";
  json += "\"soil_dry_belt_pct\": \"0\",";
  json += "\"soil_wet_belt_pct\": \"0\",";
  json += "\"loadcell_grams\": \"0\",";
  json += "\"tds_ppm\": \"0\",";
  json += "\"ultrasonic_cm\": \"100\",";
  json += "\"proximity_inductive\": \"0\",";
  json += "\"image_path\": \"\",";
  json += "\"yolo_raw\": \"-\",";
  json += "\"waste_category\": \"-\",";
  json += "\"waste_subtype\": \"-\",";
  json += "\"collection_event\": \"\",";
  json += "\"collection_bin_id\": \"\",";
  json += "\"battery_volt\": \"3.3\",";
  json += "\"rssi\": \"0\"";
  json += "}";
  return json;
}

void loop() {
  if ((millis() - lastPost) > POST_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      // Try GET-based telemetry first (dev helper) to avoid POST issues on some networks
      String q_device = "device_id=esp8266_1";
      String q_lat = "lat=22.57";
      String q_lon = "lon=88.36";
      String q_yolo = "yolo_raw=-";
      String encoded_yolo = q_yolo;
      // encode ':' if present (simple replacement)
      encoded_yolo.replace(":", "%3A");
      String getUrl = String(server) + "/telemetry_get?" + q_device + "&" + q_lat + "&" + q_lon + "&" + encoded_yolo;
      Serial.print("GET telemetry to: "); Serial.println(getUrl);
      http.begin(getUrl);
      int httpCode = http.GET();
      if (httpCode > 0) {
        String resp = http.getString();
        Serial.printf("GET %s -> %d\n", getUrl.c_str(), httpCode);
        Serial.println(resp);
      } else {
        Serial.printf("GET failed, code=%d, err=%s -- falling back to POST\n", httpCode, http.errorToString(httpCode).c_str());
        http.end();
        // fallback to POST
        String postUrl = String(server) + "/telemetry";
        Serial.print("Posting to: "); Serial.println(postUrl);
        http.begin(postUrl);
        http.addHeader("Content-Type", "application/json");
        String payload = makeTelemetryJson();
        int postCode = http.POST(payload);
        if (postCode > 0) {
          String resp2 = http.getString();
          Serial.printf("POST %s -> %d\n", postUrl.c_str(), postCode);
          Serial.println(resp2);
        } else {
          Serial.printf("POST failed, code=%d, error: %s\n", postCode, http.errorToString(postCode).c_str());
        }
      }
      http.end();
    } else {
      Serial.print("WiFi not connected (status="); Serial.print(WiFi.status()); Serial.println(")");
    }
    lastPost = millis();
  }

  // poll detected
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(server) + "/detected";
    Serial.print("Polling: "); Serial.println(url);
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode == 200) {
      String resp = http.getString();
      Serial.println("Detected: " + resp);
    } else {
      Serial.printf("Detected poll failed, code=%d, err=%s\n", httpCode, http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.print("WiFi not connected when polling (status="); Serial.print(WiFi.status()); Serial.println(")");
  }

  delay(1000);
}
