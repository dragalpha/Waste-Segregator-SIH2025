/*
  ESP8266 MQ135 Air Quality / Gas Detection with Error Handling
  
  Features:
  - Reads MQ135 analog sensor (A0)
  - Converts raw ADC to ppm using calibration
  - Detects gas type (CO2/VOC/NH3/Smoke) and level (good/elevated/high/dangerous)
  - Error handling for sensor faults and invalid readings
  - Posts telemetry to backend server with mq135_ppm field
  - Displays gas analysis on Serial Monitor
  
  Hardware:
  - ESP8266 (NodeMCU/Wemos D1 Mini)
  - MQ135 Air Quality Sensor connected to A0
  - 5V and GND connections
  
  Wiring:
  - MQ135 VCC -> 5V (or 3.3V for some modules)
  - MQ135 GND -> GND
  - MQ135 A0  -> ESP8266 A0 (analog input)
  - MQ135 D0  -> (optional digital pin for threshold trigger)
  
  Installation:
  1. Install ESP8266 board support in Arduino IDE
  2. Set your WiFi SSID and PASSWORD below
  3. Update server IP to your backend server address
  4. Upload to ESP8266
  
  Calibration Notes:
  - MQ135 requires 24-48h preheat for accurate readings
  - Default calibration assumes fresh air = ~400 ppm CO2
  - Adjust RL (load resistance) and R0 (sensor resistance in clean air) for accuracy
  - See MQ135 datasheet for gas-specific calibration curves
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// ===== CONFIGURATION =====
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* server = "http://192.168.1.100:8000";  // Backend server IP

// MQ135 sensor pins
const int MQ135_PIN = A0;  // Analog input for MQ135

// Calibration constants (adjust based on your sensor and environment)
const float RL = 10.0;           // Load resistance in kOhms (typically 10k)
const float R0_CLEAN_AIR = 76.63; // Sensor resistance in clean air (calibrate this!)
const float VCC = 3.3;           // ESP8266 ADC reference voltage
const int ADC_MAX = 1024;        // 10-bit ADC

// Preheat and sampling
const unsigned long PREHEAT_TIME = 20000;  // 20s preheat on startup (real: 24-48h)
const int SAMPLE_COUNT = 10;              // Number of samples to average
const unsigned long POST_INTERVAL = 15000; // 15s between posts

// Error thresholds
const int ADC_MIN_VALID = 10;     // Minimum valid ADC reading (detect sensor disconnect)
const int ADC_MAX_VALID = 1010;   // Maximum valid ADC reading (detect short/saturation)
const float PPM_MAX_VALID = 10000.0; // Max reasonable ppm for CO2/VOC

// ===== GLOBAL VARIABLES =====
unsigned long lastPost = 0;
bool sensorReady = false;
String lastError = "";

// ===== GAS CLASSIFICATION STRUCTURE =====
struct GasAnalysis {
  String gasName;
  String level;
  float ppm;
  String note;
  bool hasError;
  String error;
};

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n\n=== ESP8266 MQ135 Gas Detection ===");
  Serial.println("Initializing...");
  
  // Initialize sensor pin
  pinMode(MQ135_PIN, INPUT);
  
  // Connect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 20000) {
      Serial.println("\nWiFi timeout - restarting connection");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      start = millis();
    }
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Server: ");
  Serial.println(server);
  
  // Preheat sensor
  Serial.println("\nPreheating MQ135 sensor...");
  Serial.print("Wait ");
  Serial.print(PREHEAT_TIME / 1000);
  Serial.println(" seconds for sensor warmup");
  Serial.println("(Note: Full calibration requires 24-48h preheat)");
  
  unsigned long preheatStart = millis();
  while (millis() - preheatStart < PREHEAT_TIME) {
    delay(1000);
    Serial.print(".");
    if ((millis() - preheatStart) % 5000 == 0) {
      int raw = analogRead(MQ135_PIN);
      Serial.print(" ADC=");
      Serial.print(raw);
    }
  }
  Serial.println("\nSensor ready!");
  sensorReady = true;
}

// ===== SENSOR READING FUNCTIONS =====

float readMQ135Raw() {
  // Read and average multiple samples to reduce noise
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(MQ135_PIN);
    delay(10);
  }
  return sum / (float)SAMPLE_COUNT;
}

float calculatePPM(float rawADC, String &errorMsg) {
  // Error checking: ADC range validation
  if (rawADC < ADC_MIN_VALID) {
    errorMsg = "Sensor disconnected or fault (ADC too low)";
    return -1.0;
  }
  if (rawADC > ADC_MAX_VALID) {
    errorMsg = "Sensor saturated or short circuit (ADC too high)";
    return -2.0;
  }
  
  // Convert ADC to voltage
  float voltage = (rawADC / ADC_MAX) * VCC;
  
  // Avoid division by zero
  if (voltage <= 0.01) {
    errorMsg = "Invalid voltage reading";
    return -3.0;
  }
  
  // Calculate sensor resistance (Rs)
  // Rs = (Vc - V) * RL / V
  float rs = ((VCC - voltage) / voltage) * RL;
  
  if (rs <= 0) {
    errorMsg = "Invalid sensor resistance calculation";
    return -4.0;
  }
  
  // Calculate Rs/R0 ratio
  float ratio = rs / R0_CLEAN_AIR;
  
  // Convert ratio to ppm using approximate power law for CO2
  // MQ135 curve: ppm = a * (Rs/R0)^b
  // For CO2: a ≈ 116.6, b ≈ -2.769 (approximate from datasheet)
  // For general VOC/air quality: different coefficients
  
  // Using simplified CO2 curve (adjust for your calibration)
  float ppm = 116.6 * pow(ratio, -2.769);
  
  // Sanity check
  if (ppm < 0 || ppm > PPM_MAX_VALID) {
    errorMsg = "PPM out of valid range (check calibration)";
    return -5.0;
  }
  
  errorMsg = "";
  return ppm;
}

GasAnalysis analyzeGas(float ppm) {
  GasAnalysis result;
  result.ppm = ppm;
  result.hasError = false;
  
  // Error cases
  if (ppm < 0) {
    result.hasError = true;
    result.gasName = "unknown";
    result.level = "error";
    
    switch ((int)ppm) {
      case -1:
        result.error = "Sensor disconnected (ADC < " + String(ADC_MIN_VALID) + ")";
        result.note = "Check wiring and power supply";
        break;
      case -2:
        result.error = "Sensor saturated (ADC > " + String(ADC_MAX_VALID) + ")";
        result.note = "Extreme gas concentration or sensor fault";
        break;
      case -3:
        result.error = "Invalid voltage reading";
        result.note = "ADC or power supply issue";
        break;
      case -4:
        result.error = "Invalid resistance calculation";
        result.note = "Check sensor connections";
        break;
      case -5:
        result.error = "PPM out of range (recalibrate)";
        result.note = "Sensor may need recalibration or R0 adjustment";
        break;
      default:
        result.error = "Unknown sensor error";
        result.note = "Check hardware and calibration";
    }
    return result;
  }
  
  // Gas classification based on ppm levels
  // MQ135 detects: CO2, VOC, NH3, Smoke, Benzene, Alcohol
  
  if (ppm <= 400) {
    result.gasName = "fresh_air";
    result.level = "good";
    result.note = "Baseline / Clean air quality";
  } else if (ppm <= 1000) {
    result.gasName = "CO2_or_VOC";
    result.level = "elevated";
    result.note = "Mild CO2 buildup or VOCs present";
  } else if (ppm <= 2000) {
    result.gasName = "CO2_or_VOC";
    result.level = "high";
    result.note = "Poor ventilation / Strong VOCs detected";
  } else if (ppm <= 5000) {
    result.gasName = "CO2_or_VOC_hazard";
    result.level = "very_high";
    result.note = "Hazardous air quality - Ventilate immediately!";
  } else {
    result.gasName = "dangerous_gas";
    result.level = "dangerous";
    result.note = "DANGER: Extreme gas levels (>5000 ppm) - Evacuate area!";
  }
  
  result.error = "";
  return result;
}

void printGasAnalysis(const GasAnalysis &gas, float rawADC) {
  Serial.println("\n===== MQ135 GAS ANALYSIS =====");
  Serial.print("Raw ADC: ");
  Serial.println(rawADC, 2);
  
  if (gas.hasError) {
    Serial.println("STATUS: ERROR");
    Serial.print("Error: ");
    Serial.println(gas.error);
    Serial.print("Note: ");
    Serial.println(gas.note);
  } else {
    Serial.println("STATUS: OK");
    Serial.print("Detected Gas: ");
    Serial.println(gas.gasName);
    Serial.print("Concentration: ");
    Serial.print(gas.ppm, 2);
    Serial.println(" ppm");
    Serial.print("Level: ");
    Serial.println(gas.level);
    Serial.print("Note: ");
    Serial.println(gas.note);
  }
  Serial.println("==============================\n");
}

String makeTelemetryJson(const GasAnalysis &gas) {
  String json = "{";
  json += "\"device_id\": \"esp8266_mq135\",";
  json += "\"boat_id\": \"boat_1\",";
  json += "\"lat\": \"22.57\",";
  json += "\"lon\": \"88.36\",";
  json += "\"heading_deg\": \"90\",";
  
  // MQ135 data - send actual ppm or error code
  if (gas.hasError) {
    json += "\"mq135_ppm\": \"error\",";  // Backend will handle this
  } else {
    json += "\"mq135_ppm\": \"" + String(gas.ppm, 2) + "\",";
  }
  
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
  json += "\"rssi\": \"" + String(WiFi.RSSI()) + "\"";
  json += "}";
  return json;
}

void postTelemetry(const GasAnalysis &gas) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected - skipping POST");
    return;
  }
  
  HTTPClient http;
  String url = String(server) + "/telemetry";
  
  Serial.print("Posting to: ");
  Serial.println(url);
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  String payload = makeTelemetryJson(gas);
  Serial.println("Payload:");
  Serial.println(payload);
  
  int httpCode = http.POST(payload);
  
  if (httpCode > 0) {
    String response = http.getString();
    Serial.print("Response Code: ");
    Serial.println(httpCode);
    Serial.println("Response:");
    Serial.println(response);
  } else {
    Serial.print("POST failed: ");
    Serial.println(http.errorToString(httpCode));
  }
  
  http.end();
}

void loop() {
  if (!sensorReady) {
    delay(1000);
    return;
  }
  
  // Check if it's time to post
  if ((millis() - lastPost) > POST_INTERVAL) {
    // Read sensor
    float rawADC = readMQ135Raw();
    
    // Calculate PPM with error handling
    String errorMsg = "";
    float ppm = calculatePPM(rawADC, errorMsg);
    
    // Analyze gas type and level
    GasAnalysis gas = analyzeGas(ppm);
    if (errorMsg != "" && !gas.hasError) {
      gas.hasError = true;
      gas.error = errorMsg;
    }
    
    // Print analysis to Serial Monitor
    printGasAnalysis(gas, rawADC);
    
    // Post to server
    postTelemetry(gas);
    
    // Store last error for monitoring
    lastError = gas.hasError ? gas.error : "";
    
    lastPost = millis();
  }
  
  delay(1000);  // Small delay to prevent overwhelming the system
}
