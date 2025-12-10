/*
 * ESP8266 Combined MQ2 + MQ135 Gas Sensor Monitor
 * 
 * Reads both MQ2 (flammable gases) and MQ135 (air quality) sensors,
 * calculates PPM values, posts telemetry to FastAPI backend, and
 * displays combined gas analysis results.
 * 
 * HARDWARE WIRING:
 * ================
 * MQ135 (Air Quality Sensor):
 *   - VCC → 5V (requires 5V for heater)
 *   - GND → GND
 *   - A0 → ESP8266 A0 (analog input)
 *   - D0 → Not used (digital threshold output)
 * 
 * MQ2 (Flammable Gas Sensor):
 *   - VCC → 5V (requires 5V for heater)
 *   - GND → GND
 *   - A0 → ADS1115 A0 or use digital D0 output
 *   - D0 → ESP8266 D1/GPIO5 (digital threshold detection)
 * 
 * NOTE: ESP8266 has only ONE analog input (A0), so we use MQ135 on A0
 * and MQ2 digital output on D1. For analog MQ2 reading, add an ADS1115
 * I2C ADC module.
 * 
 * OPTIONAL I2C ADC (ADS1115) for dual analog reading:
 *   - VCC → 3.3V
 *   - GND → GND
 *   - SCL → D1 (GPIO5)
 *   - SDA → D2 (GPIO4)
 *   - A0 → MQ2 analog output
 * 
 * CALIBRATION:
 * - RL (Load Resistor): 10kΩ for both sensors
 * - R0 (Base Resistance in clean air):
 *   - MQ135: ~76.63Ω (calibrate in fresh air, 400ppm CO2 reference)
 *   - MQ2: ~9.83kΩ (calibrate in clean air, no flammable gases)
 * - Preheat time: 48 hours for accurate readings (minimum 20 seconds)
 */

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

// ===== WiFi Configuration =====
const char* WIFI_SSID = "YourWiFiSSID";
const char* WIFI_PASS = "YourWiFiPassword";

// ===== Backend API Configuration =====
const char* BACKEND_HOST = "192.168.1.100"; // Change to your server IP
const int BACKEND_PORT = 8000;
const char* TELEMETRY_ENDPOINT = "/telemetry";

// ===== Sensor Calibration Constants =====
// MQ135 (Air Quality)
const float MQ135_RL = 10.0;          // Load resistance in kΩ
const float MQ135_R0 = 76.63;         // Sensor resistance in clean air (Ω)
const float MQ135_VCC = 5.0;          // Supply voltage
const int MQ135_ANALOG_PIN = A0;      // ESP8266 analog input

// MQ2 (Flammable Gas)
const float MQ2_RL = 10.0;            // Load resistance in kΩ
const float MQ2_R0 = 9.83;            // Sensor resistance in clean air (kΩ)
const float MQ2_VCC = 5.0;            // Supply voltage
const int MQ2_DIGITAL_PIN = 5;        // GPIO5 (D1) - digital threshold output
// If using ADS1115 for analog MQ2, enable I2C and read from ADS1115

// ===== Timing Configuration =====
const unsigned long PREHEAT_TIME = 20000;      // 20 seconds (minimum preheat)
const unsigned long POST_INTERVAL = 15000;     // Post every 15 seconds
unsigned long lastPostTime = 0;
bool isPreheating = true;

// ===== Gas Analysis Structures =====
struct GasData {
	float ppm;
	String gasName;
	String level;
	String errorMsg;
	bool hasError;
};

struct CombinedAnalysis {
	GasData mq2;
	GasData mq135;
	String probableGas;
	String confidence;
	String hazardLevel;
	String recommendation;
};

// ===== Setup Function =====
void setup() {
	Serial.begin(115200);
	delay(100);
	Serial.println("\n\n=== ESP8266 MQ2+MQ135 Combined Gas Monitor ===");
	
	// Configure pins
	pinMode(MQ2_DIGITAL_PIN, INPUT);
	pinMode(MQ135_ANALOG_PIN, INPUT);
	
	// Connect to WiFi
	connectWiFi();
	
	// Preheat sensors
	Serial.println("\n[INFO] Preheating sensors for 20 seconds...");
	Serial.println("[INFO] (Full accuracy requires 48-hour burn-in)");
	delay(PREHEAT_TIME);
	isPreheating = false;
	Serial.println("[INFO] Preheat complete. Starting measurements.\n");
}

// ===== Main Loop =====
void loop() {
	unsigned long currentTime = millis();
	
	// Check if it's time to post telemetry
	if (currentTime - lastPostTime >= POST_INTERVAL) {
		lastPostTime = currentTime;
		
		// Read both sensors
		GasData mq135Data = readMQ135();
		GasData mq2Data = readMQ2();
		
		// Display local analysis
		printSensorData(mq135Data, mq2Data);
		
		// Post to backend and display combined analysis
		postTelemetryAndShowResults(mq135Data, mq2Data);
	}
	
	delay(100);
}

// ===== WiFi Connection =====
void connectWiFi() {
	Serial.print("[WiFi] Connecting to ");
	Serial.print(WIFI_SSID);
	
	WiFi.begin(WIFI_SSID, WIFI_PASS);
	
	int attempts = 0;
	while (WiFi.status() != WL_CONNECTED && attempts < 30) {
		delay(500);
		Serial.print(".");
		attempts++;
	}
	
	if (WiFi.status() == WL_CONNECTED) {
		Serial.println("\n[WiFi] Connected!");
		Serial.print("[WiFi] IP Address: ");
		Serial.println(WiFi.localIP());
	} else {
		Serial.println("\n[WiFi] Connection FAILED!");
	}
}

// ===== MQ135 Reading (Air Quality) =====
GasData readMQ135() {
	GasData result;
	result.hasError = false;
	
	// Average multiple samples
	long rawSum = 0;
	const int samples = 10;
	for (int i = 0; i < samples; i++) {
		rawSum += analogRead(MQ135_ANALOG_PIN);
		delay(10);
	}
	float rawADC = rawSum / (float)samples;
	
	// Convert ADC to voltage
	float voltage = (rawADC / 1024.0) * MQ135_VCC;
	
	// Error checks
	if (voltage < 0.1) {
		result.hasError = true;
		result.errorMsg = "MQ135_VOLTAGE_TOO_LOW";
		result.ppm = -1;
		return result;
	}
	
	if (voltage >= MQ135_VCC - 0.1) {
		result.hasError = true;
		result.errorMsg = "MQ135_VOLTAGE_TOO_HIGH";
		result.ppm = -2;
		return result;
	}
	
	// Calculate Rs (sensor resistance)
	float Rs = ((MQ135_VCC * MQ135_RL) / voltage) - MQ135_RL;
	
	if (Rs <= 0) {
		result.hasError = true;
		result.errorMsg = "MQ135_INVALID_RS";
		result.ppm = -3;
		return result;
	}
	
	// Calculate Rs/R0 ratio
	float ratio = Rs / MQ135_R0;
	
	if (ratio < 0.01 || ratio > 100) {
		result.hasError = true;
		result.errorMsg = "MQ135_RATIO_OUT_OF_RANGE";
		result.ppm = -4;
		return result;
	}
	
	// Convert to PPM (simplified model for CO2)
	// MQ135 datasheet: log(ppm) ≈ -0.42 * log(Rs/R0) + 3.5
	result.ppm = pow(10, (-0.42 * log10(ratio) + 3.5));
	
	// Classify gas level
	if (result.ppm <= 400) {
		result.level = "good";
		result.gasName = "clean_air";
	} else if (result.ppm <= 1000) {
		result.level = "elevated";
		result.gasName = "co2_or_voc";
	} else if (result.ppm <= 2000) {
		result.level = "high";
		result.gasName = "co2_or_voc";
	} else if (result.ppm <= 5000) {
		result.level = "very_high";
		result.gasName = "co2_or_voc_or_nh3";
	} else {
		result.level = "dangerous";
		result.gasName = "co2_or_voc_or_nh3_or_smoke";
	}
	
	return result;
}

// ===== MQ2 Reading (Flammable Gas) =====
GasData readMQ2() {
	GasData result;
	result.hasError = false;
	
	// Read digital threshold output
	int digitalValue = digitalRead(MQ2_DIGITAL_PIN);
	
	// For analog reading, use ADS1115 (I2C ADC)
	// For now, use simplified digital threshold estimation
	
	if (digitalValue == LOW) {
		// Gas detected (threshold exceeded)
		result.ppm = 1500; // Estimated threshold value
		result.level = "medium";
		result.gasName = "lpg_or_propane_or_methane";
	} else {
		// No gas detected
		result.ppm = 150; // Below threshold
		result.level = "none";
		result.gasName = "clean_air";
	}
	
	// For accurate analog reading, uncomment and add ADS1115 library:
	/*
	#include <Adafruit_ADS1X15.h>
	Adafruit_ADS1115 ads;
	
	int16_t adc0 = ads.readADC_SingleEnded(0);
	float voltage = ads.computeVolts(adc0);
	
	// Calculate Rs and ratio similar to MQ135
	float Rs = ((MQ2_VCC * MQ2_RL) / voltage) - MQ2_RL;
	float ratio = Rs / MQ2_R0;
	
	// MQ2 datasheet curve (LPG): log(ppm) ≈ -0.48 * log(Rs/R0) + 2.3
	result.ppm = pow(10, (-0.48 * log10(ratio) + 2.3));
	*/
	
	// Classify level
	if (result.ppm <= 200) {
		result.level = "none";
		result.gasName = "clean_air";
	} else if (result.ppm <= 300) {
		result.level = "trace";
		result.gasName = "lpg_or_alcohol";
	} else if (result.ppm <= 800) {
		result.level = "low";
		result.gasName = "lpg_or_propane";
	} else if (result.ppm <= 2000) {
		result.level = "medium";
		result.gasName = "lpg_or_propane_or_methane";
	} else if (result.ppm <= 5000) {
		result.level = "high";
		result.gasName = "lpg_or_propane_or_methane_or_h2";
	} else {
		result.level = "extreme";
		result.gasName = "lpg_or_propane_or_methane_or_h2";
	}
	
	return result;
}

// ===== Print Sensor Data =====
void printSensorData(const GasData& mq135, const GasData& mq2) {
	Serial.println("========================================");
	Serial.println("         SENSOR READINGS");
	Serial.println("========================================");
	
	// MQ135 Data
	Serial.println("MQ135 (Air Quality):");
	if (mq135.hasError) {
		Serial.print("  [ERROR] ");
		Serial.println(mq135.errorMsg);
		Serial.print("  Error Code: ");
		Serial.println(mq135.ppm);
	} else {
		Serial.print("  PPM: ");
		Serial.println(mq135.ppm, 2);
		Serial.print("  Gas: ");
		Serial.println(mq135.gasName);
		Serial.print("  Level: ");
		Serial.println(mq135.level);
	}
	
	Serial.println();
	
	// MQ2 Data
	Serial.println("MQ2 (Flammable Gas):");
	if (mq2.hasError) {
		Serial.print("  [ERROR] ");
		Serial.println(mq2.errorMsg);
		Serial.print("  Error Code: ");
		Serial.println(mq2.ppm);
	} else {
		Serial.print("  PPM: ");
		Serial.println(mq2.ppm, 2);
		Serial.print("  Gas: ");
		Serial.println(mq2.gasName);
		Serial.print("  Level: ");
		Serial.println(mq2.level);
	}
	
	Serial.println("========================================\n");
}

// ===== Post Telemetry and Display Results =====
void postTelemetryAndShowResults(const GasData& mq135, const GasData& mq2) {
	if (WiFi.status() != WL_CONNECTED) {
		Serial.println("[ERROR] WiFi not connected. Skipping telemetry post.");
		return;
	}
	
	WiFiClient client;
	HTTPClient http;
	
	// Build URL
	String url = "http://" + String(BACKEND_HOST) + ":" + String(BACKEND_PORT) + TELEMETRY_ENDPOINT;
	
	Serial.println("[POST] Sending telemetry to backend...");
	Serial.print("[POST] URL: ");
	Serial.println(url);
	
	// Create JSON payload
	StaticJsonDocument<512> doc;
	doc["device_id"] = "ESP8266_MQ2_MQ135";
	doc["timestamp"] = millis();
	doc["temperature"] = 25.0;  // Add DHT sensor if available
	doc["humidity"] = 60.0;     // Add DHT sensor if available
	
	// Add sensor data
	doc["mq135_ppm"] = mq135.hasError ? mq135.ppm : mq135.ppm;
	doc["mq2_ppm"] = mq2.hasError ? mq2.ppm : mq2.ppm;
	
	// Optional: add error flags
	if (mq135.hasError) {
		doc["mq135_error"] = mq135.errorMsg;
	}
	if (mq2.hasError) {
		doc["mq2_error"] = mq2.errorMsg;
	}
	
	// Serialize JSON
	String jsonPayload;
	serializeJson(doc, jsonPayload);
	
	Serial.print("[POST] Payload: ");
	Serial.println(jsonPayload);
	
	// Send HTTP POST
	http.begin(client, url);
	http.addHeader("Content-Type", "application/json");
	
	int httpCode = http.POST(jsonPayload);
	
	Serial.print("[POST] Response Code: ");
	Serial.println(httpCode);
	
	if (httpCode == 200) {
		String response = http.getString();
		Serial.println("[POST] Response Body:");
		Serial.println(response);
		
		// Parse combined analysis from backend
		parseCombinedAnalysis(response);
	} else {
		Serial.println("[POST] Request failed!");
	}
	
	http.end();
	Serial.println();
}

// ===== Parse Combined Analysis from Backend =====
void parseCombinedAnalysis(const String& jsonResponse) {
	StaticJsonDocument<1024> doc;
	DeserializationError error = deserializeJson(doc, jsonResponse);
	
	if (error) {
		Serial.print("[PARSE ERROR] Failed to parse JSON: ");
		Serial.println(error.c_str());
		return;
	}
	
	// Check if gas_analysis exists
	if (!doc.containsKey("gas_analysis")) {
		Serial.println("[INFO] No gas_analysis field in response.");
		return;
	}
	
	JsonObject analysis = doc["gas_analysis"];
	
	Serial.println("========================================");
	Serial.println("   BACKEND COMBINED GAS ANALYSIS");
	Serial.println("========================================");
	
	// Display probable gas
	if (analysis.containsKey("probable_gas")) {
		Serial.print("Probable Gas: ");
		Serial.println(analysis["probable_gas"].as<String>());
	}
	
	if (analysis.containsKey("confidence")) {
		Serial.print("Confidence: ");
		Serial.println(analysis["confidence"].as<String>());
	}
	
	if (analysis.containsKey("hazard_level")) {
		Serial.print("Hazard Level: ");
		Serial.println(analysis["hazard_level"].as<String>());
	}
	
	if (analysis.containsKey("recommendation")) {
		Serial.print("Recommendation: ");
		Serial.println(analysis["recommendation"].as<String>());
	}
	
	Serial.println("========================================\n");
}
