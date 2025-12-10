/*
 * ESP8266 LJ12A3-4-Z/BY Inductive Proximity Sensor - Metal Detection
 * 
 * Detects metallic objects using LJ12A3-4-Z/BY inductive proximity sensor
 * and reports detection status to FastAPI backend.
 * 
 * SENSOR SPECIFICATIONS:
 * ======================
 * LJ12A3-4-Z/BY Inductive Proximity Sensor:
 * - Type: Inductive proximity switch (NPN normally open)
 * - Detection Distance: 4mm (0-4mm sensing range)
 * - Detectable Objects: Ferrous metals (iron, steel), non-ferrous metals (aluminum, copper, brass)
 * - Operating Voltage: DC 6-36V (typically 12V or 24V)
 * - Output Type: NPN transistor (normally open - NO)
 * - Output Signal: Digital LOW when metal detected, HIGH when no metal
 * - Operating Current: ≤200mA
 * - Response Frequency: 500Hz
 * - Operating Temperature: -25°C to +70°C
 * - Protection: Short circuit and reverse polarity protected
 * 
 * APPLICATION: Metal Detection & Waste Sorting
 * =============================================
 * - Detects metallic waste items (cans, batteries, metal parts)
 * - Helps classify waste into metallic vs non-metallic categories
 * - Useful for automated sorting systems
 * - Can identify hazardous metal items (batteries with metal casings)
 * 
 * HARDWARE WIRING:
 * ================
 * LJ12A3-4-Z/BY Sensor (3-wire NPN):
 *   - Brown Wire → 12V DC (or 6-36V power supply)
 *   - Blue Wire → GND (common ground)
 *   - Black Wire → Signal output (NPN transistor collector)
 * 
 * ESP8266 Connection:
 *   - LJ12A3 Black Wire → 10kΩ pull-up resistor → 3.3V
 *   - LJ12A3 Black Wire → ESP8266 D2 (GPIO4) digital input
 *   - ESP8266 GND → LJ12A3 Blue Wire (common ground)
 * 
 * IMPORTANT WIRING NOTES:
 * =======================
 * 1. LJ12A3 needs 6-36V power supply (typically 12V or 24V)
 * 2. DO NOT connect 12V/24V directly to ESP8266 GPIO pins!
 * 3. Use 10kΩ pull-up resistor to 3.3V for ESP8266 compatibility
 * 4. NPN output: LOW (0V) = metal detected, HIGH (pulled to 3.3V) = no metal
 * 5. Ensure common ground between ESP8266 and LJ12A3 power supply
 * 
 * WIRING DIAGRAM:
 * 
 *              12V DC Power Supply
 *                  │
 *                  ├────────────→ Brown (LJ12A3 VCC)
 *                  │
 *                 GND
 *                  │
 *                  ├────────────→ Blue (LJ12A3 GND)
 *                  │
 *                  └────────────→ ESP8266 GND
 * 
 *              3.3V (ESP8266)
 *                  │
 *              10kΩ Resistor
 *                  │
 *                  ├────────────→ D2 (GPIO4) ESP8266
 *                  │
 *              Black Wire (LJ12A3 Signal)
 * 
 * SENSOR INSTALLATION:
 * ====================
 * - Mount sensor flush with detection surface
 * - Keep sensing face clean and unobstructed
 * - Test detection distance: typically 4mm for steel, less for aluminum
 * - Avoid mounting near other metal structures that may cause false triggers
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

// ===== LJ12A3-4-Z/BY Pin Configuration =====
const int PROXIMITY_PIN = 4; // D2 (GPIO4) - Digital input from LJ12A3

// ===== Sensor Configuration =====
// Debounce settings to avoid false triggers
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce time
unsigned long lastDebounceTime = 0;
int lastStableState = HIGH;  // Last confirmed stable state
int lastReading = HIGH;       // Last raw reading

// ===== Timing Configuration =====
const unsigned long POST_INTERVAL = 5000;  // Post every 5 seconds
unsigned long lastPostTime = 0;

// ===== Detection Statistics =====
unsigned long detectionCount = 0;
unsigned long lastDetectionTime = 0;

// ===== Metal Detection Structure =====
struct MetalDetection {
	bool metal_detected;
	String detection_state;
	unsigned long detection_count;
	unsigned long time_since_last_ms;
};

// ===== Setup Function =====
void setup() {
	Serial.begin(115200);
	delay(100);
	Serial.println("\n\n=== ESP8266 LJ12A3-4-Z/BY Metal Detector ===");
	Serial.println("Inductive Proximity Sensor");
	Serial.println("Detection Range: 0-4mm");
	Serial.println("Mode: Metal Detection & Classification\n");
	
	// Configure sensor pin with internal pull-up
	pinMode(PROXIMITY_PIN, INPUT_PULLUP);
	
	// Connect to WiFi
	connectWiFi();
	
	Serial.println("\n[INFO] System ready. Monitoring for metal objects...\n");
	delay(1000);
}

// ===== Main Loop =====
void loop() {
	unsigned long currentTime = millis();
	
	// Read sensor with debouncing
	int reading = digitalRead(PROXIMITY_PIN);
	
	// Check if reading changed (potential state change)
	if (reading != lastReading) {
		lastDebounceTime = currentTime;
	}
	
	// If reading has been stable for debounce period
	if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
		// If state actually changed
		if (reading != lastStableState) {
			lastStableState = reading;
			
			// Metal detection: LOW signal means metal detected (NPN pulls to ground)
			if (lastStableState == LOW) {
				detectionCount++;
				lastDetectionTime = currentTime;
				Serial.println(">>> METAL DETECTED! <<<");
			} else {
				Serial.println("Metal removed / No metal present");
			}
		}
	}
	
	lastReading = reading;
	
	// Post telemetry at regular intervals
	if (currentTime - lastPostTime >= POST_INTERVAL) {
		lastPostTime = currentTime;
		
		// Get current detection status
		MetalDetection detection = getDetectionStatus();
		
		// Display status
		printDetectionStatus(detection);
		
		// Post to backend
		postTelemetry(detection);
	}
	
	delay(10); // Small delay for stability
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

// ===== Get Current Detection Status =====
MetalDetection getDetectionStatus() {
	MetalDetection status;
	
	// Current state (LOW = metal detected for NPN sensor)
	status.metal_detected = (lastStableState == LOW);
	
	// Detection state string
	if (status.metal_detected) {
		status.detection_state = "active";
	} else {
		status.detection_state = "inactive";
	}
	
	// Statistics
	status.detection_count = detectionCount;
	status.time_since_last_ms = millis() - lastDetectionTime;
	
	return status;
}

// ===== Print Detection Status =====
void printDetectionStatus(const MetalDetection& detection) {
	Serial.println("========================================");
	Serial.println("    LJ12A3 METAL DETECTION STATUS");
	Serial.println("========================================");
	
	Serial.print("  Metal Detected: ");
	Serial.println(detection.metal_detected ? "YES" : "NO");
	
	Serial.print("  Detection State: ");
	Serial.println(detection.detection_state);
	
	Serial.print("  Total Detections: ");
	Serial.println(detection.detection_count);
	
	if (detection.detection_count > 0) {
		Serial.print("  Time Since Last: ");
		Serial.print(detection.time_since_last_ms / 1000.0, 2);
		Serial.println(" seconds");
	}
	
	// Material inference
	if (detection.metal_detected) {
		Serial.println("  Likely Material: Ferrous/Non-ferrous metal");
		Serial.println("  Waste Type: Metallic (can, battery, metal part)");
	} else {
		Serial.println("  Likely Material: Non-metallic");
		Serial.println("  Waste Type: Plastic, glass, organic, or empty");
	}
	
	Serial.println("========================================\n");
}

// ===== Post Telemetry to Backend =====
void postTelemetry(const MetalDetection& detection) {
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
	doc["device_id"] = "ESP8266_LJ12A3_METAL_DETECTOR";
	doc["timestamp"] = millis();
	
	// Add proximity sensor data (1 = metal detected, 0 = no metal)
	doc["proximity_inductive"] = detection.metal_detected ? 1 : 0;
	
	// Add detection statistics
	doc["detection_count"] = detection.detection_count;
	doc["time_since_last_ms"] = detection.time_since_last_ms;
	
	// Optional: Add other sensor data if available
	// doc["temperature"] = 25.0;
	// doc["ultrasonic_cm"] = 150.0;
	
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
		
		// Parse backend analysis
		parseBackendResponse(response);
	} else {
		Serial.println("[POST] Request failed!");
	}
	
	http.end();
	Serial.println();
}

// ===== Parse Backend Response =====
void parseBackendResponse(const String& jsonResponse) {
	StaticJsonDocument<1024> doc;
	DeserializationError error = deserializeJson(doc, jsonResponse);
	
	if (error) {
		Serial.print("[PARSE ERROR] Failed to parse JSON: ");
		Serial.println(error.c_str());
		return;
	}
	
	// Check if proximity_analysis exists
	if (!doc.containsKey("proximity_analysis")) {
		Serial.println("[INFO] No proximity_analysis field in response.");
		return;
	}
	
	JsonObject analysis = doc["proximity_analysis"];
	
	Serial.println("========================================");
	Serial.println("   BACKEND METAL DETECTION ANALYSIS");
	Serial.println("========================================");
	
	if (analysis.containsKey("status")) {
		Serial.print("Status: ");
		Serial.println(analysis["status"].as<String>());
	}
	
	if (analysis.containsKey("metal_detected")) {
		Serial.print("Metal Detected: ");
		Serial.println(analysis["metal_detected"].as<bool>() ? "YES" : "NO");
	}
	
	if (analysis.containsKey("detection_state")) {
		Serial.print("Detection State: ");
		Serial.println(analysis["detection_state"].as<String>());
	}
	
	if (analysis.containsKey("likely_material")) {
		Serial.print("Likely Material: ");
		Serial.println(analysis["likely_material"].as<String>());
	}
	
	if (analysis.containsKey("note")) {
		Serial.print("Note: ");
		Serial.println(analysis["note"].as<String>());
	}
	
	Serial.println("========================================\n");
}
