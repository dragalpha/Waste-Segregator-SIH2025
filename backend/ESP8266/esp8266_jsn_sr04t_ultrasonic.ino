/*
 * ESP8266 JSN-SR04T Waterproof Ultrasonic Sensor - Object Detection
 * 
 * Detects objects using JSN-SR04T ultrasonic distance sensor and reports
 * object presence and proximity to FastAPI backend.
 * 
 * SENSOR SPECIFICATIONS:
 * ======================
 * JSN-SR04T Waterproof Ultrasonic Sensor:
 * - Operating Range: 25cm to 450cm (maximum range used for detection)
 * - Operating Voltage: 5V DC
 * - Ultrasonic Frequency: 40kHz
 * - Waterproof: IP67 rated (fully sealed probe)
 * - Measuring Angle: 75 degrees
 * - Trigger Input Signal: 10µS TTL pulse
 * - Echo Output Signal: TTL pulse proportional to distance
 * 
 * APPLICATION: Pure Object Detection
 * ===================================
 * - Detects presence of objects within 25-450cm range
 * - Reports distance and proximity classification
 * - No fill-level calculation (pure detection mode)
 * - Suitable for obstacle detection, presence sensing, proximity monitoring
 * 
 * HARDWARE WIRING:
 * ================
 * JSN-SR04T Ultrasonic Sensor:
 *   - VCC → 5V (requires 5V power supply)
 *   - GND → GND
 *   - TRIG → ESP8266 D5 (GPIO14) - Trigger pin
 *   - ECHO → ESP8266 D6 (GPIO12) - Echo pin
 * 
 * WIRING NOTES:
 * - ESP8266 GPIO pins are 3.3V tolerant, but JSN-SR04T ECHO output is 5V
 * - Use a voltage divider (2kΩ + 1kΩ resistors) on ECHO pin to protect ESP8266
 * - Voltage Divider: ECHO → 2kΩ → D6 → 1kΩ → GND (reduces 5V to ~1.67V safe range)
 * - OR use a logic level shifter module (recommended for reliability)
 * 
 * INSTALLATION:
 * - Mount sensor at top of waste bin pointing downward
 * - Ensure probe is vertical for accurate readings
 * - Keep probe clean and free from obstructions
 * - 450cm max range allows monitoring of large industrial bins
 * 
 * ERROR CODES:
 * - Distance = -1: Sensor value missing
 * - Distance = -2: Invalid format
 * - Distance = -3: Negative distance reading (sensor error)
 * - Distance = -4: Below minimum range (< 25cm, object too close)
 * - Distance = -5: Above maximum range (> 450cm, no object detected or out of range)
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

// ===== JSN-SR04T Pin Configuration =====
const int TRIG_PIN = 14; // D5 (GPIO14) - Trigger pin
const int ECHO_PIN = 12; // D6 (GPIO12) - Echo pin (use voltage divider!)

// ===== Sensor Configuration =====
const float MIN_DISTANCE = 25.0;   // Minimum valid distance (cm)
const float MAX_DISTANCE = 450.0;  // Maximum valid distance (cm)
const float TIMEOUT_DISTANCE = 500.0; // Distance timeout threshold
const long ECHO_TIMEOUT = 30000;   // Echo timeout in microseconds (30ms)

// ===== Timing Configuration =====
const unsigned long POST_INTERVAL = 10000;  // Post every 10 seconds
unsigned long lastPostTime = 0;

// ===== Distance Measurement Structure =====
struct DistanceMeasurement {
	float distance_cm;
	bool valid;
	String error_msg;
	int error_code;
};

// ===== Setup Function =====
void setup() {
	Serial.begin(115200);
	delay(100);
	Serial.println("\n\n=== ESP8266 JSN-SR04T Object Detection ===");
	Serial.println("Waterproof Ultrasonic Distance Sensor");
	Serial.println("Detection Range: 25cm - 450cm");
	Serial.println("Mode: Pure Object Detection\n");
	
	// Configure pins
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	digitalWrite(TRIG_PIN, LOW);
	
	// Connect to WiFi
	connectWiFi();
	
	Serial.println("\n[INFO] System ready. Starting measurements...\n");
	delay(1000);
}

// ===== Main Loop =====
void loop() {
	unsigned long currentTime = millis();
	
	// Check if it's time to post telemetry
	if (currentTime - lastPostTime >= POST_INTERVAL) {
		lastPostTime = currentTime;
		
		// Measure distance
		DistanceMeasurement measurement = measureDistance();
		
		// Display measurement
		printMeasurement(measurement);
		
		// Post to backend
		postTelemetry(measurement);
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

// ===== Measure Distance with JSN-SR04T =====
DistanceMeasurement measureDistance() {
	DistanceMeasurement result;
	result.valid = false;
	result.error_code = 0;
	result.error_msg = "";
	
	// Take multiple samples and average (reduces noise)
	const int samples = 5;
	float distances[samples];
	int validSamples = 0;
	
	for (int i = 0; i < samples; i++) {
		// Send 10µs trigger pulse
		digitalWrite(TRIG_PIN, LOW);
		delayMicroseconds(2);
		digitalWrite(TRIG_PIN, HIGH);
		delayMicroseconds(10);
		digitalWrite(TRIG_PIN, LOW);
		
		// Read echo pulse duration
		long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);
		
		if (duration == 0) {
			// Timeout - no echo received
			continue;
		}
		
		// Calculate distance: duration (µs) / 2 / 29.1 = distance (cm)
		// Speed of sound: 343 m/s = 0.0343 cm/µs
		// Time is round-trip, so divide by 2: distance = duration * 0.0343 / 2 = duration / 58.2
		// More accurate formula: distance = duration / 58.0
		float distance = duration / 58.0;
		
		// Validate range
		if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
			distances[validSamples] = distance;
			validSamples++;
		}
		
		delay(60); // Wait 60ms between samples (sensor cycle time)
	}
	
	// Check if we got enough valid samples
	if (validSamples == 0) {
		result.error_code = -5;
		result.error_msg = "JSN-SR04T_NO_ECHO_OR_OUT_OF_RANGE";
		result.distance_cm = -5;
		return result;
	}
	
	// Calculate average of valid samples
	float sum = 0;
	for (int i = 0; i < validSamples; i++) {
		sum += distances[i];
	}
	result.distance_cm = sum / validSamples;
	
	// Final range validation
	if (result.distance_cm < MIN_DISTANCE) {
		result.error_code = -4;
		result.error_msg = "JSN-SR04T_BELOW_MIN_RANGE";
		result.distance_cm = -4;
		result.valid = false;
	} else if (result.distance_cm > MAX_DISTANCE) {
		result.error_code = -5;
		result.error_msg = "JSN-SR04T_ABOVE_MAX_RANGE";
		result.distance_cm = -5;
		result.valid = false;
	} else {
		result.valid = true;
	}
	
	return result;
}

// ===== Classify Detection Proximity =====
String classifyProximity(float distance_cm) {
	if (distance_cm < 0) {
		return "error";
	} else if (distance_cm <= 50) {
		return "immediate";
	} else if (distance_cm <= 100) {
		return "near";
	} else if (distance_cm <= 200) {
		return "moderate";
	} else if (distance_cm <= 350) {
		return "distant";
	} else {
		return "very_distant";
	}
}

// ===== Classify Detection Range =====
String classifyRange(float distance_cm) {
	if (distance_cm < 0) {
		return "error";
	} else if (distance_cm <= 50) {
		return "very_close";
	} else if (distance_cm <= 100) {
		return "close";
	} else if (distance_cm <= 200) {
		return "medium";
	} else if (distance_cm <= 350) {
		return "far";
	} else {
		return "very_far";
	}
}

// ===== Print Measurement =====
void printMeasurement(const DistanceMeasurement& measurement) {
	Serial.println("========================================");
	Serial.println("    JSN-SR04T OBJECT DETECTION");
	Serial.println("========================================");
	
	if (!measurement.valid) {
		Serial.print("  [ERROR] ");
		Serial.println(measurement.error_msg);
		Serial.print("  Error Code: ");
		Serial.println(measurement.error_code);
		Serial.println("  Object Detected: NO");
	} else {
		Serial.print("  Distance: ");
		Serial.print(measurement.distance_cm, 2);
		Serial.println(" cm");
		
		Serial.print("  Range: ");
		Serial.println(classifyRange(measurement.distance_cm));
		
		Serial.print("  Proximity: ");
		Serial.println(classifyProximity(measurement.distance_cm));
		
		Serial.println("  Object Detected: YES");
	}
	
	Serial.println("========================================\n");
}

// ===== Post Telemetry to Backend =====
void postTelemetry(const DistanceMeasurement& measurement) {
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
	doc["device_id"] = "ESP8266_JSN_SR04T_DETECTOR";
	doc["timestamp"] = millis();
	
	// Add distance measurement
	doc["ultrasonic_cm"] = measurement.distance_cm;
	
	// Add error message if present
	if (!measurement.valid && !measurement.error_msg.isEmpty()) {
		doc["ultrasonic_error"] = measurement.error_msg;
	}
	
	// Optional: Add temperature reading if you have DHT sensor
	// doc["temperature"] = 25.0;
	// doc["humidity"] = 60.0;
	
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
	
	// Check if ultrasonic_analysis exists
	if (!doc.containsKey("ultrasonic_analysis")) {
		Serial.println("[INFO] No ultrasonic_analysis field in response.");
		return;
	}
	
	JsonObject analysis = doc["ultrasonic_analysis"];
	
	Serial.println("========================================");
	Serial.println("   BACKEND OBJECT DETECTION ANALYSIS");
	Serial.println("========================================");
	
	if (analysis.containsKey("status")) {
		Serial.print("Status: ");
		Serial.println(analysis["status"].as<String>());
	}
	
	if (analysis.containsKey("distance_cm")) {
		Serial.print("Distance: ");
		Serial.print(analysis["distance_cm"].as<float>(), 2);
		Serial.println(" cm");
	}
	
	if (analysis.containsKey("range")) {
		Serial.print("Range: ");
		Serial.println(analysis["range"].as<String>());
	}
	
	if (analysis.containsKey("proximity")) {
		Serial.print("Proximity: ");
		Serial.println(analysis["proximity"].as<String>());
	}
	
	if (analysis.containsKey("object_detected")) {
		Serial.print("Object Detected: ");
		Serial.println(analysis["object_detected"].as<bool>() ? "YES" : "NO");
	}
	
	if (analysis.containsKey("note")) {
		Serial.print("Note: ");
		Serial.println(analysis["note"].as<String>());
	}
	
	Serial.println("========================================\n");
}
