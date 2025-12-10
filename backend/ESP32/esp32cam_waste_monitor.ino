/*
 * ESP32-CAM Waste Monitoring System with Multi-Sensor Integration
 * 
 * Captures images and posts to FastAPI backend with sensor telemetry.
 * Includes: MQ2, MQ135, JSN-SR04T ultrasonic, temperature, humidity sensors.
 * 
 * ============================================================================
 * FTDI PROGRAMMER WIRING FOR ESP32-CAM (AI-THINKER)
 * ============================================================================
 * 
 * UPLOADING CODE (Programming Mode):
 * -----------------------------------
 * FTDI Programmer → ESP32-CAM
 * 
 *   GND  →  GND
 *   5V   →  5V  (or 3.3V if FTDI supports it, ESP32-CAM needs stable power)
 *   TX   →  U0R (ESP32-CAM RX pin)
 *   RX   →  U0T (ESP32-CAM TX pin)
 * 
 * ** CRITICAL FOR UPLOAD MODE **
 *   IO0  →  GND (connect GPIO0 to GND to enter programming mode)
 * 
 * IMPORTANT NOTES:
 * - Remove IO0-to-GND jumper AFTER uploading, then press RESET button
 * - Use 5V power supply (ESP32-CAM camera module needs ~500mA current)
 * - Do NOT power from USB if unstable - use external 5V power supply
 * - Set Arduino IDE: Board = "AI Thinker ESP32-CAM", Upload Speed = 115200
 * 
 * RUNNING MODE (After Upload):
 * ----------------------------
 * 1. Disconnect IO0 from GND
 * 2. Press RESET button on ESP32-CAM
 * 3. Camera will boot normally and run the code
 * 
 * Optional: Connect FTDI for Serial Monitor debugging (remove IO0-GND):
 *   FTDI GND → ESP32-CAM GND
 *   FTDI RX  → ESP32-CAM U0T (to read Serial output)
 * 
 * ============================================================================
 * SENSOR WIRING
 * ============================================================================
 * 
 * MQ135 (Air Quality Sensor):
 *   VCC → 5V
 *   GND → GND
 *   A0  → GPIO 13 (analog through ADC)
 * 
 * MQ2 (Flammable Gas Sensor):
 *   VCC → 5V
 *   GND → GND
 *   D0  → GPIO 14 (digital threshold)
 * 
 * JSN-SR04T (Ultrasonic Sensor):
 *   VCC → 5V
 *   GND → GND
 *   TRIG → GPIO 12
 *   ECHO → GPIO 15 (use voltage divider if needed)
 * 
 * DHT22 (Temperature/Humidity - Optional):
 *   VCC → 3.3V
 *   GND → GND
 *   DATA → GPIO 2 (with 10kΩ pull-up resistor)
 * 
 * GPIO AVAILABILITY ON ESP32-CAM:
 * - GPIO 0, 2, 4, 12, 13, 14, 15, 16 are available
 * - GPIO 1, 3 are UART (U0T, U0R)
 * - Avoid GPIO 4 (flash LED), GPIO 33 (red LED)
 * 
 * ============================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <base64.h>

// ===== WiFi Configuration =====
const char* WIFI_SSID = "YourWiFiSSID";
const char* WIFI_PASS = "YourWiFiPassword";

// ===== Backend API Configuration =====
const char* BACKEND_HOST = "192.168.1.100"; // Change to your server IP
const int BACKEND_PORT = 8000;
const char* IMAGE_ENDPOINT = "/image_base64";

// ===== Camera Pin Configuration (AI-Thinker ESP32-CAM) =====
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

// ===== Sensor Pin Configuration =====
#define MQ135_ANALOG_PIN  13  // GPIO 13 - ADC2_CH4 (MQ135 analog output)
#define MQ2_DIGITAL_PIN   14  // GPIO 14 - MQ2 digital threshold output
#define ULTRASONIC_TRIG   12  // GPIO 12 - JSN-SR04T trigger
#define ULTRASONIC_ECHO   15  // GPIO 15 - JSN-SR04T echo
#define DHT_PIN           2   // GPIO 2 - DHT22 data pin (optional)
#define FLASH_LED_PIN     4   // GPIO 4 - Flash LED

// ===== Sensor Calibration Constants =====
// MQ135 (Air Quality)
const float MQ135_RL = 10.0;      // Load resistance (kΩ)
const float MQ135_R0 = 76.63;     // Base resistance in clean air (Ω)
const float MQ135_VCC = 3.3;      // ESP32 ADC voltage

// MQ2 (Flammable Gas)
const float MQ2_RL = 10.0;
const float MQ2_R0 = 9.83;        // Base resistance (kΩ)

// JSN-SR04T (Ultrasonic)
const float ULTRASONIC_MIN = 25.0;   // cm
const float ULTRASONIC_MAX = 450.0;  // cm
const long ECHO_TIMEOUT = 30000;     // microseconds

// ===== Timing Configuration =====
const unsigned long CAPTURE_INTERVAL = 15000;  // Capture every 15 seconds
unsigned long lastCaptureTime = 0;
bool cameraReady = false;

// ===== Sensor Data Structure =====
struct SensorData {
	float mq135_ppm;
	float mq2_ppm;
	float ultrasonic_cm;
	float temperature;
	float humidity;
	String device_id;
	String timestamp;
};

// ===== Setup Function =====
void setup() {
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	Serial.println("\n\n");
	Serial.println("============================================");
	Serial.println("  ESP32-CAM Waste Monitoring System");
	Serial.println("============================================");
	
	// Configure GPIO pins
	pinMode(MQ2_DIGITAL_PIN, INPUT);
	pinMode(ULTRASONIC_TRIG, OUTPUT);
	pinMode(ULTRASONIC_ECHO, INPUT);
	pinMode(FLASH_LED_PIN, OUTPUT);
	digitalWrite(FLASH_LED_PIN, LOW);  // Flash LED off initially
	
	// Initialize camera
	if (initCamera()) {
		Serial.println("[Camera] Initialized successfully");
		cameraReady = true;
	} else {
		Serial.println("[Camera] FAILED to initialize!");
		cameraReady = false;
	}
	
	// Connect to WiFi
	connectWiFi();
	
	Serial.println("\n[INFO] System ready. Starting image capture loop...\n");
	delay(2000);
}

// ===== Main Loop =====
void loop() {
	unsigned long currentTime = millis();
	
	// Capture and post image with sensors at regular intervals
	if (cameraReady && (currentTime - lastCaptureTime >= CAPTURE_INTERVAL)) {
		lastCaptureTime = currentTime;
		
		// Read all sensors
		SensorData sensors = readAllSensors();
		
		// Display sensor readings
		printSensorData(sensors);
		
		// Capture image
		camera_fb_t* fb = captureImage();
		
		if (fb) {
			// Post image with sensor data
			postImageToBackend(fb, sensors);
			
			// Return frame buffer to free memory
			esp_camera_fb_return(fb);
		} else {
			Serial.println("[Camera] Failed to capture image");
		}
	}
	
	delay(100);
}

// ===== Initialize Camera =====
bool initCamera() {
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
	
	// Image quality settings
	if (psramFound()) {
		config.frame_size = FRAMESIZE_UXGA;  // 1600x1200
		config.jpeg_quality = 10;            // 0-63 (lower = better quality)
		config.fb_count = 2;
		Serial.println("[Camera] PSRAM found - using high quality");
	} else {
		config.frame_size = FRAMESIZE_SVGA;  // 800x600
		config.jpeg_quality = 12;
		config.fb_count = 1;
		Serial.println("[Camera] No PSRAM - using reduced quality");
	}
	
	// Initialize camera
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
		Serial.printf("[Camera] Init failed with error 0x%x\n", err);
		return false;
	}
	
	// Adjust camera settings
	sensor_t* s = esp_camera_sensor_get();
	if (s) {
		s->set_brightness(s, 0);     // -2 to 2
		s->set_contrast(s, 0);       // -2 to 2
		s->set_saturation(s, 0);     // -2 to 2
		s->set_special_effect(s, 0); // 0 = No Effect
		s->set_whitebal(s, 1);       // 0 = disable, 1 = enable
		s->set_awb_gain(s, 1);       // 0 = disable, 1 = enable
		s->set_wb_mode(s, 0);        // 0 to 4
		s->set_exposure_ctrl(s, 1);  // 0 = disable, 1 = enable
		s->set_aec2(s, 0);           // 0 = disable, 1 = enable
		s->set_gain_ctrl(s, 1);      // 0 = disable, 1 = enable
		s->set_agc_gain(s, 0);       // 0 to 30
		s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
		s->set_bpc(s, 0);            // 0 = disable, 1 = enable
		s->set_wpc(s, 1);            // 0 = disable, 1 = enable
		s->set_raw_gma(s, 1);        // 0 = disable, 1 = enable
		s->set_lenc(s, 1);           // 0 = disable, 1 = enable
		s->set_hmirror(s, 0);        // 0 = disable, 1 = enable
		s->set_vflip(s, 0);          // 0 = disable, 1 = enable
		s->set_dcw(s, 1);            // 0 = disable, 1 = enable
		s->set_colorbar(s, 0);       // 0 = disable, 1 = enable
	}
	
	return true;
}

// ===== Capture Image =====
camera_fb_t* captureImage() {
	// Optional: Turn on flash LED for better lighting
	// digitalWrite(FLASH_LED_PIN, HIGH);
	// delay(100);
	
	camera_fb_t* fb = esp_camera_fb_get();
	
	// Turn off flash
	// digitalWrite(FLASH_LED_PIN, LOW);
	
	if (!fb) {
		Serial.println("[Camera] Frame buffer allocation failed");
		return nullptr;
	}
	
	Serial.printf("[Camera] Captured image: %dx%d, %d bytes\n", 
		fb->width, fb->height, fb->len);
	
	return fb;
}

// ===== WiFi Connection =====
void connectWiFi() {
	Serial.print("[WiFi] Connecting to ");
	Serial.println(WIFI_SSID);
	
	WiFi.mode(WIFI_STA);
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
		Serial.print("[WiFi] Signal Strength: ");
		Serial.print(WiFi.RSSI());
		Serial.println(" dBm");
	} else {
		Serial.println("\n[WiFi] Connection FAILED!");
	}
}

// ===== Read All Sensors =====
SensorData readAllSensors() {
	SensorData data;
	data.device_id = "ESP32CAM_001";
	data.timestamp = String(millis());
	
	// Read MQ135 (Air Quality)
	data.mq135_ppm = readMQ135();
	
	// Read MQ2 (Flammable Gas)
	data.mq2_ppm = readMQ2();
	
	// Read JSN-SR04T (Ultrasonic Distance)
	data.ultrasonic_cm = readUltrasonic();
	
	// Read DHT22 (Temperature/Humidity) - Optional
	// Uncomment if you have DHT sensor installed
	// #include <DHT.h>
	// DHT dht(DHT_PIN, DHT22);
	// data.temperature = dht.readTemperature();
	// data.humidity = dht.readHumidity();
	
	// Placeholder values if DHT not installed
	data.temperature = 25.0;
	data.humidity = 60.0;
	
	return data;
}

// ===== Read MQ135 (Air Quality Sensor) =====
float readMQ135() {
	// Average multiple samples
	long sum = 0;
	const int samples = 10;
	
	for (int i = 0; i < samples; i++) {
		sum += analogRead(MQ135_ANALOG_PIN);
		delay(10);
	}
	
	float rawADC = sum / (float)samples;
	
	// ESP32 ADC: 12-bit (0-4095), voltage divider may be needed
	float voltage = (rawADC / 4095.0) * MQ135_VCC;
	
	// Error check
	if (voltage < 0.1) {
		return -1;  // Error: voltage too low
	}
	
	if (voltage >= MQ135_VCC - 0.1) {
		return -2;  // Error: voltage too high
	}
	
	// Calculate Rs (sensor resistance)
	float Rs = ((MQ135_VCC * MQ135_RL) / voltage) - MQ135_RL;
	
	if (Rs <= 0) {
		return -3;  // Error: invalid Rs
	}
	
	// Calculate ratio Rs/R0
	float ratio = Rs / MQ135_R0;
	
	if (ratio < 0.01 || ratio > 100) {
		return -4;  // Error: ratio out of range
	}
	
	// Convert to PPM (CO2 approximation)
	float ppm = pow(10, (-0.42 * log10(ratio) + 3.5));
	
	return ppm;
}

// ===== Read MQ2 (Flammable Gas Sensor) =====
float readMQ2() {
	// Read digital threshold output
	int digitalValue = digitalRead(MQ2_DIGITAL_PIN);
	
	if (digitalValue == LOW) {
		// Gas detected (threshold exceeded)
		return 1500.0;  // Estimated threshold value
	} else {
		// No gas detected
		return 150.0;   // Below threshold
	}
	
	// For analog reading, connect to ADC pin and use similar calculation as MQ135
}

// ===== Read JSN-SR04T (Ultrasonic Distance Sensor) =====
float readUltrasonic() {
	// Send trigger pulse
	digitalWrite(ULTRASONIC_TRIG, LOW);
	delayMicroseconds(2);
	digitalWrite(ULTRASONIC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(ULTRASONIC_TRIG, LOW);
	
	// Read echo pulse
	long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ECHO_TIMEOUT);
	
	if (duration == 0) {
		return -5;  // Error: no echo (timeout)
	}
	
	// Calculate distance (cm)
	float distance = duration / 58.0;
	
	// Validate range
	if (distance < ULTRASONIC_MIN) {
		return -4;  // Error: below minimum range
	}
	
	if (distance > ULTRASONIC_MAX) {
		return -5;  // Error: above maximum range
	}
	
	return distance;
}

// ===== Print Sensor Data =====
void printSensorData(const SensorData& data) {
	Serial.println("\n========================================");
	Serial.println("        SENSOR READINGS");
	Serial.println("========================================");
	Serial.print("Device ID: ");
	Serial.println(data.device_id);
	Serial.print("Timestamp: ");
	Serial.println(data.timestamp);
	Serial.println();
	
	Serial.print("MQ135 (Air Quality): ");
	if (data.mq135_ppm < 0) {
		Serial.print("ERROR (");
		Serial.print(data.mq135_ppm);
		Serial.println(")");
	} else {
		Serial.print(data.mq135_ppm, 2);
		Serial.println(" ppm");
	}
	
	Serial.print("MQ2 (Flammable Gas): ");
	if (data.mq2_ppm < 0) {
		Serial.print("ERROR (");
		Serial.print(data.mq2_ppm);
		Serial.println(")");
	} else {
		Serial.print(data.mq2_ppm, 2);
		Serial.println(" ppm");
	}
	
	Serial.print("Ultrasonic Distance: ");
	if (data.ultrasonic_cm < 0) {
		Serial.print("ERROR (");
		Serial.print(data.ultrasonic_cm);
		Serial.println(")");
	} else {
		Serial.print(data.ultrasonic_cm, 2);
		Serial.println(" cm");
	}
	
	Serial.print("Temperature: ");
	Serial.print(data.temperature, 1);
	Serial.println(" °C");
	
	Serial.print("Humidity: ");
	Serial.print(data.humidity, 1);
	Serial.println(" %");
	
	Serial.println("========================================\n");
}

// ===== Post Image to Backend =====
void postImageToBackend(camera_fb_t* fb, const SensorData& sensors) {
	if (WiFi.status() != WL_CONNECTED) {
		Serial.println("[ERROR] WiFi not connected");
		return;
	}
	
	HTTPClient http;
	String url = "http://" + String(BACKEND_HOST) + ":" + String(BACKEND_PORT) + IMAGE_ENDPOINT;
	
	Serial.println("[POST] Encoding image to base64...");
	
	// Encode image to base64
	String imageBase64 = base64::encode(fb->buf, fb->len);
	
	Serial.printf("[POST] Base64 size: %d bytes\n", imageBase64.length());
	
	// Create JSON payload
	StaticJsonDocument<512> sensorDoc;
	sensorDoc["device_id"] = sensors.device_id;
	sensorDoc["timestamp"] = sensors.timestamp;
	sensorDoc["mq135_ppm"] = sensors.mq135_ppm;
	sensorDoc["mq2_ppm"] = sensors.mq2_ppm;
	sensorDoc["ultrasonic_cm"] = sensors.ultrasonic_cm;
	sensorDoc["temperature"] = sensors.temperature;
	sensorDoc["humidity"] = sensors.humidity;
	sensorDoc["lat"] = "22.5726";  // Add GPS if available
	sensorDoc["lon"] = "88.3639";
	
	String sensorsJson;
	serializeJson(sensorDoc, sensorsJson);
	
	// Create main payload
	DynamicJsonDocument mainDoc(imageBase64.length() + 1024);
	mainDoc["imageBase64"] = "data:image/jpeg;base64," + imageBase64;
	mainDoc["sensors"] = serialized(sensorsJson);
	
	String payload;
	serializeJson(mainDoc, payload);
	
	Serial.println("[POST] Sending to backend...");
	Serial.print("[POST] URL: ");
	Serial.println(url);
	
	http.begin(url);
	http.addHeader("Content-Type", "application/json");
	http.setTimeout(30000);  // 30 second timeout for large images
	
	int httpCode = http.POST(payload);
	
	Serial.print("[POST] Response Code: ");
	Serial.println(httpCode);
	
	if (httpCode == 200) {
		String response = http.getString();
		Serial.println("[POST] Response:");
		Serial.println(response);
		parseBackendResponse(response);
	} else if (httpCode > 0) {
		Serial.printf("[POST] HTTP Error: %s\n", http.errorToString(httpCode).c_str());
		String response = http.getString();
		Serial.println(response);
	} else {
		Serial.println("[POST] Connection failed!");
	}
	
	http.end();
}

// ===== Parse Backend Response =====
void parseBackendResponse(const String& jsonResponse) {
	DynamicJsonDocument doc(2048);
	DeserializationError error = deserializeJson(doc, jsonResponse);
	
	if (error) {
		Serial.print("[PARSE ERROR] ");
		Serial.println(error.c_str());
		return;
	}
	
	Serial.println("\n========================================");
	Serial.println("     BACKEND ANALYSIS RESULTS");
	Serial.println("========================================");
	
	if (doc.containsKey("prediction")) {
		Serial.print("Prediction: ");
		Serial.println(doc["prediction"].as<String>());
	}
	
	if (doc.containsKey("waste_category")) {
		Serial.print("Waste Category: ");
		Serial.println(doc["waste_category"].as<String>());
	}
	
	if (doc.containsKey("waste_subtype")) {
		Serial.print("Waste Subtype: ");
		Serial.println(doc["waste_subtype"].as<String>());
	}
	
	if (doc.containsKey("hazard")) {
		Serial.print("Hazard: ");
		Serial.println(doc["hazard"].as<int>() ? "YES" : "NO");
	}
	
	// Gas Analysis
	if (doc.containsKey("gas_analysis")) {
		Serial.println("\n--- Gas Analysis ---");
		JsonObject gas = doc["gas_analysis"];
		
		if (gas.containsKey("probable_gas")) {
			Serial.print("Probable Gas: ");
			Serial.println(gas["probable_gas"].as<String>());
		}
		
		if (gas.containsKey("hazard_level")) {
			Serial.print("Hazard Level: ");
			Serial.println(gas["hazard_level"].as<String>());
		}
		
		if (gas.containsKey("recommendation")) {
			Serial.print("Recommendation: ");
			Serial.println(gas["recommendation"].as<String>());
		}
	}
	
	// Ultrasonic Analysis
	if (doc.containsKey("ultrasonic_analysis")) {
		Serial.println("\n--- Ultrasonic Analysis ---");
		JsonObject ultra = doc["ultrasonic_analysis"];
		
		if (ultra.containsKey("distance_cm")) {
			Serial.print("Distance: ");
			Serial.print(ultra["distance_cm"].as<float>(), 2);
			Serial.println(" cm");
		}
		
		if (ultra.containsKey("fill_level_pct")) {
			Serial.print("Fill Level: ");
			Serial.print(ultra["fill_level_pct"].as<float>(), 1);
			Serial.println(" %");
		}
		
		if (ultra.containsKey("note")) {
			Serial.print("Note: ");
			Serial.println(ultra["note"].as<String>());
		}
	}
	
	Serial.println("========================================\n");
}
