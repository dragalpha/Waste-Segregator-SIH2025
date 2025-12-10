# ESP32-CAM FTDI Programmer Setup Guide

## Hardware Requirements

### Components Needed
- **ESP32-CAM** (AI-Thinker module)
- **FTDI USB to TTL Serial Adapter** (FT232RL or similar)
  - Must support 5V output or use external 5V power supply
- **Jumper wires** (female-to-female recommended)
- **External 5V Power Supply** (optional but recommended, 2A minimum)
- **Micro USB cable** (for FTDI programmer)

---

## FTDI Wiring Diagram

### Upload Mode (Programming Mode)

```
FTDI Programmer          ESP32-CAM
─────────────────        ─────────────────
GND        ────────────→ GND
5V         ────────────→ 5V
TX         ────────────→ U0R (RX)
RX         ────────────→ U0T (TX)

** CRITICAL CONNECTION **
GPIO 0     ────────────→ GND  (short IO0 to GND for upload mode)
```

### Running Mode (Normal Operation)

```
FTDI Programmer          ESP32-CAM
─────────────────        ─────────────────
GND        ────────────→ GND
5V         ────────────→ 5V
TX         ────────────→ U0R (RX)
RX         ────────────→ U0T (TX)

GPIO 0     ────────────  (DISCONNECTED - leave floating)
```

---

## Pin Identification on ESP32-CAM

### ESP32-CAM Pin Layout (AI-Thinker)

```
        ┌───────────────────┐
        │   SD Card Slot    │
        ├───────────────────┤
        │                   │
GND  ───┤ •              •  ├─── 5V
IO12 ───┤ •              •  ├─── GND
IO13 ───┤ •              •  ├─── IO15
IO15 ───┤ •              •  ├─── IO14
IO14 ───┤ •              •  ├─── IO2
IO2  ───┤ •              •  ├─── IO4
IO4  ───┤ •              •  ├─── GND
GND  ───┤ •              •  ├─── VCC (5V)
VCC  ───┤ •              •  ├─── RESET
        │                   │
U0T  ───┤ •              •  ├─── IO16
U0R  ───┤ •              •  ├─── GND
IO0  ───┤ •              •  ├─── IO1
        │                   │
        │    [Camera]       │
        │                   │
        └───────────────────┘
```

**Key Pins:**
- **5V / VCC**: Power input (red wire)
- **GND**: Ground (black wire)
- **U0T**: TX pin - transmit data FROM ESP32 (connect to FTDI RX)
- **U0R**: RX pin - receive data TO ESP32 (connect to FTDI TX)
- **IO0**: GPIO 0 - Boot mode selection (connect to GND for upload)
- **RESET**: Reset button on module

---

## Step-by-Step Upload Process

### Step 1: Physical Connections

1. **Power off everything** before connecting
2. Connect jumper wires as per wiring diagram above
3. **Important**: Cross-connect TX/RX:
   - FTDI **TX** → ESP32-CAM **U0R** (RX)
   - FTDI **RX** → ESP32-CAM **U0T** (TX)
4. **Critical**: Connect GPIO 0 (IO0) to GND to enable programming mode

### Step 2: Arduino IDE Setup

1. **Install ESP32 Board Support**:
   - Open Arduino IDE
   - Go to: **File → Preferences**
   - Add to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to: **Tools → Board → Boards Manager**
   - Search "ESP32" and install "esp32 by Espressif Systems"

2. **Board Configuration**:
   - **Board**: "AI Thinker ESP32-CAM"
   - **Upload Speed**: 115200 baud
   - **Flash Frequency**: 80MHz
   - **Flash Mode**: QIO
   - **Partition Scheme**: "Huge APP (3MB No OTA/1MB SPIFFS)"
   - **Core Debug Level**: "None"
   - **Port**: Select your FTDI COM port (e.g., COM3, COM4)

### Step 3: Install Required Libraries

Open Arduino IDE → **Tools → Manage Libraries** and install:

- **ArduinoJson** (by Benoit Blanchon) - version 6.x
- **Base64** (by Densaugeo)
- Optional: **DHT sensor library** (by Adafruit) - if using DHT22

### Step 4: Upload Code

1. Open `esp32cam_waste_monitor.ino` in Arduino IDE
2. Update WiFi credentials:
   ```cpp
   const char* WIFI_SSID = "YourWiFiSSID";
   const char* WIFI_PASS = "YourWiFiPassword";
   ```
3. Update backend host:
   ```cpp
   const char* BACKEND_HOST = "192.168.1.100";
   ```
4. **Verify GPIO 0 is connected to GND** (programming mode)
5. Click **Upload** button (→)
6. Watch for upload progress in console

### Step 5: Enter Run Mode

1. **Disconnect GPIO 0 from GND** (very important!)
2. Press **RESET** button on ESP32-CAM
3. Or power cycle: disconnect 5V, reconnect 5V
4. Open **Serial Monitor** (115200 baud) to see output

---

## Troubleshooting

### Problem: "Failed to connect to ESP32"

**Solution**:
- Verify GPIO 0 is connected to GND
- Check TX/RX are crossed (FTDI TX → ESP32 RX)
- Try lower upload speed: 9600 or 57600 baud
- Press RESET button while clicking Upload
- Ensure stable 5V power supply (minimum 500mA)

### Problem: "Brownout detector was triggered"

**Solution**:
- ESP32-CAM needs stable 5V with high current (2A recommended)
- Use external 5V power supply instead of USB power
- Add capacitor (100µF-1000µF) across 5V and GND pins
- Check for loose connections

### Problem: Camera init failed

**Solution**:
- Check camera ribbon cable is properly seated
- Camera module may be damaged
- Try lowering frame size in code:
  ```cpp
  config.frame_size = FRAMESIZE_VGA;  // 640x480
  ```

### Problem: "Guru Meditation Error"

**Solution**:
- Reduce image quality or size
- Check PSRAM availability (run example: File → Examples → ESP32 → Camera → CameraWebServer)
- Ensure GPIO pins don't conflict with camera pins

### Problem: WiFi won't connect

**Solution**:
- Check SSID and password are correct
- Ensure 2.4GHz WiFi (ESP32 doesn't support 5GHz)
- Move closer to WiFi router
- Check WiFi network allows new devices

### Problem: Upload success but no serial output

**Solution**:
- GPIO 0 must be DISCONNECTED after upload
- Press RESET button
- Check Serial Monitor baud rate is 115200
- Verify FTDI RX is connected to ESP32 TX (U0T)

---

## Power Supply Recommendations

### Option 1: FTDI Programmer Power (Basic)
- **Pros**: Simple, no external supply needed
- **Cons**: May be unstable, limited current (~500mA)
- **Use when**: Testing only, short cable runs

### Option 2: External 5V Power Supply (Recommended)
- **Pros**: Stable, high current (2A+), reliable camera operation
- **Cons**: Requires additional hardware
- **Connection**:
  ```
  5V PSU (+) → ESP32-CAM 5V
  5V PSU (-) → ESP32-CAM GND AND FTDI GND (common ground!)
  ```

### Option 3: USB Power Bank
- **Pros**: Portable, stable power
- **Cons**: Need USB to DC converter cable
- **Current**: Minimum 2A output

---

## GPIO Pin Usage Summary

### Used by Camera (DO NOT USE)
- GPIO 0, 4, 5, 18, 19, 21, 22, 23, 25, 26, 27, 32, 35, 36, 39

### Available for Sensors
- **GPIO 2**: DHT22 data pin (with pull-up)
- **GPIO 12**: Ultrasonic trigger
- **GPIO 13**: MQ135 analog (ADC2_CH4)
- **GPIO 14**: MQ2 digital threshold
- **GPIO 15**: Ultrasonic echo (use voltage divider)
- **GPIO 16**: Reserved (PSRAM if available)

### UART Pins
- **GPIO 1 (U0T)**: TX - Serial output
- **GPIO 3 (U0R)**: RX - Serial input

---

## Testing the Setup

### Test 1: Serial Output
1. Upload code in programming mode
2. Disconnect IO0 from GND, press RESET
3. Open Serial Monitor (115200 baud)
4. You should see:
   ```
   ============================================
     ESP32-CAM Waste Monitoring System
   ============================================
   [Camera] Initialized successfully
   [WiFi] Connecting to YourWiFi...
   [WiFi] Connected!
   ```

### Test 2: Camera Capture
- Check for output: `[Camera] Captured image: 1600x1200, XXXXX bytes`

### Test 3: Sensor Readings
- Verify sensor values in "SENSOR READINGS" section
- Negative values indicate errors (check wiring)

### Test 4: Backend Communication
- Look for `[POST] Response Code: 200`
- Backend should return waste classification and analysis

---

## Arduino IDE Serial Monitor Settings

```
Baud Rate: 115200
Line Ending: Both NL & CR (or Newline)
```

---

## Common Error Codes

| Code | Sensor | Meaning |
|------|--------|---------|
| -1 | MQ135/MQ2 | Sensor value missing or voltage too low |
| -2 | MQ135/MQ2 | Invalid format or voltage too high |
| -3 | MQ135/MQ2 | Invalid sensor resistance calculation |
| -4 | Ultrasonic | Below minimum range (< 25cm) |
| -5 | Ultrasonic | Above maximum range (> 450cm) or timeout |

---

## Next Steps

1. ✅ Upload code successfully
2. ✅ Verify serial output shows camera init and WiFi connection
3. ✅ Check sensor readings are valid (not negative error codes)
4. ✅ Confirm image capture every 15 seconds
5. ✅ Verify backend receives images and returns analysis
6. ✅ Mount ESP32-CAM in waste bin with sensors
7. ✅ Test in production environment

---

## Additional Resources

- **ESP32-CAM Datasheet**: [AI-Thinker ESP32-CAM](https://github.com/SeeedDocument/forum_doc/blob/master/reg/ESP32_CAM_V1.6.pdf)
- **ESP32 Arduino Core**: [GitHub Repository](https://github.com/espressif/arduino-esp32)
- **Camera Examples**: Arduino IDE → File → Examples → ESP32 → Camera
