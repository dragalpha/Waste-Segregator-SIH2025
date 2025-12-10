# ESP32-CAM Setup Checklist - Fix "esp_camera.h" Error

## ❌ Current Problem
```
fatal error: esp_camera.h: No such file or directory
```

This error means **ESP32 board support is NOT installed** in your Arduino IDE.

---

## ✅ Solution: Follow These Steps EXACTLY

### STEP 1: Install ESP32 Board Support (5-10 minutes)

1. **Open Arduino IDE**

2. **Add ESP32 Board Manager URL**:
   - Go to: `File` → `Preferences` (or press `Ctrl+Comma`)
   - Find the field: **"Additional Board Manager URLs"**
   - Paste this URL:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - If other URLs exist, click the window icon (📄) to add it on a new line
   - Click **OK**

3. **Install ESP32 Boards**:
   - Go to: `Tools` → `Board` → `Boards Manager` (or press `Ctrl+Shift+B`)
   - Wait for the package list to load (may take 30 seconds)
   - In the search box, type: **esp32**
   - Find: **"esp32 by Espressif Systems"**
   - Click **INSTALL** button
   - Wait for download and installation (5-10 minutes depending on internet speed)
   - You'll see "INSTALLED" when complete

4. **Restart Arduino IDE** ⚠️ CRITICAL STEP
   - Close Arduino IDE completely
   - Reopen Arduino IDE

---

### STEP 2: Select ESP32-CAM Board

1. **Select the Board**:
   - Go to: `Tools` → `Board` → `ESP32 Arduino` (expand this menu)
   - Scroll down and find: **"AI Thinker ESP32-CAM"**
   - Click to select it

2. **Verify Selection**:
   - Check `Tools` → `Board` shows: "AI Thinker ESP32-CAM"

---

### STEP 3: Configure Upload Settings

Set these in the `Tools` menu:

- ✅ **Board**: "AI Thinker ESP32-CAM"
- ✅ **Upload Speed**: 115200
- ✅ **Flash Frequency**: 80MHz
- ✅ **Flash Mode**: QIO
- ✅ **Partition Scheme**: "Huge APP (3MB No OTA/1MB SPIFFS)"
- ✅ **Core Debug Level**: None
- ✅ **Port**: Your FTDI COM port (e.g., COM3, COM4, COM5)

---

### STEP 4: Install Required Libraries

1. Go to: `Tools` → `Manage Libraries` (or press `Ctrl+Shift+I`)

2. Install these libraries one by one:

   **Library 1: ArduinoJson**
   - Search: `ArduinoJson`
   - Install: **"ArduinoJson" by Benoit Blanchon**
   - Version: 6.21.3 or later

   **Library 2: Base64**
   - Search: `Base64`
   - Install: **"Base64" by Densaugeo**
   - Version: 1.4.0 or later

---

### STEP 5: Test Your Setup

Before compiling the main code, test with a built-in example:

1. Go to: `File` → `Examples` → `ESP32` → `Camera` → `CameraWebServer`
2. Click **Verify** button (✓) or press `Ctrl+R`
3. Wait for compilation

**Expected Result**: Should compile successfully without errors

**If it compiles**: ✅ Your ESP32 setup is correct! Proceed to compile your code.

**If it fails**: ❌ ESP32 boards not properly installed. Repeat STEP 1.

---

## 🔧 Troubleshooting

### Problem 1: "esp32" not found in Boards Manager

**Solution**:
- Check internet connection
- Verify Board Manager URL was added correctly in Preferences
- Restart Arduino IDE and try again
- Try alternative URL:
  ```
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
  ```

### Problem 2: "AI Thinker ESP32-CAM" not in board list

**Solution**:
- ESP32 boards not installed - complete STEP 1
- Restart Arduino IDE after installation
- Check Tools → Board → ESP32 Arduino (make sure this submenu exists)

### Problem 3: Compilation still fails after installation

**Solution**:
- Verify correct board is selected: Tools → Board → "AI Thinker ESP32-CAM"
- Restart Arduino IDE
- Try uninstalling and reinstalling ESP32 boards:
  - Tools → Board → Boards Manager
  - Find "esp32 by Espressif Systems"
  - Click REMOVE
  - Restart Arduino IDE
  - Reinstall from STEP 1

### Problem 4: Arduino IDE version too old

**Solution**:
- Update to Arduino IDE 1.8.19 or newer
- Or use Arduino IDE 2.x (recommended)
- Download from: https://www.arduino.cc/en/software

---

## 📝 Quick Verification Checklist

Before attempting to compile, verify:

- [ ] ESP32 board support is installed (see Boards Manager)
- [ ] Arduino IDE was restarted after installation
- [ ] "AI Thinker ESP32-CAM" is selected in Tools → Board
- [ ] ArduinoJson library is installed
- [ ] Base64 library is installed
- [ ] Built-in CameraWebServer example compiles successfully

If all checkboxes are ticked, your code should compile!

---

## 🎯 Expected Output After Successful Compilation

When compilation succeeds, you'll see:

```
Sketch uses XXXXX bytes (XX%) of program storage space.
Global variables use XXXXX bytes (XX%) of dynamic memory.
```

---

## 🆘 Still Having Issues?

1. **Check Arduino IDE version**: Must be 1.8.13 or newer
2. **Check internet connection**: Board installation requires download
3. **Check antivirus**: May block downloads - temporarily disable
4. **Try alternative**: Use PlatformIO instead of Arduino IDE

---

## 📚 Additional Resources

- **ESP32-CAM Official Guide**: https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/
- **ESP32 Arduino Core**: https://github.com/espressif/arduino-esp32
- **FTDI Programming Guide**: See ESP32CAM_FTDI_PROGRAMMING_GUIDE.md
