# Waste Segregator - SIH 2025
# Waste-Segregator-SIH2025

## Purpose
This project demonstrates a complete pipeline:
- Laptop webcam captures images → server does YOLOv8n inference
- ESP8266 polls server for latest detection and posts telemetry including the detection
- Backend stores everything into `data/telemetry.csv`
- Heatmap generated from CSV

## Important model path
Place the model at `./ai/yolo/yolov8n.pt` or ensure `/mnt/data/yolov8n.pt` exists (uploaded path).

## Setup (Windows)
1. Create venv and install deps:
   ```powershell
   python -m venv venv
   .\venv\Scripts\Activate.ps1
   pip install --upgrade pip
   pip install -r ai/docker/requirements.txt
   ```

If `ultralytics` pulls `torch` and fails, install CPU wheel:

```powershell
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
```

2. Start server:

   ```powershell
   python ai/api/main.py
   ```

3. In another terminal, run webcam poster:

   ```powershell
   python ai/yolo/post_webcam_demo.py
   ```

4. (Optional) generate heatmap after some data:

   ```powershell
   python ai/yolo/generate_heatmap.py
   ```

5. Open browser to:

   * `http://127.0.0.1:8000/csv` to download CSV
   * or open `frontend/index.html` (make sure server is running as it fetches `/csv`)

6. Flash ESP8266 with `ESP8266/esp8266_telemetry.ino` (change SSID, password, server IP). ESP will GET `/detected` and POST `/telemetry`.

## Notes

* Model default lookup checks `./ai/yolo/yolov8n.pt` and `/mnt/data/yolov8n.pt`. The uploaded model is at `/mnt/data/yolov8n.pt`.
* For demos to judges you can host server on laptop and use `ngrok` for remote demo link.

## Reference Presentation

This project references the submitted idea presentation located at `/mnt/data/SIH2025-IDEA-Presentation25014.pdf`.

## Windows Firewall Note

On Windows, allow incoming connections to Python (or port 8000) when prompted by the firewall. If you miss the prompt, open "Windows Defender Firewall" → "Allow an app or feature through Windows Defender Firewall" and add `python.exe` for your venv.

## Generating PDF Report and Heatmap

After enough telemetry exists in `data/telemetry.csv`:

```powershell
python ai/yolo/generate_heatmap.py
python ai/reports/generate_pdf_report.py
```

The PDF will be saved to `data/report_latest.pdf` and the heatmap to `data/heatmap.html`.
