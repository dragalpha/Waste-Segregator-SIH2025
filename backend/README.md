# Backend — Waste Segregator (development)

This README covers how to run and test the FastAPI backend located under `backend/ai`.

Prerequisites
- Python 3.11+ (use a virtualenv)
- `git` (optional, to commit changes)
- (Optional) Docker if you want to build the image

Quick start (recommended, local dev)

1. Create and activate a virtualenv (Windows `cmd`):

```bat
cd C:\Users\SANTAM\Desktop\Waste-Segregator-SIH2025
python -m venv .venv
.venv\Scripts\activate
pip install --upgrade pip
pip install -r backend\ai\docker\requirements.txt
```

2. Start the FastAPI backend (dev):

```bat
# option A: run the main script (shows full tracebacks)
python backend\ai\api\main.py

# option B: use uvicorn and point at the backend app dir (recommended)
uvicorn api.main:app --app-dir backend\ai --host 127.0.0.1 --port 8000 --reload
```

Notes:
- Use `--host 0.0.0.0` with uvicorn to make the server reachable on the LAN (e.g. `http://192.168.x.y:8000`).
- If you change the host to `0.0.0.0`, you may need to allow the port through Windows Firewall.

Restarting the backend (Windows cmd)
-----------------------------------

If the server is already running in a terminal, stop it with Ctrl+C in that terminal.

If you cannot access the original terminal, find and stop the process listening on port 8000:

```bat
REM find the PID listening on port 8000
netstat -ano | findstr :8000

REM kill the process (replace <PID> with the number from the previous command)
taskkill /F /PID <PID>
```

Then start the backend (examples):

```bat
REM start for local-only development
uvicorn api.main:app --app-dir backend\ai --host 127.0.0.1 --port 8000 --reload

REM bind to all interfaces for LAN access (edit Firewall rules if prompted)
uvicorn api.main:app --app-dir backend\ai --host 0.0.0.0 --port 8000
```

Running in Docker (optional)
----------------------------

If you'd rather run inside Docker (image build may take time due to ML deps):

```bat
REM from repo root
docker build -t waste-segregator-backend -f backend\ai\docker\Dockerfile .
docker run -p 8000:8000 --rm waste-segregator-backend
```

Endpoints (dev)
- `POST /image` — form upload: `image` file plus optional `lat, lon, sensors` form fields. Returns `prediction`, `yolo_raw`, `waste_category`, `waste_subtype` and saves a telemetry row to `backend/data/telemetry.csv`.
- `POST /telemetry` — accepts canonical telemetry JSON (see schema below) and appends a row to telemetry CSV.
- `GET /detected` — returns the last detection string (read from `backend/ai/yolo/detected.txt`).
- `POST /set_detected` — dev-only: write `detected.txt` (JSON {"value":"label:0.88"}).
- `GET /csv` — download `backend/data/telemetry.csv`.
- `GET /heatmap` — serves `backend/data/heatmap.html` when generated.

Canonical telemetry CSV header (the server writes this format):

```
timestamp_utc,device_id,boat_id,lat,lon,heading_deg,mq135_ppm,mq2_ppm,soil_dry_belt_pct,soil_wet_belt_pct,loadcell_grams,tds_ppm,ultrasonic_cm,proximity_inductive,image_path,yolo_raw,waste_category,waste_subtype,collection_event,collection_bin_id,battery_volt,rssi
```

Quick test commands (run after starting server):

```bat
# check detected
curl http://127.0.0.1:8000/detected

# set a dev detection
curl -X POST -H "Content-Type: application/json" -d "{\"value\":\"plastic:0.88\"}" http://127.0.0.1:8000/set_detected

# post an image (replace with a real path)
curl -v -F "image=@C:\path\to\test.jpg" http://127.0.0.1:8000/image

# download csv
curl -O http://127.0.0.1:8000/csv

Download `telemetry.csv` from a browser or PowerShell
---------------------------------------------------

- Browser: open `http://127.0.0.1:8000/csv` or `http://<your-pc-ip>:8000/csv` to download directly.
- PowerShell (save to file):

```powershell
Invoke-WebRequest http://127.0.0.1:8000/csv -OutFile telemetry.csv
```

If the server serves a `backend/data/telemetry.csv` file directly, you can also copy it from disk:

```bat
copy backend\data\telemetry.csv .\telemetry.csv
```
```

ESP8266 sketch (mock)
- `backend/ESP8266/esp8266_telemetry.ino` is included. Edit `ssid`, `password`, and `server` (set to your PC LAN IP, e.g., `http://192.168.0.199:8000`) before uploading.

Notes and next steps
- If you need historic CSV migration, implement a migration that maps old headers to the canonical columns and backs up the original file as `telemetry.csv.bak`.
- If inference returns `{"prediction":"-"}`, verify a model file exists at `backend/ai/yolo/yolov8n.pt` or `/mnt/data/yolov8n.pt` and run the inference script directly for debugging:

```bat
python backend\ai\yolo\inference_yolov8.py --image C:\path\to\test.jpg --model backend\ai\yolo\yolov8n.pt
```

Git commit (example)

```bat
git add backend/ai/api/main.py backend/ai/yolo/post_webcam_demo.py backend/ESP8266/esp8266_telemetry.ino backend/README.md
git commit -m "Backend: canonical CSV, structured /image response, /set_detected endpoint, patch webcam poster, add ESP sketch, README"
git push origin main
```

If you want, I can:
- Add an automated CSV migration (backup + map old rows)
- Add a simple test script that posts a sample image and verifies the CSV entry
- Commit the changes for you and create a small git-friendly release note

