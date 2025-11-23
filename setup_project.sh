#!/bin/bash
set -e

echo "=== Creating directories ==="
mkdir -p backend config controllers models routes utils mqtt websocket
mkdir -p ai ai/docker ai/yolo ai/cnn ai/api ai/venv
mkdir -p data data/images frontend

echo "=== Removing old YOLOv5 files ==="
rm -f taco-yolov5s.pt || true

echo "=== Copying uploaded YOLOv8n model ==="
cp /mnt/data/yolov8n.pt ai/yolo/yolov8n.pt 2>/dev/null || true

echo "=== Copying SIH PDF ==="
cp /mnt/data/SIH2025-IDEA-Presentation25014.pdf ai/ 2>/dev/null || true

###############################################
### FASTAPI BACKEND
###############################################
cat > ai/api/main.py << 'EOF'
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse, FileResponse
import uvicorn, os, csv, datetime, json, subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data"
IMG_DIR = DATA_DIR / "images"
CSV_FILE = DATA_DIR / "telemetry.csv"

app = FastAPI()

DATA_DIR.mkdir(exist_ok=True)
IMG_DIR.mkdir(exist_ok=True)

if not CSV_FILE.exists():
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp","lat","lon","ultrasonic","temp","tds","compass","image_path","prediction"])

def append_row(r):
    with open(CSV_FILE, "a", newline="") as f:
        csv.writer(f).writerow(r)

def get_model():
    local = ROOT/"yolo"/"yolov8n.pt"
    if local.exists(): return str(local)
    fallback = Path("/mnt/data/yolov8n.pt")
    if fallback.exists(): return str(fallback)
    return "yolov8n.pt"

@app.post("/telemetry")
async def telemetry(json_data: dict):
    ts = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
    row = [
        ts,
        json_data.get("lat","-"),
        json_data.get("lon","-"),
        json_data.get("ultrasonic",""),
        json_data.get("temp",""),
        json_data.get("tds",""),
        json_data.get("compass",""),
        "-",
        "-"
    ]
    append_row(row)
    return {"status":"ok"}

@app.post("/image")
async def image(
    lat: str = Form(None),
    lon: str = Form(None),
    sensors: str = Form("{}"),
    image: UploadFile = File(...)
):
    ts = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
    path = IMG_DIR / f"{ts}.jpg"

    # Save image
    with open(path, "wb") as f:
        f.write(await image.read())

    # Parse sensors
    try:
        s = json.loads(sensors)
    except:
        s = {}

    # Run inference
    try:
        proc = subprocess.run([
            "python",
            str(ROOT/"yolo"/"inference_yolov8.py"),
            "--image", str(path),
            "--model", get_model()
        ], capture_output=True, text=True)
        pred = proc.stdout.strip() or "-"
    except:
        pred = "-"

    # Append CSV row
    row = [
        datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S"),
        lat or "-", lon or "-",
        s.get("ultrasonic",""),
        s.get("temp",""),
        s.get("tds",""),
        s.get("compass",""),
        str(path),
        pred
    ]
    append_row(row)

    return {"status":"ok","path":str(path),"prediction":pred}

@app.get("/csv")
def csv_file():
    return FileResponse(str(CSV_FILE))

@app.get("/heatmap")
def heatmap_file():
    p = DATA_DIR/"heatmap.html"
    if p.exists(): return FileResponse(str(p))
    return {"error":"heatmap not generated yet"}

if __name__ == "__main__":
    uvicorn.run(app,host="0.0.0.0",port=8000)
EOF

###############################################
### YOLO INFERENCE SCRIPT
###############################################
cat > ai/yolo/inference_yolov8.py << 'EOF'
import argparse, sys
from ultralytics import YOLO

parser = argparse.ArgumentParser()
parser.add_argument("--image", required=True)
parser.add_argument("--model", default="./ai/yolo/yolov8n.pt")
parser.add_argument("--conf", type=float, default=0.35)
args = parser.parse_args()

model = YOLO(args.model)
res = model.predict(args.image, conf=args.conf, verbose=False)[0]

if len(res.boxes) > 0:
    b = res.boxes[0]
    name = res.names.get(int(b.cls[0]), "object")
    conf = float(b.conf[0])
    print(f"{name}:{conf:.2f}")
else:
    print("-")
EOF

###############################################
### WEBCAM UPLOADER (since NO ESP32-CAM yet)
###############################################
cat > ai/yolo/post_webcam_demo.py << 'EOF'
import cv2, requests, time, json

URL = "http://127.0.0.1:8000/image"
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Cannot open webcam")

print("Posting webcam frames to:", URL)

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    _, img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    files = {"image": ("frame.jpg", img.tobytes(), "image/jpeg")}
    data = {"lat":"22.5726","lon":"88.3639","sensors": json.dumps({})}

    try:
        r = requests.post(URL, files=files, data=data, timeout=10)
        print(r.json())
    except Exception as e:
        print("POST failed:", e)

    time.sleep(5)
EOF

###############################################
### HEATMAP GENERATOR
###############################################
cat > ai/yolo/generate_heatmap.py << 'EOF'
import pandas as pd
from folium import Map
from folium.plugins import HeatMap
from pathlib import Path

CSV = Path("../data/telemetry.csv")
OUT = Path("../data/heatmap.html")

df = pd.read_csv(CSV)
df = df[pd.to_numeric(df['lat'], errors="coerce").notnull()]
df['lat'] = pd.to_numeric(df['lat'])
df['lon'] = pd.to_numeric(df['lon'])

if df.empty:
    print("No GPS data")
else:
    center = [df['lat'].mean(), df['lon'].mean()]
    m = Map(location=center, zoom_start=15)

    heat = []
    for r in df.itertuples():
        w=1
        if isinstance(r.prediction,str):
            p=r.prediction.lower()
            if "hazard" in p: w=3
            elif "wet" in p: w=2
        heat.append([r.lat, r.lon, w])

    HeatMap(heat).add_to(m)
    m.save(OUT)
    print("Saved heatmap:", OUT)
EOF

###############################################
### FRONTEND HTML
###############################################
cat > frontend/index.html << 'EOF'
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Waste Heatmap</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
  <style>#map{height:100vh; margin:0;}</style>
</head>
<body>
<div id="map"></div>
<script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
<script src="https://unpkg.com/leaflet.heat/dist/leaflet-heat.js"></script>
<script>
async function loadCSV(){
  const r = await fetch('/csv');
  const t = await r.text();
  const lines = t.split('\\n').slice(1);
  const heat = [];
  for (const ln of lines){
    if(!ln.trim()) continue;
    const c = ln.split(',');
    const lat=parseFloat(c[1]), lon=parseFloat(c[2]);
    if(!isFinite(lat)||!isFinite(lon)) continue;
    const pred=(c[8]||'').toLowerCase();
    let w=1;
    if(pred.includes('hazard')) w=3;
    else if(pred.includes('wet')) w=2;
    heat.push([lat,lon,w]);
  }
  return heat;
}

(async ()=>{
  const heat=await loadCSV();
  const map=L.map('map').setView([22.5726,88.3639],13);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
  L.heatLayer(heat,{radius:25}).addTo(map);
})();
</script>
</body>
</html>
EOF

###############################################
### SEED CSV
###############################################
echo "timestamp,lat,lon,ultrasonic,temp,tds,compass,image_path,prediction" > data/telemetry.csv

echo "=== ALL DONE ==="
echo "RUN THESE STEPS:"
echo "1) python -m venv venv"
echo "2) source venv/bin/activate   (CLINE LINUX SHELL!)"
echo "3) pip install -r ai/docker/requirements.txt"
echo "4) python ai/api/main.py"
echo "5) python ai/yolo/post_webcam_demo.py"
echo "6) python ai/yolo/generate_heatmap.py"
echo "7) Open frontend/index.html in browser
