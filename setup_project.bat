@echo off
setlocal enabledelayedexpansion

echo === Creating directories ===
mkdir backend config controllers models routes utils mqtt websocket
mkdir ai ai\docker ai\yolo ai\cnn ai\api ai\venv
mkdir data data\images frontend

echo === Removing old YOLOv5 files ===
del /Q taco-yolov5s.pt 2>nul

echo === Copying uploaded YOLOv8n model ===
copy /Y \\mnt\\data\\yolov8n.pt ai\yolo\yolov8n.pt >nul 2>&1

echo === Copying SIH PDF ===
copy /Y \\mnt\\data\\SIH2025-IDEA-Presentation25014.pdf ai\ >nul 2>&1

echo === Creating FastAPI backend ===
(
echo from fastapi import FastAPI, File, Form, UploadFile
echo from fastapi.responses import JSONResponse, FileResponse
echo import uvicorn, os, csv, datetime, json, subprocess
echo from pathlib import Path
echo.
echo ROOT = Path(__file__).resolve().parents[2]
echo DATA_DIR = ROOT / "data"
echo IMG_DIR = DATA_DIR / "images"
echo CSV_FILE = DATA_DIR / "telemetry.csv"
echo.
echo app = FastAPI()
echo.
echo DATA_DIR.mkdir(exist_ok=True)
echo IMG_DIR.mkdir(exist_ok=True)
echo.
echo if not CSV_FILE.exists():
echo     with open(CSV_FILE, "w", newline="") as f:
echo         writer = csv.writer(f)
echo         writer.writerow(["timestamp","lat","lon","ultrasonic","temp","tds","compass","image_path","prediction"])
echo.
echo def append_row(r):
echo     with open(CSV_FILE, "a", newline="") as f:
echo         csv.writer(f).writerow(r)
echo.
echo def get_model():
echo     local = ROOT/"yolo"/"yolov8n.pt"
echo     if local.exists(): return str(local)
echo     fallback = Path("/mnt/data/yolov8n.pt")
echo     if fallback.exists(): return str(fallback)
echo     return "yolov8n.pt"
echo.
echo @app.post("/telemetry")
echo async def telemetry(json_data: dict):
echo     ts = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
echo     row = [
echo         ts,
echo         json_data.get("lat","-"),
echo         json_data.get("lon","-"),
echo         json_data.get("ultrasonic",""),
echo         json_data.get("temp",""),
echo         json_data.get("tds",""),
echo         json_data.get("compass",""),
echo         "-",
echo         "-"
echo     ]
echo     append_row(row)
echo     return {"status":"ok"}
echo.
echo @app.post("/image")
echo async def image(
echo     lat: str = Form(None),
echo     lon: str = Form(None),
echo     sensors: str = Form("{}"),
echo     image: UploadFile = File(...)
echo ):
echo     ts = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
echo     path = IMG_DIR / f"{ts}.jpg"
echo.
echo     # Save image
echo     with open(path, "wb") as f:
echo         f.write(await image.read())
echo.
echo     # Parse sensors
echo     try:
echo         s = json.loads(sensors)
echo     except:
echo         s = {}
echo.
echo     # Run inference
echo     try:
echo         proc = subprocess.run([
echo             "python",
echo             str(ROOT/"yolo"/"inference_yolov8.py"),
echo             "--image", str(path),
echo             "--model", get_model()
echo         ], capture_output=True, text=True)
echo         pred = proc.stdout.strip() or "-"
echo     except:
echo         pred = "-"
echo.
echo     # Append CSV row
echo     row = [
echo         datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S"),
echo         lat or "-", lon or "-",
echo         s.get("ultrasonic",""),
echo         s.get("temp",""),
echo         s.get("tds",""),
echo         s.get("compass",""),
echo         str(path),
echo         pred
echo     ]
echo     append_row(row)
echo.
echo     return {"status":"ok","path":str(path),"prediction":pred}
echo.
echo @app.get("/csv")
echo def csv_file():
echo     return FileResponse(str(CSV_FILE))
echo.
echo @app.get("/heatmap")
echo def heatmap_file():
echo     p = DATA_DIR/"heatmap.html"
echo     if p.exists(): return FileResponse(str(p))
echo     return {"error":"heatmap not generated yet"}
echo.
echo if __name__ == "__main__":
echo     uvicorn.run(app,host="0.0.0.0",port=8000)
) > ai\api\main.py

echo === ALL DONE ===
echo RUN THESE STEPS:
echo 1) python -m venv venv
echo 2) venv\Scripts\activate
echo 3) pip install -r ai\docker\requirements.txt
echo 4) python ai\api\main.py
echo 5) python ai\yolo\post_webcam_demo.py
echo 6) python ai\yolo\generate_heatmap.py
echo 7) Open frontend\index.html in browser
