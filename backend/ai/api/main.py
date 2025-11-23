# ai/api/main.py
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse, FileResponse
import uvicorn, os, csv, datetime, json, subprocess
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]  # ai/
DATA_DIR = ROOT.parent / "data"
IMG_DIR = DATA_DIR / "images"
CSV_FILE = DATA_DIR / "telemetry.csv"
DETECTED_FILE = ROOT / "yolo" / "detected.txt"  # post_webcam_demo writes this

# ensure directories
DATA_DIR.mkdir(parents=True, exist_ok=True)
IMG_DIR.mkdir(parents=True, exist_ok=True)

# Canonical CSV header required by spec
CANONICAL_HEADER = [
	"timestamp_utc",
	"device_id",
	"boat_id",
	"lat",
	"lon",
	"heading_deg",
	"mq135_ppm",
	"mq2_ppm",
	"soil_dry_belt_pct",
	"soil_wet_belt_pct",
	"loadcell_grams",
	"tds_ppm",
	"ultrasonic_cm",
	"proximity_inductive",
	"image_path",
	"yolo_raw",
	"waste_category",
	"waste_subtype",
	"collection_event",
	"collection_bin_id",
	"battery_volt",
	"rssi",
]

if not CSV_FILE.exists():
	with open(CSV_FILE, "w", newline="") as f:
		writer = csv.writer(f)
		writer.writerow(CANONICAL_HEADER)

app = FastAPI(title="Waste Segregation API")

def append_row(row):
	# row is expected to be a list matching CANONICAL_HEADER length
	with open(CSV_FILE, "a", newline="") as f:
		csv.writer(f).writerow(row)


def classify_waste_and_hazard_from_label(label: str, conf: float = 0.0, conf_threshold: float = 0.25):
	"""Heuristic mapping from a label string (e.g. 'plastic') to waste_state and hazard.

	Returns: (waste_state, hazard:int, hazard_type)
	"""
	if not label:
		return "unknown", 0, ""
	ll = label.lower()
	keywords_wet = ('wet', 'water', 'mud', 'liquid', 'food', 'banana', 'apple', 'vegetable', 'organic', 'peel')
	keywords_dry = ('dry', 'paper', 'cardboard', 'plastic', 'metal', 'glass', 'cloth', 'fabric')
	keywords_hazard = ('battery', 'chemical', 'glass', 'sharp', 'hazard', 'flammable', 'rust', 'acid', 'alkali')

	waste_state = 'unknown'
	hazard = 0
	hazard_type = ''

	if any(k in ll for k in keywords_wet):
		waste_state = 'wet'
	elif any(k in ll for k in keywords_dry):
		waste_state = 'dry'

	for hk in keywords_hazard:
		if hk in ll:
			hazard = 1
			hazard_type = label
			break

	return waste_state, hazard, hazard_type

def get_model_path():
	candidates = [
		ROOT / "yolo" / "yolov8n.pt",
		Path("/mnt/data/yolov8n.pt")
	]
	for c in candidates:
		if c.exists():
			return str(c)
	# if not found, return default name (ultralytics may download)
	return "yolov8n.pt"

@app.post("/telemetry")
async def telemetry(json_payload: dict):
	"""
	ESP8266 posts sensor data & optionally 'detected' field.
	Ex:
	{
	  "lat":"22.57","lon":"88.36","ultrasonic":"45","temp":"27.2","tds":"390","compass":"120","detected":"plastic:0.78"
	}
	"""
	# Accept the canonical telemetry JSON fields (any missing keys will be blank)
	ts = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
	# determine yolo/waste mapping if 'detected' or 'yolo_raw' present
	raw = json_payload.get("detected") or json_payload.get("yolo_raw") or "-"
	# accept waste_state/hazard fields from device if provided
	waste_state_in = json_payload.get("waste_state")
	hazard_in = json_payload.get("hazard")
	waste_category = "-"
	waste_subtype = "-"
	if isinstance(raw, str) and raw != "-":
		try:
			from ..yolo.utils_yolo import classify_label
			cat, subtype = classify_label(raw.split(":")[0])
			waste_category = cat if cat != "unknown" else raw
			waste_subtype = subtype
		except Exception:
			waste_category = raw

		# if waste_state/hazard were provided, embed them into waste_subtype for storage
		if waste_state_in:
			# prefer storing in waste_subtype if missing
			if not waste_subtype or waste_subtype == "-":
				waste_subtype = str(waste_state_in)
			else:
				waste_subtype = f"{waste_subtype}|waste:{waste_state_in}"
		if hazard_in is not None:
			# coerce to int/str
			h = int(hazard_in) if str(hazard_in).isdigit() else (1 if str(hazard_in).lower() in ('1','true','yes') else 0)
			# append to waste_subtype
			if not waste_subtype or waste_subtype == "-":
				waste_subtype = f"hazard:{h}"
			else:
				waste_subtype = f"{waste_subtype}|hazard:{h}"

	# build row in canonical order
	def g(k):
		return json_payload.get(k, "")

	row = [
		ts,
		g("device_id"),
		g("boat_id"),
		g("lat"),
		g("lon"),
		g("heading_deg"),
		g("mq135_ppm"),
		g("mq2_ppm"),
		g("soil_dry_belt_pct"),
		g("soil_wet_belt_pct"),
		g("loadcell_grams"),
		g("tds_ppm"),
		g("ultrasonic_cm"),
		g("proximity_inductive"),
		g("image_path"),
		raw,
		waste_category,
		waste_subtype,
		g("collection_event"),
		g("collection_bin_id"),
		g("battery_volt"),
		g("rssi"),
	]

	append_row(row)
	return JSONResponse({"status":"ok"})

@app.post("/image")
async def image(
	lat: Optional[str] = Form(None),
	lon: Optional[str] = Form(None),
	sensors: Optional[str] = Form("{}"),
	image: UploadFile = File(...)
):
	# Save image
	ts_fname = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
	out_path = IMG_DIR / f"{ts_fname}.jpg"
	contents = await image.read()
	with open(out_path, "wb") as fw:
		fw.write(contents)

	# Run inference (call the local CLI-style script)
	model = get_model_path()
	pred = "-"
	try:
		proc = subprocess.run([
			sys.executable,
			str(ROOT / "yolo" / "inference_yolov8.py"),
			"--image",
			str(out_path),
			"--model",
			model,
		], capture_output=True, text=True, timeout=30)
		pred = proc.stdout.strip() or "-"
	except Exception:
		pred = "-"

	# Map pred to category & subtype
	waste_label = "-"
	waste_type = "-"
	if pred and pred != "-":
		parts = pred.split(":")
		raw_label = parts[0]
		try:
			from .yolo.utils_yolo import classify_label as classify_local
		except Exception:
			from ..yolo.utils_yolo import classify_label as classify_local
		cat, subtype = classify_local(raw_label)
		waste_label = cat if cat != "unknown" else raw_label
		waste_type = subtype

	# compute waste_state and hazard heuristics from label
	waste_state, hazard, hazard_type = classify_waste_and_hazard_from_label(raw_label if pred and pred != "-" else waste_label)

	# parse sensors (if any)
	try:
		s = json.loads(sensors)
	except:
		s = {}

	# write to CSV using canonical order
	ts = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
	def sget(k, default=""):
		return s.get(k, default)

	row = [
		ts,
		sget("device_id"),
		sget("boat_id"),
		lat or sget("lat"),
		lon or sget("lon"),
		sget("heading_deg"),
		sget("mq135_ppm"),
		sget("mq2_ppm"),
		sget("soil_dry_belt_pct"),
		sget("soil_wet_belt_pct"),
		sget("loadcell_grams"),
		sget("tds_ppm"),
		sget("ultrasonic_cm"),
		sget("proximity_inductive"),
		str(out_path),
		pred,
		waste_label,
		waste_type,
		sget("collection_event"),
		sget("collection_bin_id"),
		sget("battery_volt"),
		sget("rssi"),
	]
	append_row(row)

	# also write detected.txt for ESP8266 to read (raw_label:confidence)
	# write JSON payload so ESP/ESP32 can parse waste_state/hazard easily
	raw_for_esp = pred if pred and pred != "-" else waste_label
	detected_payload = {
		"prediction": raw_for_esp,
		"waste_state": waste_state,
		"hazard": int(hazard),
		"hazard_type": hazard_type,
	}
	try:
		with open(DETECTED_FILE, "w", encoding="utf-8") as f:
			f.write(json.dumps(detected_payload))
	except Exception:
		pass

	# Return structured info so clients can use category/subtype directly
	return JSONResponse({
		"status":"ok",
		"path": str(out_path),
		"prediction": pred,
		"yolo_raw": pred,
		"waste_category": waste_label,
		"waste_subtype": waste_type,
		"waste_state": waste_state,
		"hazard": int(hazard),
		"hazard_type": hazard_type,
	})

@app.get("/detected")
def get_detected():
	# returns last detected string for ESP to poll
	try:
		if DETECTED_FILE.exists():
			s = DETECTED_FILE.read_text().strip()
			# try to parse JSON payload written by /image or realtime script
			try:
				obj = json.loads(s)
				return obj
			except Exception:
				return {"prediction": s}
	except Exception:
		pass
	return {"prediction": "-"}


@app.post("/set_detected")
def set_detected(payload: dict):
	"""Dev-only: set the backend detected value (writes `detected.txt`).
	POST JSON: {"value":"plastic:0.88"}
	"""
	try:
		v = payload.get("value", "-")
		os.makedirs(os.path.dirname(DETECTED_FILE), exist_ok=True)
		with open(DETECTED_FILE, "w") as f:
			f.write(str(v))
		return JSONResponse({"status":"ok","written": v})
	except Exception as e:
		return JSONResponse({"status":"error","error": str(e)}, status_code=500)

@app.get("/csv")
def get_csv():
	if CSV_FILE.exists():
		return FileResponse(str(CSV_FILE))
	return JSONResponse({"error":"no csv"}, status_code=404)

@app.get("/heatmap")
def heatmap():
	out = DATA_DIR / "heatmap.html"
	if out.exists():
		return FileResponse(str(out))
	return JSONResponse({"error":"heatmap not generated yet"}, status_code=404)


@app.get("/telemetry_get")
def telemetry_get(
	device_id: Optional[str] = None,
	boat_id: Optional[str] = None,
	lat: Optional[str] = None,
	lon: Optional[str] = None,
	yolo_raw: Optional[str] = None,
	battery_volt: Optional[str] = None,
	rssi: Optional[str] = None,
):
	"""Dev helper: accept telemetry via GET query and append to CSV.
	Useful when POST from devices is unreliable. Example:
	/telemetry_get?device_id=esp1&lat=22.5&lon=88.3&yolo_raw=plastic:0.8
	"""
	ts = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
	row = [
		ts,
		device_id or "",
		boat_id or "",
		lat or "",
		lon or "",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		yolo_raw or "",
		"",
		"",
		"",
		"",
		battery_volt or "",
		rssi or "",
	]
	append_row(row)
	return JSONResponse({"status":"ok", "written": row})

if __name__ == "__main__":
	# prefer running the app object directly
	uvicorn.run(app, host="0.0.0.0", port=8000, reload=False)
