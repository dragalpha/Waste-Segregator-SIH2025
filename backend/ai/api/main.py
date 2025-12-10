# ai/api/main.py
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse, FileResponse
import uvicorn, os, csv, datetime, json, subprocess, sys, tempfile, base64
from pathlib import Path
from typing import Optional
import requests

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


def parse_sensor_ppm(value, sensor_name="sensor"):
	"""Parse sensor ppm value to float with validation; return (ppm or None, error or None)."""
	try:
		if value is None or str(value).strip() == "":
			return None, f"{sensor_name} missing"
		val_str = str(value).strip().lower()
		if val_str == "error" or val_str == "nan":
			return None, f"{sensor_name} error state"
		val = float(val_str)
		if val < 0:
			return None, f"{sensor_name} negative value"
		return val, None
	except Exception as e:
		return None, f"{sensor_name} invalid ({e})"

def parse_mq135_ppm(value):
	"""Parse mq135_ppm to float with basic validation; return (ppm or None, error or None)."""
	return parse_sensor_ppm(value, "mq135")

def parse_mq2_ppm(value):
	"""Parse mq2_ppm to float with basic validation; return (ppm or None, error or None)."""
	return parse_sensor_ppm(value, "mq2")


def classify_mq135_gas(ppm: Optional[float]):
	"""Lightweight heuristic for MQ135: returns probable gas name and level band.

	This is an approximation because MQ135 is a generic air-quality sensor without per-gas calibration.
	"""
	if ppm is None:
		return {
			"gas": "unknown",
			"level": "unknown",
			"approx_ppm": None,
			"candidates": ["CO2", "VOC", "NH3", "smoke"],
			"note": "mq135_ppm missing",
		}

	bands = [
		(400, "fresh_air", "good", "Baseline / fresh air"),
		(1000, "co2_or_voc", "elevated", "Likely CO2 buildup or mild VOCs"),
		(2000, "co2_or_voc", "high", "Poor ventilation / strong VOCs"),
		(5000, "co2_or_voc", "very_high", "Potentially hazardous; ventilate immediately"),
	]

	for limit, gas, level, note in bands:
		if ppm <= limit:
			return {
				"gas": gas,
				"level": level,
				"approx_ppm": round(ppm, 2),
				"candidates": ["CO2", "VOC", "NH3", "smoke"],
				"note": note,
			}

	# Above 5000 ppm is treated as danger-zone for CO2/VOC exposure
	return {
		"gas": "co2_or_voc",
		"level": "dangerous",
		"approx_ppm": round(ppm, 2),
		"candidates": ["CO2", "VOC", "NH3", "smoke"],
		"note": "Above 5000 ppm (danger zone)",
	}


def parse_ultrasonic_distance(value, sensor_name: str = "JSN-SR04T") -> tuple:
	"""Parse and validate ultrasonic distance sensor reading.
	
	JSN-SR04T specs:
	- Operating range: 25cm to 450cm
	- Waterproof ultrasonic sensor
	
	Returns: (distance_cm, error_message)
	  distance_cm: float if valid, negative error code if invalid
	  error_message: str describing error, empty if valid
	"""
	if value is None:
		return (-1, f"{sensor_name}_MISSING_VALUE")
	
	try:
		distance = float(value)
	except (ValueError, TypeError):
		return (-2, f"{sensor_name}_INVALID_FORMAT")
	
	# Range validation: 25cm to 450cm
	if distance < 0:
		return (-3, f"{sensor_name}_NEGATIVE_DISTANCE")
	
	if distance < 25:
		return (-4, f"{sensor_name}_BELOW_MIN_RANGE")
	
	if distance > 450:
		return (-5, f"{sensor_name}_ABOVE_MAX_RANGE")
	
	return (distance, "")


def classify_ultrasonic_detection(distance_cm: Optional[float]):
	"""Classify object detection based on JSN-SR04T ultrasonic sensor distance.
	
	Focuses purely on object detection within sensor range (25-450cm).
	Returns detection status, proximity classification, and distance information.
	"""
	if distance_cm is None:
		return {
			"status": "error",
			"distance_cm": None,
			"range": "unknown",
			"object_detected": False,
			"proximity": "unknown",
			"note": "ultrasonic_cm missing",
		}
	
	# Error code handling
	if distance_cm < 0:
		error_messages = {
			-1: "Sensor value missing",
			-2: "Invalid format",
			-3: "Negative distance reading",
			-4: "Below minimum range (25cm)",
			-5: "Above maximum range (450cm) or no object detected",
		}
		return {
			"status": "error",
			"distance_cm": distance_cm,
			"range": "error",
			"object_detected": False,
			"proximity": "error",
			"note": error_messages.get(int(distance_cm), "Unknown sensor error"),
		}
	
	# Object detection classification based on distance
	# JSN-SR04T effective range: 25cm to 450cm
	
	object_detected = True
	
	# Proximity classification (how close the object is)
	if distance_cm <= 50:
		range_class = "very_close"
		proximity = "immediate"
		note = "Object detected at very close range"
	elif distance_cm <= 100:
		range_class = "close"
		proximity = "near"
		note = "Object detected at close range"
	elif distance_cm <= 200:
		range_class = "medium"
		proximity = "moderate"
		note = "Object detected at medium distance"
	elif distance_cm <= 350:
		range_class = "far"
		proximity = "distant"
		note = "Object detected at far range"
	else:  # 350 < distance <= 450
		range_class = "very_far"
		proximity = "very_distant"
		note = "Object detected at maximum range"
	
	return {
		"status": "ok",
		"distance_cm": round(distance_cm, 2),
		"range": range_class,
		"object_detected": object_detected,
		"proximity": proximity,
		"note": note,
	}


def classify_mq2_gas(ppm: Optional[float]):
	"""MQ2 detects flammable gases: LPG, propane, methane, hydrogen, alcohol, smoke.

	Returns gas name, detection level, and approximate amount.
	"""
	if ppm is None:
		return {
			"gas": "unknown",
			"level": "unknown",
			"approx_ppm": None,
			"candidates": ["LPG", "propane", "methane", "H2", "alcohol", "smoke"],
			"detection": "none",
			"note": "mq2_ppm missing",
		}

	# MQ2 concentration bands (ppm thresholds are approximate - calibration dependent)
	bands = [
		(200, "clean_air", "none", "good", "No flammable gas detected"),
		(300, "lpg_or_smoke", "trace", "acceptable", "Trace amounts of combustible gases"),
		(800, "lpg_propane_methane", "low", "elevated", "Low concentration of flammable gas"),
		(2000, "lpg_propane_methane", "medium", "high", "Moderate flammable gas - check for leaks"),
		(5000, "flammable_gas_hazard", "high", "very_high", "High flammable gas - potential fire/explosion risk"),
	]

	for limit, gas, detection, level, note in bands:
		if ppm <= limit:
			return {
				"gas": gas,
				"level": level,
				"approx_ppm": round(ppm, 2),
				"candidates": ["LPG", "propane", "methane", "H2", "alcohol", "smoke"],
				"detection": detection,
				"note": note,
			}

	# Above 5000 ppm
	return {
		"gas": "dangerous_flammable",
		"level": "dangerous",
		"approx_ppm": round(ppm, 2),
		"candidates": ["LPG", "propane", "methane", "H2", "alcohol", "smoke"],
		"detection": "extreme",
		"note": "DANGER: Extreme flammable gas (>5000 ppm) - evacuate and ventilate!",
	}


def analyze_combined_gas(mq2_ppm: Optional[float], mq135_ppm: Optional[float]):
	"""Cross-correlate MQ2 and MQ135 readings to identify likely gas sources.

	MQ2: detects flammable gases (LPG, propane, methane, H2, alcohol, smoke)
	MQ135: detects air quality (CO2, VOC, NH3, smoke, benzene)

	Returns combined analysis with probable gas identification.
	"""
	mq2_result = classify_mq2_gas(mq2_ppm)
	mq135_result = classify_mq135_gas(mq135_ppm)

	# Combined analysis logic
	combined = {
		"mq2": mq2_result,
		"mq135": mq135_result,
		"probable_gas": "unknown",
		"confidence": "low",
		"hazard_level": "safe",
		"recommendation": "",
	}

	# Both sensors missing
	if mq2_ppm is None and mq135_ppm is None:
		combined["probable_gas"] = "no_data"
		combined["recommendation"] = "No sensor data available"
		return combined

	# Correlation patterns
	mq2_elevated = mq2_ppm and mq2_ppm > 300
	mq135_elevated = mq135_ppm and mq135_ppm > 1000
	mq2_high = mq2_ppm and mq2_ppm > 2000
	mq135_high = mq135_ppm and mq135_ppm > 2000

	# Pattern 1: Both sensors show high readings - likely smoke/fire
	if mq2_high and mq135_high:
		combined["probable_gas"] = "smoke_or_fire"
		combined["confidence"] = "high"
		combined["hazard_level"] = "critical"
		combined["recommendation"] = "URGENT: Smoke or fire detected - evacuate immediately!"

	# Pattern 2: MQ2 high, MQ135 moderate - flammable gas leak
	elif mq2_high and not mq135_high:
		combined["probable_gas"] = "lpg_or_propane_leak"
		combined["confidence"] = "high"
		combined["hazard_level"] = "high"
		combined["recommendation"] = "Check for gas leaks - ventilate area immediately"

	# Pattern 3: MQ135 high, MQ2 low - organic vapors/CO2
	elif mq135_high and not mq2_elevated:
		combined["probable_gas"] = "organic_vapor_or_co2"
		combined["confidence"] = "medium"
		combined["hazard_level"] = "medium"
		combined["recommendation"] = "Poor air quality - improve ventilation"

	# Pattern 4: Both elevated but not critical
	elif mq2_elevated and mq135_elevated:
		combined["probable_gas"] = "mixed_combustion_products"
		combined["confidence"] = "medium"
		combined["hazard_level"] = "medium"
		combined["recommendation"] = "Multiple gas sources detected - monitor closely"

	# Pattern 5: Only MQ2 trace detection
	elif mq2_elevated and not mq135_elevated:
		combined["probable_gas"] = "trace_flammable"
		combined["confidence"] = "low"
		combined["hazard_level"] = "low"
		combined["recommendation"] = "Trace flammable gas - continue monitoring"

	# Pattern 6: Only MQ135 elevated
	elif mq135_elevated and not mq2_elevated:
		combined["probable_gas"] = "air_quality_issue"
		combined["confidence"] = "medium"
		combined["hazard_level"] = "low"
		combined["recommendation"] = "Air quality degraded - improve ventilation"

	# Pattern 7: Both normal
	else:
		combined["probable_gas"] = "clean_air"
		combined["confidence"] = "high"
		combined["hazard_level"] = "safe"
		combined["recommendation"] = "Air quality normal"

	return combined

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

	# Parse both MQ2 and MQ135
	mq2_raw = g("mq2_ppm")
	mq135_raw = g("mq135_ppm")
	mq2_ppm, mq2_error = parse_mq2_ppm(mq2_raw)
	mq135_ppm, mq135_error = parse_mq135_ppm(mq135_raw)
	
	# Parse ultrasonic sensor
	ultrasonic_raw = g("ultrasonic_cm") or g("ultrasonic")
	ultrasonic_cm, ultrasonic_error = parse_ultrasonic_distance(ultrasonic_raw)
	
	# Get combined gas analysis
	gas_analysis = analyze_combined_gas(mq2_ppm, mq135_ppm)
	if mq2_error:
		gas_analysis["mq2"]["error"] = mq2_error
	if mq135_error:
		gas_analysis["mq135"]["error"] = mq135_error
	
	# Get ultrasonic detection analysis
	ultrasonic_analysis = classify_ultrasonic_detection(ultrasonic_cm)
	if ultrasonic_error:
		ultrasonic_analysis["error"] = ultrasonic_error

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
	return JSONResponse({"status":"ok", "gas_analysis": gas_analysis, "ultrasonic_analysis": ultrasonic_analysis})

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

	# Parse both MQ2 and MQ135 from sensors
	mq2_ppm, mq2_error = parse_mq2_ppm(s.get("mq2_ppm"))
	mq135_ppm, mq135_error = parse_mq135_ppm(s.get("mq135_ppm"))
	
	# Parse ultrasonic sensor
	ultrasonic_cm, ultrasonic_error = parse_ultrasonic_distance(s.get("ultrasonic_cm") or s.get("ultrasonic"))
	
	# Get combined gas analysis
	gas_analysis = analyze_combined_gas(mq2_ppm, mq135_ppm)
	if mq2_error:
		gas_analysis["mq2"]["error"] = mq2_error
	if mq135_error:
		gas_analysis["mq135"]["error"] = mq135_error
	
	# Get ultrasonic detection analysis
	ultrasonic_analysis = classify_ultrasonic_detection(ultrasonic_cm)
	if ultrasonic_error:
		ultrasonic_analysis["error"] = ultrasonic_error

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
		"gas_analysis": gas_analysis,
		"ultrasonic_analysis": ultrasonic_analysis,
	})


@app.post("/image_base64")
def image_base64(payload: dict):
	"""Accept base64 image string for ESP32-CAM posts."""
	image_b64 = payload.get("imageBase64")
	sensors = payload.get("sensors", {})
	if not image_b64:
		return JSONResponse({"status": "error", "error": "imageBase64 required"}, status_code=400)
	try:
		raw = base64.b64decode(image_b64.split(",")[-1])
		with tempfile.NamedTemporaryFile(delete=False, suffix=".jpg") as tmp:
			tmp.write(raw)
			tmp_path = Path(tmp.name)
	except Exception as e:
		return JSONResponse({"status": "error", "error": f"decode failed: {e}"}, status_code=500)

	return image_url({"imageUrl": None, "sensors": sensors, "tmp_path": str(tmp_path)})


@app.post("/image_url")
def image_url(payload: dict):
	image_url = payload.get("imageUrl")
	tmp_override = payload.get("tmp_path")
	sensors = payload.get("sensors", {})
	if not image_url and not tmp_override:
		return JSONResponse({"status": "error", "error": "imageUrl required"}, status_code=400)
	try:
		if tmp_override:
			tmp_path = Path(tmp_override)
		else:
			resp = requests.get(image_url, timeout=10)
			resp.raise_for_status()
			with tempfile.NamedTemporaryFile(delete=False, suffix=".jpg") as tmp:
				tmp.write(resp.content)
				tmp_path = Path(tmp.name)
	except Exception as e:
		return JSONResponse({"status": "error", "error": f"download failed: {e}"}, status_code=500)

	model = get_model_path()
	pred = "-"
	raw_label = "-"
	try:
		proc = subprocess.run([
			sys.executable,
			str(ROOT / "yolo" / "inference_yolov8.py"),
			"--image",
			str(tmp_path),
			"--model",
			model,
		], capture_output=True, text=True, timeout=30)
		pred = proc.stdout.strip() or "-"
	except Exception:
		pred = "-"
	if pred and pred != "-":
		raw_label = pred.split(":")[0]

	try:
		from .yolo.utils_yolo import classify_label as classify_local
	except Exception:
		from ..yolo.utils_yolo import classify_label as classify_local
	waste_label, waste_type = classify_local(raw_label) if raw_label else ("unknown", "")
	waste_state, hazard, hazard_type = classify_waste_and_hazard_from_label(raw_label)

	# write telemetry row
	ts = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
	def sget(k, default=""):
		return sensors.get(k, default)

	# Parse both MQ2 and MQ135 from sensors
	mq2_ppm, mq2_error = parse_mq2_ppm(sensors.get("mq2_ppm"))
	mq135_ppm, mq135_error = parse_mq135_ppm(sensors.get("mq135_ppm"))
	
	# Parse ultrasonic sensor
	ultrasonic_cm, ultrasonic_error = parse_ultrasonic_distance(sensors.get("ultrasonic_cm") or sensors.get("ultrasonic"))
	
	# Get combined gas analysis
	gas_analysis = analyze_combined_gas(mq2_ppm, mq135_ppm)
	if mq2_error:
		gas_analysis["mq2"]["error"] = mq2_error
	if mq135_error:
		gas_analysis["mq135"]["error"] = mq135_error
	
	# Get ultrasonic detection analysis
	ultrasonic_analysis = classify_ultrasonic_detection(ultrasonic_cm)
	if ultrasonic_error:
		ultrasonic_analysis["error"] = ultrasonic_error

	row = [
		ts, sget("device_id"), sget("boat_id"), sget("lat"), sget("lon"),
		sget("heading_deg"), sget("mq135_ppm"), sget("mq2_ppm"), sget("soil_dry_belt_pct"),
		sget("soil_wet_belt_pct"), sget("loadcell_grams"), sget("tds_ppm"), sget("ultrasonic_cm"),
		sget("proximity_inductive"), str(tmp_path), pred, waste_label, waste_type,
		sget("collection_event"), sget("collection_bin_id"), sget("battery_volt"), sget("rssi"),
	]
	append_row(row)
	try:
		os.remove(tmp_path)
	except Exception:
		pass
	return JSONResponse({
		"status": "ok",
		"path": str(tmp_path),
		"prediction": pred,
		"yolo_raw": pred,
		"waste_category": waste_label,
		"waste_subtype": waste_type,
		"waste_state": waste_state,
		"hazard": int(hazard),
		"hazard_type": hazard_type,
		"gas_analysis": gas_analysis,
		"ultrasonic_analysis": ultrasonic_analysis,
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
