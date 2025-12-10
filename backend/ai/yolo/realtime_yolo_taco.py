#!/usr/bin/env python3
"""Realtime YOLO webcam demo

Features:
- Uses `ultralytics` YOLOv8 if installed for local inference.
- If ultralytics is not available it will POST frames to a configured backend `/image` endpoint.
- Annotates frames with boxes, labels and confidences and shows a preview window.
- Writes a small `detected.txt` file with the top detection (`label:conf`) so devices can poll it.

Usage examples:
  python realtime_yolo_taco.py --model backend/ai/yolo/yolov8n.pt --write-detected backend/ai/yolo/detected.txt
  python realtime_yolo_taco.py --server-url http://192.168.0.100:8000/image --write-detected backend/ai/yolo/detected.txt
"""
import argparse
import os
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import cv2

mjpeg_server = None
latest_jpeg = None


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/stream':
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Age', 0)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                if latest_jpeg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(b'Content-Length: ' + str(len(latest_jpeg)).encode() + b'\r\n\r\n')
                self.wfile.write(latest_jpeg)
                self.wfile.write(b'\r\n')
                time.sleep(0.04)
        except Exception:
            # client disconnected
            pass


def start_mjpeg_server(host='127.0.0.1', port=8090):
    global mjpeg_server
    if mjpeg_server:
        return mjpeg_server
    try:
        mjpeg_server = HTTPServer((host, port), MJPEGHandler)
    except OSError as e:
        print(f"MJPEG server failed to start on {host}:{port}: {e}")
        mjpeg_server = None
        return None

    thread = threading.Thread(target=mjpeg_server.serve_forever, daemon=True)
    thread.start()
    print(f"MJPEG stream at http://{host}:{port}/stream")
    return mjpeg_server

# keyword sets reused across classification helpers
KEYWORDS_WET = (
    # Generic wet/organic
    'wet', 'water', 'mud', 'liquid', 'food', 'banana', 'apple', 'fruit', 'vegetable',
    'organic', 'organic_matter', 'food_waste', 'peel', 'leaf', 'plant_debris',
    # Dataset-specific wet labels
    'food_waste', 'fruit', 'vegetable', 'organic_matter', 'leaf', 'plant_debris'
)
KEYWORDS_DRY = (
    # Generic dry/recyclable
    'dry', 'paper', 'cardboard', 'plastic', 'metal', 'glass', 'wood', 'carton', 'tissue',
    'cloth', 'fabric', 'wrapper', 'bag', 'box', 'cup', 'straw', 'bottle', 'can', 'jar',
    'stick', 'leaflet', 'newspaper',
    # Dataset-specific dry labels
    'plastic_bottle', 'plastic_bag', 'plastic_wrapper', 'plastic_cup', 'paper', 'cardboard',
    'newspaper', 'metal_can', 'metal_scrap', 'glass_bottle', 'glass_jar', 'textile', 'cloth',
    'floating_plastic', 'floating_wood', 'floating_debris'
    # COCO common disposables/gear
    'fork', 'knife', 'spoon', 'bowl', 'chair', 'couch', 'sofa', 'bench', 'book',
    'handbag', 'backpack', 'suitcase', 'umbrella', 'tv', 'laptop', 'mouse', 'keyboard',
    'remote', 'cell phone', 'toothbrush', 'hair drier'
)
KEYWORDS_HAZARD = (
    'battery', 'chemical', 'glass', 'sharp', 'hazard', 'flammable'
)

def try_import_ultralytics():
    try:
        from ultralytics import YOLO

        return YOLO
    except Exception:
        return None


def post_frame_to_server(server_url, frame, timeout=10):
    import requests
    import io

    # encode as JPEG
    ret, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ret:
        return None
    files = {"image": ("frame.jpg", buf.tobytes(), "image/jpeg")}
    try:
        r = requests.post(server_url, files=files, timeout=timeout)
        if r.status_code == 200:
            return r.json()
        else:
            print("Server returned", r.status_code)
            return None
    except Exception as e:
        print("POST error:", e)
        return None


def write_detected_file(path: Path, text: str):
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
    except Exception as e:
        print("Failed to write detected file:", e)


def annotate_frame(frame, boxes, labels, confs, classes_names):
    # boxes: list of [x1,y1,x2,y2]
    for bb, lab, c in zip(boxes, labels, confs):
        x1, y1, x2, y2 = map(int, bb)
        label = f"{lab} {c:.2f}"
        color = (0, 200, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        # put label background
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
        cv2.putText(frame, label, (x1 + 3, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return frame


def classify_waste_and_hazard(labels, confs, conf_threshold=0.25):
    """Heuristic mapping from detected labels to waste_state and hazard.

    Rules (best-effort):
    - If any label indicates 'wet' or 'organic' (e.g. 'food', 'banana', 'vegetable') -> waste_state='wet'
    - If any label indicates 'dry' (e.g. 'paper', 'cardboard', 'plastic', 'metal') -> waste_state='dry'
    - Hazard types: labels like 'battery','chemical','glass','sharp' -> hazard=1 and hazard_type=label
    - Only consider labels with confidence >= conf_threshold
    Returns: (waste_state, hazard:int, hazard_type)
    """
    waste_state = 'unknown'
    hazard = 0
    hazard_type = ''

    for lab, c in zip(labels, confs):
        if c < conf_threshold:
            continue
        ll = lab.lower()
        if any(k in ll for k in KEYWORDS_WET):
            waste_state = 'wet'
        if any(k in ll for k in KEYWORDS_DRY) and waste_state == 'unknown':
            waste_state = 'dry'
        for hk in KEYWORDS_HAZARD:
            if hk in ll:
                hazard = 1
                hazard_type = lab
                break
        if hazard:
            break

    return waste_state, hazard, hazard_type


def compute_wet_dry_percentages(labels, confs, conf_threshold=0.25):
    """Return wet/dry percentages based on detected labels above the threshold."""
    wet = 0
    dry = 0
    for lab, c in zip(labels, confs):
        if c < conf_threshold:
            continue
        ll = lab.lower()
        if any(k in ll for k in KEYWORDS_WET):
            wet += 1
        elif any(k in ll for k in KEYWORDS_DRY):
            dry += 1

    total = wet + dry
    if total == 0:
        return 0.0, 0.0

    wet_pct = (wet / total) * 100.0
    dry_pct = (dry / total) * 100.0
    return wet_pct, dry_pct


def open_capture(device, width, height):
    """Try to open a webcam with sensible fallbacks (DirectShow for Windows).

    We also verify we can grab a frame; if an API opens but cannot read (common
    with MSMF on some cameras), we release and try the next option.
    """
    attempts = []
    # Prefer DirectShow first on Windows, then default, then MSMF, then any.
    attempts.append((device, cv2.CAP_DSHOW))
    attempts.append((device, None))
    attempts.append((device, cv2.CAP_MSMF))
    attempts.append((device, cv2.CAP_ANY))

    for dev, api_pref in attempts:
        cap = None
        try:
            cap = cv2.VideoCapture(dev, api_pref) if api_pref is not None else cv2.VideoCapture(dev)
        except Exception:
            cap = cv2.VideoCapture(dev)
        if not cap or not cap.isOpened():
            if cap:
                cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Verify we can actually read a frame; some APIs open but fail to grab.
        ret, _ = cap.read()
        if ret:
            return cap

        cap.release()

    return None


def run_local_model(args):
    YOLO = try_import_ultralytics()
    if YOLO is None:
        print("ultralytics not installed — local inference unavailable")
        return False

    model_path = args.model or "yolov8n.pt"
    print("Loading model:", model_path)
    model = YOLO(model_path)

    cap = open_capture(args.camera_device, args.width, args.height)
    if cap is None or not cap.isOpened():
        print("Cannot open webcam (tried default and DirectShow)")
        return False

    # start MJPEG server for browser overlay
    start_mjpeg_server(host='127.0.0.1', port=args.mjpeg_port)

    print("Press 'q' to quit. Running local inference.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        # run inference
        try:
            results = model(frame, conf=args.conf, device=args.device)
        except Exception as e:
            print("Model inference error:", e)
            break

        boxes = []
        labels = []
        confs = []
        ignore_set = {s.strip().lower() for s in args.ignore_classes.split(',') if s.strip()}

        for r in results:
            if not hasattr(r, 'boxes'):
                continue
            b = r.boxes
            # ultralytics: b.xyxy, b.cls, b.conf
            xyxy = getattr(b, 'xyxy', None)
            cls_idx = getattr(b, 'cls', None)
            confidences = getattr(b, 'conf', None)
            if xyxy is None:
                continue
            for i in range(len(xyxy)):
                box = xyxy[i].cpu().numpy() if hasattr(xyxy[i], 'cpu') else xyxy[i]
                ci = int(cls_idx[i].item()) if cls_idx is not None else 0
                name = model.names.get(ci, str(ci)) if hasattr(model, 'names') else str(ci)
                conf_val = float(confidences[i].item()) if confidences is not None else 0.0
                if name.lower() in ignore_set:
                    continue
                boxes.append(box)
                labels.append(name)
                confs.append(conf_val)

        # annotate
        annotated = annotate_frame(frame.copy(), boxes, labels, confs, getattr(model, 'names', {}))

        # compute wet/dry percentages for on-screen display
        wet_pct, dry_pct = compute_wet_dry_percentages(labels, confs, conf_threshold=args.conf)
        overlay = f"Wet: {wet_pct:.1f}%  Dry: {dry_pct:.1f}%"
        cv2.putText(
            annotated,
            overlay,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 200, 0),
            2,
            cv2.LINE_AA,
        )

        # update MJPEG buffer
        global latest_jpeg
        ok, buf = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            latest_jpeg = buf.tobytes()

        # determine top detection
        top_text = "-"
        if confs:
            best_idx = int(max(range(len(confs)), key=lambda i: confs[i]))
            top_text = f"{labels[best_idx]}:{confs[best_idx]:.2f}"

        # classify waste state and hazard from detected labels
        waste_state, hazard, hazard_type = classify_waste_and_hazard(labels, confs, conf_threshold=args.conf)

        # write extended detected info as JSON-like text for ESP32/ESP8266
        detected_payload = {
            'prediction': top_text,
            'waste_state': waste_state,
            'hazard': int(hazard),
            'hazard_type': hazard_type,
        }

        if args.write_detected:
            # write compact JSON-like single-line to detected file
            try:
                import json

                write_detected_file(Path(args.write_detected), json.dumps(detected_payload))
            except Exception:
                # fallback: write simple string
                write_detected_file(Path(args.write_detected), top_text)

        # optionally post to server
        if args.server_url:
            resp = post_frame_to_server(args.server_url, frame)
            if resp and isinstance(resp, dict) and resp.get('prediction'):
                top_text = resp.get('prediction')
                # prefer server-provided waste_state/hazard if present
                srv_waste_state = resp.get('waste_state') or resp.get('soil_state')
                srv_hazard = resp.get('hazard')
                srv_hazard_type = resp.get('hazard_type')
                if args.write_detected:
                    try:
                        import json
                        payload = {'prediction': top_text, 'waste_state': srv_waste_state or waste_state, 'hazard': int(srv_hazard) if srv_hazard is not None else int(hazard), 'hazard_type': srv_hazard_type or hazard_type}
                        write_detected_file(Path(args.write_detected), json.dumps(payload))
                    except Exception:
                        write_detected_file(Path(args.write_detected), top_text)

        if not args.no_display:
            cv2.imshow("YOLO Realtime", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(args.interval)

    cap.release()
    cv2.destroyAllWindows()
    return True


def run_fallback_posting(args):
    # fallback: capture frames and POST to server like post_webcam_demo
    import requests

    cap = open_capture(args.camera_device, args.width, args.height)
    if cap is None or not cap.isOpened():
        print("Cannot open webcam (tried default and DirectShow)")
        return False

    print("ultralytics not available — posting frames to server")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        # show preview
        if not args.no_display:
            cv2.imshow("YOLO Realtime (POST)", frame)

        # post
        resp = post_frame_to_server(args.server_url, frame) if args.server_url else None
        top_text = '-'
        if resp and isinstance(resp, dict):
            top_text = resp.get('prediction', top_text)
            # prefer server-side waste_state if provided
            waste_state = resp.get('waste_state') or resp.get('soil_state')
            hazard = resp.get('hazard')
            hazard_type = resp.get('hazard_type')
            if args.write_detected:
                try:
                    import json
                    payload = {'prediction': top_text, 'waste_state': waste_state, 'hazard': int(hazard) if hazard is not None else 0, 'hazard_type': hazard_type}
                    write_detected_file(Path(args.write_detected), json.dumps(payload))
                except Exception:
                    write_detected_file(Path(args.write_detected), top_text)
        else:
            if args.write_detected:
                write_detected_file(Path(args.write_detected), top_text)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(args.interval)

    cap.release()
    cv2.destroyAllWindows()
    return True


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--model', help='Path to YOLO model (optional)')
    p.add_argument('--server-url', help='Backend /image endpoint to POST frames (optional)')
    p.add_argument('--write-detected', help='Path to write detected label (e.g. backend/ai/yolo/detected.txt)')
    p.add_argument('--conf', type=float, default=0.35, help='Confidence threshold (higher = fewer false positives)')
    p.add_argument('--ignore-classes', default='person', help='Comma-separated class names to ignore (default: person)')
    p.add_argument('--device', default='cpu', help='YOLO device (cpu, cuda, 0, 0,1,2,3 etc)')
    p.add_argument('--camera-device', default=0, type=int, help='Webcam device index (int)')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--interval', type=float, default=0.2, help='Seconds between frames')
    p.add_argument('--no-display', action='store_true', help='Do not show preview window')
    p.add_argument('--mjpeg-port', type=int, default=8090, help='Port for MJPEG stream of annotated frames')
    return p.parse_args()


def main():
    args = parse_args()

    YOLO = try_import_ultralytics()
    if YOLO and (args.model or True):
        ok = run_local_model(args)
        if ok:
            return

    # fallback to posting to server if local isn't available
    if args.server_url:
        run_fallback_posting(args)
    else:
        print("No local model available and no --server-url provided. Nothing to do.")


if __name__ == '__main__':
    main()
# Content moved from backend_file_review/realtime_yolo_taco.py
# Placeholder content for now, will move the actual content
