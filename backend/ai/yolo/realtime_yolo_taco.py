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
from pathlib import Path
import cv2


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

    # broaden keyword lists to typical waste items
    keywords_wet = ('wet', 'water', 'mud', 'liquid', 'food', 'banana', 'apple', 'vegetable', 'organic', 'food_waste', 'peel')
    keywords_dry = ('dry', 'paper', 'cardboard', 'plastic', 'metal', 'glass', 'cloth', 'fabric')
    keywords_hazard = ('battery', 'chemical', 'glass', 'sharp', 'hazard', 'flammable', 'rust', 'acid', 'alkali')

    for lab, c in zip(labels, confs):
        if c < conf_threshold:
            continue
        ll = lab.lower()
        if any(k in ll for k in keywords_wet):
            waste_state = 'wet'
        if any(k in ll for k in keywords_dry) and waste_state == 'unknown':
            waste_state = 'dry'
        for hk in keywords_hazard:
            if hk in ll:
                hazard = 1
                hazard_type = lab
                break
        if hazard:
            break

    return waste_state, hazard, hazard_type


def run_local_model(args):
    YOLO = try_import_ultralytics()
    if YOLO is None:
        print("ultralytics not installed — local inference unavailable")
        return False

    model_path = args.model or "yolov8n.pt"
    print("Loading model:", model_path)
    model = YOLO(model_path)

    # open webcam
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print("Cannot open webcam")
        return False

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

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
                boxes.append(box)
                ci = int(cls_idx[i].item()) if cls_idx is not None else 0
                name = model.names.get(ci, str(ci)) if hasattr(model, 'names') else str(ci)
                labels.append(name)
                confs.append(float(confidences[i].item()) if confidences is not None else 0.0)

        # annotate
        annotated = annotate_frame(frame.copy(), boxes, labels, confs, getattr(model, 'names', {}))

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

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print("Cannot open webcam")
        return False

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

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
    p.add_argument('--conf', type=float, default=0.25, help='Confidence threshold')
    p.add_argument('--device', default=0, help='Webcam device index or cpu/cuda for ultralytics')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--interval', type=float, default=0.2, help='Seconds between frames')
    p.add_argument('--no-display', action='store_true', help='Do not show preview window')
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
