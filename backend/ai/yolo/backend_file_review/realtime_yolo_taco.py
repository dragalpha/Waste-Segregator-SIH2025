import os
import time
import threading
import cv2
import torch
import numpy as np
from ultralytics import YOLO
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("application.log"),
        logging.StreamHandler()
    ]
)

# --------------- CONFIG ---------------
MODEL_PATH = "./taco-yolov5s.pt"
FALLBACK_MODEL = "yolov8n.pt"
WEBCAM_INDEX = 0

# Inference tuning
INFER_SIZE = 640            # model-friendly size (square). Lower -> faster but less detail
CONF_THRESHOLD = 0.20       # detection threshold (lower for debugging)
MAP_CONF_TO_HAZARD = 0.25

# Performance tuning
SKIP_FRAMES = 1             # process every SKIP_FRAMES-th frame (1 = every frame, 2 = every other)
USE_HALF_ON_CUDA = True     # use fp16 on CUDA for speed
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720

# ---------------- mapping & helpers ----------------
LABEL_TO_CATEGORY = {
    "paper": "dry", "cardboard": "dry", "plastic": "dry", "bottle": "dry",
    "can": "dry", "metal": "dry", "glass": "dry",
    "food": "wet", "organic": "wet", "liquid": "wet", "wet_waste": "wet", "trash": "wet",
    "needle": "hazard", "broken_glass": "hazard", "battery": "hazard",
    "drum": "hazard", "oil": "hazard", "chemical": "hazard",
}
DEFAULT_CATEGORY = "dry"

def map_to_category(label, conf):
    name = label.lower()
    if name in LABEL_TO_CATEGORY:
        return LABEL_TO_CATEGORY[name]
    if "broken" in name and "glass" in name:
        return "hazard"
    if "oil" in name or "drum" in name or "battery" in name:
        return "hazard"
    if conf < MAP_CONF_TO_HAZARD:
        return "hazard"
    return DEFAULT_CATEGORY

def box_color(cat):
    if cat == "dry": return (0, 255, 0)
    if cat == "wet": return (0, 165, 255)
    return (0, 0, 255)

def preprocess_underwater(img):
    """Apply preprocessing for underwater visuals, including color correction and dehazing."""
    # Convert to LAB color space for color correction
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))
    img_corrected = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # Apply dehazing using a simple dark channel prior method
    dark_channel = cv2.min(cv2.min(img_corrected[:, :, 0], img_corrected[:, :, 1]), img_corrected[:, :, 2])
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    dark_channel = cv2.erode(dark_channel, kernel)
    transmission = 1 - (dark_channel / 255.0)
    img_dehazed = (img_corrected / transmission[:, :, None]).clip(0, 255).astype(np.uint8)

    return img_dehazed

def letterbox(img, new_size=INFER_SIZE, color=(114,114,114)):
    """Resize and pad image to square new_size (OpenCV compatible). Returns resized image and scale/pads."""
    # Apply underwater preprocessing
    img = preprocess_underwater(img)

    h0, w0 = img.shape[:2]
    r = float(new_size) / max(h0, w0)
    new_unpad = (int(round(w0 * r)), int(round(h0 * r)))
    img_resized = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    dw = new_size - new_unpad[0]
    dh = new_size - new_unpad[1]
    top, bottom = dh // 2, dh - dh // 2
    left, right = dw // 2, dw - dw // 2
    img_padded = cv2.copyMakeBorder(img_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img_padded, r, left, top

# ---------------- model loader ----------------
def load_model():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    try:
        logging.info(f"Loading YOLOv8 model: {FALLBACK_MODEL}")
        model = YOLO(FALLBACK_MODEL)
        model.to(device)  # Move model to device
        logging.info(f"YOLOv8 loaded on {device}")
        logging.debug(f"v8 names: {model.model.names if hasattr(model, 'model') and hasattr(model.model, 'names') else None}")
        return model, "v8", device
    except Exception as e:
        logging.error(f"YOLOv8 load failed: {e}")
        raise RuntimeError("YOLOv8 model loading failed.")

# ---------------- capture thread (keeps latest frame) ----------------
class CameraReader:
    def __init__(self, index=0, w=CAPTURE_WIDTH, h=CAPTURE_HEIGHT):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.lock = threading.Lock()
        self.frame = None
        self.running = False
        self.thread = None

    def start(self):
        if not self.cap.isOpened():
            raise RuntimeError("Could not open webcam")
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = frame

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=0.5)
        self.cap.release()

# ---------------- main ----------------
def main():
    model, model_type, device = load_model()
    print(f"[INFO] Device: {device} | Model type: {model_type} | INFER_SIZE: {INFER_SIZE} | SKIP_FRAMES: {SKIP_FRAMES}")

    # If CUDA available, try fp16 for speed
    use_half = False
    if device.startswith("cuda") and USE_HALF_ON_CUDA:
        try:
            if model_type == "v8":
                # ultralytics wrapper: try to set underlying model to half
                try:
                    model.model.half()
                except Exception:
                    pass
            else:
                try:
                    model.model.half()
                except Exception:
                    pass
            use_half = True
            print("[INFO] Using FP16 on GPU (half precision) for speed")
        except Exception:
            use_half = False

    # Warmup: run a fake inference to warm kernels (optional)
    try:
        if model_type == "v8":
            model.predict(np.zeros((INFER_SIZE, INFER_SIZE, 3), dtype=np.uint8), conf=CONF_THRESHOLD, verbose=False)
        else:
            dummy = torch.zeros(1, 3, INFER_SIZE, INFER_SIZE)
            if device.startswith("cuda"):
                dummy = dummy.to(device)
                if use_half: dummy = dummy.half()
            with torch.no_grad():
                model(dummy)
        print("[INFO] Warmup done")
    except Exception:
        pass

    cam = CameraReader(WEBCAM_INDEX, CAPTURE_WIDTH, CAPTURE_HEIGHT)
    try:
        cam.start()
    except RuntimeError as e:
        print("[ERROR]", e)
        return

    prev_time = time.time()
    frame_idx = 0

    try:
        while True:
            raw = cam.read()
            if raw is None:
                time.sleep(0.01)
                continue

            frame_idx += 1
            # Always display raw, but only infer on SKIP_FRAMES
            do_infer = (frame_idx % SKIP_FRAMES) == 0

            detections = []
            infer_time = 0.0
            try:
                if do_infer:
                    # Resize+pad to square INFER_SIZE for model
                    img_in, scale, pad_x, pad_y = letterbox(raw, INFER_SIZE)

                    start = time.time()
                    if model_type == "v5":
                        # prepare tensor (RGB, normalize, batch)
                        img_rgb = cv2.cvtColor(img_in, cv2.COLOR_BGR2RGB)
                        img_float = img_rgb.astype(np.float32) / 255.0
                        tensor = torch.from_numpy(img_float).permute(2,0,1).unsqueeze(0)
                        if device.startswith("cuda"):
                            tensor = tensor.to(device)
                            if use_half: tensor = tensor.half()
                        with torch.no_grad():
                            results = model(tensor)
                        # parse results
                        if hasattr(results, "xyxy") and len(results.xyxy) > 0:
                            dets = results.xyxy[0].cpu().numpy()
                            for row in dets:
                                if len(row) < 6: continue
                                x1, y1, x2, y2, conf, cls = row[:6]
                                # map back to original frame coordinates (reverse letterbox)
                                x1o = int((x1 - pad_x) / scale)
                                y1o = int((y1 - pad_y) / scale)
                                x2o = int((x2 - pad_x) / scale)
                                y2o = int((y2 - pad_y) / scale)
                                label = model.names[int(cls)] if hasattr(model, "names") else str(int(cls))
                                if conf >= CONF_THRESHOLD:
                                    detections.append(((x1o,y1o,x2o,y2o), float(conf), label))

                    else:  # v8 branch
                        try:
                            results = model.predict(img_in, conf=CONF_THRESHOLD, device=device, verbose=False)
                        except TypeError:
                            results = model.predict(img_in, conf=CONF_THRESHOLD, verbose=False)
                        if len(results) > 0:
                            r = results[0]
                            if hasattr(r, "boxes") and len(r.boxes) > 0:
                                for i in range(len(r.boxes)):
                                    xyxy = r.boxes.xyxy[i].cpu().numpy()
                                    conf = float(r.boxes.conf[i].cpu().numpy())
                                    cls_idx = int(r.boxes.cls[i].cpu().numpy())
                                    # map back to original image coordinates
                                    x1 = int((xyxy[0] - pad_x) / scale)
                                    y1 = int((xyxy[1] - pad_y) / scale)
                                    x2 = int((xyxy[2] - pad_x) / scale)
                                    y2 = int((xyxy[3] - pad_y) / scale)
                                    label = r.names[cls_idx] if hasattr(r, "names") else str(cls_idx)
                                    if conf >= CONF_THRESHOLD:
                                        detections.append(((x1,y1,x2,y2), conf, label))
                    infer_time = time.time() - start
            except Exception as e:
                print("[ERROR] Inference error:", e)

            # Draw detections
            for (x1,y1,x2,y2), conf, label in detections:
                cat = map_to_category(label, conf)
                color = box_color(cat)
                cv2.rectangle(raw, (x1,y1), (x2,y2), color, 2)
                text = f"{label} {conf:.2f} [{cat}]"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                y0 = max(0, y1 - th - 8)
                cv2.rectangle(raw, (x1, y0), (x1 + tw + 6, y0 + th + 6), color, -1)
                cv2.putText(raw, text, (x1+3, y0+th-2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            # Overlay debug info
            now = time.time()
            fps = 1.0 / (now - prev_time) if now != prev_time else 0.0
            prev_time = now
            info = f"FPS(display): {fps:.1f} | Infer(s): {infer_time:.3f} | Dets: {len(detections)}"
            cv2.putText(raw, info, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,0), 2)

            cv2.imshow("Waste Detection (optimized)", raw)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
