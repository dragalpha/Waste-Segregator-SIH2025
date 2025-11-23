# ai/yolo/inference_yolov8.py
import argparse, sys, os, json
from ultralytics import YOLO

parser = argparse.ArgumentParser()
parser.add_argument("--image", required=True, help="Path to image file")
parser.add_argument("--model", default="./backend/ai/yolo/yolov8n.pt", help="Model path")
parser.add_argument("--conf", type=float, default=0.25)
args = parser.parse_args()

MODEL_CANDIDATES = [
    args.model,
    "./backend/ai/yolo/yolov8n.pt",
    "/mnt/data/yolov8n.pt"
]

model_path = None
for p in MODEL_CANDIDATES:
    try:
        if os.path.exists(p):
            model_path = p
            break
    except Exception:
        continue

if model_path is None:
    # allow ultralytics to try a model name
    model_path = args.model

try:
    model = YOLO(model_path)
except Exception:
    print("-")
    sys.exit(0)

results = None
try:
    results = model.predict(args.image, conf=args.conf, verbose=False)
except Exception:
    print("-")
    sys.exit(0)

if not results:
    print("-")
    sys.exit(0)

res = results[0]
if len(res.boxes) == 0:
    print("-")
    sys.exit(0)

# choose box: prefer any box >= conf threshold; else return the box with lowest confidence
boxes = res.boxes
chosen_idx = 0
try:
    confidences = [float(b.conf[0].item()) if hasattr(b.conf[0], 'item') else float(b.conf[0]) for b in boxes]
except Exception:
    confidences = [float(b.conf) for b in boxes]

# find indices above threshold
above = [i for i,c in enumerate(confidences) if c >= args.conf]
if above:
    # pick the highest confidence among those
    chosen_idx = max(above, key=lambda i: confidences[i])
else:
    # pick lowest-confidence match (as requested)
    chosen_idx = min(range(len(confidences)), key=lambda i: confidences[i])

box = boxes[chosen_idx]
try:
    cls = int(box.cls[0].item())
except Exception:
    cls = int(box.cls[0])
try:
    conf = float(box.conf[0].item())
except Exception:
    conf = float(box.conf[0])
label = res.names.get(cls, f"class_{cls}") if hasattr(res, "names") else f"class_{cls}"

print(f"{label}:{conf:.2f}")
sys.exit(0)
