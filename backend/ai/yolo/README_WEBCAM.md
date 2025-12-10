# Webcam YOLO demo (wet/dry overlay)

This shows how to run the realtime YOLO webcam script that overlays wet/dry percentages and ignores people by default.

## Prerequisites (Windows)
- Python 3.11 installed at `C:\Users\RATUL\AppData\Local\Programs\Python\Python311\python.exe`
- Packages (already installed): `ultralytics`, `opencv-python`, `requests`
- Model weights: `backend/ai/yolo/yolov8n.pt` (present in repo)

## Run
From repo root:
```powershell
& "C:\Users\RATUL\AppData\Local\Programs\Python\Python311\python.exe" backend/ai/yolo/realtime_yolo_taco.py --model backend/ai/yolo/yolov8n.pt --device cpu --camera-device 0 --width 640 --height 480 --conf 0.35 --ignore-classes person --write-detected backend/ai/yolo/detected.txt
```
- `--device cpu` uses CPU (PyTorch without CUDA).
- `--camera-device 0` selects webcam index 0; change to 1/2 if needed.
- `--conf 0.35` increases confidence to reduce false positives.
- `--ignore-classes person` drops person detections; provide comma-separated list to ignore more.
- `--write-detected` writes the latest detection payload to the given file for ESP polling.

Controls: A preview window opens; press `q` to exit. Wet/dry percentages are drawn on the frame. Detections (after filtering) drive the overlay and the written file.

## Troubleshooting
- If the camera doesn’t open, try another index: `--camera-device 1`.
- If Windows blocks `python`, use the full path shown above.
- To run quieter, add `--no-display` (still writes detected.txt).
