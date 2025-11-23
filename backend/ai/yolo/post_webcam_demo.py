import cv2
import requests
import time
import os

SERVER_URL = "http://127.0.0.1:8000/image"   # your backend endpoint
# write detection to backend path so ESP polls the correct place
DETECT_FILE = "backend/ai/yolo/detected.txt"

def send_frame(frame):
    # encode frame as JPEG
    ret, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    if not ret:
        print("Encoding failed")
        return None

    files = {
        "image": ("frame.jpg", encoded.tobytes(), "image/jpeg")
    }

    try:
        r = requests.post(SERVER_URL, files=files, timeout=10)
        if r.status_code == 200:
            data = r.json()
            print("Prediction:", data.get("prediction", "-"))
            return data
        else:
            print("Server error:", r.status_code)
    except Exception as e:
        print("Request error:", e)

    return None


def main():
    print("Opening webcam...")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ ERROR: Cannot access webcam")
        return

    # reduce resolution to avoid lag
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Webcam started. Press Q to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        # show live preview
        cv2.imshow("Waste Segregator Webcam Preview", frame)

        # send frame to backend
        resp = send_frame(frame)

        # store detection for ESP8266
        if resp:
                try:
                    os.makedirs(os.path.dirname(DETECT_FILE), exist_ok=True)
                    with open(DETECT_FILE, "w") as f:
                        f.write(resp.get("prediction", "-"))
                except Exception as e:
                    print("Failed to write detected file:", e)

        # exit on Q
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.3)  # send ~3 frames/sec

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
