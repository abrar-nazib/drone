# Import libraries
import cv2
from ultralytics import YOLO
import time
import os
from environs import Env

CURRENT_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(CURRENT_FILE_DIR)
ENV_FILE_PATH = os.path.join(ROOT_DIR, ".env")

# Load environment variables from .env file
env = Env()
env.read_env(ENV_FILE_PATH)

STREAM_URL = env.str("RASPI_IP", "0")
if STREAM_URL == "0":
    STREAM_URL = 0
else:
    STREAM_URL = f"http://{STREAM_URL}:7123/stream.mjpg"

# Load a model
model = YOLO("yolov8n-seg.pt")  # load an official model
# model = YOLO('path/to/best.pt')  # load a custom model

# Open the camera
cap = cv2.VideoCapture(STREAM_URL)

# Loop over the frames
while True:
    # Read a frame
    ret, frame = cap.read()
    if not ret:
        break
    # Predict with the model
    results = model(frame, verbose=False)
    for result in results:
        # print(result.bbox)
        try:
            points = result.masks.xy[0]
            for point in points:
                cx, cy = int(point[0]), int(point[1])
                # Draw small circles
                cv2.circle(frame, (cx, cy), 2, (0, 0, 0), 1)
        # Show the predicted image
        except Exception as e:
            # print(e)
            continue
        cv2.imshow("Object Segmentation From Drone", frame)
    #   time.sleep(5)
    # Wait for a key press
    key = cv2.waitKey(1)
    if key == 27:  # ESC key
        break

# Release the camera and destroy the windows
cap.release()
cv2.destroyAllWindows()
