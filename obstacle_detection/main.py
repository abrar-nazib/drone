import cv2
from ultralytics import YOLO
import time
import os
from environs import Env
import numpy as np

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

# Open the camera
cap = cv2.VideoCapture(STREAM_URL)


# Function to generate a color based on class index
def get_color(class_index):
    hue = (class_index * 5) % 180  # Vary hue for distinct colors (0-179 in OpenCV)
    color = cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
    return tuple(int(c) for c in color)


# Loop over the frames
while True:
    for _ in range(5):
        ret, frame = cap.read()
        if not ret:
            break

    # Predict with the model
    results = model(frame, verbose=False)
    result = results[0]  # Single result per frame

    if result.masks is not None:
        # Get masks and class labels
        masks = result.masks.data.cpu().numpy()  # Shape: (num_masks, height, width)
        classes = result.boxes.cls.cpu().numpy()  # Class indices
        blended = frame.copy()
        alpha = 0.5  # Opacity of the mask (0.0 = fully transparent, 1.0 = fully opaque)

        # Process each detected object's mask
        for i in range(len(masks)):
            mask = (
                masks[i] > 0
            )  # Convert to boolean mask (assuming float32 with 0.0/1.0)
            class_index = int(classes[i])
            color = get_color(class_index)
            color_layer = np.full_like(frame, color)  # Create a solid color image

            # Blend the color layer with the frame where the mask is active
            blended[mask] = cv2.addWeighted(
                blended[mask], 1 - alpha, color_layer[mask], alpha, 0
            )

        cv2.imshow("Object Segmentation From Drone", blended)
    else:
        # No detections, show original frame
        cv2.imshow("Object Segmentation From Drone", frame)

    key = cv2.waitKey(1)
    if key == 27:  # ESC key
        break

# Release the camera and destroy the windows
cap.release()
cv2.destroyAllWindows()
