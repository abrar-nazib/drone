import cv2
import torch
import time
import numpy as np
from environs import Env
import os

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

# Load a MiDas model for depth estimation
# model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
model_type = (
    "DPT_Hybrid"  # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
)
# model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

midas = torch.hub.load("intel-isl/MiDaS", model_type)

# Move model to GPU if available
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()

# Load transforms to resize and normalize the image
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
    transform = midas_transforms.dpt_transform
else:
    transform = midas_transforms.small_transform


# Open up the video capture from a webcam
cap = cv2.VideoCapture(STREAM_URL)

while cap.isOpened():

    imgs = []
    for _ in range(5):
        success, img = cap.read()
        if not success:
            break
        imgs.append(img)
    if len(imgs) == 0:
        break
    img = imgs[-1]  # Use the last frame for processing

    start = time.time()

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Apply input transforms
    input_batch = transform(img).to(device)

    # Prediction and resize to original resolution
    with torch.no_grad():
        prediction = midas(input_batch)

        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    depth_map = prediction.cpu().numpy()

    depth_map = cv2.normalize(
        depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F
    )

    end = time.time()
    totalTime = end - start

    fps = 1 / totalTime

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    depth_map = (depth_map * 255).astype(np.uint8)
    depth_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)

    # cv2.putText(img, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
    cv2.imshow("Image", img)
    cv2.imshow("Depth Map", depth_map)

    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
