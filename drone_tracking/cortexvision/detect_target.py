from ultralytics import YOLO
import cv2
import os
import time
import numpy as np



# Get the absolute location
def absolute_path(relative_path):
    dir = os.path.dirname(__file__)
    path = os.path.join(dir, relative_path)
    return path

model = YOLO(absolute_path('car_detector.pt'))

def inference_img(img):
    [result] = model(img, verbose=False)
    classes = result.names

    boxes = result.boxes
    detections = []
    for box in boxes:
        try: 
            data = box.data[0]
            x1, y1, x2, y2, conf, cls = int(data[0]), int(data[1]), int(data[2]), int(data[3]), float(data[4]), int(data[5])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            detections.append({
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
                'cx': cx,
                'cy': cy,
                'conf': conf,
                'class': classes[cls]
            })

        except Exception as e: 
            print(e)
    return detections

if __name__ == '__main__':

    cap = cv2.VideoCapture(absolute_path('../dji_car_track.mp4'))
    while True:
        ret, frame = cap.read()

        # Inference
        inference_img(frame)

        # Show frame to screen
        cv2.imshow('frame', frame)


        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()    