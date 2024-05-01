from ultralytics import YOLO
import cv2
import pandas

model = YOLO('yolov8s.pt')

# Run inference on live camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    results = model(frame, verbose=False)

    for result in results:
        box_array = result.boxes.data
        for box in box_array:
            try:
                print(box)
                x1, y1, x2, y2, id, _, = box
                x1, y1, x2, y2, id, _= int(x1), int(y1), int(x2), int(y2), int(id), int(_)
                # Draw bounding boxes
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            except Exception as e:
                print(e)
                continue



    cv2.imshow('Object Detection Drone', frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()

