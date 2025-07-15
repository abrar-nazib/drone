import cv2

RASPBERRY_PI_IP = "192.168.0.204"


# Replace 'raspberrypi_ip' with your Raspberry Pi's IP address
stream_url = f"http://{RASPBERRY_PI_IP}:7123/stream.mjpg"

cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Error: Could not open the stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    cv2.imshow("Raspberry Pi Camera Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
