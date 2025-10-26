from ultralytics import YOLO
import cv2
import numpy as np
import serial
import time

ARDUINO_PORT = '/dev/cu.usbserial-1130' 
BAUD_RATE = 9600

timer = 0
TARGET_CLASS_ID = 67 
CLASS_NAME = 'cell phone'

last_successful_cX = 0 
last_successful_cY = 0

frames_since_last_detection = 0 
MAX_LOST_FRAMES = 10


try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0, write_timeout=0)
    time.sleep(2) 
except serial.SerialException as e:
    exit()

model = YOLO("yolov8s.pt") 
cap = cv2.VideoCapture(0)

image_center_x, image_center_y = 0, 0
has_center = False


def track_object(frame):
    global image_center_x, image_center_y, has_center, last_successful_cX, last_successful_cY, frames_since_last_detection
    
    if not has_center:
        height, width, channels = frame.shape
        image_center_x = int(width / 2)
        image_center_y = int(height / 2)
        has_center = True

    detected_centers = []
    results = model(frame, stream=False, verbose=False)
    
    target_found_in_frame = False

    if results and results[0].boxes:
        boxes = results[0].boxes
        target_detections = boxes[boxes.cls == TARGET_CLASS_ID]
        
        if len(target_detections) > 0:
            for box in target_detections:
                conf = box.conf[0].cpu().item() 
                if (conf > 0.4): 
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    
                    # Calculate center relative to image center (cX is the error signal)
                    cX = int((x1 + x2) / 2) - image_center_x
                    cY = int((y1 + y2) / 2) - image_center_y
                    
                    center_abs = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    detected_centers.append((cX, cY))
                    
                    # Update State (Found Target)
                    last_successful_cX = cX
                    last_successful_cY = cY
                    
                    frames_since_last_detection = 0
                    target_found_in_frame = True
                    
                    # Drawing/Annotation
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, center_abs, 5, (0, 0, 255), -1)
                    text_label = f"{CLASS_NAME} ({conf:.2f})"
                    coord_label = f"X:{cX}, Y:{cY}"
                    cv2.putText(frame, text_label, (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, coord_label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    break # We only need the first detection for tracking
            

        

    if not target_found_in_frame:
        frames_since_last_detection += 1
    return frame, detected_centers


while True:
    success, frame = cap.read()
    if not success:
        print("Failed to read frame from camera.")
        break
    
    # Process the frame (keeping the flip from your previous attempt
    processed_frame, positions = track_object(frame)

    timer += 1

    # Draw the absolute center line for visual reference
    if has_center:
        cv2.line(processed_frame, (image_center_x, 0), (image_center_x, frame.shape[0]), (255, 255, 0), 1)

    cv2.imshow("YOLO Tracker", processed_frame)


    if frames_since_last_detection < MAX_LOST_FRAMES:
        target_cX = last_successful_cX
        target_cY = last_successful_cY
        status_text = f"TRACKING (Last known X: {target_cX})"
    else:
        target_cX = 0 
        taret_cY = 0
        status_text = "LOST - STOPPED"

    
    data_for_arduino = f"{target_cX},{target_cY}\n"
    
    try:
        arduino.write(data_for_arduino.encode())
    except serial.SerialTimeoutException:
        pass

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
