from ultralytics import YOLO
import cv2
import numpy as np
import serial
def remove_glare(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #glare range
    lower_glare = np.array([0, 0, 200])
    upper_glare = np.array([179, 255, 255])
    glare_mask = cv2.inRange(hsv_frame, lower_glare, upper_glare)
    return cv2.inpaint(frame, glare_mask, inpaintRadius=3, flags=cv2.INPAINT_TELEA)

def track_object(frame):
    #gets results
    results = model(frame, stream=False, verbose=False)
    #have list of detected centers
    detected_centers = []
    #if results have boxes
    if results and results[0].boxes:
        boxes = results[0].boxes
        #access only phones inside detections
        target_detections = boxes[boxes.cls == TARGET_CLASS_ID]
        if len(target_detections) > 0:
            #iterate through all detected cell phones
            for box in target_detections:
                conf = box.conf[0].cpu().item() #confidence
                if (conf > 0.5):
                    #converts pytorch values to numpy int for processing, kind of like casting
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cX = int((x1 + x2) / 2)
                    cY = int((y1 + y2) / 2)
                    center = (cX, cY)
                    detected_centers.append(center)
                    
                    # Draw bounding box (Green)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw center point (Red)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1) 

                    # Annotate with class name, confidence, and coordinates
                    text_label = f"{CLASS_NAME} ({conf:.2f})"
                    coord_label = f"X:{cX}, Y:{cY}"
                    cv2.putText(frame, text_label, (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, coord_label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return frame, detected_centers


TARGET_CLASS_ID = 67 
CLASS_NAME = 'cell phone'

arduino = serial.Serial('COM3', 9600)

model = YOLO("yolov8n.pt") 

cap = cv2.VideoCapture(0)

while True:
    success, frame = cap.read()
    remove_glare_frame = remove_glare(frame)

    processed_frame, positions = track_object(remove_glare_frame)

    cv2.imshow("YOLOv8 Cell Phone Tracker", processed_frame)

    if positions:
        print(f"Detected {CLASS_NAME}(s) at centers: {positions}")
        arduino.write((str(positions[0]) + str(positions[1])))

    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()

