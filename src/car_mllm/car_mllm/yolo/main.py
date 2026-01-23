from ultralytics import YOLO
import cv2
import base64
import numpy as np

model = YOLO("yolov8s.pt")

def get_bboxes(image_base64):
    # Decode base64 string to numpy array
    img_bytes = base64.b64decode(image_base64)
    img_array = np.frombuffer(img_bytes, dtype=np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    height, width = img.shape[:2]

    results = model.predict(source=img, conf=0.25)

    for result in results:
        im_array = result.plot()
        # Convert numpy array back to base64 for Ollama
        _, buffer = cv2.imencode('.jpg', im_array)
        img_base64 = base64.b64encode(buffer).decode('utf-8')

        boxes = result.boxes
        info = {'img': img_base64, 'bboxes': {}, 'width': width, 'height': height}

        for box in boxes:
            cords = box.xyxy[0].tolist()
            x1, y1, x2, y2 = [round(x) for x in cords]

            cls_id = box.cls[0].item()
            cls_name = result.names[cls_id]

            info['bboxes'][cls_name] = {'x1': x1,'y1':y1,'x2':x2,'y2':y2}

        return info 

