from ultralytics import YOLO
import cv2
import base64
import numpy as np
from pathlib import Path

model = YOLO("yolov8s.pt")
DEFAULT_IMAGE_PATH = Path(__file__).resolve().parent / "sample_image.jpeg"
path = str(DEFAULT_IMAGE_PATH)

# Approximate fridge dimensions (meters) used to define a rectangular 3D model.
DEFAULT_FRIDGE_WIDTH_M = 0.70
DEFAULT_FRIDGE_HEIGHT_M = 1.70


def get_bboxes(image_base64=None, image_path=None):
    """Run YOLO and return annotated image+bboxes from base64 or an image path."""
    if image_base64 is not None:
        img_bytes = base64.b64decode(image_base64)
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    else:
        input_path = str(image_path) if image_path is not None else str(DEFAULT_IMAGE_PATH)
        img = cv2.imread(input_path)
        if img is None:
            raise FileNotFoundError(f"Could not read image at: {input_path}")

    height, width = img.shape[:2]

    results = model.predict(source=img, conf=0.25)

    for result in results:
        im_array = result.plot()
        # Convert numpy array back to base64 for Ollama
        _, buffer = cv2.imencode(".jpg", im_array)
        img_base64 = base64.b64encode(buffer).decode("utf-8")

        boxes = result.boxes
        info = {"img": img_base64, "bboxes": {}, "width": width, "height": height}

        for box in boxes:
            cords = box.xyxy[0].tolist()
            x1, y1, x2, y2 = [round(x) for x in cords]

            cls_id = int(box.cls[0].item())
            cls_name = result.names[cls_id]

            info["bboxes"][cls_name] = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

        return info

    # No detections: return empty bbox list and the original image.
    _, buffer = cv2.imencode(".jpg", img)
    return {
        "img": base64.b64encode(buffer).decode("utf-8"),
        "bboxes": {},
        "width": width,
        "height": height,
    }


def build_model_points(width_m=DEFAULT_FRIDGE_WIDTH_M, height_m=DEFAULT_FRIDGE_HEIGHT_M):
    """Return 4 object corners in 3D (z=0 plane), matching point order used in solvePnP."""
    half_w = width_m / 2.0
    half_h = height_m / 2.0

    # Order: bottom-left, bottom-right, top-left, top-right
    return np.array(
        [
            [-half_w, -half_h, 0.0],
            [half_w, -half_h, 0.0],
            [-half_w, half_h, 0.0],
            [half_w, half_h, 0.0],
        ],
        dtype=np.float64,
    )


def build_image_points(img_shape, bbox=None):
    """Build 2D image points in the same order as build_model_points()."""
    img_h, img_w = img_shape[:2]

    x1 = bbox['x1']
    y1 = bbox['y1']
    x2 = bbox['x2']
    y2 = bbox['y2']

    # Order: bottom-left, bottom-right, top-left, top-right
    return np.array(
        [
            [x1, y2],
            [x2, y2],
            [x1, y1],
            [x2, y1],
        ],
        dtype=np.float64,
    )


def pose_estimation(path, bbox=None, width_m=DEFAULT_FRIDGE_WIDTH_M, height_m=DEFAULT_FRIDGE_HEIGHT_M):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"Could not read image at: {path}")

    image_points = build_image_points(img.shape, bbox=bbox)
    model_points = build_model_points(width_m=width_m, height_m=height_m)

    focal_length = float(img.shape[1])
    center = (img.shape[1] / 2.0, img.shape[0] / 2.0)

    camera_matrix = np.array(
        [[focal_length, 0.0, center[0]], [0.0, focal_length, center[1]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

    dist_coeffs = np.zeros((4, 1), dtype=np.float64)  # Assume no lens distortion.

    success, rotation_vector, translation_vector = cv2.solvePnP(
        model_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    if not success:
        return {"pose": None, "success": False, "reason": "solvePnP failed"}

    axis_len_m = max(width_m, height_m)
    axis_2d, _ = cv2.projectPoints(
        np.array([(0.0, 0.0, axis_len_m)], dtype=np.float64),
        rotation_vector,
        translation_vector,
        camera_matrix,
        dist_coeffs,
    )

    p1 = (int(image_points[0][0]), int(image_points[0][1]))
    p2 = (int(axis_2d[0][0][0]), int(axis_2d[0][0][1]))

    cv2.line(img, p1, p2, (255, 0, 0), 2)
    for point in image_points:
        cv2.circle(img, (int(point[0]), int(point[1])), 5, (0, 0, 255), -1)

    cv2.imshow("Fridge Annotation", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return {
        "pose": {
            "rotation_vector": rotation_vector.reshape(-1).tolist(),
            "translation_vector": translation_vector.reshape(-1).tolist(),
        },
        "success": True,
        "image_points": image_points.tolist(),
        "model_points": model_points.tolist(),
    }


def run():
    bbox_info = get_bboxes(image_path=path)
    fridge_bbox = bbox_info["bboxes"].get("refrigerator")
    if fridge_bbox is None:
        raise ValueError("No 'refrigerator' detection found in image.")
    print(pose_estimation(path, fridge_bbox))
