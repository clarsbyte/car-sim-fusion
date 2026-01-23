import ollama
from pydantic import BaseModel, Field
from car_mllm.yolo.main import get_bboxes

class ActionBook(BaseModel):
    velocity_x: float = Field(..., description="Linear velocity of car (m/s)")
    angular_z: float = Field(..., description="Angular velocity around the z axis (rad/s)")

def get_system_prompt(width: int) -> str:
    center = width // 2
    left_third = width // 3
    right_third = (width * 2) // 3

    return f"""You control a robot with a camera. You receive:
1. An image with YOLO bounding boxes drawn on detected objects
2. A list of detected objects with their bounding box coordinates (x1, y1, x2, y2)

The image coordinate system: x=0 is LEFT edge, x increases toward RIGHT. Image center is around x={center} (for {width}px width).

## Step 1: Is the target in the detected objects list?
- If NO (target not detected): velocity_x = 0.0, angular_z = 0.5 (STOP and ROTATE to search)
- If YES: proceed to step 2

## Step 2: Use the bounding box x-coordinates to determine position
Calculate the bbox center: center_x = (x1 + x2) / 2
- center_x < {left_third} (LEFT third) → velocity_x = 0.5, angular_z = 0.3
- center_x between {left_third}-{right_third} (CENTER third) → velocity_x = 1.0, angular_z = 0.0 (go straight!)
- center_x > {right_third} (RIGHT third) → velocity_x = 0.5, angular_z = -0.3

## CRITICAL RULES
- You MUST check the detected objects list first - if target is not there, ROTATE!
- Use the bounding box coordinates for precise navigation, not just visual estimation.
- Only move forward (velocity_x > 0) when target is confirmed in detections.
"""

def process_image_with_text(image_base64: str, target: str) -> ActionBook:
    yolo_info = get_bboxes(image_base64)
    width = yolo_info['width']

    bboxes_str = "\n".join([
        f"  - {name}: x1={coords['x1']}, y1={coords['y1']}, x2={coords['x2']}, y2={coords['y2']} (center_x={(coords['x1']+coords['x2'])//2})"
        for name, coords in yolo_info['bboxes'].items()
    ]) or "  (no objects detected)"

    user_prompt = f"""TARGET: {target}

DETECTED OBJECTS (from YOLO):
{bboxes_str}

Check if "{target}" is in the detected objects list above. If not detected, rotate to search. If detected, use its bounding box center_x to navigate."""

    response = ollama.chat(
        model='gemma3',
        messages=[
            {'role': 'system', 'content': get_system_prompt(width)},
            {'role': 'user', 'content': user_prompt, 'images': [yolo_info['img']]}
        ],
        format=ActionBook.model_json_schema()
    )
    return ActionBook.model_validate_json(response['message']['content'])