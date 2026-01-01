import ollama
from pydantic import BaseModel, Field

class ActionBook(BaseModel):
    velocity_x: float = Field(..., description="Linear velocity of car (m/s)")
    angular_z: float = Field(..., description="Angular velocity around the z axis (rad/s)")

SYSTEM_PROMPT = """You control a robot with a camera. First, determine if you can see the target object in the image.

## Step 1: Can you see the target?
- If NO (target not in image): velocity_x = 0.0, angular_z = 0.5 (STOP and ROTATE to search)
- If YES: proceed to step 2

## Step 2: Where is the target in the image?
- LEFT side of image → velocity_x = 0.5, angular_z = 0.3
- CENTER of image → velocity_x = 1.0, angular_z = 0.0 (go straight!)
- RIGHT side of image → velocity_x = 0.5, angular_z = -0.3

## CRITICAL RULES
- You MUST NOT move forward (velocity_x > 0) unless you can clearly see the target in the image!
- If you cannot find the target object anywhere in the image, set velocity_x = 0 and rotate!
- Only when target is visible AND centered should you drive straight.
"""

def process_image_with_text(image_base64: str, target: str) -> ActionBook:
    user_prompt = f"TARGET: {target}\n\nFirst: Is the target visible in this image? If NO, output velocity_x=0 and angular_z=0.5 to rotate and search. If YES, output commands to navigate toward it."

    response = ollama.chat(
        model='llama3.2-vision:11b',
        messages=[
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': user_prompt, 'images': [image_base64]}
        ],
        format=ActionBook.model_json_schema()
    )
    return ActionBook.model_validate_json(response['message']['content'])
